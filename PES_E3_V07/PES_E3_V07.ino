// State machine voor Nerf lanceermechanisme met sensor- en BLE-functionaliteit, zonder TFT-scherm en met ESP32 servo-oplossing

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <FastLED.h>

StappenMotor lanceerMotor(16, 17, 5, 400);
StappenMotor verticaalMotor(18, 19, 21, 500);
const int stepsPerR = 200;



#define LED_PIN 13
#define NUM_LEDS 12
CRGB leds[NUM_LEDS];

VL53L0X sensor1;
VL53L0X sensor2;
const int pinSensor1 = 32;
const int pinSensor2 = 33;

MPU6050 mpu;
const int buzzer = 15;

Servo myServo;

bool lanceerReady = false;
bool lanceer = false;

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float uitrekkingVeren;

float afstand;
float hoogte;
float linksRechts;

  float besteHoek = 45.0;
  float besteV0 = 0.0;
  float besteHmax = 0.0;

#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

// State machine
enum State {
  IDLE,
  DATA_VERZAMELEN,
  BEREKENING,
  MIKKEN,
  WACHT_OP_TREK,
  READY_TO_FIRE,
  LANCEER,
  RESET
};

State currentState = IDLE;

class StappenMotor{
  public:
    int stepPin; // stappen zetten
    int dirPin; // met of tegen de klok draaien
    int enablePin; // aan of uit
    int stepdelay; // delay bepaald de snelheid
    int position;

  StappenMotor(int SP, int DP, int EP, int D, int POS){
    stepPin = SP;
    dirPin = DP;
    enablePin = EP;
    stepdelay = D;
    position = POS;

  }
  StappenMotor(int SP, int DP, int EP, int D) : StappenMotor(SP, DP, EP, D, 0) {}
  StappenMotor(int SP, int DP, int EP) : StappenMotor(SP, DP, EP, 400, 0) {} // standaard waardes.

  void setupMotor(){
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    uit();
  }
  // Direction fucties
  void setDirClockMee(){
    digitalWrite(dirPin, HIGH);
  }
  void setDirTegenClockMee(){
    digitalWrite(dirPin, LOW);
  }

  // ENable Fucties
  void aan(){
    Serial.println("motor aan");
    digitalWrite(enablePin, LOW);
  }
  void uit(){
    Serial.println("motor uit");
    digitalWrite(enablePin, HIGH); /// verbruikt geen stoom
  }

  void draai(float toeren){
    int aantalStappen = (int)(toeren*stepsPerR);
    Serial.print("toeren: ");
    Serial.println(toeren);
    Serial.print("aantalStappen: ");
    Serial.println(aantalStappen);
    if (toeren > 0){
      setDirClockMee();
    }else if (toeren < 0){
      setDirTegenClockMee();
      aantalStappen *= -1; // zodat het aantalstappen een positieve getal is
    }else{
      return; // als 0 toeren moet draaien dan niks doen
    }
    aan();
    for (int i = 0; i < aantalStappen; i++){
      // digitalWrite(stepPin, HIGH);
      // delayMicroseconds(stepdelay);
      // digitalWrite(stepPin, LOW);
      // delayMicroseconds(stepdelay);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepdelay*2);
    }
    uit();
  }

  void lanceer(){ // functie voor de lanceer motor
    draai(-10); // tegen de clock mee 10 keer draaien
  }


};


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void ChangeLED(int led, CRGB color){
  if (led >= 0 && led < NUM_LEDS) {
    leds[led] = color;  // Specifieke LED
  }
  FastLED.show();  // Update de LEDs
}

//hoogte heeft stap groote van 1,8°
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  /*
  if (Serial.available()){

  String input = Serial.readStringUntil('\n');
  input.trim();
  Serial.println("reading input as: " + input);

  if (input.length() > 1) {
    char motor = input.charAt(0);
    StappenMotor* SM;

    if (motor == 'l' || motor == 'L'){
      SM = &lanceerMotor;
    }else if (motor == 'v' || motor == 'V'){
      SM = &verticaalMotor;
    }else{
      Serial.println("---no valid motor---");
      return;
    }

    char command = input.charAt(1);

    if (command == 'd' || command == 'D'){
      if (input.length()==2){// to get the current delay
        Serial.println("delay voor motor: " + String(motor) + " is " + String(SM->stepdelay));
        return;
      }
      float newDelay = input.substring(2).toFloat();
      if (newDelay > 0) {
        SM->stepdelay = newDelay;
        Serial.println("Nieuwe delay: " + String(newDelay));
      } else {
        Serial.println("---not a valid number---");
      }
    }else if (command == 'l' || command == 'L'){
      Serial.println("Launching...");
      SM->lanceer();
    }else{
      float toeren = input.substring(1).toFloat();
      if (toeren != 0.0) {
        SM->draai(toeren);
      } else {
        Serial.println("---not a valid number or command---");
      }
    }
  }}
  
  */
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    String value = pLedCharacteristic->getValue();  // Get the value as String
    if (value.length() > 0) {
      Serial.print("Received data: ");
      Serial.println(value);

      // Parse the received data
      int separatorIndex1 = value.indexOf('&');
      int separatorIndex2 = value.indexOf('&', separatorIndex1 + 1);
      int separatorIndex3 = value.indexOf('&', separatorIndex2 + 1);
      int separatorIndex4 = value.indexOf('&', separatorIndex3 + 1);
      int separatorIndex5 = value.indexOf('&', separatorIndex4 + 1);
      int separatorIndex6 = value.indexOf('&', separatorIndex5 + 1);

      if (separatorIndex1 != -1 && separatorIndex2 != -1 && separatorIndex3 != -1) {
        String distanceStr = value.substring(0, separatorIndex1);
        String heightStr = value.substring(separatorIndex1 + 1, separatorIndex2);
        String leftRightStr = value.substring(separatorIndex2 + 1, separatorIndex3);
        String needToBuzzer = value.substring(separatorIndex3 + 1, separatorIndex4);
        String ledCommands = value.substring(separatorIndex4 + 1, separatorIndex5); // Bijv: "1:255,0,0|2:0,255,0|3:0,0,255"
        String lanceer = value.substring(separatorIndex5 + 1, separatorIndex6);

        if(distanceStr != NULL){
          currentState = BEREKENING;
        }

        if (lanceer == "lanceer") {
          currentState = LANCEER;
        }

        if(needToBuzzer == "1"){
          digitalWrite(buzzer, HIGH);
        }else{
          digitalWrite(buzzer, LOW);
        }

        // Split op '|' voor meerdere commando's
        int startIdx = 0;
        while (true) {
          int pipePos = ledCommands.indexOf('|', startIdx);
          String cmd = (pipePos == -1) ? ledCommands.substring(startIdx) : ledCommands.substring(startIdx, pipePos);

          if (cmd.length() > 0) {
            // Parse LED:R,G,B
            int colonPos = cmd.indexOf(':');
            if (colonPos == -1) return; // Fout: geen colon gevonden

            int ledNum = cmd.substring(0, colonPos).toInt();
            String colorStr = cmd.substring(colonPos + 1);

            // Split de rest in 4 delen (R,G,B,A)
            int parts[4];
            int lastIndex = 0;
            for (int i = 0; i < 4; i++) {
              int nextComma = colorStr.indexOf(',', lastIndex);
              if (nextComma == -1 && i < 3) return; // Fout: niet genoeg delen
              
              String partStr = colorStr.substring(lastIndex, nextComma != -1 ? nextComma : colorStr.length());
              parts[i] = i < 3 ? partStr.toInt() : (int)(partStr.toFloat() * 255); // Converteer opacity naar 0-255
              
              lastIndex = nextComma + 1;
            }

            // Zorg ervoor dat waarden binnen bereik zijn
            parts[0] = constrain(parts[0], 0, 255); // R
            parts[1] = constrain(parts[1], 0, 255); // G
            parts[2] = constrain(parts[2], 0, 255); // B
            parts[3] = constrain(parts[3], 0, 255); // A (nu 0-255)

            Serial.printf("LED %d: R=%d, G=%d, B=%d, A=%d\n", ledNum, parts[0], parts[1], parts[2], parts[3]);

            // Pas de kleur aan met opacity
            CRGB color = CRGB(
              (parts[0] * parts[3]) >> 8, // Gelijk aan * (opacity/255)
              (parts[1] * parts[3]) >> 8,
              (parts[2] * parts[3]) >> 8
            );
            ChangeLED(ledNum, color);
          }

          if (pipePos == -1) break;
          startIdx = pipePos + 1;
        }



        afstand = distanceStr.toFloat();
        hoogte = heightStr.toFloat();
        linksRechts = leftRightStr.toFloat();

        Serial.print("Parsed values - Distance: ");
        Serial.print(afstand);
        Serial.print(", Height: ");
        Serial.print(hoogte);
        Serial.print(", Left/Right: ");
        Serial.println(linksRechts);



      } else {
        Serial.println("Data niet correct binnengekregen");
      }
    }
  }
};


void setup() {
  Serial.begin(115200);

  lanceerMotor.setupMotor();
  verticaalMotor.setupMotor();
  

  Serial.println("Motoren zijn gereed");

  Wire.begin();

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  pinMode(pinSensor1, OUTPUT);
  pinMode(pinSensor2, OUTPUT);
  digitalWrite(pinSensor1, LOW);
  digitalWrite(pinSensor2, LOW);

  delay(150);
  digitalWrite(pinSensor1, HIGH);
  sensor1.init(true);
  sensor1.setAddress(0x30);
  delay(150);
  digitalWrite(pinSensor2, HIGH);
  sensor2.init(true);

  sensor1.startContinuous();
  sensor2.startContinuous();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 niet gevonden!");
    while (1)
      ;
  }
  mpu.CalibrateGyro();

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
    LED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  switch (currentState) {
    case IDLE:
      ledsGroen();
      if (deviceConnected) currentState = DATA_VERZAMELEN;
      break;

    case DATA_VERZAMELEN:
      if (!deviceConnected) {
        currentState = IDLE;
        break;
      }
      ledsGroen();
      updateSensors();
      break;

    case BEREKENING:
      ledsOranje();
      uitrekkingVeren = berekenVeerAfstand();
      currentState = MIKKEN;
      break;

    case MIKKEN:
      ledsOranje();
      mikkenOpHoek();
      currentState = WACHT_OP_TREK;
      break;

    case WACHT_OP_TREK:
      ledsOranje();
      if (afstandAchterSensorCorrect()) {
        digitalWrite(buzzer, HIGH);
        currentState = READY_TO_FIRE;
      }
      break;

    case READY_TO_FIRE:
      ledsRood();
      digitalWrite(buzzer, LOW);
      break;

    case LANCEER:
      ledsRood();
      draaiServo90();
      delay(2000);
      currentState = RESET;
      break;

    case RESET:
      ledsGroen();
      resetServo();
      delay(500);
      currentState = IDLE;
      break;
  }

  if (deviceConnected) {
    String value = updateSensors();
    if (currentState == READY_TO_FIRE) value += "&lanceerReady";
    pSensorCharacteristic->setValue(value.c_str());
    pSensorCharacteristic->notify();
    delay(300);
  }

  checkBLEConnection();
}

String updateSensors() {
  String strDistance = updateDistance();
  String strTilt = updateTilt();
  return strDistance + "&" + strTilt;
}

String updateDistance() {
  if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) return "TIMEOUT";
  return String(sensor1.readRangeContinuousMillimeters()) + "&" + String(sensor2.readRangeContinuousMillimeters());
}

String updateTilt() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float normAccelX = ax / 16384.0;
  float normAccelY = ay / 16384.0;
  float normAccelZ = az / 16384.0;
  float pitch = atan2(normAccelY, normAccelZ) * 180.0 / PI;
  return String(pitch, 2);
}

void ledsGroen() {
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
}
void ledsOranje() {
  fill_solid(leds, NUM_LEDS, CRGB::Orange);
  FastLED.show();
}
void ledsRood() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
}

void draaiServo90() {
  myServo.write(90);
}
void resetServo() {
  myServo.write(0);
}

bool afstandAchterSensorCorrect() {
  int sensorAfstand = sensor1.readRangeContinuousMillimeters();
  return abs(sensorAfstand - uitrekkingVeren) < 10;  // voorbeeldwaarde
}
struct SchotResultaat {
  float hoek;     // in graden
  float v0;       // in m/s
  float h_max;    // maximale hoogte
};

void berekenHoekEnSnelheid(float afstand, float minHoogte) {
  float massa = 0.3; // kg
  float g = 9.81;    // m/s²
  besteHoek = 45.0;
  besteV0 = 0.0;
  besteHmax = 0.0;

  // Zoek tussen 30° en 80°
  for (float hoek = 30.0; hoek <= 80.0; hoek += 0.5) {
    float rad = hoek * (PI / 180.0);
    float sin2theta = sin(2 * rad);
    if (sin2theta <= 0) continue;

    float v0 = sqrt((afstand * g) / sin2theta);

    float v0y = v0 * sin(rad);
    float h_max = (v0y * v0y) / (2 * g);

    if (minHoogte != -1 && h_max < minHoogte) continue;

    besteHoek = hoek;
    besteV0 = v0;
    besteHmax = h_max;
    break; // eerste goede gevonden
  }
}

float berekenVeerAfstand() {
  float k = 0.66 * 4; // N/m (gegeven)

  berekenHoekEnSnelheid(afstand, hoogte);

  float E_kin = 0.5 * 0.3 * besteV0 * besteV0; // massa = 0.3kg
  float deltaX = sqrt((2 * E_kin) / k);

  Serial.print("Gekozen hoek: ");
  Serial.println(besteHoek);
  Serial.print("Benodigde beginsnelheid v0: ");
  Serial.println(besteV0);
  Serial.print("Maximale hoogte: ");
  Serial.println(besteHmax);
  Serial.print("Benodigde veeruitrekking: ");
  Serial.println(deltaX);

  return deltaX;
}

void mikkenOpHoek() {
  berekenHoekEnSnelheid(afstand, hoogte);

  // Veronderstel dat servo 0° is horizontaal, 90° is verticaal.
  int servoPositie = map(besteHoek, 0, 90, 0, 180); // of pas aan als je setup anders is
  servoPositie = constrain(servoPositie, 0, 180);

  myServo.write(servoPositie);

  Serial.print("Servo gemikt op ");
  Serial.print(besteHoek);
  Serial.println(" graden");
}

void checkBLEConnection() {
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}