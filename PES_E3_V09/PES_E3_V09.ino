// State machine voor Nerf lanceermechanisme met sensor- en BLE-functionaliteit, zonder TFT-scherm en met ESP32 servo-oplossing

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <FastLED.h>

const int stepsPerR = 200;

#define LED_PIN 27
#define NUM_LEDS 12
CRGB leds[NUM_LEDS];

const int buzzer = 14;

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class StappenMotor {
public:
  int stepPin;    // stappen zetten
  int dirPin;     // met of tegen de klok draaien
  int enablePin;  // aan of uit
  int stepdelay;  // delay bepaald de snelheid
  int position;

  StappenMotor(int SP, int DP, int EP, int D, int POS) {
    stepPin = SP;
    dirPin = DP;
    enablePin = EP;
    stepdelay = D;
    position = POS;
  }
  StappenMotor(int SP, int DP, int EP, int D)
    : StappenMotor(SP, DP, EP, D, 0) {}
  StappenMotor(int SP, int DP, int EP)
    : StappenMotor(SP, DP, EP, 400, 0) {}  // standaard waardes.

  void setupMotor() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    uit();
  }
  // Direction fucties
  void setDirClockMee() {
    digitalWrite(dirPin, HIGH);
  }
  void setDirTegenClockMee() {
    digitalWrite(dirPin, LOW);
  }

  // ENable Fucties
  void aan() {
    Serial.println("motor aan");
    digitalWrite(enablePin, LOW);
  }
  void uit() {
    Serial.println("motor uit");
    digitalWrite(enablePin, HIGH);  /// verbruikt geen stoom
  }

  void draai(float toeren) {
    int aantalStappen = (int)(toeren * stepsPerR);
    Serial.print("toeren: ");
    Serial.println(toeren);
    Serial.print("aantalStappen: ");
    Serial.println(aantalStappen);
    if (toeren > 0) {
      setDirClockMee();
    } else if (toeren < 0) {
      setDirTegenClockMee();
      aantalStappen *= -1;  // zodat het aantalstappen een positieve getal is
    } else {
      return;  // als 0 toeren moet draaien dan niks doen
    }
    aan();
    for (int i = 0; i < aantalStappen; i++) {
      // digitalWrite(stepPin, HIGH);
      // delayMicroseconds(stepdelay);
      // digitalWrite(stepPin, LOW);
      // delayMicroseconds(stepdelay);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepdelay * 2);
    }
    uit();
  }

  void lanceer() {  // functie voor de lanceer motor
    draai(-10);     // tegen de clock mee 10 keer draaien
  }
};

StappenMotor lanceerMotor(16, 17, 5, 400);
StappenMotor verticaalMotor(18, 19, 23, 500);

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

void ChangeLED(int led, CRGB color) {
  if (led >= 0 && led < NUM_LEDS) {
    leds[led] = color;  // Specifieke LED
  }
  FastLED.show();  // Update de LEDs
}

//hoogte heeft stap groote van 1,8°
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
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


      String motor = value.substring(0, separatorIndex1);
      String afstand = value.substring(separatorIndex1 + 1, separatorIndex2);
      String needToBuzzer = value.substring(separatorIndex2 + 1, separatorIndex3);
      String ledCommands = value.substring(separatorIndex3 + 1, separatorIndex4);  // Bijv: "1:255,0,0|2:0,255,0|3:0,0,255"

      StappenMotor* SM;

      if (motor == "l" || motor == "L") {
        SM = &lanceerMotor;
      } else if (motor == "v" || motor == "V") {
        SM = &verticaalMotor;
      } else {
        Serial.println("---no valid motor---");
        return;
      }

      float toeren = afstand.toFloat();
      if (toeren != 0.0) {
        SM->draai(toeren);
      } else {
        Serial.println("---not a valid number or command---");
      }
    
      if (needToBuzzer == "1") {
        digitalWrite(buzzer, HIGH);
      } else {
        digitalWrite(buzzer, LOW);
      }

      // Split op '|' voor merdere commando's
      int startIdx = 0;
      while (true) {
        int pipePos = ledCommands.indexOf('|', startIdx);
        String cmd = (pipePos == -1) ? ledCommands.substring(startIdx) : ledCommands.substring(startIdx, pipePos);

        if (cmd.length() > 0) {
          // Parse LED:R,G,B
          int colonPos = cmd.indexOf(':');
          if (colonPos == -1) return;  // Fout: geen colon gevonden

          int ledNum = cmd.substring(0, colonPos).toInt();
          String colorStr = cmd.substring(colonPos + 1);

          // Split de rest in 4 delen (R,G,B,A)
          int parts[4];
          int lastIndex = 0;
          for (int i = 0; i < 4; i++) {
            int nextComma = colorStr.indexOf(',', lastIndex);
            if (nextComma == -1 && i < 3) return;  // Fout: niet genoeg delen

            String partStr = colorStr.substring(lastIndex, nextComma != -1 ? nextComma : colorStr.length());
            parts[i] = i < 3 ? partStr.toInt() : (int)(partStr.toFloat() * 255);  // Converteer opacity naar 0-255

            lastIndex = nextComma + 1;
          }

          // Zorg ervoor dat waarden binnen bereik zijn
          parts[0] = constrain(parts[0], 0, 255);  // R
          parts[1] = constrain(parts[1], 0, 255);  // G
          parts[2] = constrain(parts[2], 0, 255);  // B
          parts[3] = constrain(parts[3], 0, 255);  // A (nu 0-255)

          Serial.printf("LED %d: R=%d, G=%d, B=%d, A=%d\n", ledNum, parts[0], parts[1], parts[2], parts[3]);

          // Pas de kleur aan met opacity
          CRGB color = CRGB(
            (parts[0] * parts[3]) >> 8,  // Gelijk aan * (opacity/255)
            (parts[1] * parts[3]) >> 8,
            (parts[2] * parts[3]) >> 8);
          ChangeLED(ledNum, color);
        }

        if (pipePos == -1) break;
        startIdx = pipePos + 1;
      }
    }
  }
};


void setup() {
  Serial.begin(115200);
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
  pAdvertising->start();
  Serial.println("Waiting a client connection to notify...");

  lanceerMotor.setupMotor();
  verticaalMotor.setupMotor();

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
}

void loop() {
  checkBLEConnection();
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
