


/*
READ ME

om de motoren te gebruiken 
kan via de Serial Monitor (rechts boven)
je kan commands ingeven

-de eerste letter van de commando moet weergeven welke motor je wilt laten draaien

dus v/V   voor de motor om verticaal terichten
en l/L voor de motor om te lanceeren

-om de motor te laten draaien
kan je na de letter van de motor een getal zetten om zoveel toeren te maken
postieve is met de clock mee (bij verticaal richten omhoog)
negatieve is tegen de clock mee (bij verticaal richten omlaag)

-om de snelheid te veranderen van de motors
kan je na de letter van de motor de letter d/D doen
en hier na de nieuwe delay zetten

de range om de delay tezetten zit best tussen de 350-600
de motor voor het verticaal te richten staat best op de 500 of trager

-extra methode als je voor de tweede letter in de comando l/L doet dan draait die standaardt 10 toeren de juiste richting uit

als je geen feedback krijkt op een commando dan is er geen verbinding tussen de esp en computer
en als het nog niet werkt just get good
*/

int stepsPerR = 200;

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

StappenMotor lanceerMotor(16, 17, 5, 400);
StappenMotor verticaalMotor(18, 19, 21, 500);



void setup(){
  Serial.begin(115200);
  lanceerMotor.setupMotor();
  verticaalMotor.setupMotor();
  

  Serial.println("Motoren zijn gereed");
}
void loop(){


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
}

    // if (dir == '+'){
    //   setDirection(1);
    //   draai(touren);
    // }else if (dir == '-'){
    //   setDirection(0);
    //   draai(touren);
    // }else{
    //   draai(touren);
    // }}else{
      // Serial.println("not valid");
      // draai();
      // switchDirection();
      // draai();




// #define STEP_PIN  18  // Change as needed
// #define DIR_PIN   19  // Change as needed
// // #define ENABLE_PIN  4  // Optional, if used

// void setup() {
//   Serial.begin(115200);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   // pinMode(ENABLE_PIN, OUTPUT);

//   // digitalWrite(ENABLE_PIN, LOW); // Enable driver
//   digitalWrite(DIR_PIN, LOW);   // Set direction (HIGH = one way, LOW = other way)
//   Serial.println("Stepper test starting...");
// }


// void loop() {
//   for (int i = 0 ; i < 200 ; i++){
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(D);

//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(D);
//   }
//   delay(500);
//   digitalWrite(dirPin,!digitalRead(dirPin));
//   Serial.println(D);
  
// }

// int speed = 1000;

// void loop() {
//   for (int i = 0; i < 200; i++) { // 200 steps = 1 revolution for a 1.8° stepper
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(speed*2); // Adjust speed (Lower = Faster)
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(speed);
//     Serial.println("stepping");
//   }
  
//   delay(500); // Wait 1 second
//   // digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); // Change direction
// }
// int stepsPerSec = (RPM * stepsPerR) / 60.0;
// int D = 1000000 / (2 * stepsPerSec); // delay in microseconds
// int RPM = 200.0;



// #define stepPin 16   // PUL+
// #define dirPin  17   // DIR+
// #define stepPin 17   // PUL+
// #define dirPin  19   // DIR+
// #define stepPin2 16   // PUL+
// #define dirPin2  17   // DIR+

/* direction:

HIGH met de clock
LOW tegen de clock

*/
// const int DELAY = 400;
// const int stepsPerR = 200.0;

// void setup() {

//   Serial.begin(115200);
  
//   // setup steppermotor:
//   // pinMode(stepPin, OUTPUT);
//   // pinMode(dirPin, OUTPUT);
//   // digitalWrite(dirPin, HIGH);
//   //
//   //test:
//   // draai(1);

// }

// // functions steppermotor:
// void switchDirection(){
//   digitalWrite(dirPin, !digitalRead(dirPin));
// }
// void setDirection(int dir){
//   digitalWrite(dirPin, dir);
// }
// void steps(int steps){
//   for (int i = 0; i < steps; i++) {
//       digitalWrite(stepPin, HIGH);
//       delayMicroseconds(DELAY);
//       digitalWrite(stepPin, LOW);
//       delayMicroseconds(DELAY);
//     }
// }
// void draai(){
//   steps(stepsPerR);
// }
// void steps(int steps, int delay){
//   for (int i = 0; i < steps; i++) {
//       digitalWrite(stepPin, HIGH);
//       delayMicroseconds(delay);
//       digitalWrite(stepPin, LOW);
//       delayMicroseconds(delay);
//     }
// }
// void draai(float rotaties, int delay){
//   int voleSteps = (int)(round(rotaties*stepsPerR));
//   steps(voleSteps, delay);
// }
// void draai(float rotaties){
//   draai(rotaties,DELAY);
// }
// //

// void loop() {
//   if (Serial.available()){
//     String input = Serial.readStringUntil('\n');

//     if (input.length() > 1) {
//       char dir = input.charAt(0);   //            '+' of '-'
//       float touren = input.substring(1).toFloat();

//     if (dir == '+'){
//       setDirection(1);
//       draai(touren);
//     }else if (dir == '-'){
//       setDirection(0);
//       draai(touren);
//     }else{
//       draai(touren);
//     }}else{
//       Serial.println("not valid");
//       draai();
//       switchDirection();
//       draai();
//     }
//   }
// }
  // // Ramp-up the speed
  // stepDelay = currentDelay;  Update delay
  // for (int i = 0 ; i < 50; i++){
  //   steps();
  //   delay(1000);
  // }
  // delay(3000);
  // Serial.println("50 steps");



  // Serial.print("Speed: ");
  // Serial.println(currentDelay);

  // delay(2000);  // Pause before restarting ramp-up
