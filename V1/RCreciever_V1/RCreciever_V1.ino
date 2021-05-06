
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>

int X_STEP_PIN     =    54; // Stepper pins on the shield
int X_DIR_PIN       =   55;
int X_ENABLE_PIN    =   38;
int Y_STEP_PIN       =  60;
int Y_DIR_PIN       =   61;
int Y_ENABLE_PIN    =   56;
int Z_STEP_PIN        = 46;
int Z_DIR_PIN        =  48;
int Z_ENABLE_PIN     =  62;

unsigned long mapval = 0; // Placeholder variable for motor speeds
unsigned long turnval = 0; // Placeholder variable for turn motor adjustments


AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN); // Enable AccelStepper libray Objects
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);





//Initialize Radio Subsystem and Transmission Package structure
RF24 radio(41, 43); // CE, CSN
const byte address[6] = "00001";
struct dataStruct {
  int Lxt = 0;
  int Lyt = 0;
  int Rxt = 0;
  int Ryt = 0;
  int Lswt = 0;
  int Rswt = 0;
  char KB = 'N';
} data;


void setup() {
  //pinMode(X_STEP_PIN, OUTPUT);
  //pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  //pinMode(Y_STEP_PIN, OUTPUT);
  //pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  //pinMode(Z_STEP_PIN, OUTPUT);
  //pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW); //Actives Low, this enables the three stepper drivers
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  //digitalWrite(X_DIR_PIN, LOW); //Step Direction
  //digitalWrite(Y_DIR_PIN, LOW); //Step Direction
  //digitalWrite(Z_DIR_PIN, LOW); //Step Direction
  //digitalWrite(X_STEP_PIN, LOW); //Step PULSE pin
  //digitalWrite(Y_STEP_PIN, LOW); //Step PULSE pin
  //digitalWrite(Z_STEP_PIN, LOW); //Step PULSE pin


  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();



  stepperY.setMaxSpeed(0.001);
  stepperX.setMaxSpeed(0.001);
  stepperX.setAcceleration(500);
  stepperY.setAcceleration(500);
  stepperX.move(100000);
  stepperY.move(100000);
}



void loop() {


  if (radio.available()) {
    radio.read(&data, sizeof(data));
    //Serial.println(text);
  }








  if (data.Lxt > 506 && data.Lxt < 538) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~Joystick Deadzone

    if (data.Ryt > 539) {
      turnval = 400;
      stepperX.setMaxSpeed(turnval);
      stepperX.move(-100000);
      stepperY.setMaxSpeed(turnval);
      stepperY.move(100000);
    }
    if (data.Ryt < 510) {
      turnval = 400;
      stepperX.setMaxSpeed(turnval);
      stepperX.move(100000);
      stepperY.setMaxSpeed(turnval);
      stepperY.move(-100000);
    }
    if (data.Ryt > 515 && data.Ryt < 530) {
      stepperX.setMaxSpeed(0.001);
      stepperX.move(100000);
      stepperY.setMaxSpeed(0.001);
      stepperY.move(100000);
    }
  }

  if (data.Lxt > 539) { //~~~~~~~~~~~~~~~~~~~~ Joystick Throttle up
    if (data.Ryt > 539) {
      turnval = map(data.Ryt, 535, 1020, 1, 30);
      mapval = map(data.Lxt, 531, 900, 300, 900);
      turnval = 400;
      stepperX.setMaxSpeed(mapval - turnval);
      stepperX.move(100000);
      stepperY.setMaxSpeed(mapval + turnval);
      stepperY.move(100000);
    }
    if (data.Ryt < 510) {
      turnval = map(data.Ryt, 0, 510, 30
                    , 1);
      mapval = map(data.Lxt, 531, 900, 300, 900);
      turnval = 400;
      stepperX.setMaxSpeed(mapval + turnval);
      stepperX.move(100000);
      stepperY.setMaxSpeed(mapval - turnval);
      stepperY.move(100000);
    }
    if (data.Ryt > 515 && data.Ryt < 530) {
      mapval = map(data.Lxt, 531, 900, 300, 900);
      stepperX.setMaxSpeed(mapval);
      stepperX.move(100000);
      stepperY.setMaxSpeed(mapval);
      stepperY.move(100000);
    }
  }



  if (data.Lxt < 505) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Joystick Throttle rev
    mapval = map(data.Lxt, 521, 14, 100, 700);
    stepperX.setMaxSpeed(mapval);
    stepperX.move(-100000);
    stepperY.setMaxSpeed(mapval);
    stepperY.move(-100000);
  }






  stepperX.run();
  stepperY.run();















} //Void Loop End ~~~~~~~~~~~~~~~~~~
