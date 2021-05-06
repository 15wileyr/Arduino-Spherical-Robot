
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


bool Xhigh = true;
bool Xlow = true;
bool XNextStep = true;
bool Xstart = true;
bool reverse=false;
bool stoppeddeadinitstracks=true;
bool Xdirlow=true;

int X_STEP_PIN     =    54; // Stepper pins on the shield
int X_DIR_PIN       =   55;
int X_ENABLE_PIN    =   38;
int Y_STEP_PIN       =  60;
int Y_DIR_PIN       =   61;
int Y_ENABLE_PIN    =   56;
int Z_STEP_PIN        = 46;
int Z_DIR_PIN        =  48;
int Z_ENABLE_PIN     =  62;
int i = 0;

int n = 200; //Acceleration!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!!

float pi = 3.1415926535897932384626433832795;
float radius = 150000; // [micrometers] Outer Shell Radius !!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!!
float steplength = pi * 2 * radius * 0.005; //[micrometers]
float Xaccel = 0.001; // [meters/sec^2]  Max AcceleratSHEEON !!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!!
float Xstepaccel = Xaccel * (1 / (pi * 2 * radius * 0.0025)); // [steps/sec^2]
float Throttle = 0;
float Xlaststepspeed = 0; //[step/sec]
float Xlastspeed = 0;
float Xsteplength = 5000;
float Xhalfsteplength = 2500;


unsigned long mapval = 0; // Placeholder variable for motor speeds
unsigned long turnval = 0; // Placeholder variable for turn motor adjustments
unsigned long Xstartstep = 0;
unsigned long Xendstep = 1000000;
unsigned long Xlasttime = 0;
unsigned long Xcurrenttime = 0;




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
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW); //Actives Low, this enables the three stepper drivers
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(X_DIR_PIN, LOW); //Step Direction
  digitalWrite(Y_DIR_PIN, LOW); //Step Direction
  digitalWrite(Z_DIR_PIN, LOW); //Step Direction
  digitalWrite(X_STEP_PIN, LOW); //Step PULSE pin
  digitalWrite(Y_STEP_PIN, LOW); //Step PULSE pin
  digitalWrite(Z_STEP_PIN, LOW); //Step PULSE pin


  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();




}



void loop() {


  if (radio.available()) {
    radio.read(&data, sizeof(data));
  }



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Throttle Read, then ya set speed.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  if (data.Lxt > 506 && data.Lxt < 538) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~DEADZONE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Throttle = 100000;
  }

  if (data.Lxt > 539) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~FORWARD~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Throttle = map(data.Lxt, 531, 1020, 5000, 500);
  reverse=false;
  
 if(stoppeddeadinitstracks==true){
  digitalWrite(X_DIR_PIN, LOW);
  Xdirlow=true;
  stoppeddeadinitstracks=false;
  
  }
  }

  if (data.Lxt < 505) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~REVERSE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Throttle = map(data.Lxt, 521, 0, 5000, 500);
  reverse=true;
  
  if(stoppeddeadinitstracks==true){
  digitalWrite(X_DIR_PIN, HIGH);
  Xdirlow=false;
  stoppeddeadinitstracks=false;
  
  }
  }


if (Xdirlow==true  && data.Lxt < 505) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Extra check to prevent bad stuff~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Throttle = 100000;
  }
if (Xdirlow==false && data.Lxt > 539) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Extra check to prevent bad stuff~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Throttle = 100000;
  }







if ( Xsteplength >= 400 && Xsteplength > Throttle ){  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X ACCELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (XNextStep == true) {                                   //Prepare for next step
      Xlastspeed = steplength / Xsteplength; //[meters/second]    [micrometers/microsecond]
      Xsteplength = Xsteplength - ((2.0 * Xsteplength) / ((4.0 * n) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
      Xhalfsteplength = Xsteplength / 2;
      XNextStep = false;
      Xstart = true;
      Xhigh = true;
      Xlow = true;
    }
    if (Xstart == true) { //Get Step Start time
      Xstart = false;
      Xstartstep = micros();
    }
    if (micros() - Xstartstep < Xhalfsteplength && Xhigh == 1) { //Step pin ON
      digitalWrite(X_STEP_PIN, HIGH);
      Xhigh = false;
    }
    if (micros() - Xstartstep > Xhalfsteplength && micros() - Xstartstep < Xsteplength && Xlow == 1) { //Step pin OFF
      digitalWrite(X_STEP_PIN, LOW);
      Xlow = false;
    }
    if (micros() - Xstartstep > Xsteplength) { //Flag for next X Step and Get Step end time
      XNextStep = true;
      Xendstep = micros();
    }
}


if (Xsteplength < Throttle && Xsteplength < 5100){  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X DECELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (XNextStep == true) {                                   //Prepare for next step
      Xlastspeed = steplength / Xsteplength; //[meters/second]    [micrometers/microsecond]
      Xsteplength = Xsteplength - ((2.0 * Xsteplength) / ((4.0 * -n) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
      Xhalfsteplength = Xsteplength / 2;
      XNextStep = false;
      Xstart = true;
      Xhigh = true;
      Xlow = true;
    }
    if (Xstart == true) { //Get Step Start time
      Xstart = false;
      Xstartstep = micros();
    }
    if (micros() - Xstartstep < Xhalfsteplength && Xhigh == 1) { //Step pin ON
      digitalWrite(X_STEP_PIN, HIGH);
      Xhigh = false;
    }
    if (micros() - Xstartstep > Xhalfsteplength && micros() - Xstartstep < Xsteplength && Xlow == 1) { //Step pin OFF
      digitalWrite(X_STEP_PIN, LOW);
      Xlow = false;
    }
    if (micros() - Xstartstep > Xsteplength) { //Flag for next X Step and Get Step end time
      XNextStep = true;
      Xendstep = micros();
    }
}


if (Throttle > 5100 && Xsteplength > 5100){  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X BRAKE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      digitalWrite(X_STEP_PIN, LOW);      
stoppeddeadinitstracks=true;
}












} //Void Loop End ~~~~~~~~~~~~~~~~~~
