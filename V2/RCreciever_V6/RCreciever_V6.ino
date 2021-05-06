
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


bool Xhigh = true;
bool Xlow = true;
bool XNextStep = true;
bool Xstart = true;
bool reverse = false;
bool Xstoppeddeadinitstracks = true;
bool Xdirlow = true;

bool Yhigh = true;
bool Ylow = true;
bool YNextStep = true;
bool Ystart = true;
bool Ystoppeddeadinitstracks = true;
bool Ydirlow = true;




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



int DrivingAccel = 50;       //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Acceleration Value when in motion  (smaller is higher acceleration)
int TurninplaceAccel = 70;    //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Acceleration Value when turning in place (smaller is higher acceleration)
int Slowspeed = 30000;         //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Minimum Driving speed smaller is faster
int Topspeed = 7000;           //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Highest Driving Speed smaller is faster
int Reversemaxspeed = 7000;   //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Highest Driving speed in reverse smaller is faster

int Maxturninplace = 27000;    //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Maximum turning pulse length smaller means turn faster
float radius = 150000;        //!!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!! Outer sphere/shell radius [micrometers] Outer Shell Radius

int CurrentAccel = DrivingAccel; //don't set this for accel
int mincutoffspeed = max(Slowspeed, Maxturninplace); //don't set this true cuttoff for accel and decel
int Maxspeed = Topspeed - 100; //don't set this needs to smaller than Topspeed
int Throttlemap = 0;
float Turnvalue = 0.0;

float Yawfactor = 0;


float pi = 3.1415926535897932384626433832795;

float steplength = pi * 2 * radius * 0.005; //[micrometers]

float XThrottle = 0;
float Xlastspeed = 0;
float Xsteplength = mincutoffspeed;//needs to be initialized as nonzero
float Xhalfsteplength = mincutoffspeed/2;//needs to be initialized as nonzero

float YThrottle = 0;
float Ylastspeed = 0;
float Ysteplength = mincutoffspeed;//needs to be initialized as nonzero
float Yhalfsteplength = mincutoffspeed/2;//needs to be initialized as nonzero

unsigned long Xstartstep = 0;
unsigned long Xendstep = 1000000;
unsigned long Ystartstep = 0;
unsigned long Yendstep = 1000000;




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



  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //Serial.begin(9600);


}



void loop() {


  if (radio.available()) {   // read the throttle data from bluetooth
    radio.read(&data, sizeof(data));
  }

  //___________________________________________________________________________________________________________________________________________________________________________________________________________________
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~THE BELOW SECTION HANDLES THE ASSIGNMENT OF THROTTLE VALUES (Sets a target speed)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //___________________________________________________________________________________________________________________________________________________________________________________________________________________




  if (data.Lxt > 539) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~FORWARD DRIVING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    CurrentAccel = DrivingAccel;
    if (mincutoffspeed == Maxturninplace) {//fixes acceleration by converting the last Step Length
      Xsteplength = Slowspeed;
      Ysteplength = Slowspeed;
    }
    mincutoffspeed = Slowspeed;
    if (data.Ryt > 539) { // ~~~~~~~~~~~~TURN RIGHT~~~~~~~~~~~~~
      Throttlemap = map(data.Lxt, 531, 1020, Slowspeed, Topspeed); //Value of the throttle
      //Turnvalue = (0.000043758234806 * pow(Throttlemap, 2) - 0.052695928245088 * Throttlemap + 63.374980344081571); //*mapf(data.Ryt, 539, 1024, 1, 0.8); //Function to determine a value to turn at
      Turnvalue =1000;
      XThrottle = Throttlemap + Turnvalue;
      YThrottle = Throttlemap - Turnvalue;
      reverse = false;

      if (Xstoppeddeadinitstracks == true) {
        digitalWrite(X_DIR_PIN, LOW);
        Xdirlow = true;
        Xstoppeddeadinitstracks = false;
      }

      if (Ystoppeddeadinitstracks == true) {
        digitalWrite(Y_DIR_PIN, LOW);
        Ydirlow = true;
        Ystoppeddeadinitstracks = false;
      }
    }

    if (data.Ryt < 510) { // ~~~~~~~~~~~~TURN LEFT~~~~~~~~~~~~~
      Throttlemap = map(data.Lxt, 531, 1020, Slowspeed, Topspeed); //Value of the throttle
      //Turnvalue = (0.000043758234806 * pow(Throttlemap, 2) - 0.052695928245088 * Throttlemap + 63.374980344081571); //*mapf(data.Ryt, 510, 0, 1, 0.8); //Function to determine a value to turn at
      Turnvalue =1000;
      XThrottle = Throttlemap - Turnvalue;
      YThrottle = Throttlemap + Turnvalue;
      reverse = false;

      if (Xstoppeddeadinitstracks == true) {
        digitalWrite(X_DIR_PIN, LOW);
        Xdirlow = true;
        Xstoppeddeadinitstracks = false;
      }

      if (Ystoppeddeadinitstracks == true) {
        digitalWrite(Y_DIR_PIN, LOW);
        Ydirlow = true;
        Ystoppeddeadinitstracks = false;
      }
    }
    if (data.Ryt > 505 && data.Ryt < 530) {   // ~~~~~~~~~~~~DON'T TURN SUCKA~~~~~~~~~~~~~
      XThrottle = map(data.Lxt, 531, 1020, Slowspeed, Topspeed);
      YThrottle = map(data.Lxt, 531, 1020, Slowspeed, Topspeed);
      reverse = false;

      if (Xstoppeddeadinitstracks == true) {
        digitalWrite(X_DIR_PIN, LOW);
        Xdirlow = true;
        Xstoppeddeadinitstracks = false;

      }

      if (Ystoppeddeadinitstracks == true) {
        digitalWrite(Y_DIR_PIN, LOW);
        Ydirlow = true;
        Ystoppeddeadinitstracks = false;
      }
    }
  }




  if (data.Lxt < 505 && data.Lxt > 0) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~REVERSE DRIVING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    XThrottle = map(data.Lxt, 521, 0, Slowspeed, Reversemaxspeed);
    YThrottle = map(data.Lxt, 521, 0, Slowspeed, Reversemaxspeed);
    reverse = true;
    CurrentAccel = DrivingAccel;

    if (mincutoffspeed == Maxturninplace) {
      Xsteplength = Slowspeed;
      Ysteplength = Slowspeed;
    }
    mincutoffspeed = Slowspeed;
    if (Xstoppeddeadinitstracks == true) {
      digitalWrite(X_DIR_PIN, HIGH);
      Xdirlow = false;
      Xstoppeddeadinitstracks = false;
    }

    if (Ystoppeddeadinitstracks == true) {
      digitalWrite(Y_DIR_PIN, HIGH);
      Ydirlow = false;
      Ystoppeddeadinitstracks = false;
    }
  }

















  if (data.Lxt > 507 && data.Lxt < 538) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~ THROTTLE DEADZONE && TURNING IN PLACE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    if (data.Ryt > 539  && Xstoppeddeadinitstracks == true && Ystoppeddeadinitstracks == true) { // ~~~~~~~~~~~~TURN RIGHT~~~~~~~~~~~~~
      digitalWrite(X_DIR_PIN, LOW);
      digitalWrite(Y_DIR_PIN, HIGH);
      CurrentAccel = TurninplaceAccel;

      if (mincutoffspeed == Slowspeed) {
        Xsteplength = Maxturninplace;
        Ysteplength = Maxturninplace;
      }
      mincutoffspeed = Maxturninplace;


      Yawfactor = mapf(data.Ryt, 539, 1024, 1, 0.8);
      XThrottle = Maxturninplace * Yawfactor;
      YThrottle = Maxturninplace * Yawfactor;

    }

    if (data.Ryt < 505  && Xstoppeddeadinitstracks == true && Ystoppeddeadinitstracks == true) { // ~~~~~~~~~~~~TURN LEFT~~~~~~~~~~~~~
      digitalWrite(X_DIR_PIN, HIGH);
      digitalWrite(Y_DIR_PIN, LOW);
      CurrentAccel = TurninplaceAccel;

      if (mincutoffspeed == Slowspeed) {
        Xsteplength = Maxturninplace;
        Ysteplength = Maxturninplace;
      }
      mincutoffspeed = Maxturninplace;


      Yawfactor = mapf(data.Ryt, 505, 5, 1, 0.8);
      XThrottle = Maxturninplace * Yawfactor ;
      YThrottle = Maxturninplace * Yawfactor ;
    }


    if (data.Ryt > 505 && data.Ryt < 530 && Xstoppeddeadinitstracks == true) {   // ~~~~~~~~~~~~DON'T TURN SUCKA~~~~~~~~~~~~~

      XThrottle = 100000;
      YThrottle = 100000;
    }

    if (Xstoppeddeadinitstracks == false) {   // ~~~~~~~~~~~~DON'T TURN SUCKA~~~~~~~~~~~~~
      DrivingAccel = DrivingAccel;

      XThrottle = 100000;
      YThrottle = 100000;
    }
  }








  if (Xdirlow == true  && data.Lxt < 505) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Forces throttle to 0 before switchin direction when the throttle stick switches~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    XThrottle = 100000;
  }
  if (Xdirlow == false && data.Lxt > 539) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Same as above~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    XThrottle = 100000;
  }
  if (Ydirlow == true  && data.Lxt < 505) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Same as above~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    YThrottle = 100000;
  }
  if (Ydirlow == false && data.Lxt > 539) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Same as above~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    YThrottle = 100000;
  }

































  //mincutoffspeed=Slowspeed;


  //___________________________________________________________________________________________________________________________________________________________________________________________________________________
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~THE BELOW SECTION HANDLES THE STEPPING OF MOTORS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //___________________________________________________________________________________________________________________________________________________________________________________________________________________


  if ( Xsteplength >= Maxspeed && Xsteplength > XThrottle ) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X ACCELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (XNextStep == true) {                                   //Prepare for next step
      Xlastspeed = steplength / Xsteplength; //[meters/second]    [micrometers/microsecond]
      Xsteplength = Xsteplength - ((2.0 * Xsteplength) / ((4.0 * CurrentAccel) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
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


  if (Xsteplength < XThrottle && Xsteplength < (mincutoffspeed)) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X DECELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (XNextStep == true) {                                   //Prepare for next step
      Xlastspeed = steplength / Xsteplength; //[meters/second]    [micrometers/microsecond]
      Xsteplength = Xsteplength - ((2.0 * Xsteplength) / ((4.0 * -CurrentAccel) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
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


  if (XThrottle > (mincutoffspeed) && Xsteplength > (mincutoffspeed)) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~X BRAKE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    digitalWrite(X_STEP_PIN, LOW);
    Xstoppeddeadinitstracks = true;
  }




  if ( Ysteplength >= Maxspeed && Ysteplength > YThrottle ) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Y ACCELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (YNextStep == true) {                                   //Prepare for next step
      Ylastspeed = steplength / Ysteplength; //[meters/second]    [micrometers/microsecond]
      Ysteplength = Ysteplength - ((2.0 * Ysteplength) / ((4.0 * CurrentAccel) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
      Yhalfsteplength = Ysteplength / 2;
      YNextStep = false;
      Ystart = true;
      Yhigh = true;
      Ylow = true;
    }
    if (Ystart == true) { //Get Step Start time
      Ystart = false;
      Ystartstep = micros();
    }
    if (micros() - Ystartstep < Yhalfsteplength && Yhigh == 1) { //Step pin ON
      digitalWrite(Y_STEP_PIN, HIGH);
      Yhigh = false;
    }
    if (micros() - Ystartstep > Yhalfsteplength && micros() - Ystartstep < Ysteplength && Ylow == 1) { //Step pin OFF
      digitalWrite(Y_STEP_PIN, LOW);
      Ylow = false;
    }
    if (micros() - Ystartstep > Ysteplength) { //Flag for next Y Step and Get Step end time
      YNextStep = true;
      Yendstep = micros();
    }
  }


  if (Ysteplength < YThrottle && Ysteplength < (mincutoffspeed)) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Y DECELERATION STEPPING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (YNextStep == true) {                                   //Prepare for next step
      Ylastspeed = steplength / Ysteplength; //[meters/second]    [micrometers/microsecond]
      Ysteplength = Ysteplength - ((2.0 * Ysteplength) / ((4.0 * -CurrentAccel) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
      Yhalfsteplength = Ysteplength / 2;
      YNextStep = false;
      Ystart = true;
      Yhigh = true;
      Ylow = true;
    }
    if (Ystart == true) { //Get Step Start time
      Ystart = false;
      Ystartstep = micros();
    }
    if (micros() - Ystartstep < Yhalfsteplength && Yhigh == 1) { //Step pin ON
      digitalWrite(Y_STEP_PIN, HIGH);
      Yhigh = false;
    }
    if (micros() - Ystartstep > Yhalfsteplength && micros() - Ystartstep < Ysteplength && Ylow == 1) { //Step pin OFF
      digitalWrite(Y_STEP_PIN, LOW);
      Ylow = false;
    }
    if (micros() - Ystartstep > Ysteplength) { //Flag for next Y Step and Get Step end time
      YNextStep = true;
      Yendstep = micros();
    }
  }


  if (YThrottle > (mincutoffspeed) && Ysteplength > (mincutoffspeed)) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Y BRAKE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    digitalWrite(Y_STEP_PIN, LOW);
    Ystoppeddeadinitstracks = true;
  }









} //Void Loop End ~~~~~~~~~~~~~~~~~~




float mapf(float x, float in_min, float in_max, float out_min, float out_max) {  // The map equation re written to work with float variables
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
