
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


bool Xhigh=true;
bool Xlow=true;
bool XNextStep=true;
bool Xstart =true;




int X_STEP_PIN     =    54; // Stepper pins on the shield
int X_DIR_PIN       =   55;
int X_ENABLE_PIN    =   38;
int Y_STEP_PIN       =  60;
int Y_DIR_PIN       =   61;
int Y_ENABLE_PIN    =   56;
int Z_STEP_PIN        = 46;
int Z_DIR_PIN        =  48;
int Z_ENABLE_PIN     =  62;
int i=0;

int n=400;

float pi=3.1415926535897932384626433832795;
float radius=0.15; // [meters] Outer Shell Radius !!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!!
float steplength= pi*2*radius*0.005;  //[meters]
float Xaccel =0.001; // [meters/sec^2]  Max AcceleratSHEEON !!!~~~~~~~~~~~~~~~~~~~SET ME PLZ~~~~~~~~~~~~~~~~~~~!!!
float Xstepaccel = Xaccel*(1/(pi*2*radius*0.0025));// [steps/sec^2]
float ThrottleRead = 0;
float Xlaststepspeed = 0; //[step/sec]
float Xlastspeed = 0;
float Xsteplength=5000;
float Xhalfsteplength=2500;


unsigned long mapval = 0; // Placeholder variable for motor speeds
unsigned long turnval = 0; // Placeholder variable for turn motor adjustments
unsigned long Xstartstep =0;
unsigned long Xendstep=1000000;
unsigned long Xlasttime=0;
unsigned long Xcurrenttime=0;




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
    //Serial.println(text);
  }





delay(1000); //!!!!!!!!!!!!!!!!!!!!! DEBUG

ThrottleRead=data.Lxt;



Xstartstep=micros();
while (i <= 2000) {
   


 //~~~~~~~~~~~~~~~~~~~~~~~~~~X ACCELERATION STEP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Xcurrenttime=micros()-Xstartstep;
if(XNextStep==true){                                       //Prepare for next step
Xsteplength = Xsteplength - ((2.0 * Xsteplength) / ((4.0 * n) + 1)); // [mircoseconds]  Calculate Step length for 1 new step
Xhalfsteplength=Xsteplength/2;
XNextStep=false;
Xstart=true;
Xhigh=true;
Xlow=true;
i=i+1;
}
if(Xstart==true){  //Get Step Start time
   Xstart=false;
   Xstartstep=micros();
}
if(micros()-Xstartstep < Xhalfsteplength && Xhigh==1){  //Step pin ON
   digitalWrite(X_STEP_PIN, HIGH);
   Xhigh=false;
}
if(micros()-Xstartstep > Xhalfsteplength && micros()-Xstartstep < Xsteplength && Xlow==1){ //Step pin OFF
   digitalWrite(X_STEP_PIN, LOW);
   Xlow=false;
}
if(micros()-Xstartstep > Xsteplength){  //Flag for next X Step and Get Step end time
   XNextStep=true;
   Xendstep=micros();
}



}


delay(1000); //!!!!!!!!!!!!!!!!!!!!! DEBUG

Serial.print("pi "); //!!!!!!!!!!!!!!!!!!!!! DEBUG
Serial.println(pi);
Serial.print("steplength   ");
Serial.println(steplength);
Serial.print("Xaccel   ");
Serial.println(Xaccel);
Serial.print("Xstepaccel   ");
Serial.println(Xstepaccel);
Serial.print("ThrottleRead   ");
Serial.println(ThrottleRead);
Serial.print("Xlaststepspeed   ");
Serial.println(Xlaststepspeed);
Serial.print("Xlastspeed   ");
Serial.println(Xlastspeed);
Serial.print("Xsteplength   ");
Serial.println(Xsteplength);
Serial.print("Xsteplength   ");
Serial.println(Xsteplength);
Serial.print("Xhalfsteplength   ");
Serial.println(Xhalfsteplength);
Serial.print("Xstartstep   ");
Serial.println(Xstartstep);
Serial.print("Xendstep   ");
Serial.println(Xendstep);
Serial.print("Xlasttime   ");
Serial.println(Xlasttime);
Serial.print("Xcurrenttime   ");
Serial.println(Xcurrenttime);
Serial.print("Xhigh   ");
Serial.println(Xhigh);
Serial.print("Xlow   ");
Serial.println(Xlow);
Serial.print("XNextStep   ");
Serial.println(XNextStep);
Serial.print("Xstart   ");
Serial.println(Xstart);



delay(3600000); //!!!!!!!!!!!!!!!!!!!!! DEBUG


} //Void Loop End ~~~~~~~~~~~~~~~~~~
