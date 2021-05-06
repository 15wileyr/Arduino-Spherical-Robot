

int Lvrx=A0; //potientiameter values from joysticks
int Lvry=A1;
int Rvrx=A2;
int Rvry=A3;
int leftjoyswitch=6; //Joystick clicks
int rightjoyswitch=7;
bool joypoll=1;// Enable Joystick Polling
int Lx=0; // Actual joystick read values
int Ly=0;
int Rx=0;
int Ry=0;
int Lsw=0;
int Rsw=0;


void setup() {
  pinMode(leftjoyswitch, INPUT); // initializes Joystick pins
  pinMode(rightjoyswitch, INPUT);
  pinMode(Lvrx, INPUT);
  pinMode(Lvry, INPUT);
  pinMode(Rvrx, INPUT);
  pinMode(Rvry, INPUT);
  
  Serial.begin(9600);
}



//Below is main code loop
void loop() {


if (joypoll == 1) {
Lx=analogRead(Lvrx);
Ly=analogRead(Lvry);
Rx=analogRead(Rvrx);
Ry=analogRead(Rvry);
Lsw=digitalRead(leftjoyswitch);
Rsw=digitalRead(rightjoyswitch);
}


Serial.print("Lx");
Serial.print(Lx);
Serial.print("\t");
Serial.print("Ly");
Serial.print(Ly);
Serial.print("\t");
Serial.print("Rx");
Serial.print(Rx);
Serial.print("\t");
Serial.print("Ry");
Serial.print(Ry);
Serial.print("\t");
Serial.print("Lsw");
Serial.print(Lsw);
Serial.print("\t");
Serial.print("Rsw");
Serial.print(Rsw);
Serial.println("\t");

} //Void Loop End ~~~~~~~~~~~~~~~~~~
