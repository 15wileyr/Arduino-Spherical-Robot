
#include <SPI.h> //radio
#include <nRF24L01.h> //radio
#include <RF24.h> //radio
#include <Keypad.h> //keypad
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const byte ROWS = 4; // keypad stuff
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {22, 24, 26, 28}; //keyboard pins
byte colPins[COLS] = {30, 32, 34, 36};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

int Lvrx = A0; //potientiameter values from joysticks
int Lvry = A1;
int Rvrx = A2;
int Rvry = A3;
int leftjoyswitch = 6; //Joystick clicks
int rightjoyswitch = 7;
bool joypoll = 1; // Enable Joystick Polling
int RadioRate = 40; // Milliseconds between radio transmition
int lcdRate = 900;// Milliseconds between lcd refresh
unsigned long cycleclock = 0; //millisec
unsigned long radioclock = 0; //millisec
unsigned long lcdclock = 0;

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
struct dataStruct {
  int Lxt = 0; // data to send over radio
  int Lyt = 0;
  int Rxt = 0;
  int Ryt = 0;
  int Lswt = 0;
  int Rswt = 0;
  char KB = 'N';
} data;

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display



void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); // Power Level
  radio.stopListening();

  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();

  Serial.begin(9600);
}
void loop() {


  cycleclock = millis();


  if (millis() - radioclock >= RadioRate) {
    //Serial.println(millis() - radioclock);//debug
    radioclock = millis();

    data.Lxt = analogRead(Lvrx);
    data.Lyt = analogRead(Lvry);
    data.Rxt = analogRead(Rvrx);
    data.Ryt = analogRead(Rvry);
    data.Lswt = digitalRead(leftjoyswitch);
    data.Rswt = digitalRead(rightjoyswitch);
    data.KB = customKeypad.getKey();
    radio.write(&data, sizeof(data));
  }


  if (millis() - lcdclock >= lcdRate) { //LCD display code
    lcdclock = millis();
    lcd.clear();
    for (int i = 0; i <= 16; i++) {
      lcd.setCursor(i, 0);
      lcd.print((char)random(166, 218));
    }
    for (int i = 0; i <= 16; i++) {
      lcd.setCursor(i, 1);
      lcd.print((char)random(166, 218));
    }

  }


/*
//Shows Sent radio signal through serial monitor Usually leave this commented out
  Serial.print("Lx");
  Serial.print(data.Lxt);
  Serial.print("\t");
  Serial.print("Ly");
  Serial.print(data.Lyt);
  Serial.print("\t");
  Serial.print("Rx");
  Serial.print(data.Rxt);
  Serial.print("\t");
  Serial.print("Ry");
  Serial.print(data.Ryt);
  Serial.print("\t");
  Serial.print("Lsw");
  Serial.print(data.Lswt);
  Serial.print("\t");
  Serial.print("Rsw");
  Serial.print(data.Rswt);
  Serial.print("\t");
  Serial.println(data.KB);
  Serial.print("\t");

*/





  //Serial.println(cycleclock);//debug
}
