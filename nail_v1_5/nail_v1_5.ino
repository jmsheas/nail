#include <PID_v1.h> // PID Contol Algorithm
#include <max6675.h> // Interface Thermocouple Chip
#include "SevSeg.h" // For the SevSeg display to work
#include <EEPROM.h> //To save desired value in memory

SevSeg sevseg;

// Pinout for thermocuple
int thermoDO = 10;
int thermoCS = 11;
int thermoCLK = 12;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO); // Declare and initialize thermocouple object

// PID Setup
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=3, aggKi=0.15, aggKd=.75;
double medKp=2, medKi=0.10, medKd=.50;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//settings for pid and sevenseg
int WindowSize = 5000;
unsigned long windowStartTime;
unsigned long previousMillis = 0;
int interval = 1000;
int printout;

double temp;

//timer for knob readout
int timer = 0;

//mem location of temp
int tempad = 1;
double temp1;

void setup() {

  //sevenseg settup
  byte numDigits = 4;
  byte digitPins[] = {2, 3, 4, 5};
  byte segmentPins[] = {6, 7, 8, 9, A3, A4, A5};
  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins);
  
  Serial.begin(9600);
  pinMode(A1, OUTPUT); // Relay pin A1

  windowStartTime = millis();
  
  //check if there is a previously stored value
  if(EEPROM.read(tempad)<0 || EEPROM.read(tempad)>1000){
    Setpoint = 710;
  }
  else{
    Setpoint = EEPROM.read(tempad); // Temperature Setpoint
  }
  
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //dlay for thermocouple to turn on
  delay(500);
}

void loop() {
  //gets the wanted temp
  temp1= analogRead(A2); // Knob pin A2
  temp1= map(temp1, 0, 1023, 0, 1000);
  Serial.println(temp1);
  EEPROM.write(tempad, temp1);
  
  //display temp
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;
    temp = readTemp(); 
  }
  
  //if a new value is wanted print that otherwise print 
  //the temp it is
  if(abs(temp1-Setpoint) > 5){
    timer = 100;
    //set the new temp;
    Setpoint = temp1;
  }
  
  if(timer != 0){
    printout = (int) Setpoint;
    timer--;
  }
  else{
    printout = (int) temp;
  }

  //set brightness and value to print
  sevseg.setNumber(printout,0);
  sevseg.refreshDisplay();
  sevseg.setBrightness(90);
  
  Input = temp;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<75 && gap>=10)
  {
    myPID.SetTunings(medKp, medKi, consKd);
  }
  else if(gap<10)
  {  
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  //computer what the PID should do
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(A1, HIGH);
  else digitalWrite(A1, LOW);
}

double readTemp(){
  return thermocouple.readFahrenheit();
}
