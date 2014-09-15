// This is a simple test sketch which increases the targetHeading about 25degress every 1.4seconds.
// Afterwards the tracker should point into this direction.
//
// Please note that you will have to disable DEBUG in config.h otherwise the serial classes will conflict.

#include <EEPROM.h>
#include <Wire.h>
#include "compass.h"
#include "servos.h"

uint16_t targetHeading = 0;
uint16_t heading = 0;
uint16_t heading10 = 0;
unsigned long time;

int P = 2200;
int I = 280;
int D = 20000;

//PID stuff
long Error[11];
long Accumulator;
long PID;
uint8_t Divider = 15;
int PWMOutput;
long Dk;

void setup()
{
  pinMode(6, INPUT);
  digitalWrite(6, HIGH);
  Wire.begin();
  Serial.begin(57600);
  initServos();
  delay(200);
  Serial.println("Init compass");
  initCompass();
  Serial.println("Init done");
  time = millis();
}



void loop()
{
  if (millis() > time){
    time = millis() + 14;
    heading10 = getHeading();
    heading=heading10/10;

    Serial.print("H:");Serial.print(heading);Serial.print(" T: ");Serial.print(targetHeading/10);
    Serial.print(" P: ");Serial.print(P);Serial.print(" I: ");Serial.print(I);Serial.print(" D: "); Serial.println(D);    
    
    getError();       // Get position error
    calculatePID();   // Calculate the PID output from the error
    SET_PAN_SERVO_SPEED(PWMOutput);
  }

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'H' || c == 'h'){
      int speed = Serial.parseInt(); 
      targetHeading = speed*10;
    } else if( c == 'c'){
       calibrate(); 
    }else if( c == 'p'){
      Accumulator=0; 
      P = Serial.parseInt(); 
    }else if( c == 'i'){
      Accumulator=0;
       I = Serial.parseInt(); 
    }else if( c == 'd'){
      Accumulator=0;
       D = Serial.parseInt();  
    } else if (c == 't'){
      int tilt =  Serial.parseInt();
      tilt = map(tilt, 0, 180, 1000, 2000);
      SET_TILT_SERVO_ANGLE(tilt);
    }
  }
}

void getError(void)
{
  // shift error values
  for(byte i = 0; i < 10; i++){
    Error[i+1] = Error[i];
  }

  int16_t delta = targetHeading - heading10;
  if (delta > 1800){
    delta -= 3600; 
  } 
  else if (delta < -1800){
    delta += 3600;
  }  
  // load new error into top array spot  
  Error[0] = delta;
}

void calculatePID(void)
{
  // Calculate the PID  
  PID = Error[0] * P;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  if(Accumulator>5000)
    Accumulator=5000;
   if(Accumulator<-5000)
    Accumulator=-5000; 
  PID += I * Accumulator; // add integral gain and error accumulation
  Dk=D * (Error[0]-Error[10]);
  PID += Dk; // differential gain comes next
  PID = PID >> Divider; // scale PID down with divider
  // limit the PID to the resolution we have for the PWM variable
  if(PID >= 500)
    PID = 500;
  if(PID <= -500)
    PID = -500;
  if (Error[0] > 10){
    PWMOutput = PAN_0+PID;
  }else if(Error[0] < -10){
    PWMOutput = PAN_0+PID;
  } else {
    PWMOutput = PAN_0;
  }
}



