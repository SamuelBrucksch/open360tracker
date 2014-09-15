/**
 * Created by Samuel Brucksch
 *
 * Compass calibration: http://forum.bildr.org/viewtopic.php?f=29&t=650&start=0
 * Compass code taken from http://bluelemonlabs.blogspot.de/2013/08/arduino-simple-compass-with-hmc5883l.html
 * 
 * and adapted by Samuel Brucksch
 */
#include "compass.h"
#include <inttypes.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "servos.h"
#ifdef DEBUG
#include "uart.h"
#endif

static float   magGain[3] = {1.0,1.0,1.0};
int16_t magADC[3];
float smoothed[3];
int16_t magZero[3];
static uint8_t magInit = 0;

#define filterSamples 11
int xSmooth [filterSamples];
int ySmooth [filterSamples];
int zSmooth [filterSamples];

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int temp;
  uint8_t j, k, top, bottom;
  long total;
  static uint8_t i;
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }
  return total / k;    // divide by number of samples
}

void write(int address, int data)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

bool readRawAxis(){
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(DataRegisterBegin);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_Address);
  Wire.requestFrom(HMC5883L_Address, 6);

  if(Wire.available() == 6) {
    //X
    magADC[0] = (Wire.read() << 8) | Wire.read();
    //Z
    magADC[2] = (Wire.read() << 8) | Wire.read();
    //Y
    magADC[1] = (Wire.read() << 8) | Wire.read();
  }
  Wire.endTransmission();
  
  if (magADC[0] == -4096 || magADC[1] == -4096 || magADC[2] == -4096) {
        // no valid data available
        return false;
    }
    return true;
}

unsigned long timer = 0;


void initCompass(){
  int32_t xyz_total[3]={0,0,0}; 
  bool bret = true;
  write(HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);
  write(HMC58X3_R_CONFB, 2 << 5);
  write(HMC58X3_R_MODE, 1);
  delay(100);
  while (!readRawAxis());

  for (uint8_t i = 0; i < 10; i++){
    write(HMC58X3_R_MODE, 1);
    delay(100);
    while (!readRawAxis());
    xyz_total[0] += magADC[0];
    xyz_total[1] += magADC[1];
    xyz_total[2] += magADC[2];
    if (-(1<<12) >= min(magADC[0],min(magADC[1], magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }
  
  write(HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS);
  
  for (uint8_t i=0; i<10; i++) { 
    write(HMC58X3_R_MODE, 1);
    delay(100);
    while (!readRawAxis());
                
    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=magADC[0];
    xyz_total[1]-=magADC[1];
    xyz_total[2]-=magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(magADC[0],min(magADC[1], magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }
  
  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  write(HMC58X3_R_CONFA ,0x78 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  write(HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  write(HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  
  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
}

void calibrate(){
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    byte axis = 0;
    while (!readRawAxis());
    for(axis=0;axis<3;axis++) {
      magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    timer = millis();
    SET_PAN_SERVO_SPEED(2000);
    while (millis() - timer  < 5000) { 
      while (!readRawAxis());
      for(axis=0;axis < 3; axis++) {
        if (magADC[axis] < magZeroTempMin[axis]){
          magZeroTempMin[axis] = magADC[axis];
        }
        if (magADC[axis] > magZeroTempMax[axis]) {
          magZeroTempMax[axis] = magADC[axis];
        }
      }
      delay(14);
    }
    SET_PAN_SERVO_SPEED(PAN_0);
    delay(1000);
    SET_PAN_SERVO_SPEED(1000);
    timer = millis();
    while (millis() - timer  < 5000) { 
      while (!readRawAxis());
      for(axis=0;axis < 3; axis++) {
        if (magADC[axis] < magZeroTempMin[axis]){
          magZeroTempMin[axis] = magADC[axis];
        }
        if (magADC[axis] > magZeroTempMax[axis]) {
          magZeroTempMax[axis] = magADC[axis];
        }
      }
      delay(14);
    }
    SET_PAN_SERVO_SPEED(PAN_0);
    for(axis=0;axis<3;axis++){
      magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      Serial.print(magZero[axis]);Serial.print(" ");
    }
    Serial.println();
      //writeGlobalSet(1);
      //TODO write to eeprom
}

int getHeading(){
  while (!readRawAxis());
  
  smoothed[0] = (digitalSmooth(magADC[0], xSmooth) * magGain[0]) - magZero[0];
  smoothed[1] = (digitalSmooth(magADC[1], ySmooth) * magGain[1]) - magZero[1];
  smoothed[2] = (digitalSmooth(magADC[2], zSmooth) * magGain[2]) - magZero[2];
  /*magADC[0] = magADC[0] * magGain[0];
  magADC[1] = magADC[1] * magGain[1];
  magADC[2] = magADC[2] * magGain[2];*/
  double heading = atan2(smoothed[1], smoothed[0]) ;

  if(heading < 0)
    heading += 2*M_PI;

  if(heading > 2*M_PI)
    heading -= 2*M_PI;

  return (int) ((heading * 1800.0/M_PI)+ DECLINATION + 1800) % 3600;
}
