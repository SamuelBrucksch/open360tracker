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
#include "eeprom_functions.h"

static float magGain[3] = {1.0, 1.0, 1.0};
int16_t magADC[3];
float smoothed[3];
int magZero[3];
static uint8_t magInit = 0;

/*#define filterSamples 11
int xSmooth[filterSamples];
int ySmooth[filterSamples];
int zSmooth[filterSamples];

int digitalSmooth(int rawIn, int *sensSmoothArray) {    // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int temp;
  uint8_t j, k, top, bottom;
  long total;
  static uint8_t i;
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j = 0; j < filterSamples; j++) { // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting
  while (done != 1) {      // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++) {
      if (sorted[j] > sorted[j + 1]) {    // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j + 1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }
  bottom = max(((filterSamples * 15)  / 100), 1);
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j < top; j++) {
    total += sorted[j];  // total remaining indices
    k++;
  }
  return total / k;    // divide by number of samples
}*/
#ifdef MPU6050
void initMpu6050(){
  //disable sleep
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(MPU6050_RA_PWR_MGMT_1);
  Wire.write(MPU6050_PWR1_SLEEP_BIT);
  Wire.write(0);
  Wire.endTransmission();

  //disable master mode
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(MPU6050_RA_USER_CTRL);
  Wire.write(MPU6050_USERCTRL_I2C_MST_EN_BIT);
  Wire.write(0);
  Wire.endTransmission();
  
  //enable slave mode
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(MPU6050_RA_INT_PIN_CFG);
  Wire.write(MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
  Wire.write(1);
  Wire.endTransmission();
}
#endif

void write(int address, int data)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

bool readRawAxis() {
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(DataRegisterBegin);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_Address);
  Wire.requestFrom(HMC5883L_Address, 6);

  if (Wire.available() == 6) {
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

static int32_t xyz_total[3] = { 0, 0, 0 };
static uint8_t bias_collect(uint8_t bias) {
  int16_t abs_magADC;

  write(HMC58X3_R_CONFA, bias);            // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
  for (uint8_t i=0; i<10; i++) {                               // Collect 10 samples
    write(HMC58X3_R_MODE, 1);
    delay(100);
    while(!readRawAxis());                                                  // Get the raw values in case the scales have already been changed.
    for (uint8_t axis=0; axis<3; axis++) {
      abs_magADC =  abs(magADC[axis]);
      xyz_total[axis]+= abs_magADC;                            // Since the measurements are noisy, they should be averaged rather than taking the max.
      if ((int16_t)(1<<12) < abs_magADC) return false;         // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }
  return true;
}

void initCompass() {
  #ifdef MPU6050
    initMpu6050();
  #endif
  
  bool bret = true;
  write(HMC58X3_R_CONFB, 2 << 5);
  write(HMC58X3_R_MODE, 1);
  delay(100);
  
  //get one sample and discard it
  while (!readRawAxis());
  
  if (bias_collect(0x010 + HMC_POS_BIAS)) bret = false;
  if (bias_collect(0x010 + HMC_NEG_BIAS)) bret = false;
  
  if (bret){
   for (uint8_t axis=0; axis<3; axis++)
      magGain[axis]=820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[axis];  // note: xyz_total[axis] is always positive
  }else{
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
  write(HMC58X3_R_CONFA , 0x78 ); //Configuration Register A  -- output rate: 75Hz ; normal measurement mode
  write(HMC58X3_R_CONFB , 0x20 ); //Configuration Register B  -- configuration gain 1.3Ga
  write(HMC58X3_R_MODE  , 0x00 ); //Mode register             -- continuous Conversion Mode
  delay(100);


  for (uint8_t axis = 0; axis < 3; axis++) {
    magZero[axis] = LoadIntegerFromEEPROM(axis * 2);
  }
}

void calibrate_compass() {
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  byte axis = 0;
  while (!readRawAxis());
  for (axis = 0; axis < 3; axis++) {
    magZero[axis] = 0;
    magZeroTempMin[axis] = magADC[axis];
    magZeroTempMax[axis] = magADC[axis];
  }
  timer = millis();
  SET_PAN_SERVO_SPEED(2000);
  while (millis() - timer  < 5000) {
    while (!readRawAxis());
    for (axis = 0; axis < 3; axis++) {
      if (magADC[axis] < magZeroTempMin[axis]) {
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
    for (axis = 0; axis < 3; axis++) {
      if (magADC[axis] < magZeroTempMin[axis]) {
        magZeroTempMin[axis] = magADC[axis];
      }
      if (magADC[axis] > magZeroTempMax[axis]) {
        magZeroTempMax[axis] = magADC[axis];
      }
    }
    delay(14);
  }
  SET_PAN_SERVO_SPEED(PAN_0);
  for (axis = 0; axis < 3; axis++) {
    magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
    StoreIntegerToEEPROM(magZero[axis], axis * 2);
  }
}

int getHeading() {
  while (!readRawAxis());

  /*smoothed[0] = (digitalSmooth(magADC[0], xSmooth) * magGain[0]) - magZero[0];
  smoothed[1] = (digitalSmooth(magADC[1], ySmooth) * magGain[1]) - magZero[1];
  smoothed[2] = (digitalSmooth(magADC[2], zSmooth) * magGain[2]) - magZero[2];*/

  //double heading = atan2(smoothed[1], smoothed[0]) ;
  double heading = atan2((magADC[1]* magGain[1]) - magZero[1], (magADC[0]* magGain[0]) - magZero[0]) ;
  
  
  if (heading < 0)
    heading += 2 * M_PI;

  if (heading > 2 * M_PI)
    heading -= 2 * M_PI;

  return (int) ((heading * 1800.0 / M_PI) + DECLINATION + OFFSET) % 3600;
}


