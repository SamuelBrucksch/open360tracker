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

void initMpu6050(){
  Wire.beginTransmission(MPU6050_Address); //PWR_MGMT_1    -- DEVICE_RESET 1
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(MPU6050_Address); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  Wire.write(0x6B);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_Address); ///CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  Wire.write(0x1A);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_Address); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission();
  
  
  // enable I2C bypass for AUX I2C
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
}

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
  initMpu6050();
  
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
