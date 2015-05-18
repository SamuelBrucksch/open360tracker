#ifndef COMPASS_H
#define COMPASS_H
/**
 * Created by Samuel Brucksch
 *
 * Compass code taken from http://bluelemonlabs.blogspot.de/2013/08/arduino-simple-compass-with-hmc5883l.html
 * and adapted by Samuel Brucksch
 */
#include <inttypes.h>

#define HMC5883L_Address           0x1E
#define ModeRegister               0x02
#define DataRegisterBegin          0x03
#define HMC58X3_R_CONFA            0x00
#define HMC58X3_R_CONFB            0x01
#define HMC58X3_R_MODE             0x02
#define HMC58X3_X_SELF_TEST_GAUSS  (+1.16)   //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS  (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS  (+1.08)   //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT        (243.0/390.0)  //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT       (575.0/390.0)  //!< High limit when gain is 5.
#define HMC_POS_BIAS               0x01
#define HMC_NEG_BIAS               0x02

#define MPU6050_ADDRESS_AD0_LOW           0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH          0x69 // address pin high (VCC)
#define MPU6050_Address                   MPU6050_ADDRESS_AD0_LOW
#define MPU6050_RA_PWR_MGMT_1             0x6B
#define MPU6050_PWR1_SLEEP_BIT            0x06
#define MPU6050_RA_USER_CTRL              0x6A
#define MPU6050_USERCTRL_I2C_MST_EN_BIT   0x05
#define MPU6050_RA_INT_PIN_CFG            0x37
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT  0x01

//MagnetometerScaled readScaledAxis();
bool readRawAxis();
void initCompass();
int  getHeading();
void compassMaxMin(int xRaw, int yRaw);
void calcScaleFactor_Offset();
void writeCalibration();
void readCalibration();
void calibrate_compass(void);

#endif


