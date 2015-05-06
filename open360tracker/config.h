#ifndef CONFIG_H
#define CONFIG_H

/* Config file
 * created by Samuel Brucksch
 *
 */
//#define DEBUG

/** PID Values
*
*/
#define P 5000 //default 2200
#define I 100 //default 280
#define D 1000 //default 20000

/* #### Protocol ####
 *
 *  FRSKY_D, FRSKY_X, HOTT, EXTERNAL
 *
 *  FRSKY_D -> D-Series
 *  FRSKY_X -> Taranis / XJT
 *  HOTT -> MX12, MX16 and all other HoTT transmitters with telemetry
 *  RVOSD
 *  MFD -> MFD protocol will not work with local GPS!!!!
 *  MAVLINK -> Mavlink protocol (APM/Pixhawk/...)
 *  SERVOTEST
 */
#define FRSKY_X

/* #### Baud Rate ####
 *
 * baud rate of telemetry input
 * 9600 for FRSKY_D -> D-Series
 * 57600 for FRSKY_X -> Taranis/XJT and MAVLINK
 * 115200 for RVOSD (RVGS)
 * ??? for HoTT
 */
#define BAUD 57600

/* #### Tilt servo 0° adjustment ####
 *
 *  Enter PWM value of Servo for pointing straight forward
 */
#define TILT_0 1050

/* #### Tilt servo 90° adjustment ####
 *
 *  Enter PWM value of Servo for pointing 90° up
 */
#define TILT_90 2025

/* #### Pan servo 0° adjustment ####
 *
 *  Enter PWM value of Servo for not moving
 */
#define PAN_0 1485

/* #### Pan servo minimum required speed ####
 *
 *  If the servo has problems to start a rotation when the speed is slow adjust this value until the tracker moves directly from each position
 */
#define MIN_PAN_SPEED 50

/* #### Compass declination ####
 *
 * http://magnetic-declination.com/
 * Enter your city and then get the value for Magnetic declination
 * for example [Magnetic declination: 3° 2' EAST]
 *
 * now enter the value in the format DEGREE.MINUTE * 10 -> 3.2 * 10 = 32
 *
 * set to 0 if you cannot find your declination!
 */
#define DECLINATION 32

/* #### Compass offset ####
 *
 * If you did not mount your compass with the arrow pointing to the front you can set an offset here.
 *
 * Needs to be multiplied by 10 -> 90° = 900
 *
 * Range: 0 ... 3599
 *
 */
#define OFFSET 0

/* #### DIY GPS / Fix Type ####
*
* If you use the diy GPS the fix type is transmitted with the satellites on Temp2. The value is calculated like this:
* Num of Sats: 7
* Fix Type: 3
* Value = Sats * 10 + Fix Type = 7*10 + 3 = 73
*
* If you use the native frsky gps or fixtype is not present comment to disable.
*/
//#define DIY_GPS

#ifndef MFD
/* #### Ground GPS ####
 *
 * !!!!!!NOT SUPPORTED YET!!!!!!!
 *
 * needed for ground gps so home does not need to be manually set
 *
 * Types:
 * MTK, UBX
 * UBX not implemented yet
 *
 * does not work when in MFD mode
 */
//#define LOCAL_GPS
#define MTK
#define GPS_BAUDRATE 38400
#endif

/* #### Tracker Setup ####
 *
 * Start tracking when plane is XXX m away from tracker
 *
 * It is recommended to start tracking only if plane moved a few meters already. Default: 10m
 *
 */
#define START_TRACKING_DISTANCE 10

/* ### LCD Display ###
 *
 * Uncomment to display data on LCD Display
 *
 * Please choose for the Display Type:
 * I2C
 * SPI
 *
 * LCD Display is required for this.
 *
 *  Requires modified LiquidCrystal library: https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
 *
 */
#define LCD_DISPLAY I2C

/* ### Battery monitoring ###
 *
 * Uncomment to monitor your Battery
 *
 * Voltage divider is required for this.
 *
 */
//#define BATTERYMONITORING
#ifdef BATTERYMONITORING
  #define BATTERYMONITORING_RESISTOR_1 18000
  #define BATTERYMONITORING_RESISTOR_2 1000
  #define BATTERYMONITORING_CORRECTION 1.0    // default 1.0
  #define BATTERYMONITORING_ALERT 3
#endif

/* ### BUZZER ###
 *
 * Uncomment to activate Buzzer
 *
 * A connected Buzzer is required
 */
//#define BUZZER

/* #### Do not edit below this line */
#if TILT_0 < 800 || TILT_0 > 2200 || TILT_90 > 2200  || TILT_90 < 800
#error "Tilt servo range invalid. Must be between 800 and 2200."
#endif

#if OFFSET < 0 || OFFSET > 3599
#error "Offset invalid. Must be between 0° and 359°."
#endif

#endif
