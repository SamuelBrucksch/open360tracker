#ifndef CONFIG_H
#define CONFIG_H

/* Config file
 * created by Samuel Brucksch
 *
 */
#define DEBUG

/** PID Values
*
*/
#define P 2200 //default 2200
#define I 280 //default 280
#define D 20000 //default 20000

/* #### Protocol ####
 *
 *  FRSKY_D, FRSKY_X, HOTT, EXTERNAL
 * 
 *  FRSKY_D -> D-Series
 *  FRSKY_X -> Taranis / XJT
 *  HOTT -> MX12, MX16 and all other HoTT transmitters with telemetry
 *  EXTERNAL -> implement your own protocol
 *  RVOSD
 *  GPS_TELEMETRY -> NMEA/Ublox protocol over serial transmission system (e.g. 3DR radio)
 *  MFD -> MFD protocol will not work with local GPS!!!!
 */
#define FRSKY_X

/* #### Baud Rate ####
 *
 * baud rate of telemetry input
 * 9600 for FRSKY_D -> D-Series
 * 57600 for FRSKY_X -> Taranis / XJT
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
 *  Enter PWM value of Servo for pointing 90Â° up
 */
#define TILT_90 2000

/* #### Pan servo 0° adjustment ####
 *
 *  Enter PWM value of Servo for not moving
 */
#define PAN_0 1470

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

/* #### Ground GPS ####
 *
 * needed for ground gps so home does not need to be manually set
 *
 * uncomment #define LOCAL_GPS to disable local GPS
 */
#ifndef MFD
  //#define LOCAL_GPS
#endif
#define GPS_BAUDRATE 38400

/* #### Local GPS type ####
 *
 * Needed for GPS configuration
 *
 * Types:
 * MTK, UBX
 * UBX not implemented yet
 */
#define MTK

/* #### Tracker Setup ####
 * 
 * Start tracking when plane is XXX m away from tracker
 *
 */
#define START_TRACKING_DISTANCE 0

/* ### Vario Altitude ###
 *
 * Uncomment to use baro alt instead of GPS alt. Only works if Vario is present
 *
 */
//#define VARIO

/* ### LCD Display ###
 *
 * Uncomment to display data on LCD Display
 *
 * I2C LCD Display is required.
 *
 */
#define LCD_DISPLAY

/* #### Do not edit below this line */
#if TILT_0 < 1000 || TILT_0 > 2000 || TILT_90 > 2000  || TILT_90 < 1000
  #error "Servo range invalid. Must be between 1000 and 2000."
#endif

#endif
