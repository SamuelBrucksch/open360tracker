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
//#define P 2500
//#define I 40 //40
//#define D 10 //10

/* #### Protocol ####
 *
 *  FRSKY_D, FRSKY_X, HOTT, EXTERNAL
 * 
 *  FRSKY_D -> D-Series
 *  FRSKY_X -> Taranis / XJT
 *  HOTT -> MX12, MX16 and all other HoTT transmitters with telemetry
 *  EXTERNAL -> implement your own protocol
 */
#define FRSKY_D

/* #### Baud Rate ####
 *
 * baud rate of telemetry input
 * 9600 for FRSKY_D -> D-Series
 * 57600 for FRSKY_X -> Taranis / XJT
 * ??? for HoTT
 */
#define BAUD 9600

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
#define PAN_0 1492

/* #### Compass declination ####
 *
 * http://magnetic-declination.com/
 * Enter your city and then get the value for Magnetic declination
 * for example [Magnetic declination: 3° 2' EAST]
 *
 * now enter the value in the format DEGREE.MINUTE -> 3.2
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
//#define LOCAL_GPS
#define GPS_BAUDRATE 38400

/* #### Local GPS type ####
 *
 * Needed for GPS configuration
 *
 * Types:
 * MTK, UBX
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
 * Uncomment to use baro alt instead of GPS alt. Only works if Vario is presemt
 *
 */
//#define VARIO

#endif
