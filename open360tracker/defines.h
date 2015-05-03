#ifndef DEFINES_H
#define DEFINES_H

/* Defines file
 * created by Samuel Brucksch
 *
 */
#include <inttypes.h>
#include "config.h"

#define FMW_VERSION "0.1"

#define HOME_BUTTON 5
#define CALIB_BUTTON 6
#define LED_PIN 13
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7

//lat and lon required in units of millionths of a degree -> precision of 5 digits after '.'
// for example 52.52081 -> 5252081
//             13.40945 -> 1340945
typedef struct {
  // latitude in units of millionths of a degree
  int32_t lat;
  // longitude in units of millionths of a degree
  int32_t lon;
  // altitude ranging from -32.768m to 32.767m
  int16_t alt;
  // heading in 0-359° *10
  uint16_t heading;
  // distance from 0 ... 64km
  uint16_t distance;
}
geoCoordinate_t;

typedef struct {
  bool f0: 1;
  bool f1: 1;
  bool f2: 1;
  bool f3: 1;
  bool f4: 1;
  bool f5: 1;
  bool f6: 1;
  bool f7: 1;
} PackedBool;

#define HOME_SET          ( (volatile PackedBool*)(&GPIOR0) )->f0
#define TRACKING_STARTED  ( (volatile PackedBool*)(&GPIOR0) )->f1
#define CURRENT_STATE     ( (volatile PackedBool*)(&GPIOR0) )->f2
#define PREVIOUS_STATE    ( (volatile PackedBool*)(&GPIOR0) )->f3
#define HAS_FIX           ( (volatile PackedBool*)(&GPIOR0) )->f4
#define HAS_ALT           ( (volatile PackedBool*)(&GPIOR0) )->f5
#define SETTING_HOME      ( (volatile PackedBool*)(&GPIOR0) )->f6
#define NEW_HEADING       ( (volatile PackedBool*)(&GPIOR0) )->f7

#ifdef MFD
#define TEST_MODE            ( (volatile PackedBool*)(&GPIOR1) )->f0
#endif
//#define exampleBool_1_2    ( (volatile PackedBool*)(&GPIOR1) )->f1
//#define exampleBool_1_3    ( (volatile PackedBool*)(&GPIOR1) )->f2
//#define exampleBool_1_3    ( (volatile PackedBool*)(&GPIOR1) )->f3
//#define exampleBool_1_4    ( (volatile PackedBool*)(&GPIOR1) )->f4
//#define exampleBool_1_5    ( (volatile PackedBool*)(&GPIOR1) )->f5
//#define exampleBool_1_6    ( (volatile PackedBool*)(&GPIOR1) )->f6
//#define exampleBool_1_7    ( (volatile PackedBool*)(&GPIOR1) )->f7

#define toRad(val) val * PI/180.0f
#define toDeg(val) val * 180.0f/PI

#define meter2feet(value) value * 3.2808399
#define feet2meter(value) value * 0.3048
#endif


