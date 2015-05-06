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
#define VOLTAGEDIVIDER A0
#define BUZZER_PIN A1

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

#define I2C 1
#define SPI 2

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

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
