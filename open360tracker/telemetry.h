#ifndef TELEMETRY_H
#define TELEMETRY_H


#include "inttypes.h"
#include "defines.h"
#include "Arduino.h"


//alt in meter
int16_t getTargetAlt();
void encodeTargetData(uint8_t c);
  
#ifdef MFD
  uint16_t getDistance();
  uint16_t getAzimuth();
#else
  //lat and lon required in units of millionths of a degree -> precision of 5 digits after '.'
  // for example 52.52081 -> 5252081
  //             13.40945 -> 1340945
  int32_t getTargetLat();
  int32_t getTargetLon();
  int16_t getSats();
  int16_t getFix();
#endif

#endif


