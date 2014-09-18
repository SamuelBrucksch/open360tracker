#ifndef TELEMETRY_H
#define TELEMETRY_H


#include "inttypes.h"
#include "defines.h"
#include "Arduino.h"

//lat and lon required in units of millionths of a degree
int32_t getTargetLat();
int32_t getTargetLon();

//alt in meter
int16_t getTargetAlt();

void encodeTargetData(uint8_t c);

#endif

