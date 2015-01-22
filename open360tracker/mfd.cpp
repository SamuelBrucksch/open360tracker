#include "config.h"

#ifdef MFD
#include "telemetry.h"
#include <avr/io.h>

int8_t alt_neg = 1;
int16_t alt;
uint16_t mfd_distance;
uint16_t azimuth;


uint8_t read_checksum = 125;
uint8_t calc_checksum = 255;

uint16_t getDistance(){
  return mfd_distance;
}

uint16_t getAzimuth(){
  return azimuth; 
}

int16_t getTargetAlt(){
  return alt_neg * alt;
}

uint8_t state = 0;

void encodeTargetData(uint8_t c){
  if (c == 'D'){
    //distance
    state = 1;
    mfd_distance = 0;
    calc_checksum = 0;
    return;
  } else if (c == 'H'){
    //altitude
    state = 2;
    alt = 0;
    return;
  } else if (c == 'A'){
    //azimuth
    state = 3;
    azimuth = 0;
    return;
  } else if (c == '*'){
    //checksum
    state = 4;
    read_checksum = 0;
    return;
  } else if (c == '\r'){
    //newline

    if (calc_checksum == read_checksum){
      hasAlt = 1;
    }
    state = 0;
    return;
  } else if (c == 'X'){
    //home set
    SETTING_HOME = 1;
    return;
  } else if (c == '#'){
    //no coordinates
    return; 
  }

  switch(state){
  case 1: 
    mfd_distance = (mfd_distance * 10) + c;
    calc_checksum += c;
    break;
  case 2: 
    if (c == '-'){
      alt_neg = -1;
      return;
    }
    alt = (alt * 10) + c;
    calc_checksum += c;
    break;
  case 3:
    azimuth = (azimuth * 10) + c;
    calc_checksum += c;
    break;
  case 4: 
    read_checksum = (read_checksum * 10) + c;
    break;
  }
}
#endif


