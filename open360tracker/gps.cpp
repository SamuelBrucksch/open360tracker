#include "config.h"
#ifdef GPS_TELEMETRY
#include "telemetry.h"
#include <avr/io.h>

#include <TinyGPS.h>
TinyGPS gps;

int16_t alt;
int32_t lat;
int32_t lon;

int32_t getTargetLat(){
  return lat;
}

int32_t getTargetLon(){
  return lon;
}

int16_t getTargetAlt(){
  return alt;
}

void encodeTargetData(uint8_t c){
  if (gps.encode(c)){
    float flat, flon;
    unsigned long fix_age;
    gps.f_get_position(&flat, &flon, &fix_age);
    
    if (fix_age == TinyGPS::GPS_INVALID_AGE){
      hasFix = false;
    }
    else if (fix_age > 5000){
      hasFix = false;
    }
    else{
      gps.get_position(&lat,&lon);
      hasFix = true;
    }
    
    if(gps.altitude() != TinyGPS::GPS_INVALID_ALTITUDE) {
      alt = (int16_t)gps.altitude();
      hasAlt = true;
    }else{
      hasAlt = false;
    }
  }
}

#endif


