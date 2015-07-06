#include "config.h"
#ifdef GPS_TELEMETRY
#include "telemetry.h"
#include <avr/io.h>

#include <TinyGPS.h>
TinyGPS remoteGps;

int16_t alt;
int32_t lat;
int32_t lon;

int32_t getTargetLat() {
  return lat;
}

int32_t getTargetLon() {
  return lon;
}

int16_t getTargetAlt() {
  return alt;
}

void encodeTargetData(uint8_t c) {
  if (remoteGps.encode(c)) {
    unsigned long fix_age;
    remoteGps.get_position(&lat, &lon, &fix_age);
    
    if (fix_age == TinyGPS::GPS_INVALID_AGE) {
      HAS_FIX = false;
    } else if (fix_age > 5000) {
      HAS_FIX = false;
    } else {
      //TODO valid data
      lat = lat / 10;
      lon = lon / 10;

      if (remoteGps.altitude() != TinyGPS::GPS_INVALID_ALTITUDE) {
        alt = int16_t(remoteGps.altitude() / 100);
        HAS_ALT = true;
      }
      HAS_FIX = true;
    }
  }
}

#endif
