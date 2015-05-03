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

uint16_t getDistance() {
  return mfd_distance;
}

uint16_t getAzimuth() {
  return azimuth;
}

int16_t getTargetAlt() {
  return alt_neg * alt;
}

uint8_t state = 0;

void encodeTargetData(uint8_t c) {
#ifdef DEBUG
  Serial.write(c);
#endif
  if (c == 'D') {
    //distance
    state = 1;
    mfd_distance = 0;
    read_checksum = 0;
    calc_checksum = c;
    alt_neg = 1;
    TEST_MODE = false;
    return;
  } else if (c == 'H') {
    //altitude
    state = 2;
    alt = 0;
    calc_checksum += c;
    return;
  } else if (c == 'A') {
    //azimuth
    state = 3;
    azimuth = 0;
    calc_checksum += c;
    return;
  } else if (c == '*') {
    //checksum
    state = 4;
    return;
  } else if (c == '\r') {
    //newline
#ifdef DEBUG
    Serial.print("\nDebug values: [Dist "); Serial.print(mfd_distance); Serial.print("] [Alt "); Serial.print(alt_neg * alt); Serial.print("] [Azimuth "); Serial.print(azimuth); Serial.print("] [Read"); Serial.print(read_checksum); Serial.print("] [Calc"); Serial.print(calc_checksum); Serial.println("]");
#endif
    if (calc_checksum == read_checksum) {
      HAS_FIX = true;
    }
    state = 0;
    return;
  } else if (c == 'X') {
    //home set
    TEST_MODE = false;
    SETTING_HOME = true;
    return;
  } else if (c == '#') {
    //no coordinates
    TEST_MODE = false;
    return;
  } else if (c == '@') {
    //connection failure
    TEST_MODE = false;
    return;
  } else if (c == 'N') {
    azimuth = 0;
    mfd_distance = 1000;
    alt = 0;
    TEST_MODE = true;
    return;
  } else if (c == 'E') {
    azimuth = 90;
    mfd_distance = 1000;
    alt = 1000;
    TEST_MODE = true;
    return;
  } else if (c == 'S') {
    azimuth = 180;
    mfd_distance = 0;
    alt = 1000;
    TEST_MODE = true;
    return;
  } else if (c == 'W') {
    azimuth = 270;
    mfd_distance = 1000;
    alt = 0;
    TEST_MODE = true;
    return;
  }

  switch (state) {
    case 1:
      mfd_distance = (mfd_distance * 10) + (c - 48);
      calc_checksum += c;
      break;
    case 2:
      calc_checksum += c;
      if (c == '-') {
        alt_neg = -1;
        break;
      }
      alt = (alt * 10) + (c - 48);
      break;
    case 3:
      azimuth = (azimuth * 10) + (c - 48);
      calc_checksum += c;
      break;
    case 4:
      read_checksum = (read_checksum * 10) + (c - 48);
      break;
  }
}
#endif
