/*
This code is written by Samuel Brucksch
 
 it will decode the RVOSD telemetry protocol and will convert it into values the open360tracker can understand
 
 $1,1,00040291,00.000000,N,000.000000,W,+00000,+00000,000,0000,0000,000,+000,000,089,089,089,089,1160,0000,00004,0004,00,000,0026,*00
 
 It is sent 25 times/S, 115200, 8N1.
 
 $
 Validity (1 valid, 0 invalid)
 Units (1 metric, 0 imperial)
 HHMMSSmm (hour|minutes|seconds|tenth of seconds)
 ddmm.mmmm, 
 N/S (Latitude)
 dddmm.mmmm,
 E/W (Longitude)
 +/- altitude (relative)
 +/- altitude (absolute)
 Roll (0 to 255)
 Pitch (0 to 4096)
 Airspeed
 GroundSpeed
 +/- Variometer
 Heading (0 to 360)
 Rudder
 Elevator
 Aileron
 Throttle (mS / 100)
 Main battery voltage
 Aux battery voltage (V * 100)
 Current
 mAh
 Sats
 RSSI
 Temp
 *checksum
 
 
 Checksum is everything XORed from "$" to "temperature".
 */
#include "config.h"
#ifdef RVOSD

#include "telemetry.h"
#include "frsky_common.h"

uint8_t dot_found = 0;

uint16_t checksum_calculation = 0;
uint16_t checksum_read = 0;

//data needed to buffer
int16_t alt;
int8_t altsign;

int8_t latsign;
int8_t lonsign;
uint16_t lat_bp;
uint16_t lon_bp;
uint32_t lat_ap;
uint32_t lon_ap;

int32_t lat = 0;
int32_t lon = 0;
uint8_t sats = 0;

uint8_t index = 0;
uint8_t frame_started = 0;
uint8_t checksum_started = 0;
uint8_t metric = 0;
uint8_t checksum_index = 0;

int32_t getTargetLat() {
  return lat;
}

int32_t getTargetLon() {
  return lon;
}

int16_t getTargetAlt() {
  return altsign*alt;
}

int16_t getSats() {
  return (int16_t)sats;
}



void encodeTargetData(uint8_t c) {
  if (c == '$' && !frame_started){
    frame_started = true;
    checksum_started = false;
    checksum_calculation = 0;
    checksum_read = 0;
    lat_bp = 0;
    lat_ap = 0;
    lon_bp = 0;
    lon_ap = 0;
    latsign = 0;
    lonsign = 0;
    alt = 0;
    sats = 0;
    altsign = 0;
    checksum_index = 0;
    return;
  } 
  else if (c == 'R' && frame_started){
    checksum_calculation ^= c;
    return;
  } 
  else if (c == 'V' && frame_started){
    checksum_calculation ^= c;
    return;
  }
  else if (c == '*' && frame_started){
    checksum_started = true;
    return;
  }
  /*else if ((c == '\r' || c == '\n') && frame_started){
    if (checksum_read == checksum_calculation){
      lat = gpsToLong(latsign, lat_bp, lat_ap);
      lon = gpsToLong(lonsign, lon_bp, lon_ap);

      // data is ready
      HAS_FIX = true;
      HAS_ALT = true;
    }
    else{
      //needed?
    }
    frame_started = false;
  }*/
  else if (checksum_index == 2){
    if (checksum_read == checksum_calculation){
      lat = gpsToLong(latsign, lat_bp, uint16_t(lat_ap/100));
      lon = gpsToLong(lonsign, lon_bp, uint16_t(lon_ap/100));

      // data is ready
      HAS_FIX = true;
      HAS_ALT = true;
    }
    else{
      //needed?
    }
    frame_started = false;
  }

  if (frame_started){
    if (checksum_started){
      checksum_index++;
      checksum_read *= 16;
      if (c >= '0' && c <= '9'){
        checksum_read += (c - '0');
      }
      else{
        checksum_read += (c-'A'+10);
      }
    }
    else{
      //we get data
      checksum_calculation ^= c;
    }
  }
  else{
    return;
  }

  if (c == ','){
    index++;
    return;
  }

  switch (index) {
  case 0:
    //frame valid?
    if ((c - '0') == 0){
      //frame invalid, skip...
      frame_started = false;
      return;
    }
    else if ((c - '0') == 1){
      //frame valid, do nothing
    }
    break;
  case 1:
    //units, 0 = imperial, 1 = metric
    if (c - '0' == 0){
      metric = false;
    }
    else if (c - '0' == 1){
      metric = true;
    }
    break;
  case 3:
    //lat
    if (c == '.'){
      dot_found = true;
      return;
    }
    if (!dot_found){
      lat_bp *= 10;
      lat_bp += c-'0';
    }
    else{
      lat_ap *= 10;
      lat_ap += c-'0';
    }
    break;
  case 4:
    //N/S
    if (c == 'N'){
      latsign = 1;
    }
    else if (c == 'S'){
      latsign = -1;
    }
    break;
  case 5:
    //lon
    if (c == '.'){
      dot_found = true;
      return;
    }
    if (!dot_found){
      lon_bp *= 10;
      lon_bp += c-'0';
    }
    else{
      lon_ap *= 10;
      lon_ap += c-'0';
    }
    break;
  case 6:
    //E/W
    if (c == 'E'){
      lonsign = 1;
    }
    else if (c == 'W'){
      lonsign = -1;
    }
    break;
  case 8:
    //absolute alt;
    if (c == '.'){
      dot_found = true;
      return;
    }
    else if (c == '-'){
      altsign = -1;
      return;
    }
    else if (c == '+'){
      altsign = 1;
      return;
    }
    if (!dot_found){
      alt *= 10;
      alt += c- '0';
    }
    else{
      //no after comma i think
    }
    break;
  case 23:
    sats *= 10;
    sats += c - '0';
    break;
  default:
    break;
  }
}
#endif



