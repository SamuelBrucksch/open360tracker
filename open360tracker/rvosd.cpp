/*
This code is written by Samuel Brucksch
 
 it will decode the RVOSD telemetry protocol and will convert it into values the open360tracker can understand
 
 $1,1,00040291,00.000000,N,000.000000,W,+00000,+00000,000,0000,0000,000,+000,000,089,089,089,089,1160,0000,00004,0004,00,000,0026,*00
 
 It is sent 25 times/S, 115200, 8N1.
 
 $
 Validity (1 valid, 0 invalid)
 Units (1 metric, 0 imperial)
 HHMMSSmm (hour|minutes|seconds|tenth of seconds)
 ddmm.mmmm, N/S (Latitude)
 dddmm.mmmm,E/W (Longitude)
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
#include "uart.h"

unsigned char buffer_index = 0;
unsigned char commacount = 0;
unsigned char buffer[10];
uint8_t checksum = 0;
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
uint16_t lat_ap;
uint16_t lon_ap;

int32_t lat = 0;
int32_t lon = 0;
int16_t altitude = 0;

int32_t getTargetLat(){
  return lat;
}

int32_t getTargetLon(){
  return lon;
}

int16_t getTargetAlt(){
  return altitude;
}

void encodeTargetData(uint8_t c){
  //only enable this to debug telemtry data. Else debug output will be flooded with rvosd protocol output.
  #ifdef DEBUG
    //uart_puts(c);
  #endif
  
  
  //wait for frame to start
  if(c != '$' && buffer_index == 0){
    return;
  }

  if (c == '$'){
    //frame started
    checksum = false;
    commacount = 0;
    buffer_index = 0;
    checksum_calculation = 0;
    checksum_read = 0;
  } 
  else if (c == '*'){
    //frame end -> checksum
    checksum = true;
  }

  if (buffer_index < sizeof(buffer)){
    buffer[buffer_index] = c;
  } 
  else if(buffer_index > 9){
    buffer_index = 0;
    return;
  }

  //calculate checksum
  if (buffer_index == 1){
    //checksum is everything xored except * and checksum itself
    checksum_calculation = c;
  } 
  else if (buffer_index > 1){
    if (!checksum){
      checksum_calculation ^= c; 
    }
    else if (c!= '*' && c != '\r' && c != '\n'){
      //we reached checksum
      checksum_read *= 10;
      checksum_read += c;
    }
  }

  //valid data?
  if (commacount == 0 && buffer_index > 1){
    if (buffer[1] != '1'){
      //no valid data
      buffer_index = 0;

      //TODO warning (beeper??)
      return;
    }
  }

  if (c == ','){
    // first value after comma
    commacount++;
    switch(commacount){
    case 1:
      // Units (1 metric, 0 imperial)
      break;
    case 3:
      // ddmm.mmmm Lat
      dot_found = 0;
      lat_bp = 0;
      lat_ap = 0;
      break;
    case 4:
      // N/S
      if (c == 'N'){
        latsign = 1;
      }
      else if (c == 'S'){
        latsign = -1;
      } 
      else{
        //invalid data
        buffer_index == 0;
        return; 
      }
      break;
    case 5:
      // dddmm.mmmm Lon
      dot_found = 0;
      lat_bp = 0;
      lat_ap = 0;
      break;
    case 6:
      // E/W
      if (c == 'E'){
        lonsign = 1;
      } 
      else if ( c == 'W'){
        lonsign = -1;
      } 
      else{
        //invalid data
        buffer_index == 0;
        return; 
      }
      break;
    case 7: 
      // +/- altitude (relative)
      break;
    case 8:
      // +/- altitude (absolute)
      alt = 0;
      break;
    case 23:
      // Sats 
      break;
    } 
  } 
  else{
    // values behind first value
    switch(commacount){
    case 3:
      // ddmm.mmmm Lat
      if (c == '.' || dot_found){
        dot_found++;
      }

      if (c != '.' && !dot_found){
        lat_bp *= 10;
        lat_bp += (c - '0');
      } 
      else if (c != '.' && dot_found < 7){
        lat_ap *= 10;
        lat_ap += (c-'0');
      } 
      else{
        //invalid data
        buffer_index == 0;
        return;
      }
      break;
    case 5:
      // dddmm.mmmm Lon
      if (c == '.' || dot_found){
        dot_found = 1;
      }

      if (c != '.' && !dot_found){
        lon_bp *= 10;
        lon_bp += (c - '0');
      } 
      else if (c != '.' && dot_found < 7){
        lon_ap *= 10;
        lon_ap += (c-'0');
      } 
      else{
        //invalid data
        buffer_index == 0;
        return;
      }
      break;
    case 7: 
      // +/- altitude (relative)
      break;
    case 8:
      // +/- altitude (absolute)
      if (c == '+'){
        altsign = 1;
        break;
      } 
      else if (c == '-'){
        altsign = -1;
        break;
      }

      alt *= 10;
      alt += (c - '0');
      break;
    case 23:
      // Sats 
      break;
    } 
  }

  if (c == '\n'){
    //end of line -> checksum is available
    if (checksum_calculation != checksum_read){
        #ifdef DEBUG
          uart_puts("RVOSD: Checksum wrong. \n");
        #endif
        buffer_index = 0;
        return;
    }
  } 
  else{
    buffer_index++; 
    return;
  }

  #ifdef DEBUG
    uart_puts("RVOSD: Data valid and ready. \n");
  #endif


  lat = gpsToLong(latsign, lat_bp, lat_ap);
  lon = gpsToLong(lonsign, lon_bp, lon_ap);
  altitude = alt*altsign;

  // data is ready
  hasAlt = true;
  hasLat = true;
  hasLon = true;
}
#endif

