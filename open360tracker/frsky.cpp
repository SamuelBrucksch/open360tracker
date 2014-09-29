#include "config.h"
#ifdef FRSKY_D
#include "telemetry.h"
#include <avr/io.h>
//#include "uart.h"
#include "frsky_common.h"

void processFrskyPacket(uint8_t *packet);
void parseTelemHubByte(uint8_t c);
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap);

// Enumerate FrSky packet codes
#define LINKPKT         0xfe
#define USRPKT          0xfd
#define A11PKT          0xfc
#define A12PKT          0xfb
#define A21PKT          0xfa
#define A22PKT          0xf9
#define RSSI1PKT        0xf7
#define RSSI2PKT        0xf6

#define START_STOP      0x7e
#define BYTESTUFF       0x7d
#define STUFF_MASK      0x20

#define GPS_ALT_BP_ID      0x01
#define GPS_ALT_AP_ID      0x09
#define BARO_ALT_BP_ID     0x10
#define BARO_ALT_AP_ID     0x21
#define GPS_LON_BP_ID     0x12
#define GPS_LAT_BP_ID      0x13
#define GPS_LON_AP_ID     0x1A
#define GPS_LAT_AP_ID      0x1B
#define GPS_LON_EW_ID     0x22
#define GPS_LAT_NS_ID      0x23

#define FRSKY_RX_PACKET_SIZE 11
uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];

enum FrSkyDataState {
  STATE_DATA_IDLE,
  STATE_DATA_START,
  STATE_DATA_IN_FRAME,
  STATE_DATA_XOR,
};


int16_t alt;
int8_t latsign;
int8_t lonsign;

uint16_t lat_bp;
uint16_t lon_bp;
uint16_t lat_ap;
uint16_t lon_ap;

int32_t getTargetLat(){
  return gpsToLong(latsign, lat_bp, lat_ap);
}

int32_t getTargetLon(){
  return gpsToLong(lonsign, lon_bp, lon_ap);
}

int16_t getTargetAlt(){
  return alt;
}

bool stuff;
void encodeTargetData(uint8_t c){
  static uint8_t bufferIndex = 0;
  if (c == 0x7e){
      //message end
    if (bufferIndex == FRSKY_RX_PACKET_SIZE){
       processFrskyPacket(frskyRxBuffer);
       bufferIndex = 0;
       return;
    } else if (bufferIndex >0 && bufferIndex < FRSKY_RX_PACKET_SIZE){
       //0x7e in data -> something is wrong
       bufferIndex = 0;
    } else {
       //msg begin
       frskyRxBuffer[bufferIndex++] = c;
    }
  } else if (c == 0x7D){
    stuff = 1;
  } else if (stuff){
    frskyRxBuffer[bufferIndex++] = c ^ 0x20;
    stuff = 0;
  } else {
    frskyRxBuffer[bufferIndex++] = c;
  }
}

void processFrskyPacket(uint8_t *packet){
  //get package ID
  switch (packet[1]){
    case LINKPKT: // A1/A2/RSSI values
      //maybe we can use RSSI here to start an alarm when RSSI level gets low
      break;
    case USRPKT:
      uint8_t numBytes = packet[2];
      for (uint8_t i = 3; i < numBytes; i++) {
          parseTelemHubByte(packet[i]);
      }
      break; 
  }
}

typedef enum{
  IDLE = 0,
  DATA_ID,
  DATA_LOW,
  DATA_HIGH,
  STUFF = 0x80
}STATE;
  
#ifdef DEBUG
  char s[10];
#endif
  
void parseTelemHubByte(uint8_t c){
  static uint8_t dataId;
  static uint8_t byte0;
  static STATE state = IDLE;

  if (c == 0x5e) {
    state = DATA_ID;
    return;
  }
  if (state == IDLE) {
    return;
  }
  if (state & STUFF) {
    c = c ^ 0x60;
    state = (STATE)(state - STUFF);
  }
  if (c == 0x5d) {
    state = (STATE)(state | STUFF);
    return;
  }
  if (state == DATA_ID) {
    if (c > 0x3f) {
      state = IDLE;
    }
    else {
      dataId = c;
      state = DATA_LOW;
    }
    return;
  }
  if (state == DATA_LOW) {
    byte0 = c;
    state = DATA_HIGH;
    return;
  }
  state = IDLE;
   
  switch(dataId){
    case GPS_ALT_BP_ID:
      #ifndef VARIO
        alt = int16_t((byte0 << 8) + c);
        hasAlt = true;
      #endif
      break;
    case GPS_ALT_AP_ID:
      #ifndef VARIO
      //not really needed as we only use int as alt
      #endif
      break;
    case BARO_ALT_BP_ID:
      #ifdef VARIO
        alt = (int32_t)(byte0 << 8) + c) * 100;
        hasAlt = true;
      #endif
      break;
    case BARO_ALT_AP_ID:
      #ifdef VARIO
      //not really needed as we only use int as alt
      #endif
      break;
    case GPS_LON_BP_ID:
      //if we have values we should have a sat fix here
      if (byte0 || c){
        hasFix = true;
      } else {
        hasFix = false; 
      }
      lon_bp = (c << 8) + byte0;
      break;
    case GPS_LON_AP_ID:
      lon_ap = (c << 8) + byte0;
      break;
    case GPS_LAT_BP_ID:
      //if we have values we should have a sat fix here
      if (byte0 || c){
        hasFix = true;
      } else {
        hasFix = false; 
      }
      lat_bp = (c << 8) + byte0;
      break;
    case GPS_LAT_AP_ID:
      lat_ap = (c << 8) + byte0;
      break;
    case GPS_LON_EW_ID:
      if (byte0 == 'E'){
        lonsign = 1;
      } else {
        lonsign = -1;
      }
      hasLon = true;
      break;
    case GPS_LAT_NS_ID:
      if (byte0 == 'N'){
        latsign = 1;
      } else {
        latsign = -1;
      }
      hasLat = true;
      break;
  }
}


#if defined(VARIO)
  void evalVario(int16_t altitude_bp, uint16_t altitude_ap){
    alt = (int32_t)altitude_bp * 100 + (altitude_bp > 0 ? altitude_ap : -altitude_ap);
  }
#endif
#endif
