#include "config.h"
#ifdef FRSKY_X
#include "telemetry.h"
#include "frsky_common.h"

void processFrskyPacket(uint8_t *packet);
void parseTelemHubByte(uint8_t c);
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap);

#define START_STOP         0x7e
#define BYTESTUFF          0x7d
#define STUFF_MASK         0x20

// FrSky PRIM IDs (1 byte)
#define DATA_FRAME         0x10

// FrSky old DATA IDs (1 byte)
#define GPS_ALT_BP_ID      0x01
#define GPS_ALT_AP_ID      0x09
#define BARO_ALT_BP_ID     0x10
#define BARO_ALT_AP_ID     0x21
#define GPS_LONG_BP_ID     0x12
#define GPS_LONG_AP_ID     0x1A
#define GPS_LAT_BP_ID      0x13
#define GPS_LAT_AP_ID      0x1B
#define GPS_LONG_EW_ID     0x22
#define GPS_LAT_NS_ID      0x23
#define FRSKY_LAST_ID      0x3F
//used for sats and fix type
#define TEMP2              0x05

// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID       0x0100
#define ALT_LAST_ID        0x010f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f


#define TELEMETRY_INIT    0
#define TELEMETRY_OK      1
#define TELEMETRY_KO      2

#define FRSKY_RX_PACKET_SIZE 9

uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];
uint8_t telemetryState = TELEMETRY_INIT;

//alt in m
int16_t alt;
uint8_t sats;
uint8_t fix;

uint16_t NS;
uint16_t EW;

//lat lon in decimal degree dd.ddddd
uint16_t lat_bp;
uint16_t lon_bp;
uint16_t lat_ap;
uint16_t lon_ap;

int16_t getSats() {
  return (int16_t)sats;
}

int32_t getTargetLat(){
  int32_t value = gpsToLong(NS=='N'?1:-1, lat_bp, lat_ap);
  NS = 0;
  return value;
}

int32_t getTargetLon(){
  int32_t value = gpsToLong(EW=='E'?1:-1, lon_bp, lon_ap);
  EW = 0;
  return value;
}

int16_t getTargetAlt() {
  return alt;
}

void processHubPacket(uint8_t id, uint16_t value)
{
  if (id > FRSKY_LAST_ID)
    return;
  switch (id) {
  case GPS_ALT_BP_ID:
    alt = value;
    HAS_ALT = true;
    break;
  case GPS_LONG_BP_ID:
    lon_bp = value;
    break;
  case GPS_LONG_AP_ID:
    lon_ap = value;
    break;
  case GPS_LAT_BP_ID:
    lat_bp = value;
    break;
  case GPS_LAT_AP_ID:
    lat_ap = value;
    break;
  case GPS_LONG_EW_ID:
    EW = value;
    break;
  case GPS_LAT_NS_ID:
    NS = value;
    break;
  case TEMP2:
#ifdef DIY_GPS
    sats = value / 10;
    fix = value % 10;
#else
    sats = value;
#endif
    break;
  }
  if ((NS == 'N' || NS == 'S') && (EW == 'E' || EW == 'W')){
    HAS_FIX = true;
  }
}

bool checkSportPacket(uint8_t *packet)
{
  short crc = 0;
  for (int i = 1; i < FRSKY_RX_PACKET_SIZE; i++) {
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
    crc += crc >> 8; //0-0FF
    crc &= 0x00ff;
  }
  return (crc == 0x00ff);
}

#define SPORT_DATA_U8(packet)   (packet[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(packet+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(packet+4)))
#define HUB_DATA_U16(packet)    (*((uint16_t *)(packet+4)))

void processSportPacket(uint8_t *packet)
{
  uint8_t  prim   = packet[1];
  uint16_t appId  = *((uint16_t *)(packet + 2));

  if (!checkSportPacket(packet))
    return;

  switch (prim)
  {
  case DATA_FRAME:
    if ((appId >> 8) == 0) {
      // The old FrSky IDs
      uint8_t  id = (uint8_t)appId;
      uint16_t value = HUB_DATA_U16(packet);
      processHubPacket(id, value);
    }
    else if (appId >= T2_FIRST_ID && appId <= T2_LAST_ID) {
      #ifdef DIY_GPS
        sats = SPORT_DATA_S32(packet) / 10;
        fix = SPORT_DATA_S32(packet) % 10;
      #endif
      #ifndef DIY_GPS
        //we assume that other systems just send the sats over temp2 and fix over temp1
        sats = SPORT_DATA_S32(packet);
      #endif
    }
    else if (appId >= GPS_SPEED_FIRST_ID && appId <= GPS_SPEED_LAST_ID) {
      //frskyData.hub.gpsSpeed_bp = (uint16_t) (SPORT_DATA_U32(packet) / 1000);
    }
    else if (appId >= GPS_COURS_FIRST_ID && appId <= GPS_COURS_LAST_ID) {
      //uint32_t course = SPORT_DATA_U32(packet)/100;
    }
    else if (appId >= GPS_ALT_FIRST_ID && appId <= GPS_ALT_LAST_ID) {
      alt = SPORT_DATA_S32(packet)/100;
      HAS_ALT = true;
    }
    else if (appId >= GPS_LONG_LATI_FIRST_ID && appId <= GPS_LONG_LATI_LAST_ID) {
      uint32_t gps_long_lati_data = SPORT_DATA_U32(packet);
      uint32_t gps_long_lati_b1w, gps_long_lati_a1w;
      gps_long_lati_b1w = (gps_long_lati_data & 0x3fffffff) / 10000;
      gps_long_lati_a1w = (gps_long_lati_data & 0x3fffffff) % 10000;

      switch ((gps_long_lati_data & 0xc0000000) >> 30) {
      case 0:
        lat_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
        lat_ap = gps_long_lati_a1w;
        NS = 'N';
        break;
      case 1:
        lat_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
        lat_ap = gps_long_lati_a1w;
        NS = 'S';
        break;
      case 2:
        lon_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
        lon_ap = gps_long_lati_a1w;
        EW = 'E';
        break;
      case 3:
        lon_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
        lon_ap = gps_long_lati_a1w;
        EW = 'W';
        break;
      }
      if ((NS == 'N' || NS == 'S') && (EW == 'E' || EW == 'W')){
        HAS_FIX = true;
      }
    }
    break;
  }
}

// Receive buffer state machine state enum
enum FrSkyDataState {
  STATE_DATA_IDLE,
  STATE_DATA_IN_FRAME,
  STATE_DATA_XOR,
};

void encodeTargetData(uint8_t data) {
  static uint8_t numPktBytes = 0;
  static uint8_t dataState = STATE_DATA_IDLE;

  if (data == START_STOP) {
    dataState = STATE_DATA_IN_FRAME;
    numPktBytes = 0;
  }
  else {
    switch (dataState) {
    case STATE_DATA_XOR:
      frskyRxBuffer[numPktBytes++] = data ^ STUFF_MASK;
      dataState = STATE_DATA_IN_FRAME;
      break;

    case STATE_DATA_IN_FRAME:
      if (data == BYTESTUFF)
        dataState = STATE_DATA_XOR; // XOR next byte
      else
        frskyRxBuffer[numPktBytes++] = data;
      break;
    }
  }

  if (numPktBytes == FRSKY_RX_PACKET_SIZE) {
    processSportPacket(frskyRxBuffer);
    dataState = STATE_DATA_IDLE;
  }
}
#endif
