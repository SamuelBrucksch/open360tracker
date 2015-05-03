#include "config.h"

// Arduino-Mavlink-Library: http://www.multixmedia.org/test/mavlink-tracker-library.tgz

#ifdef MAVLINK
#include "telemetry.h"
#include <avr/io.h>

#define MAVLINK_MAX_PAYLOAD_LEN 36
#include <Mavlink.h>

static long p_lat = 0.0;
static long p_long = 0.0;
static long p_alt = 0.0;
static int16_t sats = 0;

int32_t getTargetLat() {
  return p_lat;
}

int32_t getTargetLon() {
  return p_long;
}

int16_t getTargetAlt() {
  return p_alt;
}

int16_t getSats() {
  return sats;
}

void mavlink_handleMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        p_lat = (float)mavlink_msg_gps_raw_int_get_lat(msg) / 10000000.0;
        p_long = (float)mavlink_msg_gps_raw_int_get_lon(msg) / 10000000.0;
        p_alt = (float)mavlink_msg_gps_raw_int_get_alt(msg) / 1000.0;
        sats = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
        break;
      }
  }
}

void encodeTargetData(uint8_t c) {
  mavlink_status_t status;
  mavlink_message_t msg;
  if (mavlink_parse_char(0, c, &msg, &status)) {
    mavlink_handleMessage(&msg);
  }
}

#endif
