#include "config.h"

// Arduino-Mavlink-Library: http://www.multixmedia.org/test/mavlink-tracker-library.tgz

#ifdef MAVLINK
#include "telemetry.h"
#include <avr/io.h>

#define MAVLINK_MAX_PAYLOAD_LEN 36
#include <Mavlink.h>

static int32_t p_lat = 0; // latitude
static int32_t p_lon = 0; // longitude
static int32_t p_alt = 0; // altitude
static int16_t p_sats = 0; // number of satelites

int32_t getTargetLat() {
  return p_lat;
}

int32_t getTargetLon() {
  return p_lon;
}

int16_t getTargetAlt() {
  return p_alt;
}

int16_t getSats() {
  return p_sats;
}

void mavlink_handleMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      p_lat = mavlink_msg_gps_raw_int_get_lat(msg) / 100;
      p_lon = mavlink_msg_gps_raw_int_get_lon(msg) / 100;
      p_sats = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
      p_alt = mavlink_msg_gps_raw_int_get_alt(msg) / 10;
      HAS_ALT = true;

      // fix_type: GPS lock 0-1=no fix, 2=2D, 3=3D
      if (mavlink_msg_gps_raw_int_get_fix_type(msg) > 1) {
        HAS_FIX = true;
      } else {
        HAS_FIX = false;
      }
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      p_lat = mavlink_msg_global_position_int_get_lat(msg) / 100;
      p_lon = mavlink_msg_global_position_int_get_lon(msg) / 100;
      p_alt = mavlink_msg_global_position_int_get_alt(msg) / 10;
      HAS_ALT = true;

      if (p_lat != 0 || p_lon != 0) {
        HAS_FIX = true;
      } else {
        HAS_FIX = false;
      }
      break;
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
