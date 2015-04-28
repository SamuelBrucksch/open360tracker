/**
*
* Calculation from cesco1: https://github.com/Cesco1/ManualRTH/blob/master/Nav.ino
*/

#include "math.h"

float lonScale = 1.0f;

void calcTargetDistanceAndHeading(geoCoordinate_t *tracker, geoCoordinate_t *target) {
  int16_t dLat = tracker->lat - target->lat;
  int16_t dLon = (tracker->lon - target->lon);// * lonScale;
  target->distance = uint16_t(sqrt(sq(dLat) + sq(dLon)) * 1.113195f); 
  target->heading = uint16_t(atan2(dLon,dLat) * 572.90f);
}

void setHome(geoCoordinate_t *tracker, geoCoordinate_t *target){
  //todo check if this is correct
  float rads = (abs(float(target->lat)) / 10000000.0) * 0.0174532925;
  lonScale = cos(rads);
  //Serial.print("lon scale: ");
  //Serial.print(lonScale, 6);

  tracker->lat = target->lat;
  tracker->lon = target->lon;
  tracker->alt = target->alt;
  HOME_SET = true;
}
