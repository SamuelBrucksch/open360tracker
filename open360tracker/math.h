#ifndef MATH_H
#define MATH_H

#include "inttypes.h"
#include "defines.h"
#include "Arduino.h"

void calcTargetDistanceAndHeading(geoCoordinate_t *tracker, geoCoordinate_t *target);
void setHome(geoCoordinate_t *tracker, geoCoordinate_t *target);
#endif
