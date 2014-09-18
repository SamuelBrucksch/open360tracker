#ifndef FRSKY_COMMON_H
#define FRSKY_COMMON_H


//TODO check if this is the right conversion
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap){
  // we want convert from frsky to millionth of degrees
  // 0302.7846 -> 03 + (02.7846/60) = 3.04641 -> 3046410
  // first the upper part
  uint32_t result = ((bp / 100) * 100000);
  // now the lower part
  result = result + (((bp % 100) * 100000 + ap*10)  ) / 60;
  // take sign into account
  return ((int32_t)result*neg);
}


#endif

