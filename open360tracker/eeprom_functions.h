#ifndef EEPROM_FUNCTIONS
#define EEPROM_FUNCTIONS

#include <inttypes.h>


// functions to store a byte/int/float to an eeprom position
short LoadByteFromEEPROM(uint8_t position);
int   LoadIntegerFromEEPROM(uint8_t position);
float LoadFloatFromEEPROM(uint8_t position);

// functions to load a byte/int/float from an eeprom position
void  StoreShortToEEPROM(short value, uint8_t position);
void  StoreIntegerToEEPROM(int value, uint8_t position);
void  StoreFloatToEEPROM(float value, uint8_t position);


#endif


