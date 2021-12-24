/*
Credentials definition
*/

#include "credentials.h"
#ifndef USE_OTAA
#error "Only OTAA is supported for Helium!"
#endif

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. Note that this is reversed from the default order (msb) shown in Helium Console
const uint8_t PROGMEM APPEUI[8] = {0xA0, 0x2E, 0x8E, 0x90, 0xBF, 0xF9, 0x81, 0x60};  // (lsb)

// This be in little endian format (lsb).
// Note: If all values are zero, the DevEUI will be generated automatically based on the device macaddr [Recommended]
uint8_t DEVEUI[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// This key should be in big endian format (msb)
// The key shown here is the semtech default key.  You should probably change it to a lesser-known (random) value
const uint8_t PROGMEM APPKEY[16] = {0xCF, 0x4B, 0x3E, 0x8F, 0x8F, 0xCB, 0x77, 0x9C, 0x8E, 0x1C, 0xAE, 0xE3, 0x11, 0x71, 0x2A, 0xE5};
