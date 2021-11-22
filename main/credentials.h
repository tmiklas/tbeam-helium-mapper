/*

Credentials file

*/

#pragma once

// Only one of these settings must be defined
//#define USE_ABP
#define USE_OTAA

#ifdef USE_ABP

    // LoRaWAN NwkSKey, network session key
    static const u1_t PROGMEM NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // LoRaWAN AppSKey, application session key
    static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // LoRaWAN end-device address (DevAddr)
    // This has to be unique for every node
    static const u4_t DEVADDR = 0x00000000;

#endif

#ifdef USE_OTAA

    // This EUI must be in little-endian format, so least-significant-byte (lsb)
    // first. When copying an EUI from ttnctl output, this means to reverse
    // the bytes. For TTN issued EUIs the last bytes should be 0x00, 0x00,
    // 0x00.
    static const u1_t PROGMEM APPEUI[8]  = { 0xA0, 0x2E, 0x8E, 0x90, 0xBF, 0xF9, 0x81, 0x60 }; // Reverse Helium Console

    // This should also be in little endian format (lsb), see above.
    // Note: If all values are zero, the DevEUI will be generated automatically based on the device macaddr
    static u1_t DEVEUI[8]  = { 0, 0, 0, 0, 0, 0, 0, 0 };

    // This key should be in big endian format (msb) (or, since it is not really a
    // number but a block of memory, endianness does not really apply). In
    // practice, a key taken from ttnctl can be copied as-is.
    // The key shown here is the semtech default key.
    static const u1_t PROGMEM APPKEY[16] = { 0xCF, 0x4B, 0x3E, 0x8F, 0x8F, 0xCB, 0x77, 0x9C, 0x8E, 0x1C, 0xAE, 0xE3, 0x11, 0x71, 0x2A, 0xE5 };
    
#endif
