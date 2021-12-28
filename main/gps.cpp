/* Revised to use UBlox native binary protocol to engage NMEA */
#include "gps.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#include "SparkFun_Ublox_Arduino_Library_Series_6_7.h"
#include "configuration.h"

HardwareSerial gpsSerial(GPS_SERIAL_NUM);

SFE_UBLOX_GPS myGNSS;
TinyGPSPlus _gps;

void gps_time(char* buffer, uint8_t size) {
  snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude() {
  return _gps.location.lat();
}

float gps_distanceBetween(float last_lat, float last_lon, float lat, float lon) {
  return _gps.distanceBetween(last_lat, last_lon, lat, lon);
}

float gps_longitude() {
  return _gps.location.lng();
}

float gps_altitude() {
  return _gps.altitude.meters();
}

float gps_hdop() {
  return _gps.hdop.hdop();
}

uint8_t gps_sats() {
  return _gps.satellites.value();
}

float gps_speed() {
  return _gps.speed.kmph();
}

boolean fresh_gps = false;

void gps_setup(void) {
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsSerial.setRxBufferSize(2048);  // Default is 256

  if (0)
    myGNSS.enableDebugging();

  bool changed_speed = false;

  // Check all the possible GPS bitrates to get in sync
  do {
    gpsSerial.updateBaudRate(GPS_BAUDRATE);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS connected.");
      break;
    }

    // Well, wasn't where we expected it
    changed_speed = true;

    Serial.println("Trying 115200...");
    gpsSerial.updateBaudRate(115200);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS found at 115200 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Trying 9600...");
    gpsSerial.updateBaudRate(9600);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS found at 9600 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Trying 38400...");
    gpsSerial.updateBaudRate(38400);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS found at 38400 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Trying 57600...");
    gpsSerial.updateBaudRate(57600);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS found at 57600 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Could not connect to GPS. Retrying all speeds...");
  } while (1);

  myGNSS.setUART1Output(COM_TYPE_NMEA);  // We do want NMEA

  if (0)
    myGNSS.factoryReset();

  myGNSS.setNavigationFrequency(2);  // Produce X solutions per second

#if 0
  // On the Neo6M, these are all off by default anyway:
  myGNSS.disableNMEAMessage(UBX_NMEA_DTM, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GAQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GBQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GBS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GLQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GPQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_TXT, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VLW, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
#endif  

  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1); // Don't need SV list (on by default)
  myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);  // For Speed
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);  // For Time & Location & SV count

  if (changed_speed)
    myGNSS.saveConfiguration();  // Save the current settings to flash and BBR
}

void gps_passthrough(void) {
  Serial.println("GPS Passthrough forever...");
  while (1) {
    if (gpsSerial.available())
      Serial.write(gpsSerial.read());
    if (Serial.available())
      gpsSerial.write(Serial.read());
  }
}

void gps_loop(void) {
  while (gpsSerial.available()) {
    _gps.encode(gpsSerial.read());
  }
}