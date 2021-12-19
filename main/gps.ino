/* Revised to use UBlox native binary protocol instead of NMEA */
#include "configuration.h"
#include "gps.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

HardwareSerial gpsSerial(GPS_SERIAL_NUM);
SFE_UBLOX_GNSS myGNSS;

/* Copied from https://github.com/mikalhart/TinyGPSPlus/blob/master/src/TinyGPS%2B%2B.cpp 
(GPL v2.1)
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.
*/
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

void gps_time(char * buffer, uint8_t size) {
    //snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude() {
    //return _gps.location.lat();
    return 0.0;
}

float gps_distanceBetween(float last_lat, float last_lon, float lat, float lon) {
    return (float) distanceBetween(last_lat, last_lon, lat, lon);
}

float gps_longitude() {
    //return _gps.location.lng();
    return 0.0;
}

float gps_altitude() {
    //return _gps.altitude.meters();
    return 0.0;
}

float gps_hdop() {
    //return _gps.hdop.hdop();
    return 0.0;
}

uint8_t gps_sats() {
    //return _gps.satellites.value();
    return 0;
}

float gps_speed() {
    return 0.0;
}

boolean fresh_gps = false;

#if 0
void foofreshPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
      fresh_gps = true;
      Serial.println();

      Serial.print(F("Time: "));        // Print the time
      uint8_t hms = ubxDataStruct.hour; // Print the hours
      if (hms < 10)
          Serial.print(F("0")); // Print a leading zero if required
      Serial.print(hms);
      Serial.print(F(":"));
      hms = ubxDataStruct.min; // Print the minutes
      if (hms < 10)
          Serial.print(F("0")); // Print a leading zero if required
      Serial.print(hms);
      Serial.print(F(":"));
      hms = ubxDataStruct.sec; // Print the seconds
      if (hms < 10)
          Serial.print(F("0")); // Print a leading zero if required
      Serial.print(hms);
      Serial.print(F("."));
      unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
      if (millisecs < 100)
          Serial.print(F("0")); // Print the trailing zeros correctly
      if (millisecs < 10)
          Serial.print(F("0"));
      Serial.print(millisecs);

      long latitude = ubxDataStruct.lat; // Print the latitude
      Serial.print(F(" Lat: "));
      Serial.print(latitude);

      long longitude = ubxDataStruct.lon; // Print the longitude
      Serial.print(F(" Long: "));
      Serial.print(longitude);
      Serial.print(F(" (degrees * 10^-7)"));

      long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
      Serial.print(F(" Height above MSL: "));
      Serial.print(altitude);
      Serial.println(F(" (mm)"));
}
#endif

void gps_setup(void) {
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsSerial.setRxBufferSize(2048); // Default is 256
  // myGNSS.enableDebugging();

  bool changed_speed = false;

  // Check all the possible GPS bitrates to get in sync
  do {
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
      myGNSS.setSerialRate(115200);
      continue;
    }
    
    Serial.println("Trying 38400...");
    gpsSerial.updateBaudRate(38400);
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GPS found at 38400 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      myGNSS.setSerialRate(115200);
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

    myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    if (0)
        myGNSS.factoryReset();
    
    /* Not so interesting, since it's always version 0.0 */
    #if 0
    Serial.print(F("GPS Protocol Version: "));
    byte versionHigh = myGNSS.getProtocolVersionHigh();
    Serial.print(versionHigh);
    Serial.print(".");
    byte versionLow = myGNSS.getProtocolVersionLow();
    Serial.println(versionLow);
    #endif
    
    if (changed_speed)
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
        
 //   myGNSS.setAutoPVTcallback(&freshPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
    myGNSS.setNavigationFrequency(1); //Produce one solution per second
  }


void gps_loop(void) {
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();

    if (!fresh_gps)
        return;
    fresh_gps = false;

    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
}
