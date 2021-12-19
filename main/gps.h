
#pragma once

void gps_loop(void);
void gps_setup(void);
void gps_time(char *buffer, uint8_t size);
float gps_latitude(void);
float gps_distanceBetween(double lat1, double long1, double lat2, double long2);
float gps_longitude(void);
float gps_altitude(void);
float gps_hdop(void);
uint8_t gps_sats(void);
float gps_hdop(void);
