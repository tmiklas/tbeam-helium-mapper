
#pragma once

#include <Arduino.h>

void gps_loop(void);
void gps_setup(void);
void gps_time(char *buffer, uint8_t size);
float gps_latitude(void);
float gps_distanceBetween(float lat1, float long1, float lat2, float long2);
float gps_longitude(void);
float gps_altitude(void);
float gps_hdop(void);
uint8_t gps_sats(void);
float gps_hdop(void);
float gps_speed(void);
void gps_passthrough(void);

