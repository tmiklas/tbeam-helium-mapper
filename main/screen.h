#pragma once
#include <Arduino.h>
#include <SSD1306.h>

void screen_print(const char *text);
void screen_print(const char *text, uint8_t x, uint8_t y);

void screen_update(void);
void screen_loop(unsigned long int tx_interval_ms, float min_dist_moved, char *cached_sf_name, int sats);
void screen_off(void);

void screen_show_logo(void);
void screen_setup(void);
