#pragma once
#include <Arduino.h>


void screen_print(const char *text);
void screen_print(const char *text, uint8_t x, uint8_t y);
void screen_print(const char *text, uint8_t x, uint8_t y, uint8_t alignment);

void screen_update(void);
void screen_loop(unsigned long int tx_interval_ms, float min_dist_moved, char *cached_sf_name, int sats, boolean in_menu, const char *menu_prev,
                 const char *menu_cur, const char *menu_next, boolean highlighted);
void screen_off(void);
void screen_on(void);

void screen_show_logo(void);
void screen_setup(void);
