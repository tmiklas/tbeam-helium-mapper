#pragma once
#include <Arduino.h>

void screen_print(const char *text);
void screen_print(const char *text, uint8_t x, uint8_t y);
void screen_print(const char *text, uint8_t x, uint8_t y, uint8_t alignment);

void screen_update(void);

void screen_body(boolean in_menu, const char *menu_prev, const char *menu_cur, const char *menu_next,
                 boolean highlighted);
void screen_header(unsigned int tx_interval_s, float min_dist_moved, char *cached_sf_name, boolean in_deadzone,
                   boolean stay_on, boolean never_rest);

void screen_off(void);
void screen_on(void);

void screen_show_logo(void);
void screen_setup(void);
void screen_end(void);
