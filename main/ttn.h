#pragma once

#include <Arduino.h>
#include <lmic.h>

uint32_t ttn_get_count();
boolean ttn_send(uint8_t *data, uint8_t data_size, uint8_t port, bool confirmed);
void ttn_set_sf(dr_t sf);
void ttn_get_sf_name(char *b, size_t len);
size_t ttn_response_len(void);
void ttn_erase_prefs(void);
void ttn_loop(void);
void ttn_write_prefs(void);
void ttn_response(uint8_t *port, uint8_t *buffer, size_t len);
void ttn_adr(bool enabled);
void ttn_join(void);
bool ttn_setup(void);
