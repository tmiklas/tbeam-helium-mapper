#pragma once

#include <Arduino.h>

void sleep_interrupt(uint8_t gpio, uint8_t mode);
void sleep_interrupt_mask(uint64_t mask, uint8_t mode);
void sleep_millis(uint64_t ms);
void sleep_seconds(uint32_t seconds);
void sleep_forever(void);
