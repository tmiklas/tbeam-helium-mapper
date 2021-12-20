/*
TTGO T-Beam Mapper for Helium
Copyright (C) 2021 by @Max_Plastix

Forked from:
TTGO T-BEAM Tracker for The Things Network
Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

This code requires LMIC library by Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

// -----------------------------------------------------------------------------
// CONFIGURATION  
// Stuff you might reasonably want to change is here:
// -----------------------------------------------------------------------------

// Select which T-Beam board is being used. Only uncomment one.
//#define T_BEAM_V07  // AKA Rev0 (first board released)
#define T_BEAM_V10  // AKA Rev1 (second board released), also for "v1.1"

#define MIN_DIST                68.0           // Minimum distance in meters from the last sent location before we can send again. A hex is about 340m.
#define STATIONARY_TX_INTERVAL  ( 2 * 60)      // If no minimum movement, the LoRa frame will still be sent once every N seconds
#define REST_WAIT               (30 * 60)      // If we still haven't moved in this many seconds, start sending even slower
#define REST_TX_INTERVAL        (10 * 60)      // Slow resting ping frequency in seconds

#define BATTERY_LOW_VOLTAGE     3.4             // Below this voltage, power off until USB power allows charging

#define LORAWAN_PORT            2               // FPort for Uplink messages -- must match Helium Console Decoder script!
#define LORAWAN_CONFIRMED_EVERY 0               // Send confirmed message for ACK every N messages (0 means never, 1 means always)
#define LORAWAN_SF              DR_SF7          // Spreading factor (recommended DR_SF7 for network map purposes, DR_SF10 is slower/more-reach)

// Uncomment to enable discarding network settings by long pressing second button
#define PREFS_DISCARD

// -----------------------------------------------------------------------------
// Version
// -----------------------------------------------------------------------------

#define APP_NAME                "Helium TTGO"
#define APP_VERSION             "1.5.3 MaxP"

// -----------------------------------------------------------------------------
// Less common Configuration iteams
// -----------------------------------------------------------------------------

#define ALWAYS_SHOW_LOGO                        // It's a great logo.  Display it with pride.
#define LOGO_DELAY              2000            // Time to show logo on first boot (ms)

#define DEBUG_PORT              Serial          // Serial debug port
#define SERIAL_BAUD             115200          // Serial debug baud rate (note that bootloader is fixed at 115200)

#define LORAWAN_ADR             0               // Do not enable ADR

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             5

// If using a single-channel gateway, uncomment this next option and set to your gateway's channel
//#define SINGLE_CHANNEL_GATEWAY  0

// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------
#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

// Verbose LoRa message callback reporting
// #define DEBUG_LORA_MESSAGES

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED       100
#define EV_PENDING      101
#define EV_ACK          102
#define EV_RESPONSE     103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------

#define REQUIRE_RADIO           true            // If true, we will fail to start if the radio is not found

#define I2C_SDA         21
#define I2C_SCL         22

#if defined(T_BEAM_V07)
#define LED_PIN         14
#define MIDDLE_BUTTON_PIN      39
#elif defined(T_BEAM_V10)
#define MIDDLE_BUTTON_PIN      38 // Middle button SW5
#endif

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

#define SSD1306_ADDRESS 0x3C

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM  1 // SerialX
#define GPS_BAUDRATE    115200 // Make haste!  NMEA is big.. go fast
#define USE_GPS         1

#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10) // Or T-Beam v1.1
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#if defined(T_BEAM_V10)
#define RESET_GPIO      14
#else
#define RESET_GPIO      23
#endif
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

// #define AXP192_SLAVE_ADDRESS  0x34 // Now defined in axp20x.h
#define GPS_POWER_CTRL_CH     3
#define LORA_POWER_CTRL_CH    2
#define PMU_IRQ               35
