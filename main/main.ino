/*
  This module and those attached with it have been modified for the Helium Network by Fizzy. The following has been changed from the original modifications for Helium, by longfi-arduino:
  - Added Helium Startup Logo
  - Changed App Name and Version of device to reflect more of a device name and number scheme.
  - Enabled long press middle button to Discard Prefs by default for future troubleshooting on device.
  - Changed Text output to reflect Helium, and not TTL (Code referances ttn, just to prevent brakes in this awesome code)
  - Changed credentials file to use OTAA by default.
  - Changed GPS metric output text "Error", to "Accuracy/HDOP".
*/
/*
  Modifications by Max-Plastix
  - DevEUI is auto-generated per-device
  - Print all three ID values to console at boot, in a format that pastes into Helium Console. 
  - Change Payload to #2 and match CubeCell Mapper format, including battery voltage. (Shared decoder with CubeCell)
*/
/*

  Main module

  # Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

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

#include <Arduino.h>
#include "configuration.h"
#include "rom/rtc.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <axp20x.h>

bool          movementTrackingActive  = false;  // have we got a lock and sent first frame...
unsigned int  stationaryTickCounter   = 1;  // count how many we should have TX but didn't because we didn't move
bool          justSendNow             = false; // if set to true, this will force TX - no more
unsigned int  stationaryTxInterval    = STATIONARY_TX_INTERVAL;
float         last_send_lat           = 0;
float         last_send_lon           = 0;
float         min_dist_moved          = MIN_DIST;
float         dist_moved              = UINT32_MAX;
unsigned int  adjusted_SEND_INTERVAL  = SEND_INTERVAL;

// do we want to auto-scale transmit window size?
bool autoScaleTX = false;

AXP20X_Class axp;
bool pmu_irq = false;
const char *baChStatus = "unknown";

bool ssd1306_found = false;
bool axp192_found = false;

bool packetSent, packetQueued;

#if defined(PAYLOAD_USE_FULL)
// includes number of satellites and accuracy
static uint8_t txBuffer[10];
#elif defined(PAYLOAD_USE_CAYENNE)
// CAYENNE DF
static uint8_t txBuffer[11] = {0x03, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#elif defined (PAYLOAD_USE_MAPPER)
  static uint8_t txBuffer[11];
#endif

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause; // the reason we booted this time

// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

void buildPacket(uint8_t txBuffer[]); // needed for platformio

/**
   If we have a valid position send it to the server.
   @return true if we decided to send.
*/
bool trySend() {
  packetSent = false;

  // distance from last transmitted location
  float dist_moved = gps_distanceBetween(last_send_lat, last_send_lon, gps_latitude(), gps_longitude());

  // We also wait for altitude being not exactly zero, because the GPS chip generates a bogus 0 alt report when first powered on
  if (0 < gps_hdop() && gps_hdop() < 50 && gps_latitude() != 0 && gps_longitude() != 0 && gps_altitude() != 0)
  {
    char buffer[40];
    snprintf(buffer, sizeof(buffer), "Lat: %10.6f\n", gps_latitude());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "Long: %10.6f\n", gps_longitude());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "HDOP: %4.2fm\n", gps_hdop());
    screen_print(buffer);

    // check if we should transmit based on distance covered since last TX or are there other reasons:
    // - TX when distance traveled above required threshold
    // - TX when we are not tracking distance yet - happens on first trySend() after boot-up/reset - default is distance to (0,0) so more than usual 50m
    // - TX when stationary (not met movement requirement) but waited enough TX cycles (report I'm still alive)
    // - TX when USR button short presset to force send right now
    // in all other cases sleep and return false

    if (dist_moved > min_dist_moved || justSendNow || stationaryTickCounter % stationaryTxInterval == 0) {

      // rudimentary debug to serial port
      if (dist_moved > min_dist_moved) {
        Serial.println("** MOVING");
      } else if (justSendNow) {
        Serial.println("** JUST_SEND_NOW");
      } else if (stationaryTickCounter % stationaryTxInterval != 0) {
        Serial.println("** TICK_COUNT_MODULO_ZERO");
      }

      // showing movement on screen depends on if we track it yet or not
      // first TX after boot-up does not have it, but later TX will do
      if (movementTrackingActive) {
        snprintf(buffer, sizeof(buffer), "Movement: %4.1fm\n", dist_moved);
        screen_print(buffer);
      }

      // TX window auto-scaling
      // desired ping distance is 200m
      if (autoScaleTX && !justSendNow) {
        float newWindow = DISTANCE_TARGET / (dist_moved / adjusted_SEND_INTERVAL);  // seconds
        if (newWindow > SEND_INTERVAL) { // both are in millis!
          // we're too slow... default back to old mode with ping every SEND_INTERVAL
          newWindow = SEND_INTERVAL;
        } else if (newWindow < 2000) {
          // this is 100m/s movement or 360kph or 220mph :-o
          newWindow = 2000;
        }
        snprintf(buffer, sizeof(buffer), "TX window: %4.1fsec\n", newWindow / 1000);
        screen_print(buffer);
        adjusted_SEND_INTERVAL = newWindow; // millis
      }

      // set back to normal mode
      justSendNow = false;

      // start movement tracking
      movementTrackingActive = true;

      // reset stationary cycle counter
      stationaryTickCounter = 1;

      // set last transmit location for later
      last_send_lat = gps_latitude();
      last_send_lon = gps_longitude();

      // prepare LoRa frame
      buildPacket(txBuffer);

      bool confirmed = (LORAWAN_CONFIRMED_EVERY > 0) && (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);
      if (confirmed) {
        Serial.println("ACK requested");
      }

      // send it!
      packetQueued = true;
      ttn_send(txBuffer, sizeof(txBuffer), LORAWAN_PORT, confirmed);
      return true;

    } else {

      // we are stationary and have nothing to report to network
      // so we only update screen and show how many cycles untill next stationary TX 
      // this is the "Next TTL" countdown

      // showing movement on screen depends on if we track it yet or not
      if (movementTrackingActive) {
        snprintf(buffer, sizeof(buffer), "Movement: %4.1fm\n", dist_moved);
        screen_print(buffer);
        snprintf(buffer, sizeof(buffer), "Next TTL: %03d\n", stationaryTxInterval - stationaryTickCounter);
        screen_print(buffer);        
      }
      // update counter
      stationaryTickCounter++;
      Serial.println("(stationary)");
      return false;
    }

  }
  else {
    // it was bogus read so not transmitting either
    return false;
  }
}


void doDeepSleep(uint64_t msecToWake)
{
  Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

  // not using wifi yet, but once we are this is needed to shutoff the radio hw
  // esp_wifi_stop();

  screen_off(); // datasheet says this will draw only 10ua
  LMIC_shutdown(); // cleanly shutdown the radio

  if (axp192_found) {
    // turn on after initial testing with real hardware
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
  }

  // FIXME - use an external 10k pulldown so we can leave the RTC peripherals powered off
  // until then we need the following lines
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // Only GPIOs which are have RTC functionality can be used in this bit map: 0,2,4,12-15,25-27,32-39.
  uint64_t gpioMask = (1ULL << MIDDLE_BUTTON_PIN);

  // FIXME change polarity so we can wake on ANY_HIGH instead - that would allow us to use all three buttons (instead of just the first)
  gpio_pullup_en((gpio_num_t) MIDDLE_BUTTON_PIN);

  esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // call expects usecs
  esp_deep_sleep_start();                              // TBD mA sleep current (battery)
}


void sleep() {
  float_t batt_volts = axp.getBattVoltage() / 1000.0;
  float_t charge_ma = axp.getBattChargeCurrent();
  float_t discharge_ma = axp.getBattDischargeCurrent();
  static boolean screen_sleep = false;

  if (1)
  {
    char buffer[30];
    snprintf(buffer, sizeof(buffer), "%.2fv %.1fmA\n", batt_volts, charge_ma - discharge_ma);
    Serial.println(buffer);
  }

  if ((batt_volts > SLEEP_VOLTAGE) || (charge_ma - discharge_ma > 1.0)) {
    if (screen_sleep) {
      screen_sleep = false;
      screen_on();
    }
  } else {
    if (!screen_sleep) {
      screen_sleep = true;
      screen_off();
    }
  }

#if 0  
  // Set the user button to wake the board
  sleep_interrupt(MIDDLE_BUTTON_PIN, LOW);

  doDeepSleep(SEND_INTERVAL);
#endif
}


void callback(uint8_t message) {
#if 0
  {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "MSG %d\n", message);
    screen_print(buffer);
  }
#endif
  if (EV_JOIN_TXCOMPLETE == message) Serial.println("# JOIN_TXCOMPLETE");
  if (EV_TXCOMPLETE == message) Serial.println("# TXCOMPLETE");
  if (EV_RXSTART == message) Serial.println("# RXSTART");
  if (EV_TXCANCELED == message) Serial.println("# TXCANCELED");
  if (EV_TXSTART == message) Serial.println("# TXSTART");
  if (EV_JOINING == message) Serial.println("# JOINING");
  if (EV_JOINED == message) Serial.println("# JOINED");
  if (EV_JOIN_FAILED == message) Serial.println("# JOIN_FAILED");
  if (EV_REJOIN_FAILED == message) Serial.println("# REJOIN_FAILED");
  if (EV_RESET == message) Serial.println("# RESET");
  if (EV_LINK_DEAD == message) Serial.println("# LINK_DEAD");
  if (EV_ACK == message) Serial.println("# ACK");
  if (EV_PENDING == message) Serial.println("# PENDING");
  if (EV_QUEUED == message) Serial.println("# QUEUED");

  if (EV_TXSTART == message) {
    screen_print("Sending.. ");
  }
  // We only want to say 'packetSent' for our packets (not packets needed for joining)
  if (EV_TXCOMPLETE == message && packetQueued) {
    screen_print("sent.\n");
    packetQueued = false;
    packetSent = true;
  }

  if (EV_RESPONSE == message) {
    screen_print("[Helium] Response: ");

    size_t len = ttn_response_len();
    uint8_t data[len];
    ttn_response(data, len);

    char buffer[6];
    for (uint8_t i = 0; i < len; i++) {
      snprintf(buffer, sizeof(buffer), "%02X", data[i]);
      screen_print(buffer);
    }
    screen_print("\n");
  }
}


void scanI2Cdevice(void)
{
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
      nDevices++;

      if (addr == SSD1306_ADDRESS) {
        ssd1306_found = true;
        Serial.println("ssd1306 display found");
      }
      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("axp192 PMU found");
      }
    } else if (err == 4) {
      Serial.print("Unknow error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

/**
   Init the power manager chip

   axp192 power
    DCDC1 0.7-3.5V @ 1200mA max -> OLED // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> unused
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery // charges the tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS
*/

void axp192Init() {
  if (axp192_found) {
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      Serial.println("AXP192 Begin PASS");
    } else {
      Serial.println("AXP192 Begin FAIL");
    }
    // axp.setChgLEDMode(LED_BLINK_4HZ);
  #if 0
    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
    Serial.println("----------------------------------------");
#endif

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED & AXP192 power
    axp.setDCDC1Voltage(3300); // for the OLED power

    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    //Serial.printf("LDO1: %s\n", axp.isLDO1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
      pmu_irq = true;
    }, FALLING);

    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
    axp.clearIRQ();

    if (axp.isChargeing()) {
      baChStatus = "Charging: Yes";
    } else {
      baChStatus = "Charging: No";
    }
  } else {
    Serial.println("AXP192 not found");
  }
}


// Perform power on init that we do on each wake from deep sleep
void initDeepSleep() {
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();
  /*
    Not using yet because we are using wake on all buttons being low

    wakeButtons = esp_sleep_get_ext1_wakeup_status();       // If one of these buttons is set it was the reason we woke
    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons) // we must have been using the 'all buttons rule for waking' to support busted boards, assume button one was pressed
      wakeButtons = ((uint64_t)1) << buttons.gpios[0];
  */

  Serial.printf("booted, wake cause %d (boot count %d)\n", wakeCause, bootCount);
}


void setup() {
  // Debug
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
#endif

  initDeepSleep();

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  axp192Init();

  // Buttons & LED
  pinMode(MIDDLE_BUTTON_PIN, INPUT_PULLUP);

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
#endif

  // Hello
  DEBUG_MSG(APP_NAME " " APP_VERSION "\n");

  // Don't init display if we don't have one or we are waking headless due to a timer event
  if (0 && wakeCause == ESP_SLEEP_WAKEUP_TIMER)
    ssd1306_found = false; // forget we even have the hardware

  if (ssd1306_found) {
    screen_setup();
  }

  // Init GPS
  gps_setup();

  // Show logo on first boot after removing battery
#ifndef ALWAYS_SHOW_LOGO
  if (bootCount == 0) {
#endif
    screen_print(APP_NAME " " APP_VERSION, 0, 0);
    screen_show_logo();
    screen_update();
    delay(LOGO_DELAY);
#ifndef ALWAYS_SHOW_LOGO
  }
#endif

  // Helium setup
  if (!ttn_setup()) {
    screen_print("[ERR] Radio module not found!\n");

    if (REQUIRE_RADIO) {
      delay(MESSAGE_TO_SLEEP_DELAY);
      screen_off();
      sleep_forever();
    }
  }
  else {
    ttn_register(callback);
    // ttn_erase_prefs();
    ttn_join();
    ttn_adr(LORAWAN_ADR);
  }
}

void loop() {
  gps_loop();
  ttn_loop();
  screen_loop();

  if (packetSent) {
    packetSent = false;
    sleep();
  }

  // Short press on power button (near USB) also causes PMIC IRQ
  if (axp192_found && pmu_irq) {
    const char *irq_name = "Unrecognized";
    pmu_irq = false;
    axp.readIRQ();
    if (axp.isChargingIRQ()) {
       irq_name = "ChargingIRQ";
    }
    if (axp.isVbusRemoveIRQ()) {
      irq_name = "VbusRemoveIRQ";
    }
    if (axp.isPEKShortPressIRQ()) {
      irq_name = "PEKShortPressIRQ";
    }
    char buffer[40];
    snprintf(buffer, sizeof(buffer), "%s\n", irq_name);
    screen_print(buffer);

    // digitalWrite(2, !digitalRead(2));  No idea what this does
    axp.clearIRQ();
  }

  // if user presses button for more than 3 secs, discard our network prefs and reboot (FIXME, use a debounce lib instead of this boilerplate)
  static uint32_t pressTime = 0; // what tick should we call this press long enough
  if (!digitalRead(MIDDLE_BUTTON_PIN)) {
    // Pressure is on
    if (!pressTime) { // just started a new press
      pressTime = millis();
    }
  } else if (pressTime) {
    // we just did a release
    if (millis() - pressTime > 1000) {
      // held long enough
      Serial.println("Long press!");
      if (autoScaleTX) {
        // turn off auto scaling to static trigger
        autoScaleTX = false;
        adjusted_SEND_INTERVAL = SEND_INTERVAL;
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "TX scaling OFF\n");
        screen_print(buffer);        
        // screen_print("TX Scaling OFF");
      } else {
        // enable auto-scaling
        autoScaleTX = true;
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "TX scaling ON\n");
        screen_print(buffer);        
        // screen_print("TX Scaling ON");
      }
// #ifndef PREFS_DISCARD
//       screen_print("Discarding prefs disabled\n");
// #endif
// #ifdef PREFS_DISCARD
//       screen_print("Discarding prefs!\n");
//       ttn_erase_prefs();
//       delay(5000); // Give some time to read the screen
//       ESP.restart();
// #endif
    } else {
      // short press, send beacon
      Serial.println("Short press.");
      justSendNow = true;
      trySend();
    }
    pressTime = 0;  // Released
  }

  // Send every SEND_INTERVAL millis
  static uint32_t last = 0;
  static bool first = true;
  if (0 == last || millis() - last > adjusted_SEND_INTERVAL) {
    if (trySend()) {
      last = millis();
      first = false;
      //      Serial.println("TRANSMITTED");
    } else {
      if (first) {
        screen_print("Waiting GPS lock\n");
        first = false;
      }

      // update time of last transmission time also when not transmitted, 
      // to make sure we don't fall into a fast loop trying to send when not moving
      // no need to waste battery cycling empty...
      last = millis();

#ifdef GPS_WAIT_FOR_LOCK
      if (millis() > GPS_WAIT_FOR_LOCK) {
        sleep();
      }
#endif

      // No GPS lock yet, let the OS put the main CPU in low power mode for 100ms (or until another interrupt comes in)
      // i.e. don't just keep spinning in loop as fast as we can.
      delay(100);
    }
  }
}
