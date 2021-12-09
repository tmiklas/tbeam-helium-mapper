<!-- ## TTGO T-Beam Tracker for The Things Network and/or The Helium Network  -->
## TTGO T-Beam Tracker for the Helium LoRaWAN Network and Mapper integrations.
by [Max-Plastix](https://github.com/Max-Plastix/tbeam-helium-mapper/)

This build is a modification of work by many experts, with input from the [Helium Discord](https://discord.gg/helium) `#mappers` community.  Thanks to @Kicko and @tmiklas especially, along with the work done on similar builds for the Heltec CubeCell mappers.

The goal of this version is to have a TTGO T-Beam that's ideally suited to vehicle mapping and tracking, taking cues from the USB Power source and position movement for activity level.  It's intended to be highly active while the vehicle is in motion.  If you want to conserve DC or power, tune the default configuration to send less frequently.

### Mandatory Configuration
Before Buliding and Uploading, you will probably want to inspect or change some items in these three files:
  - `configuration.h`
  - `platformio.ini`
  - `credentials.h`

The comments and text below will guide you on what values to look out for.  By default, this builds is for the **US915** region.  Tune accordingly.
You should choose your own AppKey value in `credentials.h`, either take the random one suggested by the Console Device entry, or make one up.

### Goals of Operation
When your car is started, and USB Power appears, the Mapper will power on, acquire GPS, and continue mapping.
It re-uses the last network Join state for faster connection and fewer packets.  (Hold down the middle button to erase the last-known network state, in case this gets botched.)

When running, the Mapper will send out a packet every time GPS indicates it has moved `MIN_DIST` meters.  This is the primary knob to turn for more/fewer packets.  A hex cell is about 340meters across, so the default 50.0 meter packet distance will send quite a few.  DC is incredibly cheap, but adjust to taste.

This is the normal operation of the Mapper in motion.. every 50 meters, one ping reporting position, battery charging from USB to a steady 4.1+ volts.  If the speed of motion is fast, it may even result in back-to-back packet sends.

When the Mapper comes to a stop, staying within `MIN_DIST` meters, it sends a hearbeat pin every `STATIONARY_TX_INTERVAL` seconds (default 60).  This serves to keep it visible on the map and report battery voltage.  Too often?  Dial up the `STATIONARY_TX_INTERVAL` to a longer timespan.

After being stationary a long time (parked) with a decreasing battery voltage, we change to a slower pace of updates.  This happens after `REST_WAIT` seconds (default: 30 minutes).  In the Rest state, the Mapper transmits every `REST_TX_INTERVAL` seconds (default: 5 minutes).

It stays in this REST state until motion resumes or USB power tops up the battery over `BATTERY_HI_VOLTAGE`.  If instead, the battery falls below `BATTERY_LOW_VOLTAGE`, the Mapper completely powers OFF.  It will power on and resume only when USB power appears.

### Building
The fantastic state of PlatformIO tools makes it easiest to build and load your TTGO from source code; no pre-built binary is necessary.

By default, the DevEUI is generated automatically to be unique to each unit, but you may want to hardcode it in `credentials.h` instead.

I tested only on a LilyGo TTGO T-Beam v1.1 on **US915**.  If you have an older v0.7 board or different region, adjust the configuration to match.

### Operation
On startup, the USB Serial port will print the DevEUI, AppID, and AppKey values, suitable for cut & paste entry into the Helium Console for your Device.

The OLED screen is always on when operating (as it uses only 10mA), and displays alternating status in the top row:
- `#ABC` is the last three hex digits of your DevEUI, so you can match it to the right device in Console.
- `4.10v` is the battery voltage
- `48mA` is the charge or discharge current to the battery.  The TTGO charges the battery cell at around 750mA from USB, when possible.
- Satellite Count is displayed on the right at all times.
- Packet Count and Time of Day (UTC) alternate on the display line every 2 seconds.

The lower part of the OLED screen shows a scrolling display of five messages.  Most often, it shows the Time and Distance between packet sends.
`60s 5m Tx.. sent.`

## Payload
The Payload Port and byte content have been changed to match that in common use by CubeCell mappers: 
Lat, Long, Altitude, Speed, Battery, and Sats.  This common decoder function can now be used for both TTGO and CubeCell mappers:

```
// From https://github.com/hkicko/CubeCell-GPS-Helium-Mapper
function Decoder(bytes, port) {
  var decoded = {};
  
  var latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  latitude = (latitude / 16777215.0 * 180) - 90;
  
  var longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  longitude = (longitude / 16777215.0 * 360) - 180;
  
  switch (port)
  {
    case 2:
      decoded.latitude = latitude;
      decoded.longitude = longitude; 
      
      var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if(sign) decoded.altitude = 0xFFFF0000 | altValue;
      else decoded.altitude = altValue;
      
      decoded.speed = parseFloat((((bytes[8]))/1.609).toFixed(2));
      decoded.battery = parseFloat((bytes[9]/100 + 2).toFixed(2));
      decoded.sats = bytes[10];
      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      break;
  }
     
  return decoded;  
}
```


### Downlink
This builds adds the option to reconfigure the Mapper remotely via Helium Downlink (network to device).  You can change the maximum Time Interval, Distance, and Battery Cut-off voltage remotely.

#### Format your Downlink Payload.

You can use the `downlink_encoder.py` Python script to convert your intent into a Base64 Payload.
```
% python downlink_encoder.py --help
usage: downlink_encoder.py [-h] [--distance DISTANCE] [--time TIME] [--cutoffvolts CUTOFFVOLTS]

Encode a downlink payload for a Helium mapper.

options:
  -h, --help            show this help message and exit
  --distance DISTANCE, -d DISTANCE
                        Map distance interval (meters)
  --time TIME, -t TIME  Minimum time interval (seconds)
  --cutoffvolts CUTOFFVOLTS, -c CUTOFFVOLTS
                        Low Voltage Power Off (volts)
 ```

For example, you might want to change the Mapper to 75 meter distance, and 600 second maximum time:
 ```
% python downlink_encoder.py -d 75 -t 600
00 4B 02 58 00
AEsCWAA=
```
That last output `AEsCWAA=` is the Base64-encoded payload, read to use.

#### Queue the Downlink packet for transmission
Paste that payload into the Helium Console under the Downlink panel for that device.  Select a specific device, then the "Cloud Down-arrow" icon on the right ("Send a manual downlink to this device") to open the Downlink panel.

Leave FPort set to the default (1), and type as default (Base64).
Queue it for transmission using the Cloud down-arrow button, and the packet should appear in the Download Queue.

When the mapper next reports (uplink), it will receive this directive and show the updates on-screen.
To rush things along, you can cause an immediate Uplink (& Downlink) by pressing the middle button on the TTGO.

#### Allowed values
Setting any value to zero will leave the present value unchanged by the downlink.
Maximum Distance interval can be 1 to 65,534 meters.  Time interval can be 1 to 65,534 seconds.  A special time interval of `-1` indicates that you want to remove any override and revert to the time interval in the software build configuration.

Battery voltage cutoff can range from 2.0 to 4.5 volts.  If you set a cutoff higher than the present battery voltage, the Mapper will immediately power down.  When it reboots (on USB power present), the default battery cutoff will be restored from the software build.

None of the Downlink values persist across power-off & on; the device always reverts to compiled-in values on startup.

# The remainder of this README is copied from earlier authors that developed this codebase:

-------------
This well thought out README, original code, and the prior versions of code all have great people behind this and I have only sought to copy and revise it further for my own personal amusement and esthetics. 
Here are the changes to the code:
  - Added Helium Startup Logo
  - Changed App Name and Version of device to reflect more of a device name and number scheme.
  - Changed Text output to reflect Helium, and not TTN (Code referances ttn, just to prevent brakes in this awesome code)
  - Changed credentials file to use OTAA by default.
  - Changed GPS metric output text "Error", to "Accuracy/HDOP".
      
In no way do I claim to have created this code, or own it. 
Proper gratitude is given to all that has worked on this before; and I am just the person that drunk the bottled water sitting on the throne. 
The next person may edit and make this even greater, and hopefuly TLDR with proper credits.
-Fizzy

Same as Fizzy I have not done any of the coding. My only idea with this fork is to make it easier for Newbies like me.
    - Removing TTN references as it confused me
    - Adding instructions where I found it hard to figure it out eg. Libraries
    - Corrected some file edit paths

-------------

Further modifications added by [tmiklas](https://github.com/tmiklas/tbeam-helium-mapper):

**2021-11-18** - Introducing **distance target** mode (a.k.a. TX window scaling)

Verison: `1.1-tm`

Most mappers usually operate in **time target** mode, where they send data to network every set interval (i.e. `SEND_INTERVAL`). There is however a different use case for fast moving mappers - like in a car at motorway speeds...

This feature allows you to switch mapping mode from time target to distace target and back. You can configure your desired distance target in `configuration.h` and turn ON/OFF by pressing the `USR` button for over 1sec:

```
#define DISTANCE_TARGET          200.0     // MUST be decimal number; distance target in meters
```

![T-Beam buttons](img/t-beam-buttons.jpeg)

Once moving, you will be able to see it operating properly and reporting TX window:

![TX Window Scaling](img/TX-window-scaling.jpeg)

In simple terms, with distance target set to 200m (as default in this code), window scaling starts working once you travel at speed over 10m/s (36k/h or 22.3mph). There's also a lower limit - do not transmit more often than every 2sec... so with 200m target you are good up to 100m/s (360k/h or 223mph) - good luck :-P

**2021-11-14** - Added some new features

Verison: `1.0-tm`

> Send Now - transmit on deman by short-pressing 2nd button

Reacts to short-press of the central button and overrides any travel distance requirement (see 2 below).
Intended for use when you are just skimming the hex you want to light up and don't want to or can't wait
for the usual 30sec cycle.

> Not moving, not transmitting (configurable)

Normally mapper sends location every 30sec, but if you are stationary, there location doesn't change (except possible GPS drift), so ongoing transmissions makes little if any sense, only burns DC. To avoid that, mapper will no longer report position every cycle unless it is actually moving or once every N cycles if stationary to say it's still alive and working.

This is controlled by 2 variables in `configuration.h` file, `MIN_DIST` and `STATIONARY_TX_INTERVAL`.

Example:

```
#define SEND_INTERVAL           (20 * 1000)     // Sleep for these many millis

// -----------------------------------------------------------------------------
// LoRa send criteria
// -----------------------------------------------------------------------------
#define MIN_DIST                 50.0      // MUST be decimal number; minimum distance in meters from the last sent location before we can send again. A hex is about 340m, divide by this value to get the pings per hex.
#define STATIONARY_TX_INTERVAL   60        // If stationary the LoRa frame will be sent once every N cycles... with 20sec cycle, interval of 60 means to transmit once every 20min

```

With the default cycle time of 30sec means that mapper will transmit location if:

- it has moved at least 50m from previously transmitted coordinates,
- when it was stationary for 60 cycles (or 30 minutes),
- ... or user short-pressed 2nd button, which ignores distance/time requirements



-------------
Ref: https://github.com/helium/longfi-arduino/tree/master/TTGO-TBeam-Tracker

This code was originally developed for use on The Things Network (TTN) it has been editied/repurposed for use with the Helium Network.

This TTGO device application uploads GPS data from the TTGO T-Beam to be used for tracking and determining signal strength of LoRaWAN gateways and nodes. When using the device and application on the Helium Network one can contribute to the [Helium Network](https://www.helium.com) Mapper or Cargo projects. Details for the Mapper project can be found [here](https://mappers.helium.com/) and details for Cargo can be found [here](https://cargo.helium.com/)

Current version: 1.2.1

#### This version is based on a forked repo from github user [kizniche] https://github.com/kizniche/ttgo-tbeam-ttn-tracker. Which in turn is based on the code from [xoseperez/ttgo-beam-tracker](https://github.com/xoseperez/ttgo-beam-tracker), with excerpts from [dermatthias/Lora-TTNMapper-T-Beam](https://github.com/dermatthias/Lora-TTNMapper-T-Beam) to fix an issue with incorrect GPS data being transmitted to the network. Support was also added for the 915 MHz frequency (North and South America). [lewisxhe/TTGO-T-Beam](https://github.com/lewisxhe/TTGO-T-Beam) was referenced for enabling use on the newer T-Beam board (Rev1).

This is a LoRaWAN node based on the [TTGO T-Beam](https://github.com/LilyGO/TTGO-T-Beam) development platform using the SSD1306 I2C OLED display.
It uses a RFM95 by HopeRF and the MCCI LoRaWAN LMIC stack. This sample code is configured to connect to The LoRaWan network using the US 915 MHz frequency by default, but can be changed to EU 868 MHz.

NOTE: There are now 2 versions of the TTGO T-BEAM, the first version (Rev0) and a newer version (Rev1). The GPS module on Rev1 is connected to different pins than Rev0. This code has been successfully tested on REV0, and is in the process of being tested on REV1. See the end of this README for photos of each board.

### Setup

1. Install VisualStudio Code (https://code.visualstudio.com/)

2. Add the PlattformIO extension within VS Code (https://platformio.org/install/ide?install=vscode)

3. Check Device Manager if a serialdevice appear on conntecting the T-Beam. If nothing appears a driver for the USB to serial Adpater need to be installed.(https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

3. Check and edit platformio.ini to use the right bandplan for your region.

4. Edit this project file from Arduino IDE ```main/configuration.h``` under the **Configuration** section and select your correct board revision, either T_BEAM_V07 or T_BEAM_V10 (see [T-BEAM Board Versions](#t-beam-board-versions) to determine which board revision you have).

5. Within your project edit ```main/credentials.h``` to add the device OTAA keys, ```Device EUI, App EUI and App Key```. These can be found within the device configuration within the Helium console. Be sure to pay special attention to the required format when adding these credentials.
* Change to the following
*     // Only one of these settings must be defined
*     //#define USE_ABP
*     #define USE_OTAA
* Then under * #idef USE_OTAA
*     Copy over the APP EUI (LSB), DEV EUI (LSB), APP KEY (MSB) that you copy from the Helium console 

6. Within the Helium Console, add a Mapper or Cargo integration.
- step by step details for setting up a Mapper integration can be found [here](https://docs.helium.com/use-the-network/coverage-mapping/mappers-quickstart/#mappers-quickstart).
- detail for setting up a Cargo integration can be found [here](https://docs.helium.com/use-the-network/console/integrations/cargo).

The specific details for adding a Mapper or Cargo integration use a different edge node device than the one detailed here. When prompted to add a function decoder, be sure to use the following decoder. Note: This decoder can also be found within this project in the console-decoders directory.

```C
function Decoder(bytes, port) {
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) decoded.altitude = 0xFFFF0000 | altValue;
    else decoded.altitude = altValue;

    decoded.hdop = bytes[8] / 10.0;
    decoded.sats = bytes[9];
    decoded.accuracy = bytes[8] / 10.0;
    return decoded;
}
```

7. Open this project file ```main/main.ino``` with the Arduino IDE Verify/Compile the project. If the compile is successful upload the application to your TTGO T-Beam.

8. Disconnect and turn on the device and once a GPS lock is acquired, the device should start sending data to the Helium network and Helium Mapper or Helium Cargo depending upon which you configured in step 6.


### Using the Mapping Data

Now that your device is hopefully connecting to the Helium network refer to the following for more details about interpreting the mapping data.
- For the Helium Mapping effort visit [here](https://docs.helium.com/use-the-network/coverage-mapping)
- For the Helium Cargo effort visit [here](https://docs.helium.com/use-the-network/console/integrations/cargo). Pay particular attention to the "Info" note found on this page.


### T-BEAM Board Versions

#### Rev0

![TTGO T-Beam 01](img/TTGO-TBeam-01.jpg)

![TTGO T-Beam 02](img/TTGO-TBeam-02.jpg)

![TTGO T-Beam 03](img/TTGO-TBeam-03.jpg)

#### Rev1

![T-BEAM-Rev1-01](img/T-BEAM-Rev1-01.jpg)

![T-BEAM-Rev1-02](img/T-BEAM-Rev1-02.jpg)
