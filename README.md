<!-- ## TTGO T-Beam Tracker for The Things Network and/or The Helium Network  -->
## TTGO T-Beam Tracker for use with a LoRaWan Network (Helium)

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
Ref: https://github.com/helium/longfi-arduino/tree/master/TTGO-TBeam-Tracker

This code was originally developed for use on The Things Network (TTN) it has been editied/repurposed for use with the Helium Network.

This TTGO device application uploads GPS data from the TTGO T-Beam to be used for tracking and determining signal strength of LoRaWAN gateways and nodes. When using the device and application on the Helium Network one can contribute to the [Helium Network](https://www.helium.com) Mapper or Cargo projects. Details for the Mapper project can be found [here](https://mappers.helium.com/) and details for Cargo can be found [here](https://cargo.helium.com/)

Current version: 1.2.1

#### This version is based on a forked repo from github user [kizniche] https://github.com/kizniche/ttgo-tbeam-ttn-tracker. Which in turn is based on the code from [xoseperez/ttgo-beam-tracker](https://github.com/xoseperez/ttgo-beam-tracker), with excerpts from [dermatthias/Lora-TTNMapper-T-Beam](https://github.com/dermatthias/Lora-TTNMapper-T-Beam) to fix an issue with incorrect GPS data being transmitted to the network. Support was also added for the 915 MHz frequency (North and South America). [lewisxhe/TTGO-T-Beam](https://github.com/lewisxhe/TTGO-T-Beam) was referenced for enabling use on the newer T-Beam board (Rev1).

This is a LoRaWAN node based on the [TTGO T-Beam](https://github.com/LilyGO/TTGO-T-Beam) development platform using the SSD1306 I2C OLED display.
It uses a RFM95 by HopeRF and the MCCI LoRaWAN LMIC stack. This sample code is configured to connect to The LoRaWan network using the US 915 MHz frequency by default, but can be changed to EU 868 MHz.

NOTE: There are now 2 versions of the TTGO T-BEAM, the first version (Rev0) and a newer version (Rev1). The GPS module on Rev1 is connected to different pins than Rev0. This code has been successfully tested on REV0, and is in the process of being tested on REV1. See the end of this README for photos of each board.

### Setup

1. Follow the directions at [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) to install the board to the Arduino IDE and use board 'T-Beam'.  
In summary, within the Arduino IDE open Boards Manager from Tools > Board menu and search for and install the "esp32" platform (and don't forget to select your ESP32 board from Tools > Board menu after installation).

2. Install the following Arduino IDE libraries: 
***You find Libraries under the Arduino folder*** 
In my case documents\arduino\libraries
To run the below commnads you will need to download and install Git from https://git-scm.com/downloads
If you use the "git clone" command below and using the Git Bash App then ensure you are in the librarires folder by using cd documents/arduino/libraries
I downloaded the zip files from the links and used the Arduino IDE librariess manager to install the library

   1. [mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic) (for Rev0 and Rev1)  
    Executing ```git clone https://github.com/mcci-catena/arduino-lmic.git``` within the Arduino project's libraries directory should install the proper and latest version of the library.
    
   2. [ThingPulse/esp8266-oled-ssd1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) (for Rev0 and Rev1)  
    Executing ```git clone https://github.com/ThingPulse/esp8266-oled-ssd1306.git``` within the Arduino project's libraries directory should install proper and the latest version of the library.
    
   3. [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) (for Rev0 and Rev1)  
    Executing ```git clone https://github.com/mikalhart/TinyGPSPlus.git``` within the Arduino project's libraries directory should install the proper and latest version of the library.
   4. [lewisxhe/AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library) (or Rev0 and Rev1)  
    Executing ```git clone https://github.com/lewisxhe/AXP202X_Library.git``` within the Arduino project's libraries directory should install the proper and latest version of the library.



3. Edit from the libraries folder on your computer ```arduino-lmic-master/project_config/lmic_project_config.h``` and uncomment the proper frequency for your region.

4. Edit this project file from Arduino IDE ```main/configuration.h``` under the **Configuration** section and select your correct board revision, either T_BEAM_V07 or T_BEAM_V10 (see [T-BEAM Board Versions](#t-beam-board-versions) to determine which board revision you have).

5. Within your project edit ```main/credentials.h``` to add the device OTAA keys, ```Device EUI, App EUI and App Key```. These can be found within the device configuration within the Helium console. Be sure to pay special attention to the required format when adding these credentials.
* Change to the following
*  // Only one of these settings must be defined
*  //#define USE_ABP
*  #define USE_OTAA
* Then under * #idef USE_OTAA
*   Copy over the APP EUI, DEV EUI, APP KEY that you copy from the Helium console 

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
