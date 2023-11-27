# IOTA-Flasher
----------------------------
The IOTA-Flasher project implements GPS based timing for cameras that generate image sequences (e.g. video).  The IOTA-Flasher device is an Arduino Mega 2560 microcontroller along with a GPS receiver.  This repository holds the Arduino source code.

## Licensing
------------
 This program is free software: you can redistribute it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.  However, if you wish to make modifications, we add a further restriction.  If you are going to use this device to measure times for occultation observations, you may only modify the files which contain only text strings.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.


## Building the IOTA-Flasher software
-------------------------------------
Prerequisites:
- Arduino IDE


## Building the IOTA-Flasher hardware
--------------------------------------
The hardware:
- Arduino Mega 2560
- GPS Module based on U-blox receiver:
    <https://www.amazon.com/Gowoops-Module-Antenna-Arduino-Microcomputer/dp/B01MRNN3YZ>
- LED attached to a cable 
- current limiting resitor for LED
- USB cable to connect the Flasher to a computer
- enclosure


The source file includes notes on how these components should be interconnected.

## Basic Usage
----------------------------------------
The basic role of the flasher box is still to provide timed "goalpost" flashes on either side of an occultation.  For simple use it is unaware of the camera and just makes flashes at the appropriate times.  

User interaction is via USB and a serial terminal program.  The Arduino Serial Monitor works directly but you have to copy the output log to save it.  PuTTY is a pretty good alternative after you make a few adjustments to the default PuTTY profile and save it so it can be launched directly from a desktop icon.  The whole PuTTY session -- user input as well as output data -- is logged automatically to a date-stamped csv file.  

Flash intensity should be adjusted to avoid saturation of the imaging pixels.  The IOTA-Flasher adjusts flash intensity via software control with PWM.  


## Developers & Contributors
----------------------------
- Aart Olsen
- Bob Anderson
- Steve Preston


