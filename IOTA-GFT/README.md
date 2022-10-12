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
- TinyGPS+ library <https://github.com/mikalhart/TinyGPSPlus>


## Building the IOTA-Flasher hardware
--------------------------------------
The hardware:
- Arduino Mega 2560
- GPS Module based on U-blox receiver:
    <https://www.amazon.com/Gowoops-Module-Antenna-Arduino-Microcomputer/dp/B01MRNN3YZ>
- LED attached to a cable 
- current limiting resitor for LED
- trimmer pot to match current limiting resistor for LED
- USB cable to connect the Flasher to a computer
- enclosure
- optional : cable to connect Exposure start signal from camera

The source file includes notes on how these components should be interconnected.

## Basic Usage
----------------------------------------
The basic role of the flasher box is still to provide timed "goalpost" flashes on either side of an occultation.  For simple use it is unaware of the camera and just makes flashes at the appropriate times.  For cameras that output a shutter/strobe/frame start signal, the flasher logs the times of these and makes the goalpost flashes coincide with the frame instead of the PPS.  This allows the registration of images with their respective times in the flasher log file.

User interaction is via USB and a serial terminal program.  The Arduino Serial Monitor works directly but you have to copy the output log to save it.  PuTTY is a pretty good alternative after you make a few adjustments to the default PuTTY profile and save it so it can be launched directly from a desktop icon.  The whole PuTTY session -- user input as well as output data -- is logged automatically to a date-stamped csv file.  

The flasher may also be operated autonomously, without a laptop.  When used this way it won't log times but because the flash times are known beforehand (and confirmed in the video).

When operated with a laptop both the startup factory default GPS-UTC leap seconds and current, correct difference are reported, so the user sees if/when this changes.

When used with a camera frame start signal output, the time logging works reliably up to 300 frames/second, slowing to about 250 frames/sec if geolocation (lat, long, altitude) is displayed.

Flash intensity should be adjusted to avoid saturation of the imaging pixels.  The IOTA-Flasher adjusts flash intensity via software control with PWM.  We also recommend you include a trimmer pot to provide some "coarse" adjustment of the flash intensity.

The flasher times are based on PPS and the micros() system variable count since the previous PPS, but depending on the brightness level the first pulse of a flash pulse train may start anywhere within the 10 microsecond period of the 100kHz PWM frequency, so the precision is no better than 10 microseconds.  The micros() period is continuously calibrated against PPS so the timer free-runs quite accurately if the GPS signal is lost.

The flasher works the same way for either PPS timed flashes or camera shutter signals -- basically you just say when you would like your goalpost / registration flashes to happen.

The sketch is started in one of three ways:
1. Data may be entered beforehand and used subsequently, i.e. set up at home and later used on-site.  Power is not needed to retain the data.  On-station, if the user does nothing within 15 seconds after power-up the flasher will automatically start its countdown to the stored prediction time.  The flasher is thus self-starting and in this mode it can be used in a standalone manner.  Of course the user can't adjust the LED, etc. unless a laptop is plugged in.

2. The user may enter a new prediction.  After the receiver gets live GPS data and if the OW prediction numbers are handy this should take less than a minute.  Entered values are automatically stored so if the flasher needs to be restarted it won't be necessary to re-enter the prediction again.

3. Or the user may ignore any prediction data and simply make flashes and timestamps on demand via the laptop, or simply rely on the automated zero-second flashes (these may be turned on/off as needed).

When on station, during the countdown, the user may do the following at any time, all from the laptop:
- make a manual flash
- adjust the LED brightness
- save the brightness setting to the Arduino storage for subsequent use
- toggle LED blink on/off at 1/sec
- toggle the LED continuously on/off
- toggle zero-second flashes on/off
- toggle geolocation (latitude, longitude, altitude) on/off
All of these operations are timestamped in the output log.


## Developers & Contributors
----------------------------
- Aart Olsen


