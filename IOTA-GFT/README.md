# IOTA-GFT Device
----------------------------
The IOTA-GFT device connects to the user's computer via USB and drives an LED which is attached to the front of the telescope.  The user's computer initiates sequences of LED "flashes" which illuminate a set of frames on recorded video sequences.  The IOTA-GFT device employs an internal GPS receiver to record the timing of the flash sequences and emit a log with timing information to the user's computer.  After video recording, this timing log is compared to the recorded video to determine the UTC times for each frame of the video recording.

The IOTA-GFT device hardware includes the following components:
- IOTA-GFT controller, USB cable, enclosure and USB cable
- Active GPS antenna
- LED bracket and 3.5mm pin cable

This repository holds the Arduino firmware for the IOTA-GFT controller.  The IOTA-GFT controller combines an Arduino Mega 2560 with the IOTA-GFT shield.

## Building the IOTA-Flasher software
-------------------------------------
Prerequisites:
- Arduino IDE


## Licensing
------------
 This program is free software: you can redistribute it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.  However, if you wish to make modifications, we add a further restriction.  If you are going to use this device to measure times for occultation observations, you must agree to not modify the code from this repository.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.



## Developers & Contributors
----------------------------
- Bob Anderson
- Steve Preston


