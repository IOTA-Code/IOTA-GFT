# IOTA-GFT Project
----------------------------
The IOTA-GFT project (IOTA GPS Flash Timer) implements a GPS based solution for adding UTC timestamps to sequential image captures (e.g. video).  The project includes both an IOTA-GFT hardware device and various software components for the computer used to capture the video data.

The IOTA-GFT device connects to the video recording computer and drives an LED which is attached to the front of the optics.  The IOTA-GFT software on the recording computer controls the LED flash sequence from the device.  The recording computer software also logs the timing information from the GPS receiver in the IOTA-GFT device.  After recording a video sequence, IOTA-GFT software analyzes the video and timing log to determine the times for each video frame.

This repository includes the following components:
- IOTA-GFT Device
    - Device Firmware
    - IOTA-GFT Shield PCB design
    - LED bracket components
- Recording computer software
    - ASCOM Driver for the IOTA-GFT device
    - FileStamper utility for inserting timestamps
- Documentation
    - Usage Guide(s)

## Licensing
------------
 This program is free software: you can redistribute it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.  However, if you wish to make modifications, we add a further restriction.  If you make changes to the code from this repository, you must agree to NOT use the modified code for timing occultation observations.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.



## Developers & Contributors
----------------------------

- Bob Anderson
- Steve Preston


