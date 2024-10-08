# IOTA-GFT Shield
----------------------------
The IOTA-GFT Arduino shield includes the GPS receiver and driver circuit for the LED.  This shield is designed for and requires an Arduino Mega 2560 R3 board.  The IOTA-GFT controller consists of this shield mounted on an Arduino Mega.  This repository carries the Eagle design files for the shield.

## Components
--------------
- IOTA-GFT shield PCB
- Female stacking headers for Arduino Mega ( https://a.co/d/8ICgdAu )
- GT-U7 GPS breakout board based on ublox GPS receiver (https://a.co/d/9eoqRfC) 
- Active GPS Antenna + pigtail ( https://a.co/d/hI4Iglh )
- SparkFun Voltage-Level Translator Breakout - TXB0104 ( https://www.sparkfun.com/products/11771 )
- (2) seven pin male headers - 0.1" (can break off from longer header : https://a.co/d/7kpX5g9 )
- (1) six pin male header - 0.1"
- (2) JST-XH PCB jack , CONN HEADER VERT 2POS 2.5MM, JST part# B2B-XH-A ( https://www.digikey.com/short/nwvmtq8r )
- Resistors (1/4w)
  - (1) 150 ohm
  - (1) 1k ohm
  - (1) 5.6k ohm
  - (1) 200k ohm
- Capacitors (50v)
  - (1) 30 pf
- Transistors
  - (1) 2N3906
- Plastic zip tie (4")

## Schematic and Board layout
------------
![schematic](./IOTA-GFT-Shield-a-schematic.png)
![board layout](./IOTA-GFT-Shield-a-board.png)

## Assembly
------------
- Solder the Arduino shield stacking headers onto the PCB.
- Solder the two 7 pin male headers onto the PCB at the location of U2 (level shifter)
- Solder the one 6 pin male header onto the PCB at the location of the pins for the GT-U7 breakout board.
- Solder the resistors, capacitor, and transitor onto the PCB at the marked locations.
- Solder the two JST connectors onto the PCB with the open side facing away from the Arduino's USB and power connectors.
- Solder the TXB0104 breakout board onto the two male headers with the OE marked end facing the two JST connectors.
- Attach the GPS antenna pigtail to the antenna connector on the GT-U7 breakout board.
- Solder the GT-U7 GPS breakout board to the single six pin male header.
- Snug down the GPS antenna pigtail to the breakout board using the plastic zip tie.

## Licensing
------------
 This design is free: you can redistribute it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.  However, if you wish to make modifications, we add a further restriction.  If you are going to use this design to measure times for occultation observations, you must agree to not modify the design.

 This design is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.


## Developers & Contributors
----------------------------
- Bob Anderson
- Steve Preston
- Aart Olsen

