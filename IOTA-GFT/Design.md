# IOTA-GFT Design Document
This design document provides an overview of the basic theory of operation for the IOTA-GFT device.  
## Components
The IOTA-GFT comprises the following components:
- PIC microprocessor (Arduino Mega 2560)
- GPS receiver (ublox)
- Serial I/O to an external computer via USB
- LED driver circuit for driving an external LED
- Optional input from an external 5V logic trigger signal
## General Overview
The IOTA-GFT collects timing information from the PIC microprocessor, the GPS receiver, and (optionally) the external trigger input.  This timing information, or timing log, is sent across the USB connection to the recording computer where it is stored and later used to determine the UTC time of various system events.  Each line of output in this timing log, from the IOTA-GFT device, includes a timestamp and a data content portion.  The log timestamps are 32 bit counts (in hex) from the 16mhz system timer of the PIC processor. The IOTA-GFT emits logging sentences with the system time for each of the following system events:
- GPS data
    - NMEA sentences via serial I/O comm with the GPS receiver
    - PPS via 1pps signal generated by the GPS receiver
- LED activity
    - LED turns ON
    - LED turns OFF
- External input trigger (EXP signal)
- USB Commands
    - USB Commands sent to the device
    - Device responses to USB Commands 
## Operating Modes
The IOTA-GFT firmware implements a simple state machine with five "operating modes": InitMode, WaitingForGPS, Syncing, TimeValid, and FatalError.  The device boots into InitMode and quickly transitions to WaitingForGPS.  In WaitingForGPS mode, the code monitors the NMEA data and PPS signal waiting until both of these inputs imply that the GPS data is deemed "valid" for timing purposes.  At this point, the device moves to "Syncing" mode where it monitors several seconds of data to confirm that the GPS data is stable.  Assuming the GPS data looks good for several seconds of Syncing mode, the device moves to TimeValid mode and is ready for operation.  If transient errors occur (e.g. bad NMEA data or missing PPS pulses), the device will drop back to WaitingForGPS mode to "restart" toward TimeValid mode.  The FatalError mode only occurs in case of a non-recoverable error (e.g. unable to initialize the GPS receiver).State changes, between the five operating modes, are primarily driven by the GPS receiver - both the NMEA data and the PPS interrupt.

The IOTA-GFT firmware follows the standard Arduino coding structure: a one-time setup() routine and a loop() routine which is exectued repeatedly.  The setup() routing initializes the GPS receiver, the Arduino timer/counters, the Arduino I/O ports and various other elements.  The loop() routine repeatedly checks for input from the GPS receiver and the USB serial from the recording computer.  In addition, the loop() routine regularly outputs pending timing log data to the recording computer. 

## Timers
IOTA-GFT uses four of the Arduino timer/counters: Timer 0, Timer 2, Timer 3, Timer 4 and Timer 5.

Timer 0 is the default Arduino timer for the delay() and millis() functions.  These functions are used in the gps communication code.

Timer 2 is used in PWM mode to implement a software based intensity control for the LED.

Timer 3 is used for LED pulse duration in certain flash modes.

Timer 4 and Timer 5 are synchronized system timer counters running at 16 mhz.  The Input Capture signal for Timer 4 is tied to the PPS input. The Input Capture signal for Timer 5 is tied to the EXP signal input.  The firmware uses the overflow interupts to implement 32 bit system timers based on Timer 4 and 5.  Because these timers are synchronized, both system time counts are always identical.  The input capture register assures that the PPS and EXP interupt times are highly accurate (no latency to the ISR routine).

## Logging buffer
All information which is output to the recording computer (via USB serial) is first placed in an internal buffer.  The buffer is output to the recording computer during each pass through the main loop() routine.  Most of the logging sentences will be written to the buffer without interruption.  However, logging based on PPS and EXP interrupts might "interrupt" another sentence in the log.

## Flash Operation
The IOTA-GFT is designed to support two flash modes: PPS and EXP.  The majority of users will use PPS flashes.  The EXP mode is currently in development.  It is aimed at cameras which provide an output signal marking the beginning of an exposure.  

IOTA-GFT operates in PPS flash mode by default.  Flash sequences are specified as X seconds long.

In either flash mode, the device can be programmed to a wide range of flash intensity values.

In addition, there is an option to turn the LED ON and OFF via explicit commands from the recording computer (over the USB port).

## LED Driver Circuit
The LED intensity is controlled via two appoaches: a LED current control "driver" circuit and a PWM modulation of the power driving this LED driver circuit.  The LED driver circuit is documented in the schematic for the PCB.

## USB I/O Device Commands
The recording computer sends commands to the IOTA-GFT over the USB connection. Commands are terminated with a LF (linefeed) character ('\n').  The commands may optionally include an XOR checksum in the form of "*XX" where XX are two ASCII hex chars for the one byte checksum. Commands are NOT case sensitive.

IOTA-GFT will return two sentences to all commands.  The first sentence will be an echo of the command sent to the device.  The second sentence will be a response to the command and include a simple one byte XOR checksum (two ASCII hex chars) at the end, followed by CR,LF ('\r\n'). Successful responses will be either the sentence "[DONE]" or "[command: value]".  Error responses will be of the form "[ERROR ...".  

### general commands
    status - get current device mode (e.g. TimeValid, WaitingForGPS, Syncing, etc)
    device - get device name
    version - get version info for the device
    null - ignore this command string if it includes the word "null" - useful for clearing the input buffer

 ### flash commands
    flash now - start flash sequence at the next PPS or EXP pulse
    flash duration X - X is seconds in PPS mode or starting number of pulses in EXP mode
    flash level X - get/set the current flash intensity level [ 0 to 255]
    flash range [X] -get/set the current range of flash intensity to low, medium or high [ 0 to 2]
    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)

    pulse duration [X] - get/set the flash pulse duration (us)
    pulse interval [X] - get/set the flash pulse interval = # of exp interrupts between pulse sequences

    led [ON | OFF] - turn the LED on/off right now

 ### logging commands
    log [on | off ] - enable disable data logging (default = ON)

## Timing Log Format
After startup, the IOTA-GFT constantly emits logging information - a timing log.  The timing log includes all NMEA data from the GPS receiver, all PPS and EXP interrupts, all LED flash timing, all commands sent to the device, all responses to these commands, and a regular report on the current operating mode of the device.  Each of these outputs is in the form of a single line of ASCII characters terminated with a *XX checksum and CRLF characters.

### Operating mode sentence
IOTA-GFT will output an operating mode sentence at the start of each set of NMEA sentences (which occur at the beginning of each UTC second).  

The format (excluding the terminating CRLF):

{MODE mmm fff}*XX
- mmm = operating mode 
- fff = current flash mode (only when operating mode = TimeValid)
- XX = hex checksum in ASCII

examples: 
- {MODE Sync}*02
- {MODE TimeValid PPS}*35

At power-up, IOTA-GFT emits the following sentence to the logging stream:

[STARTING!]*27

### GPS NMEA sentences
IOTA-GFT outputs a log sentence for each of the following NMEA sentences: $GPDTM, $GPRMC, $GPGGA, $PUBX04 (proprietary ublox sentence).  

The format (excluding terminating CRLF):

{TTTTTTTT nnn}*XX
- TTTTTTTT = 8 digit Hex system timer count when NMEA data received
- nnn = NMEA sentence data (without terminating CR or LF)
- XX - hex checksum in ASCII

examples:
- {006F03C0 $GPDTM,W84,,0.0,N,0.0,E,0.0,W84*6F}*37
- {00702831 $GPRMC,211252.00,A,4738.34269,N,12214.22164,W,0.114,,- 140324,,,A*64}*41
- {00725188 $GPGGA,211252.00,4738.34269,N,12214.22164,W,1,08,1.14,32.3,M,-18.7,M,,*51}*7C
- {0074B4E8 $PUBX,04,211252.00,140324,421972.00,2305,18,-305489,101.109,21*3D}*6A

### Interrupt timing sentences
An interrupt timing sentence is logged at the time of PPS and EXP interrupts.  These interrupt sentences are injected into the logging buffer at "interrupt level" and may sometimes appear "inside" other sentences (interrupting another logging sentence).

The format (excluding terminating CRLF):

{TTTTTTTT P}*XX
- TTTTTTTT = 8 digit Hex system timer count at time of interrupt
- P => this is a PPS interrupt
- XX - hex checksum in ASCII

{TTTTTTTT E}*XX
- TTTTTTTT = 8 digit Hex system timer count at time of interrupt
- E => this is an EXP interrupt
- XX - hex checksum in ASCII

Example:
- {1550D7B3 P}*75


### Flash timing sentences
A flash timing sentence is logged whenever the LED changes state (from OFF to ON or from ON to OFF).

The format (excluding terminating CRLF):

{TTTTTTTT +}*XX
- TTTTTTTT = 8 digit Hex system timer count when flash changed state
- '+' => LED turned ON
- XX - hex checksum in ASCII

{TTTTTTTT !}*XX
- TTTTTTTT = 8 digit Hex system timer count when flash changed state
- ! => LED turned OFF at the end of a flash sequence
- XX - hex checksum in ASCII

{TTTTTTTT -}*XX
- TTTTTTTT = 8 digit Hex system timer count when flash changed state
- '-' => LED turned OFF as part of a multi-flash sequence
- XX - hex checksum in ASCII

Example:
- {1550D7CB +}*7E

### Commands/Responses
Commands sent to the device will be echoed to the logging stream and enclosed in square brackets ('[', ']') and starting with the phrase CMD and a terminating checksum.  Responses to these commands are also returned in square backets and include a checksum.

example:
- [CMD flash now]*4A
- [DONE]*06
