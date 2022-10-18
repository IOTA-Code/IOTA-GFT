//======================================================================
//  IOTA-GFT - GPS Exposure TIME LOGGER
//  
//  NOTES: 
//  *** ASSUMES NEO-6 GPS module
//  *** ASSUMES MEGA2560 R3 Arduino
//  *** ASSUMES SD card 
//   
//
//====================================================================== 
//
//

// Mega Pin Connections
//
//  GPS comm
//   Connect the GPS TX (transmit) to Digital pin 19 (RX1)
//   Connect the GPS RX (receive) to Digital pin 18 (TX1)
//   Connect the GPS 1pps Digital pin 49 (ICP4)
//
//	Exposure start signal
//   Connect the Digital pin 48 to the exposure interrupt input (or GND if no exposure int input).  Most cameras will require a pull-up resistor.
//
//  LED
//   Connect LED positive to Digital pin 9
//
//  BUTTON
//   Connect one push button input to GND.
//   Connect second push button input to pin 6.
// 
//  SD card 
//    MISO - pin 50
//    MOSI - pin 51
//    SCLK - pin 52
//    CS   - pin 53
//

//=======================================
//  INCLUDES
//=======================================
#include "iota-gft.h"
#include "ublox.h"
#include "gpsComm.h"
#include "logger.h"
#include "usbComm.h"

//==========================================
// PIN definitions
//===========================================
int BUTTON_PIN = 6;         // Input from Button.  internal Pull up.  button press => low
int PPS_PIN = 49;           // PPS signal input from GPS
int EXP_PIN = 48;           // EXP signal input from camera
int LED_PIN = 9;            // PIN for LED output


//=======================================
//  GLOBAL variables and definitions
//=======================================

volatile OperatingMode DeviceMode;    // current operating mode

volatile FlashingMode FlashMode;		// current flashing mode

short int SYNC_SECONDS = 4;            // # of seconds for syncing to GPS time
volatile short int TimeSync;          // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)

bool fEventDefined  = false;	// true if event defined


//******************
// Timers
//

volatile unsigned short timer4_ov;    // timer 4 overflow count = high word of "time" (32ms)
volatile unsigned short timer5_ov;    // timer 5 overflow count  

const byte OCR2Alevel = 159;  		  			// PWM frequency set; on Timer 2 with prescaler = 1, 99 makes 160kHz, 159 = 100kHz, 199 = 80kHz
byte flashlevel = 100;                    // flash brightness level in percent (0 to 100)

const unsigned long Timer3_us = 64;				// microseconds per led timer clock count (x1024 prescaler)

//*************
//  Time
//
volatile unsigned long Timer_Second = 2000000;                // # of ticks for 1 second
volatile unsigned long PPS_TOLERANCE = 2000;                  // tolerance for PPS interval - 1ms
volatile unsigned long CLOCK_TOLERANCE = 20000;               // tolerance for arduino clock frequency interval - currently 10ms

volatile unsigned long tk_pps_interval_total=0;         // sum of pps intervals
volatile unsigned long tk_pps_interval_count=0;         // # of pps interval
volatile unsigned long tk_pps_interval_ave=0;           // average pps interval

// YY-MM-DD HH:MM:SS time of current second
//   note: all values are less than 255 => don't need to protect individual changes from interrupts during read/write
//         an ISR might use these values, so we should only update the groups of these variables with interrupts OFF
//
volatile bool time_UTC = false;         // true => time is currently UTC , false => time is currently GPS
volatile int sec_Year;                  // 2 digit
volatile int sec_Mon;
volatile int sec_Day;
volatile int sec_hh;
volatile int sec_mm;
volatile int sec_ss;
volatile int offsetUTC_Default = -1;      // receiver default value for GPS-UTC offset
volatile int offsetUTC_Current = -1;      // current/valid GPS-UTC offset


//***************
// PPS event
//
volatile unsigned long tk_PPS_valid;  // time of last VALID PPS int
volatile unsigned long tk_PPS;        // tick "time" of most recent PPS int


volatile boolean pps_led_state = false;       // current state of on-board LED
volatile unsigned long pps_time;
volatile unsigned long pps_count = 0;         // # of pps ints since power on
volatile boolean pps_new = false;
volatile boolean pps_flash = false;           // true IFF LED was flashed with this PPS

//****************************
// Flash variables
//    There are two flashing modes: PPS and EXP.  In both cases, the LED is driven by a 160khz PWM cycle.
//    PPS mode => Flashes are single LED On/Off sequences (via enabing/disabling PWM) where both the start and stop are closely aligned with a PPS signal.
//    EXP mode => Flashes are a series of short duration LED On/Off pulses
//    
//
volatile boolean LED_ON = false;      			// LED state

volatile int PPS_Flash_Countdown_Sec;           // # of seconds remaining in a PPS flash


//***********
//  Button
//
byte buttonState;                                 // current reading
byte lastButtonState = HIGH;
unsigned long debounceDuration = 100; 				    // millis
unsigned long lastTimeButtonStateChanged = 0;


//*********
//  INPUTS
//
volatile bool blnEchoNMEA = false;
volatile bool blnEchoPPS = false;

//********
//  Logging outputs
//
char logNMEA[] = "TTTTTTTT ";         // just the prefix to the rest of the NMEA string
#define len_logNMEA 9

char logPPS[] = "TTTTTTTT P\r\n";
#define len_logPPS 12

char logFlashON[] = "TTTTTTTT +\r\n";
#define len_logFlashON 12

char logFlashOFF[] = "TTTTTTTT -\r\n";
#define len_logFlashOFF 12

unsigned long LastFlush = 0;            // millis() time of last flush

//***********
// Misc
//
int errorCode = 0;
unsigned long errorLong;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  INTERRUPT SERVICE ROUTINES
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//========================================
// ISR for LED done (timer 3 output compare A interrupt)
//=========================================
ISR(TIMER3_COMPA_vect)
{

  // turn OFF LED & decrement count
  //
    // now turn off OC0B
  TCCR0A = (1<< COM0B1);   // normal mode & CLEAR 0C0B on match
  TCCR0B = (1<< FOC0B) | (1<< CS02) | (1<< CS01) | (1<< CS00);     // Force 0C0B low
    // now setup for next incoming 1pps to SET 0C0B
    //
  TCCR0A = (1<< COM0B1) | (1<< COM0B0);   // normal mode & SET 0C0B on match


//  digitalWrite(LED_PIN,false);
  bitClear(TCCR2A, COM2B1);         // disable PWM => turn OFF LED
  LED_ON = false;

  // turn OFF Timer 3
  //
  TCCR3B = (1 << WGM32);    // CTC set => mode 4 AND CS = 0 (no input => clock stopped)
  
} // end of LED_done_interrupt

//========================================
// ISR for Timer 4 overflow
//=========================================
ISR( TIMER4_OVF_vect)
{
  
  timer4_ov++;   // just increment overflow count
  
} // end of Timer4 overflow vector

//=================================================================
// PPS signal ISR
//
// ISR for Timer 4 input capture interrupt 
//==================================================================
ISR( TIMER4_CAPT_vect)
{

  unsigned long timeCurrent;
  unsigned long timePrev;
  unsigned long timeDiff;
  unsigned long ppsDiff;
  bool blnIntervalError;

  unsigned long tk_LED;       // LED time in ticks

 
  //*****************
  //  If init mode or FatalError, just leave...
  //
  if ((DeviceMode == FatalError) || (DeviceMode == InitMode))
  {
    // do nothing in these modes
    return;
  }

  //******************
  // Check for start/end of LED pulse
  //
  if (PPS_Flash_Countdown_Sec == 0)
  {
    // (Countdown == 0)  => flash should be disabled now
    //

    // if LED is ON, turn it OFF
    // else do nothing
    //
    if (LED_ON)
    {
      // no more flashes left
      //
      bitClear(TCCR2A, COM2B1);             // disable PWM - turn OFF LED
      tk_LED = GetTicks(CNT4);              // time LED turned OFF
      LED_ON = false;

      // LOG time LED went OFF
      //
      ultohexA(logFlashOFF,tk_LED);
      LogTextWrite(logFlashOFF,len_logFlashOFF);

    }
  }
  else
  {
    // (Countdown > 0)  => flash should be enabled
    //
    // if (LED OFF)
    //   turn it on
    //
    if (!LED_ON)
    {
      bitSet(TCCR2A,COM2B1);                // enable PWM mode => turn on LED
      tk_LED = GetTicks(CNT4);              // time LED turned ON
      LED_ON = true;

      // log time LED went ON
      //
      ultohexA(logFlashON,tk_LED);
      LogTextWrite(logFlashON,len_logFlashON);

    }

    // one less second left in this pulse
    //
    PPS_Flash_Countdown_Sec--;

  } // end of LED status update

  //************************
  // Get TIME of PPS event from input capture register
  //
  timeCurrent = GetTicks(IC4);

  //************************************
  //  Validate PPS interval
  //    if too long or too short => Error Mode (PPS error)
  //   

  // save the previous value for compare
  //
  timePrev = tk_PPS;
  tk_PPS = timeCurrent;

  // delay from last PPS
  //
  if (timeCurrent > timePrev)
  {
    timeDiff = timeCurrent - timePrev;
  }
  else
  {
    timeDiff = 0 - (timePrev - timeCurrent);
  }
  
  // PPS interval between now and the last PPS
  //   save this to be used by averaging code
  //
  ppsDiff = timeDiff;       

  // check interval since last PPS
  // if Syncing , check delay since last PPS pulse...
  //  if too short or too long, restart SYNC
  // In other modes... just keep going and let the post-processing catch the errors
  //
  blnIntervalError = false;
  if (DeviceMode == Syncing)
  {
    if ((timeDiff < (Timer_Second - CLOCK_TOLERANCE)) || (timeDiff > (Timer_Second + CLOCK_TOLERANCE)))
    {
      // report error?
      //
//*** TBD
      errorCode = 1;
      errorLong = timeDiff;

      // sync error - restart sync
      //
      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;

      DeviceMode = Syncing;
      TimeSync = SYNC_SECONDS;


    }
  }
  else if (DeviceMode == TimeValid) 
  {
    if ((timeDiff < (Timer_Second - PPS_TOLERANCE)) || (timeDiff > (Timer_Second + PPS_TOLERANCE)))
    {
      // report error?
      //
//*** TBD
      errorCode = 2;
      errorLong = timeDiff;

      // sync error - restart sync
      //
      DeviceMode = Syncing;
      TimeSync = SYNC_SECONDS;

      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;
    }
  }

  // ***** PPS validated
  //
  tk_PPS_valid = tk_PPS;

  // log the PPS time
  //
  ultohexA(logPPS,tk_PPS);
  LogTextWrite(logPPS,len_logPPS);


  //********************************
  // DeviceMode logic
  //
  //  *TimeValid
  //    Update internal time values
  //    - check for GPS/UTC switch
  //
  //  *WaitingForGPS Mode
  //    - if GPS data good
  //        move to Syncing mode
  //    - else
  //        leave (keep waiting)
  //
  //  *Syncing Mode
  //    - validate data and restart if necessary
  //
  //
  //
  if ( DeviceMode == TimeValid )
  {
    int ErrorFound;
    
    // TimeValid Mode
    //

    // update PPS interval statistics
    //
    tk_pps_interval_total += ppsDiff;
    tk_pps_interval_count++;

    // check for switch from GPS to UTC time
    //
    if (!time_UTC)
    {
      // time has been GPS based
      //  - do we have a valid almanac now?
      //
      if (gpsPUBX04.valid && gpsPUBX04.blnLeapValid)
      {
        // aha... we now have a current value for of GPS-UTC offset
        //  decrement time by this number of seconds and mark it as UTC time now...
        //
        for( int i = 0; i < gpsPUBX04.usLeapSec; i++)
        {
          SecDec();            
        }

        // now verify that this time matches the values from the previous RMC sentence (which should now be UTC)
        //
        if ((gpsRMC.hh != sec_hh) || (gpsRMC.mm != sec_mm) || (gpsRMC.ss != sec_ss))
        {
          // Opps! the internal UTC time does not match the RMC values... display error and start over again with a new SYNC
          //
          ErrorFound = 6;
          DeviceMode = Syncing;
          TimeSync = SYNC_SECONDS;
    
          tk_pps_interval_total = 0;
          tk_pps_interval_count = 0;
          
          // Error message
          //
          errorCode = 3;
//*** TBD          
          return;
        }

        // set flag to indicate we are now on UTC time
        //
        time_UTC = true;      // UTC now
      }
    }

    // NOW increment the time by one second for this PPS
    //
    SecInc();
    
  }
  else if ( DeviceMode == WaitingForGPS )
  {
    // WaitingForGPS mode
    //
    
    // we are waiting to synchronize
    //  if we have a valid time from the serial data, start the process
    //
    if ((!gpsRMC.valid) || (!gpsPUBX04.valid))
    {
      // no valid RMC or PUBX04, keep waiting
      return;
    }
    else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
    {
      // wait for a good fix
      return;
    }
    else
    {
      // RMC time should correspond to the PREVIOUS second
      //
      sec_ss = gpsRMC.ss;
      sec_mm = gpsRMC.mm;
      sec_hh = gpsRMC.hh;
      SecInc();               // bump the count by one to match the second for THIS PPS signal    
    }

    // GPS data looks good => move to Syncing mode (we will check the next few seconds for consistency)
    //
    DeviceMode = Syncing;
    TimeSync = SYNC_SECONDS;
    tk_pps_interval_total = 0;
    tk_pps_interval_count = 0;
        
  }
  else if ( DeviceMode == Syncing )
  {
    int ErrorFound;
    
    // SYNCING mode
    //
    ErrorFound = 0;
            
    // validation 1 : was last PPS about one second away?  we checked this earlier
    //

    // validation 2: gpsRMC, PUBX04 good?
    //
    if ((!gpsRMC.valid) || (!gpsPUBX04.valid))
    {
      ErrorFound = 1;
    }
    else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
    {
      ErrorFound = 3;
    }
    else
    {
      // compare time of RMC, PUBX04 vs time of PPS
      //
      if (timeCurrent > tk_GPSRMC)
      {
        timeDiff = timeCurrent - tk_GPSRMC;
      }
      else
      {
        timeDiff = 0 - (tk_GPSRMC - timeCurrent);
      }
      
      if (timeDiff > Timer_Second)
      {
        // oops... this RMC was sent MORE than one second befor this PPS
        ErrorFound = 4;    
      }
      else
      {
        if (timeCurrent > tk_PUBX04)
        {
          timeDiff = timeCurrent - tk_PUBX04;
        }
        else
        {
          timeDiff = 0 - (tk_PUBX04 - timeCurrent);
        }
        
        if (timeDiff > Timer_Second)
        {
          // oops... this RMC was sent MORE than one second befor this PPS
          ErrorFound = 5;
        }
        
      } // end of PUBX04 check 

      // ok... both RMC and PUBX04 look good
      
    } // end of checking timing to RMC/PUBX sentence
    
    
    // validation 3 : check the time stamp
    //   we have not yet incremented the time, so it should match the current NMEA value
    //   * note... if UTC offset updates in this timeframe it will break the sync ... but that is OK... it just restarts...
    //
    if (ErrorFound == 0)
    {
      if ((gpsRMC.hh != sec_hh) || (gpsRMC.mm != sec_mm) || (gpsRMC.ss != sec_ss))
      {
        ErrorFound = 2;
      }
      else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
      {
        ErrorFound = 3;
      }
    }
    
    // if error, report it and return
    //
    if (ErrorFound > 0)
    {
      // failed the test - report error and restart sync
      //
      DeviceMode = Syncing;
      TimeSync = SYNC_SECONDS;

      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;
      
      // Error message
      //
      errorCode = ErrorFound + 10;
//*** TBD ****

      // and done with this PPS logic
      return;
    }
    
    // this pps passed the test: bump the time and decrement the count
    //
    SecInc();  
    TimeSync --;

    // update time average
    //
    tk_pps_interval_total += ppsDiff;
    tk_pps_interval_count++;

    // are we all done with the sync?
    //
    if (TimeSync == 0)
    {
      // Set internal time according to most recent PPS and Leap Second status
      //

      // RMC time should correspond to the PREVIOUS second
      //
      sec_ss = gpsRMC.ss;
      sec_mm = gpsRMC.mm;
      sec_hh = gpsRMC.hh;
      SecInc();               // bump the count by one to match the second for THIS PPS signal
      
      // check for leap second status of the time from RMC
      //
      if (gpsPUBX04.blnLeapValid)
      {
        // we have an almanac => UTC time
        //
        time_UTC = true;
      }
      else
      {
        // using default leap seconds... increment time by leap seconds to match GPS time
        //
        time_UTC = false;
        for(int i = 0; i < gpsPUBX04.usLeapSec; i++)
        {
          SecInc();
        }
        
      } // end of check for leap second status

      // update PPS statistics
      //
      tk_pps_interval_ave = tk_pps_interval_total / tk_pps_interval_count;      // compute the average delay between PPS intervals
      Timer_Second = tk_pps_interval_ave;                                       // use this average as the new definition of a second
      PPS_TOLERANCE = Timer_Second / 1000;                                      // reset PPS tolerance to 1ms
      
      tk_pps_interval_total = 0;                                                // reset
      tk_pps_interval_count = 0;

      // We now have a valid time base... 
      //
      DeviceMode = TimeValid;
      
    }  // end of TimeSync = 0 section

  }
  else
  {
    // WHAT???
    //
    DeviceMode = FatalError;    
        
    //  failure message
    //
//*** TBD ***
    
  }  // end of check for current mode

  
} // end of PPS/Timer4 input capture interrupt

//========================================
// ISR for Timer 5 overflow
//=========================================
ISR( TIMER5_OVF_vect)
{
  
  timer5_ov++;   // just increment overflow count
  
} // end of Timer5 overflow vector

//=================================================================
// ISR for Timer 5 input capture interrupt - capture external EVENT input 
//==================================================================
ISR( TIMER5_CAPT_vect)
{
  unsigned long cTime;

  // get current time and save it to the event buffer
  //
  cTime = GetTicks(IC5);

  
} // end of Timer5 input capture interrupt


//++++++++++++++++++++++++++++++++++++++++++++++++++
//
// MISC Utility routines
//
//++++++++++++++++++++++++++++++++++++++++++++++++++

//===========================================================================
// ultohexA - convert unsigned long to 8 hex ASCII characters in a character array
//
//===========================================================================
uint8_t hexA[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46};
void ultohexA(uint8_t *dest, unsigned long ul)
{


  uint8_t *pn;
  unsigned long nibble;

  pn= dest + 7;
  
  for(int i = 0; i < 8; i++)
  {

    // get nibble 
    //
    nibble = (ul & 0xF);
    *pn = hexA[nibble];

    // move to next nibble
    //
    ul = ul >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ultohex

//===========================================================================
// ustohex - convert unsigned short to 4 hex MAX7456 characters in a character array
//
//===========================================================================
void ustohexA(uint8_t *dest, unsigned short us)
{


  uint8_t *pn;
  unsigned short nibble;

  pn= dest + 3;
  
  for(int i = 0; i < 4; i++)
  {

    // get nibble 
    //
    nibble = (us & 0xF);
    *pn = hexA[nibble];

    // move to next nibble
    //
    us = us >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ustohexA



//++++++++++++++++++++++++++++++++++++++++++++
//
// MISC Timing routines
//
//++++++++++++++++++++++++++++++++++++++++++++

//========================================
// GetTicks() - get timer tick count from counter 4 or 5 with check for overflows
//    *** call with interrupts OFF
//=========================================
unsigned long GetTicks(CountSource src)
{
  unsigned long tickCount;
  bool ov;

  // get count and check overflow status
  //
  switch( src ) {
    case CNT4:
      tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)TCNT4;
      ov = TIFR4 & (1 << TOV4);
      break;
      
    case IC4:
      tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)ICR4;
      ov = TIFR4 & (1 << TOV4);
      break;
      
    case CNT5:
      tickCount = ((unsigned long)timer5_ov << 16) + (unsigned long)TCNT5;
      ov = TIFR5 & (1 << TOV5);
      break;
      
    case IC5:
      tickCount = ((unsigned long)timer5_ov << 16) + (unsigned long)ICR5;
      ov = TIFR5 & (1 << TOV5);
      break;
      
    default:
      return 0;     // return a 0
      break;
  }

  // if overflow and count is low, assume we have a pending oveflow interrupt and bump this count by one word
  //
  if (ov & (tickCount < 0xF000))
  {
    tickCount += 0x0010000;
  }

  return( tickCount );

} // end of GetTicks()


//=================================================
//  SecInc - increment second 
//   note: interrupts are disabled while making this change across multiple values
//=================================================
void SecInc()
{
  uint8_t savSREG;

  savSREG = SREG;
  noInterrupts();       // disable all interrupts to protect this operation
  sec_ss++;
  if (sec_ss == 60)
  {
    sec_ss = 0;
    sec_mm++;
    if (sec_mm == 60)
    {
      sec_mm = 0;
      sec_hh++;
      if (sec_hh == 24)
      {
        sec_hh = 0;
      }
    }
  }
  SREG = savSREG;     // restore interrupt status
  
} // end of SecInc

//=================================================
//  SecDec - decrement second 
//   note: interrupts are disabled while making this change across multiple values
//=================================================
void SecDec()
{
  uint8_t savSREG;

  savSREG = SREG;
  noInterrupts();       // disable all interrupts to protect this operation
  sec_ss--;
  if (sec_ss == -1)
  {
    sec_ss = 59;
    sec_mm--;
    if (sec_mm == -1)
    {
      sec_mm = 59;
      sec_hh--;
      if (sec_hh == -1)
      {
        sec_hh = 23;
      }
    }
  }
  SREG = savSREG;     // restore interrupt status
  
} // end of SecDec


//=========================================
//  DateToMJD - convert YYYY,MM,DD to MJD
//    Note: adapted from van Flandern and Pulkkinen http://adsabs.harvard.edu/abs/1979ApJS...41..391V
//=========================================
unsigned long DateToMJD(int yy, int mm, int dd)
{
  unsigned long mjd;

  // Modified Julian Day
  mjd = 367L * yy - 7 * (yy + (mm + 9) / 12) / 4 + 275 * mm / 9 + dd - 678987;

  return(mjd);

} // end of DateToMJD

//=========================================
//  TimeToSecOfDay - convert HH:MM:SS time to SIGNED seconds of day
//=========================================
long TimeToSecOfDay(int hh, int mm, int ss)
{
  long sod;

  sod = hh*3600 + mm*60 + ss;

  return(sod);

} // end of TimeToSecOfDay

//===============================================================
// SETUP
//===============================================================
void setup()  
{    
  unsigned long tNow;
  unsigned long tBegin;
  
  unsigned int sReg;

  int retVal;
  
  //*************************************
  // general Init
  //

  // initializing now...
  //
  DeviceMode = InitMode;
  FlashMode = PPS;

  // set PIN modes
  //
  pinMode(LED_BUILTIN,OUTPUT);          // setup built-in LED 
  pinMode(LED_PIN,OUTPUT);              // setup external LED pin
  pinMode(BUTTON_PIN,INPUT_PULLUP);		  // button input pin
  pinMode(PPS_PIN,INPUT);                    // ICP4 = pin 49 as input
  pinMode(EXP_PIN,INPUT);                    // ICP5 = pin 48 as input

  // connect to PC at 115200 so we send data quickly
  //
  //
  Serial.begin(115200);
  Serial.println("STARTING!");

  // 9600 NMEA is the default rate for the GPS
  //
  Serial1.begin(9600);

  //*********************
  //  Init GPS
  //
  retVal = gpsCommInit();
  if (!gpsCommInit())
  {
    Serial.print("Fatal error initializing GPS.");
  }
 

  //**********************
  //  Init timers
  //	  Timer 2 - PWM for LED
  //    Timer 3 - LED flash duration
  //    Timer 4 - 1pps logging
  //    Timer 5 - exposure marker logging
  //  

  // Timer 2 - PWM for LED (to enable some s/w of control brightness)
  //
  // set flash pwm frequency using Timer 2 in fast PWM mode; when on, TCNT2 continuously counts from 0 to OCR2A
  // output OC2B (pin 9) starts high at TCNT2 = 0, goes low when TCNT2 = OCR2B and resets to high at TCNT2 = OCR2A
  //
  // pwm pulses are switched on/off by setting/clearing the COM2B1 bit of TCCR2A
  //
  TCCR2A = bit(WGM20) | bit(WGM21);  		              // clear OC2B on compare, fast PWM top = OCR2A (with WGM22 below), COM2B1 = 0 => OFF for now
  TCCR2B = bit(WGM22) | bit(CS20);                    // fast pwm, prescaler = 1
  OCR2A = OCR2Alevel;  									              // top level of TCNT2
  OCR2B = map(flashlevel, 0, 100, 0, OCR2Alevel);  		// compare at flashlevel mapped to 0<>OCR2A range; user adjusts this
  
  //  Timer 3 - for LED flash duration in EXP mode only
  //    CTC mode 4
  //    prescaler OFF => timer OFF for now / but will be set to f/1024 for actual timing
  //    OCR3A set to desired duration
  //    PRR1 disabled to allow timer
  //
  sReg = SREG;
  noInterrupts();
  TCCR3A = 0;               // compare output disconnected, WGM bits 1 and 0 set to 0
  TCCR3B = (1 << WGM32);    // CTC3 set => mode 4 AND CS3x = 000 (no input => clock stopped)
  PRR1 &= ~(1 << PRTIM3);   // disable power reduction for timer 3 to enable timer
  SREG = sReg;              // back on again

  //  Timer 4 - used to capture times of 1pps interrrupts
  //    ICP4 pin (digital 49) set to input
  //    Normal mode
  //    Prescaler = f/8
  //    Input Capture = edge detect
  //
  
  TIMSK4 = 0;                             // mask off all interrupts for now
  TCCR4A = 0;                             // all OC ports off and normal mode
  TCCR4B = (1 << ICES4) | (1 << CS41);    // positive edge trigger IC & f/8 prescaler
  PRR1 &= ~(1 << PRTIM4);                 // turn OFF power reduction for timer 4 => turn it ON!
  
  //  Timer 5 - used to capture times of exposure interrrupts
  //    ICP5 pin (digital 48) set to input
  //    Normal mode
  //    Prescaler = f/8
  //    Input Capture = edge detect AND noise canceller ON
  //
     
  TIMSK5 = 0;                             // mask off all interrupts for now
  TCCR5A = 0;                             // all OC ports off and normal mode
  TCCR5B = (1 << ICNC5) | (1 << ICES5) | (1 << CS51);    // noise cancel on, positive edge trigger IC & f/8 prescaler
  PRR1 &= ~(1 << PRTIM5);                 // turn OFF power reduction for timer 5 => turn it ON!

  //  SYNC all synchronous timers via Prescaler reset
  //
  GTCCR = (1 << TSM) | (1 << PSRSYNC);    // STOP prescaler and all syncronous timers

      // PPS interupt enable
  TCNT4 = 0;        // timer4: reset count
  timer4_ov = 0;    // timer4: reser overflow count
  TIFR4 = 0;        // timer4: reset all pending interrupts
  TIMSK4 = (1 << ICIE4) | (1 << TOIE4);   // timer 4: turn on IC capture and overflow interrupts

      // EXP interrupt enable
  TCNT5 = 0;        // timer5: reset count
  timer5_ov = 0;    // timer5: reser overflow count
  TIFR5 = 0;        // timer5: reset all pending interrupts
//***
//*** leave exp disabled for initial device testing, just enable overflow
//  TIMSK5 = (1 << ICIE5) | (1 << TOIE5);   // timer 5: turn on IC capture and overflow interrupts
  TIMSK5 = (1 << TOIE5);

  GTCCR = 0;    // RESTART prescaler and all synchronous timers
  
  //******************
  //  Waiting for GPS
  //  - Wait for PPS interrupts AND NMEA data valid 
  //  - ReadGPS will parse the NMEA data and set valid status for NMEA data
  //  - after NMEA data valid, next PPS ISR will change the device mode to "Syncing" 
  //
  DeviceMode = WaitingForGPS;
  Serial.println("Waiting for GPS...");

  tBegin = millis();
  while( DeviceMode == WaitingForGPS)
  {
    // get pending serial data from GPS
    //
    ReadGPS();

    // wait up to 10 minutes for valid GPS
    //
    tNow = millis();
    if ((tNow - tBegin) > (10 * 60000))
    {
      Serial.println("Timeout waiting for GPS.");
      DeviceMode = FatalError;
      while(1);
    }
  } // end of waiting for GPS 

  //******************
  //  Now wait for Sync to finish
  //  - PPS ISR will change mode to TimeValid after sync period finishes 
  //
  tBegin = millis();
  while(DeviceMode == Syncing)
  {
    // get pending serial data from GPS
    //
    ReadGPS();

    // wait up to 10 minutes for Sync
    //
    tNow = millis();
    if ((tNow - tBegin) > (10 * 60000))
    {
      Serial.println("Sync failed.");
      DeviceMode = FatalError;
      while(1);
    }
  }

  //******************
  //  Now initialize logging
  //
  if (!LogInit())
  {
    Serial.println("Fatal error - stopping!");
    while(1);
  }

  //******************
  //   TESTING - open log file now
  //******************
  if (!LogFileOpen())
  {
    Serial.println("Fatal error - stopping!");
    while(1);
  }

  //*****************
  //  Now start timing...
  //
  Serial.println("Ready for timing...");
  

} // end of setup


//==========================================================================
//  LOOP
//
//  Basic algorithm
//	- check for button press
//  - Keep checking for GPS data until there is none
//  - Then check for incomming commands from PC
//  - Then execute PC commands
//
//============================================================================
void loop()                     // run over and over again
{
  unsigned long now_ms;
  byte buttonReading;

  
  //******************************************************
  // check for Button Press (with debouncing)
  //   => if not flashing, initiate a flash
  //   note: button only works in TimeValid mode
  //
  //******************************************************

  if (DeviceMode == TimeValid)
  {
    buttonReading = digitalRead(BUTTON_PIN);

    // check to see if you just pressed the button
    // (i.e. the input went from HIGH to LOW), and you've waited long enough
    // since the last press to ignore any noise:
    //
    if (buttonReading != lastButtonState)
    {
      // reset the debouncing timer
        lastTimeButtonStateChanged = millis();
    }
    if ((millis() - lastTimeButtonStateChanged) > debounceDuration) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (buttonReading != buttonState) {
        buttonState = buttonReading;

        if (buttonState == LOW)
        {
 
          // debug...
          Serial.println("BUTTON PRESSED!");

          // button PRESSED!
          //  start flashing at next PPS
          //
          // if LED already ON, extend the time...
          // else
          //   setup one duration
          //
          if (LED_ON)
          {
            PPS_Flash_Countdown_Sec += PPS_Flash_Duration_Sec;
          }
          else
          {
            PPS_Flash_Countdown_Sec = PPS_Flash_Duration_Sec;
          }
        }
      }
    }
    lastButtonState = buttonReading;    // save the current reading for next time in the loop
    
  }  // end of checking button while TimeValid

  //******************************************************
  //  check for pending data from GPS
  //
  ReadGPS();

  //******************************************************
  // Output logging info
  //
  now_ms = millis();
  LogFlushFull();         // flush any full buffers to the file...

  // in PPS mode, update disk every second...
  //
  if ((now_ms - LastFlush) > 2000)
  {
    LastFlush = now_ms;
    LogFlushToFile();       // update the file with the current data

    if (DeviceMode == TimeValid)
    {
      Serial.print("DeviceMode = TimeValid : ");
      Serial.println(sec_ss);
      
    }
    else if (DeviceMode == Syncing)
    {
      Serial.println("DeviceMode == Syncing");
    }
    else if (DeviceMode == WaitingForGPS)
    {
      Serial.println("DeviceMode == WaitingForGPS");
    }
    else if (DeviceMode == FatalError)
    {
      Serial.println("DeviceMode == FatalError");
    }
    else if (DeviceMode == InitMode)
    {
      Serial.println("DeviceMode == InitMode");
    }
    else
    {
      Serial.println("unknown mode!");
    }
    
 } 

  //***********************
  //  now check for an incomming command from USB port
  //
  ReadCMD();
 

} // end of loop()


