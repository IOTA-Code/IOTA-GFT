//======================================================================
//  IOTA-GFT - GPS Exposure TIME LOGGER
//  
//  NOTES: 
//  *** ASSUMES NEO-6 GPS module
//  *** ASSUMES MEGA2560 R3 Arduino
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
//   Connect Digital pin 9, Digital pin 10, and Analog pin 1 to the LED driver circuit
//   
//
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
int PPS_PIN = 49;           // PPS signal input from GPS
int EXP_PIN = 48;           // EXP signal input from camera

  // LED circuit pins
#define flashPinB 9        // This PWM pin is controlled by OCR2B
#define flashPinA 10       // This PWM pin is controlled by OCR2A

#define midRange  55       // Controls switch transistor that provides middle current for LED (D55 = Analog 1)

//=======================================
//  GLOBAL variables and definitions
//=======================================

//  VERSION
//
const char *strDeviceName = "IOTA-GFT";
const char *strVersion = "v2024-03-10-2";

volatile OperatingMode DeviceMode;    // current operating mode
volatile bool blnReportMode;          // true => report current mode in log (enabled with each NMEA set)

unsigned long tBeginWait;
bool fStarted;                        // set to true when we first enter TimeValid mode


volatile FlashingMode FlashMode;		// current flashing mode

short int SYNC_SECONDS = 4;            // # of seconds for syncing to GPS time
volatile short int TimeSync;          // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)



//******************
// Timers
//

volatile unsigned short timer4_ov;    // timer 4 overflow count = high word of "time" (32ms)
volatile unsigned short timer5_ov;    // timer 5 overflow count  

byte flashlevel = 255;                    // flash brightness level in percent (0 to 255)
byte flashrange = 2;                      //  range of flash intensity ( 0 - 2 )

uint16_t Timer3_us = 16;				          // microseconds per led timer clock count (x256 prescaler)
uint16_t OCR3A_pulse = 5000/Timer3_us;    // default 5ms pulse duration for EXP mode

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
volatile unsigned long tk_PPS;        // tick "time" of most recent PPS int


volatile boolean pps_led_state = false;       // current state of on-board LED
volatile unsigned long pps_time;
volatile unsigned long pps_count = 0;         // # of pps ints since power on
volatile boolean pps_new = false;
volatile boolean pps_flash = false;           // true IFF LED was flashed with this PPS

//****************************
// Flash variables
//    There are two flashing modes: PPS and EXP.  In both cases, the LED is driven by a PWM cycle.
//    PPS mode => Flashes are single LED On/Off sequences (via enabing/disabling PWM) where both the start and stop are closely aligned with a PPS signal.
//    EXP mode => Flashes are a series of short duration LED On/Off pulses
//    
//
volatile boolean LED_ON = false;      			// LED state

volatile int PPS_Flash_Countdown_Sec;           // # of seconds remaining in a PPS flash
                                                //  (countdown < 0) => not in PPS flash sequence

                                            // EXP flash sequences are short pulses separated by D EXP intervals for a total count of X pulses
                                            //      sequence is enabled when pulse_count is non-zero
                                            //
int pulse_duration_us = 5000;               // duration of one shutter (EXP) pulse in EXP mode
volatile int pulse_interval = 5;            // number of EXP events between pulses
volatile uint16_t pulse_countdown;          // # of pulses left in sequence, 0 => no EXP flash sequence in progress
volatile uint16_t pulse_interval_countdown; // # of EXP events until next pulse, 0=> flash (pulse) on next EXP event



//*********
//  INPUTS
//
volatile bool blnEchoNMEA = false;


//********
//  serial outputs
//

char logPPS[] = "{TTTTTTTT P}*XX\r\n";
int len_logPPS = 17;
int offset_logPPS = 1;
int chksum_logPPS = 13;

char logEXP[] = "{TTTTTTTT E}*XX\r\n";
int len_logEXP = 17;
int offset_logEXP = 1;
int chksum_logEXP = 13;

char logFlashON[] = "{TTTTTTTT +}*XX\r\n";
int len_logFlashON = 17;
int offset_logFlashON = 1;
int chksum_logFlashON = 13;

char logFlashOFF[] = "{TTTTTTTT -}*XX\r\n";
int len_logFlashOFF = 17;
int offset_logFlashOFF = 1;
int chksum_logFlashOFF = 13;

char logFlashFINAL[] = "{TTTTTTTT !}*XX\r\n";    // end of final flash pulse in sequence
int len_logFlashFINAL = 17;
int offset_logFlashFINAL = 1;
int chksum_logFlashFINAL = 13;

char logModeInit[] = "{MODE Init}*1F\r\n";
int len_logModeInit = 16;
int chksum_logModeInit = 12;

char logModeWaitingForGPS[] = "{MODE WaitingForGPS}*71\r\n";
int len_logModeWaitingForGPS = 25;
int chksum_logModeWaitingForGPS = 21;

char logModeSync[] = "{MODE Sync}*02\r\n";
int len_logModeSync = 16;
int chksum_logModeSync = 12;

char logModeTimeValid[] = "{MODE TimeValid FFF}*XX\r\n";
int len_logModeTimeValid = 25;
int fmode_logModeTimeValid = 16;
int chksum_logModeTimeValid = 21;

char logModeFatal[] = "{MODE Fatal}*7B\r\n";
int len_logModeFatal = 17;
int chksum_logModeFatal = 13;

char logERROR[] = "{ERROR 0000}*XX\r\n";
int len_logERROR = 17;
int offset_logERROR = 7;
int chksum_logERROR = 13;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  INTERRUPT SERVICE ROUTINES
//
//    Timer 3
//      output compare for LED pulse duration
//    Timer 4
//      input capture for PPS logging
//      overflow for tracking clock count
//    Timer 5
//      input capture for EXP logging
//      overflow for tracking clock count
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//========================================
// ISR for LED done (timer 3 output compare A interrupt)
//=========================================
ISR(TIMER3_COMPA_vect)
{
  unsigned long tk_LED;
  byte chk;

  // turn OFF LED 
  //
  OCR2A = OCR2B = 0;
  LED_ON = false;

  // turn OFF Timer 3
  //
  TCCR3B = (1 << WGM32);    // CTC set => mode 4 AND CS = 0 (no input => clock stopped)

  // log time flash went off
  //
  tk_LED = GetTicks(CNT4);              // time LED turned OFF

  if (pulse_countdown == 0)
  {
    ultohexA(logFlashFINAL + offset_logFlashFINAL,tk_LED);
    chk = chksum_b(logFlashFINAL,chksum_logFlashFINAL-1);   // compute checksum
    btohexA(logFlashFINAL + chksum_logFlashFINAL, chk);
    LogTextWrite(logFlashFINAL,len_logFlashFINAL);
  }
  else
  {
    ultohexA(logFlashOFF + offset_logFlashOFF,tk_LED);
    chk = chksum_b(logFlashOFF,chksum_logFlashOFF-1);   // compute checksum
    btohexA(logFlashOFF + chksum_logFlashOFF, chk);
    LogTextWrite(logFlashOFF,len_logFlashOFF);
  }
  
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

  unsigned long tk_LED;       // LED time in ticks
  bool blnLogFlashON = false;
  bool blnLogFlashOFF = false;
  byte chk;

  
  //************************
  // Get TIME of PPS event from input capture register
  //
  timeCurrent = GetTicks(IC4);

  //*****************
  //  If init mode or FatalError, just leave...
  //
  if ((DeviceMode == FatalError) || (DeviceMode == InitMode))
  {
    // do nothing in these modes
    return;
  }

  //*****************
  //  DISABLE FURTHER PPS interrpts at ICP pin and ENABLE system interrupts
  //  *** WARNING - RE-ENABLE these interrupts before leaving this ISR
  //
  TIMSK4 &= ~(1 << ICIE4);    // turn off ICP for timer 4 => no more PPS interrupts
  interrupts();               // enable interrupts again

  // Check for start/end of LED pulse in PPS mode
  //
  //******************
  if (FlashMode == PPS)
  {

    // check status of PPS flash sequence
    //  countdown >= 0 => in a flash sequence
    //  countdown == 0 => time to disable flash
    //  countdown < 0 => not in flash sequence now
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
        OCR2A = OCR2B = 0;                    // LED OFF
        tk_LED = GetTicks(CNT4);              // time LED turned OFF
        LED_ON = false;

        // LOG time LED went OFF
        //
        ultohexA(logFlashFINAL + offset_logFlashFINAL,tk_LED);
        chk = chksum_b(logFlashFINAL,chksum_logFlashFINAL-1);   // compute checksum
        btohexA(logFlashFINAL + chksum_logFlashFINAL, chk);
        blnLogFlashOFF = true;

      }
      PPS_Flash_Countdown_Sec--;            //  decrement this count => no longer in PPS flash sequence

    }
    else if (PPS_Flash_Countdown_Sec > 0)
    {
      // (Countdown > 0)  => flash should be enabled
      //
      // if (LED OFF)
      //   turn it on
      //
      if (!LED_ON)
      {
        OCR2A = OCR2B = flashlevel;
        tk_LED = GetTicks(CNT4);              // time LED turned ON
        LED_ON = true;

        // log time LED went ON
        //
        ultohexA(logFlashON + offset_logFlashON,tk_LED);
        chk = chksum_b(logFlashON,chksum_logFlashON-1);   // compute checksum
        btohexA(logFlashON + chksum_logFlashON, chk);
        blnLogFlashON = true;

      }

      // one less second left in this pulse
      //
      PPS_Flash_Countdown_Sec--;

    } // end of LED status update

  } // end of check for PPS flash mode


  //************************************
  //  Validate PPS interval
  //    if too long or too short => Error Mode (PPS error)
  //   

  // save the previous value for compare
  //
  timePrev = tk_PPS;
  tk_PPS = timeCurrent;

  // log the PPS time
  //
  ultohexA(logPPS + offset_logPPS,tk_PPS);
  chk = chksum_b(logPPS,chksum_logPPS-1);   // compute checksum
  btohexA(logPPS + chksum_logPPS, chk);
  LogTextWrite(logPPS,len_logPPS);

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
  if (DeviceMode == Syncing)
  {
    if ((timeDiff < (Timer_Second - CLOCK_TOLERANCE)) || (timeDiff > (Timer_Second + CLOCK_TOLERANCE)))
    {
      // report error
      //
      ustohexA(logERROR + offset_logERROR,err_pps_interval_clock);
      chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
      btohexA(logERROR + chksum_logERROR, chk);
      LogTextWrite(logERROR,len_logERROR);

      // sync error - restart sync
      //
      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;

      DeviceMode = WaitingForGPS;
      TimeSync = SYNC_SECONDS;
      ppsDiff = 0;                // reset this to no error for now.


    }
  }
  else if (DeviceMode == TimeValid) 
  {
    if ((timeDiff < (Timer_Second - PPS_TOLERANCE)) || (timeDiff > (Timer_Second + PPS_TOLERANCE)))
    {
      // report error
      //
      ustohexA(logERROR + offset_logERROR,err_pps_interval_tolerance);
      chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
      btohexA(logERROR + chksum_logERROR, chk);
      LogTextWrite(logERROR,len_logERROR);

      // sync error - restart sync
      //
      DeviceMode = WaitingForGPS;
      TimeSync = SYNC_SECONDS;
      ppsDiff = 0;                // reset this to no error for now.

      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;
    }
  }



  // check to see if we should log the time of flash ON or OFF
  //
  if (blnLogFlashON)
  {
    LogTextWrite(logFlashON,len_logFlashON);
    blnLogFlashON = false;
  }
  if (blnLogFlashOFF)
  {
    LogTextWrite(logFlashFINAL,len_logFlashFINAL);
    blnLogFlashOFF = false;
  }


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
  //    - NOTE: should always move here from WaitingForGPS => errors should go to WaitingForGPS mode first
  //
  //
  //
  if ( DeviceMode == TimeValid )
  {
    
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

          DeviceMode = WaitingForGPS;
          TimeSync = SYNC_SECONDS;
    
          tk_pps_interval_total = 0;
          tk_pps_interval_count = 0;
          
          // report error
          //
          ustohexA(logERROR + offset_logERROR,err_rmc_time);
          chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
          btohexA(logERROR + chksum_logERROR, chk);
          LogTextWrite(logERROR,len_logERROR);

          // leave PPS interrupt routine now... 
          //
          TIMSK4 |= (1 << ICIE4);   // RE-ENBALE PPS interrupts first!
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
      TIMSK4 |= (1 << ICIE4);   // RE-ENBALE PPS interrupts first!
      return;
    }
    else if (gpsRMC.status != 'A')
    {
      // wait for a good fix
      TIMSK4 |= (1 << ICIE4);   // RE-ENBALE PPS interrupts first!
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
      ErrorFound = err_sync_NotValid;
    }
    else if (gpsRMC.status != 'A')
    {
      ErrorFound = err_sync_Mode;
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
        ErrorFound = err_sync_rmcSecDiff;    
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
          ErrorFound = err_sync_pubxSecDiff;
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
        ErrorFound = err_sync_rmcTimeDiff;
      }
    }
    
    // if error, report it and return
    //
    if (ErrorFound > 0)
    {
      // failed the test - report error and restart sync
      //
      DeviceMode = WaitingForGPS;
      TimeSync = SYNC_SECONDS;

      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;
      
      // report error
      //
      ustohexA(logERROR + offset_logERROR,ErrorFound);
      chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
      btohexA(logERROR + chksum_logERROR, chk);
      LogTextWrite(logERROR,len_logERROR);

      // and done with this PPS logic
      TIMSK4 |= (1 << ICIE4);   // RE-ENBALE PPS interrupts first!
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
        
      // report error
      //
      ustohexA(logERROR + offset_logERROR,err_invalidMode);
      chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
      btohexA(logERROR + chksum_logERROR, chk);
      LogTextWrite(logERROR,len_logERROR);
    
  }  // end of check for current mode

  // all done with PPS capture interrupt
  //
  TIMSK4 |= (1 << ICIE4);   // RE-ENBALE PPS interrupts before leaving

} // end of PPS/Timer4 input capture interrupt

//========================================
// ISR for Timer 5 overflow
//=========================================
ISR( TIMER5_OVF_vect)
{
  
  timer5_ov++;   // just increment overflow count
  
} // end of Timer5 overflow vector

//=================================================================
// ISR for Timer 5 input capture interrupt - capture EXP interrupt
//==================================================================
ISR( TIMER5_CAPT_vect)
{
  unsigned long tk_EXP;
  unsigned long tk_LED;
  bool blnLogFlash;
  byte chk;

  // get current time and save it to the event buffer
  //
  tk_EXP = GetTicks(IC5);

  //*****************
  //  DISABLE FURTHER EXP interrpts at ICP pin and ENABLE system interrupts
  //  *** WARNING - RE-ENABLE these interrupts before leaving this ISR
  //
  TIMSK5 &= ~(1 << ICIE5);    // turn off ICP for timer 5 => no more EXP interrupts
  interrupts();               // enable interrupts again

  // Is an EXP flash sequence currently active?
  //
  blnLogFlash = false;
  if ((FlashMode == EXP) && (pulse_countdown > 0) )
  {
    // EXP flash cycle active and not done yet
    //

    // a flash sequence is active, is it time for a pulse?
    //
    if (pulse_interval_countdown > 0)
    {
      // not yet time for a pulse
      // decrement interval count
      //
      pulse_interval_countdown--;

    }
    else
    {
      // it is time for a flash pulse
      //

       // turn on LED
      //
      OCR2A = OCR2B = flashlevel;
      tk_LED = GetTicks(CNT4);              // time LED turned ON
      LED_ON = true;

      // start flash timer = timer 3
      TCCR3B = 0;                             // no source => clock stopped
      TCNT3 = 0;                              // start count at 0
      TIFR3 = 0;                              // clear all pending ints
      OCR3A = OCR3A_pulse;                    // set duration   
      TCCR3B |= (1 << CS32);                  // f/256 clock source => timer is ON now   
      TIMSK3 |= (1 << OCIE3A);                // enable timer compare interrupt
      
      // reset interval countdown
      //
      pulse_interval_countdown = pulse_interval;

      // decrement total pulse count
      //
      pulse_countdown--;

      // log the flash start too...
      //
      blnLogFlash = true;
    }
    

  }

  // logging - EXP time and (optional) flash time
  //
  ultohexA(logEXP + offset_logEXP,tk_EXP);
  chk = chksum_b(logEXP,chksum_logEXP-1);   // compute checksum
  btohexA(logEXP + chksum_logEXP, chk);
  LogTextWrite(logEXP,len_logEXP);

  if (blnLogFlash)
  {
      ultohexA(logFlashON + offset_logFlashON,tk_LED);
      chk = chksum_b(logFlashON,chksum_logFlashON-1);   // compute checksum
      btohexA(logFlashON + chksum_logFlashON, chk);
      LogTextWrite(logFlashON,len_logFlashON);

  }

  // all done with EXP capture ISR
  //    - optionally re-enable capture for EXP
  if (blnLogEXP)
  {
    TIMSK5 |= (1 << ICIE5);   // RE-ENBALE EXP interrupts! 
  }


} // end of Timer5 input capture interrupt


//++++++++++++++++++++++++++++++++++++++++++++++++++
//
// MISC Utility routines
//
//++++++++++++++++++++++++++++++++++++++++++++++++++

//===========================================================================
// chksum_b() - return checksum of byte array
//  INPUTS:
//    arrayB = pointer to byte array
//    lenB = # of array elements for checksum
//===========================================================================
byte chksum_b( char *arrayB, int lenB)
{

  byte chksum = 0;
  for( int i = 0; i < lenB; i++)
  {
    chksum = chksum ^ (byte)arrayB[i];
  }

  return( chksum );

} // end of chksum()


//===========================================================================
// ultohexA - convert unsigned long to 8 hex ASCII characters
//
//===========================================================================
char hexA[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46};
void ultohexA(char *dest, unsigned long ul)
{


  char *pn;
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
// ustohexA - convert unsigned short to 4 hex ASCII characters
//
//===========================================================================
void ustohexA(char *dest, unsigned short us)
{


  char *pn;
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

//===========================================================================
// btohexA - convert byte to 2 hex ASCII characters
//
//===========================================================================
void btohexA(char *dest, byte b)
{

  char *pn = dest + 1;
  unsigned short nibble;
  
  // lower nibble
  //
  nibble = (b & 0x0F);
  *pn = hexA[nibble];

  // upper nibble
  //
  b = b >> 4;
  nibble = (b & 0x0F);
  pn--;
  *pn = hexA[nibble];
  
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

//=========================================
//  setLED routines
//=========================================

void setLEDtoHighRange() 
{
  digitalWrite(midRange, HIGH); // Turn off the middle current transistor
  pinMode(flashPinB, INPUT);    // Turn off the low/middle current PWM output
  pinMode(flashPinA, OUTPUT);   // Turn on the high current PWM output
} // setLEDtoHighRange()

void setLEDtoMidRange() 
{
  pinMode(flashPinA, INPUT);    // Turn off the high current PWM output
  digitalWrite(midRange, LOW);  // Turn on the middle current selection transistor
  pinMode(flashPinB, OUTPUT);   // Turn on the low/middle current PWM output
} // setLEDtoMidRange()

void setLEDtoLowRange()
{
  pinMode(flashPinA, INPUT);    // Turn off the high current PWM output
  digitalWrite(midRange, HIGH);    // Turn off the middle current transistor
  pinMode(flashPinB, OUTPUT);   // Turn on the low/middle current PWM output
} // setLEDtoLowRange()

//===============================================================
// SETUP
//===============================================================
void setup()  
{    
  unsigned int sReg;
  
  //*************************************
  // general Init
  //

  // initializing now...
  //
  DeviceMode = InitMode;
  FlashMode = PPS;
  pulse_countdown = 0;          // no EXP sequence active
  pulse_interval_countdown = 0;

  // set PIN modes
  //
  pinMode(LED_BUILTIN,OUTPUT);          // setup built-in LED 
  digitalWrite(LED_BUILTIN, LOW);
  
  pinMode(PPS_PIN,INPUT);                    // ICP4 = pin 49 as input
  pinMode(EXP_PIN,INPUT_PULLUP);             // ICP5 = pin 48 as input WITH PULL-UP 

  // setup LED driver circuit
  //

  // Select highest current drive to LED 
  TCCR2A = 0;//reset the register => PWM off
  TCCR2B = 0;//reset the register
  setLEDtoHighRange();
  OCR2A = OCR2B = 255;

  // flashpinA and flashpinB are PWM outputs (Timer 2 OC2A and OC2B output respectively)
  // We start the pins as inputs so that the LED cannot turn on until we are through setting up the PWM
  pinMode(flashPinA, INPUT);  // Connect flashpin to the LED cathode (the lead on the flat edge of the package). 
  pinMode(flashPinB, INPUT);  // Connect flashpin to the LED cathode (the lead on the flat edge of the package). 

  // Turn off the mid range current switch transistor
  pinMode(midRange, OUTPUT);
  digitalWrite(midRange, HIGH);

  pinMode(flashPinA, OUTPUT);   // Turn on the high current PWM output

  // connect to PC at 250k to reduce transmission errors with 16mhz Arduino clock
  //
  //
  Serial.begin(250000);
  Serial.println("[STARTING!]");

  // 9600 NMEA is the default rate for the ublox GPS
  //
  Serial1.begin(9600);

  //*********************
  //  Init GPS
  //
  if (!gpsCommInit())
  {
    Serial.print("[FATAL error initializing GPS.]");
  }
 

  //**********************
  //  Init timers
  //	  Timer 2 - PWM for LED
  //    Timer 3 - LED pulse duration
  //    Timer 4 - PPS logging
  //    Timer 5 - exposure marker logging
  //  

  // Timer 2 - PWM for LED (to enable some s/w of control brightness)
  //

  TCCR2A = 0;//reset the register
  TCCR2B = 0;//reset the register
  TCNT2  = 0;

  TCCR2A = 0b10100001;  // COM2A1 COM2A0 COM2B1 COM2B0    -      -    WGM21  WGM20
                        //    1      0      1      0      0      0      0      0
                        // Behavior: PWM on OC2A and OC2B (Pins 9 and 10) ; Waveform generation mode 1 (Phase correct PWM ; TOP = 255)
                        
  TCCR2B = 0b00000001;  //  FOC2A  FOC2B    -      -    WGM22   CS22   CS21   CS20
                        //    0      0      0      0      0      0      0      1
                        // Behavior: Waveform generation mode 1 (Phase correct PWM ; TOP = 255) ; Prescaler -> clk/1

  // Note: fpwm = fclk/(N·2·255)  --> N = pre-scaler factor
  //            = 16M /(1·2·255) = 31.36 kHz
                        
  OCR2A  = 0;         // duty cycle value
  OCR2B  = 0;         // duty cycle value
 
  //  Timer 3 - for LED flash duration in EXP mode only
  //    CTC mode 4
  //    prescaler OFF => timer OFF for now / but will be set to f/256 for actual timing
  //                      f/256 => 64khz => us/16 = value for OCR3A register, 16us minimum, 1024ms maximum
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
  TIMSK5 = (1 << TOIE5);   // timer 5: turn on overflow interrupts

      // optionally turn on ICP capture
  if (blnLogEXP)
  {
    TIMSK5 |= (1 << ICIE5);   // timer 5: turn on IC capture 
  }

  // Timers begin...
  //
  GTCCR = 0;    // RESTART prescaler and all synchronous timers
  
  //******************
  //  Waiting for GPS mode
  //  - Wait for PPS interrupts AND NMEA data valid 
  //  - ReadGPS will parse the NMEA data and set valid status for NMEA data
  //  - after NMEA data valid, next PPS ISR will change the device mode to "Syncing" 
  //
  DeviceMode = WaitingForGPS;
  tBeginWait = millis();

  fStarted = false;       // startup logic will happen later after we have GPS lock
  blnLogEnable = true;
  blnReportMode = false;

  //*****************
  //  clear pending data from the serial input
  //
  ClearSerialInput();
  

} // end of setup


//==========================================================================
//  LOOP
//
//  - check for GPS serial data
//  - Then check for incomming commands from PC
//  - output logging data
//  - Then execute PC commands
//  
//============================================================================
void loop()                     // run over and over again
{
  int retVal;
  byte chk;
  //******************************************************
  //  check for pending data from GPS
  //
  retVal = ReadGPS();
  if ((retVal > 0) && (DeviceMode == TimeValid))
  {
    // report error
    //
    ustohexA(logERROR + offset_logERROR,retVal);
    chk = chksum_b(logERROR,chksum_logERROR-1);   // compute checksum
    btohexA(logERROR + chksum_logERROR, chk);
    LogTextWrite(logERROR,len_logERROR);

  }
  
  //***************************
  //  report current operating mode?
  //
  if (blnReportMode)
  {
      if (DeviceMode == TimeValid)
      {
        char *p = logModeTimeValid + fmode_logModeTimeValid;

        // report current flash mode
        //
        if (FlashMode == PPS)
        {
          *p++ = 'P';
          *p++ = 'P';
          *p = 'S';
        }
        else if (FlashMode == EXP)
        {
          *p++ = 'E';
          *p++ = 'X';
          *p = 'P';
        }
        else
        {
          *p++ = 'E';
          *p++ = 'R';
          *p = 'R';
        }

        chk = chksum_b(logModeTimeValid,chksum_logModeTimeValid-1);   // compute checksum
        btohexA(logModeTimeValid + chksum_logModeTimeValid, chk);
        LogTextWrite(logModeTimeValid,len_logModeTimeValid);
      }
      else if (DeviceMode == Syncing)
      {
        LogTextWrite(logModeSync,len_logModeSync);
      }
      else if (DeviceMode == WaitingForGPS)
      {
        LogTextWrite(logModeWaitingForGPS,len_logModeWaitingForGPS);
      }
      else if (DeviceMode == FatalError)
      {
        LogTextWrite(logModeFatal,len_logModeFatal);
      }
      else if (DeviceMode == InitMode)
      {
        LogTextWrite(logModeInit,len_logModeInit);
      }
      blnReportMode = false;      // turn OFF until next NMEA set
  }

  //******************************************************
  // Output pending data from logging buffer
  //
  if (blnLogEnable)
  {

    LogFlushAll();         // flush logging buffer

  }

  //***********************
  //  now check for an incomming command from USB port
  //
  ReadCMD();
 

} // end of loop()


