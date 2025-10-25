/*
	iota-gft.h 
	
	include for main module 
	
*/

//  make sure this include is invoked only once
//
#if !defined(__iotagft_h)
  #define __iotagft_h

  //******************************
  //  VERSION
  //
  extern const char *strVersion;
  extern const char *strDeviceName;

  //*****************************
  //  Operating Mode
  //
  //  InitMode => initializing the device
  //  WaitingForGPS => waiting for valid GPS data and PPS
  //  Timing => normal timing operation
  //  FatalError => fatal error occured
  //
  enum OperatingMode
  {
    InitMode,               // => Power up initialization
    WaitingForGPS,          // => Waiting to receive GPS data
    Syncing,                // => syncing to PPS time
    TimeValid,              // => Timing data is OK
    FatalError              // => Fatal error occurred
  };
  extern volatile OperatingMode DeviceMode;    // current operating mode
  extern volatile bool blnReportMode;

  extern short int SYNC_SECONDS;                // # of seconds for syncing to GPS time
  extern volatile short int TimeSync;           // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)

  //****************************************
  // Flashing Modes
  //  CLK mode => flash now and turn off after X seconds by internal clock counts
  //  PPS mode => flash on next PPS signal and off after X PPS signals
  //	EXP mode => short duration flashes on EXP interrupt (exposure signal)
  //
  //
  enum FlashingMode
  {
    CLK,
    PPS,
    EXP
  };
  extern volatile FlashingMode FlashMode;		// current flashing mode

  extern volatile boolean LED_ON;      			// LED state
  extern volatile int PPS_Flash_Countdown_Sec;           // # of seconds remaining in a PPS flash

  // flashlevel = PWM adjustment
  //
  extern byte flashlevel;
  extern byte flashrange;
  extern byte OCR2Alevel;
  extern uint16_t Timer3_us;
  extern uint16_t OCR3A_pulse;

  //******************
  // Timer info
  //
  enum CountSource
  {
    CNT4,
    IC4,
    CNT5,
    IC5
  };

  //****************
  //  Time variables
  //

  extern volatile unsigned long Timer_Second;                 // # of ticks for 1 second
  extern volatile unsigned long PPS_TOLERANCE;                // tolerance for PPS interval - 1ms
  extern volatile unsigned long CLOCK_TOLERANCE;              // tolerance for arduino clock frequency interval - currently 10ms

  extern volatile unsigned long tk_pps_interval_total;         // sum of pps intervals
  extern volatile unsigned long tk_pps_interval_count;         // # of pps interval
  extern volatile unsigned long tk_pps_interval_ave;           // average pps interval

  extern int pulse_duration_us;                         // duration of one shutter pulse (microseconds)
  extern volatile int pulse_interval;                   // time between pulses (miliseconds)
  extern volatile int pulse_countdown;             // # of pulses left in EXP sequence, 0 => no EXP flash sequence in progress


  extern volatile bool time_UTC;                  // true => time is currently UTC , false => time is currently GPS
  extern volatile int sec_Year;                   // 2 digit
  extern volatile int sec_Mon;
  extern volatile int sec_Day;
  extern volatile int sec_hh;
  extern volatile int sec_mm;
  extern volatile int sec_ss;
  extern volatile int offsetUTC_Default;      // receiver default value for GPS-UTC offset
  extern volatile int offsetUTC_Current;      // current/valid GPS-UTC offset

  // output strings
  //
  extern char logFlashON[];
  extern int len_logFlashON;
  extern int offset_logFlashON;
  extern int chksum_logFlashON;

  extern char logFlashFINAL[];
  extern int len_logFlashFINAL;
  extern int offset_logFlashFINAL;
  extern int chksum_logFlashFINAL;

  //****************
  // ERROR CODES
  //
  #define err_pps_interval_clock  0
  #define err_pps_interval_tolerance 1
  #define err_rmc_time 2
  #define err_sync_NotValid 3
  #define err_sync_Mode 4
  #define err_sync_rmcTimeDiff 5
  #define err_sync_rmcSecDiff 6
  #define err_sync_pubxSecDiff 7
  #define err_gps_ubxTime 8
  #define err_gps_utcMatch 9
  #define err_gps_parseNMEA 10
  #define err_gps_nmeaCount 11
  #define err_invalidMode 255

  //******************
  // function prototypes
  //
  extern unsigned long GetTicks(CountSource);
  extern void ultohexA(char *, unsigned long);
  extern void ustohexA(char *, unsigned short);
  extern void btohexA(char *, byte);
  extern byte chksum_b( char *, int);

  extern void setLEDtoHighRange();
  extern void setLEDtoMidRange();
  extern void setLEDtoLowRange();

#endif // end of block containing entire include file