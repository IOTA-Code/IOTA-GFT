/*
	iota-gft.h 
	
	include for main module 
	
*/

//  make sure this include is invoked only once
//
#if !defined(__iotagft_h)
  #define __iotagft_h

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

  extern short int SYNC_SECONDS;                // # of seconds for syncing to GPS time
  extern volatile short int TimeSync;           // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)


  //****************************************
  // Flashing Modes
  //  PPS mode => flash on PPS signal for X whole seconds
  //	EXP mode => short duration flashes on EXP interrupt (exposure signal)
  //
  //	fEventDefined => True if an future event definition active
  //
  enum FlashingMode
  {
    PPS,
    EXP
  };
  extern volatile FlashingMode FlashMode;		// current flashing mode

  extern volatile boolean LED_ON;      			// LED state

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


  extern volatile bool time_UTC;                  // true => time is currently UTC , false => time is currently GPS
  extern volatile int sec_Year;                   // 2 digit
  extern volatile int sec_Mon;
  extern volatile int sec_Day;
  extern volatile int sec_hh;
  extern volatile int sec_mm;
  extern volatile int sec_ss;
  extern volatile int offsetUTC_Default;      // receiver default value for GPS-UTC offset
  extern volatile int offsetUTC_Current;      // current/valid GPS-UTC offset

  //******************
  //  Input parms
  //
  extern volatile bool blnEchoNMEA;

  //***********
  // debug
  //
  extern int errorCode;

  //******************
  // function prototypes
  //
  extern unsigned long GetTicks(CountSource);
  extern void ultohexA(uint8_t *, unsigned long);


#endif // end of block containing entire include file