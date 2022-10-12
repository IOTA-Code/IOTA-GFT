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
//   Connect the Digital pin 48 to the exposure interrupt input (or GND if no exposure int input)
//
//  LED
//   Connect LED positive to Digital pin 9
//
//  BUTTON
//   Connect one push button input to +5V.
//   Connect second push button input to pin 4 AND to 10K resister tied to GND (pull down)
// 


//=======================================
//  GLOBAL variables and definitions
//=======================================


// Operating Modes
//  PPS mode => flash on PPS signal
//	EXP mode => flash on EXP interrupt (exposure signal)
//
//	fEventDefined => True if an future event definition active
//
enum OperatingMode
{
  InitMode,               // => Power up initialization
  WaitingForGPS,          // => Waiting to receive GPS data
  Syncing,                // => Synchronizing internal time base with GPS data
  Ready,                  // => Synced and ready
  FatalError,             // => Fatal error occurred
	PPS,
	EXP
};
volatile OperatingMode FlashMode;		// current flashing mode

#define SYNC_SECONDS 4                // # of seconds for syncing to GPS time
volatile short int TimeSync;          // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)

bool fEventDefined  = false;	// true if event defined

// Misc
//
int BUTTON_PIN = 4;

// Timers
//
enum CountSource
{
  CNT4,
  IC4,
  CNT5,
  IC5
};

volatile unsigned short timer4_ov;    // timer 4 overflow count = high word of "time" (32ms)
volatile unsigned short timer5_ov;    // timer 5 overflow count  
#define Timer_Second 2000000          // timers are running at about 2mhz
#define Timer_Milli 2000              // approx ticks per millisecond

const byte OCR2Alevel = 159;  		  			// PWM frequency set; on Timer 2 with prescaler = 1, 99 makes 160kHz, 159 = 100kHz, 199 = 80kHz
byte flashlevel;
const unsigned long Timer3_us = 64;				// microseconds per led timer clock count (x1024 prescaler)

// 1 PPS event
//
volatile boolean pps_led_state = false;       // current state of on-board LED
volatile unsigned long pps_time;
volatile unsigned long pps_count = 0;         // # of pps ints since power on
volatile boolean pps_new = false;
volatile boolean pps_flash = false;           // true IFF LED was flashed with this PPS

volatile boolean timer45_error = false;       // true if we detect a mismatch between timer 4 and 5
volatile unsigned long timer45_values;              // high 16 bits = TCNT5 , low 16 bits = TCNT4

// LED / Flash variables
//    each flash is a sequence of LED pulses separated by an interval
//
int LED_PIN = 9;                      			// PIN for LED output
volatile boolean LED_ON = false;      			// LED state

          // PPS pulse settings
int PPS_Pulse_Duration = 5;						// duration of one LED pulse in seconds
int PPS_Pulse_Count = 1;						  // # of pulses in a flash
int PPS_Pulse_Interval = 1;						// # of seconds between PPS pulses

int Flash_Test_Interval = 60;					// # of seconds between flash sequences while emitting test flashes

volatile int Pulse_Duration = 0;                // # of seconds of LED pulse remaining
volatile int Pulse_Count = 0;           			  // # of LED pulses remaining

// Command string from the PC
//
#define CommandSize 30            // max size of a command string 
char strCommand[ CommandSize + 1 ];
int Cmd_Next;                     // speed optimization ( -1 => full command pending )
volatile unsigned long cmd_time;


//  List for NMEA sentences (managed as a circular buffer)
//    S_BufferHead = first valid entry in buffer
//    S_BufferTail = next spot to use in buffer
//    S_Count = # of valid entries
//
typedef struct
{
  unsigned long RecvTime;     // time of first char received for this sentence
  int IdxFirst;               // starting index of this sentence in the NMEA char buffer (-1 => empty)
  boolean Complete;           // is this sentence complete?
} NMEA_Sentence;

#define S_BufferSize 8
volatile NMEA_Sentence S_Buffer[S_BufferSize];
volatile int S_BufferHead = -1;
volatile int S_BufferTail = 0;
volatile int S_Count = 0;       // # of entries in use (including partial sentence
volatile int S_Current = -1;    // current entry for reading data ( -1 > no active sentence now )

//  Buffer for NMEA sentence characters
//    N_Buffer[] - buffer holding sentences
//    N_BufferHead - head of used portion of buffer
//    N_BufferTail - tail - next spot for a character
//    N_Count = # of characters in the buffer

#define N_BufferSize 500    // buffer of sentence chars 
                            //   must be greater than 120 (best is > 150) to capture one full sentence
volatile char N_Buffer[N_BufferSize];
volatile int N_BufferHead = -1;
volatile int N_BufferTail = 0;
volatile int N_Count = 0;
volatile boolean N_Start = false;      // true after seeing first $

// Event buffer
//    E_Buffer[] - buffer holding sentences
//    E_BufferHead - head of used portion of buffer
//    E_BufferTail - tail - next spot to use in buffer
//
#define E_BufferSize 20
volatile unsigned long E_Buffer[E_BufferSize];
volatile int E_BufferHead = -1;
volatile int E_BufferTail = 0;
volatile int E_Count = 0;

volatile boolean OneTime = true;

volatile int mCount = 0;

// debug flags
//
boolean f_Save1 = false;
boolean f_Save2 = false;
boolean f_Save3 = false;
boolean f_Save4 = false;

volatile unsigned long ulTime1 = 0;
volatile unsigned long ulTime2 = 0;
volatile int tailTest = -1;


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
  FlashMode = InitMode;

  // set PIN modes
  //
  pinMode(LED_BUILTIN,OUTPUT);    // setup built-in LED 
  pinMode(LED_PIN,OUTPUT);        // setup external LED pin
  pinMode(BUTTON_PIN,INPUT);		  // button input pin

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
  retVal = gpsInit();
  if (retVal > 0)
  {
    Serial.print("ERROR initializing GPS module : ");
    Serial.println(retVal,HEX);
  }
 

  //**********************
  //  Init timers
  //	  Timer 2 - PWM for LED
  //    Timer 3 - LED flash duration
  //    Timer 4 - 1pps logging
  //    Timer 5 - exposure marker logging
  //  

  // Timer 2 - PWM for LED (to control brightness)
  //
  // set flash pwm frequency using Timer 2 in fast PWM mode; when on, TCNT2 continuously counts from 0 to OCR2A
  // output OC2B (pin 9) starts high at TCNT2 = 0, goes low when TCNT2 = OCR2B and resets to high at TCNT2 = OCR2A
  //
  // pwm pulses are switched on/off by setting/clearing the CS20 bit of TCCR2B (prescaler)
  //
  TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  		// clear OC2B on compare, fast PWM top = OCR2A (with WGM22 below)
  TCCR2B = bit(WGM22);   								// fast pwm, prescaler = 0 NOW, set CS20 bit to start PWM
  OCR2A = OCR2Alevel;  									// top level of TCNT2
  OCR2B = map(flashlevel, 0, 100, 0, OCR2Alevel);  		// compare at flashlevel mapped to 0<>OCR2A range; user adjusts this
  
  //  Timer 3 - for LED flash duration
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
  pinMode(49,INPUT);                    // ICP4 = pin 49 as input
  
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
  pinMode(48,INPUT);                    // ICP5 = pin 48 as input
  
  TIMSK5 = 0;                             // mask off all interrupts for now
  TCCR5A = 0;                             // all OC ports off and normal mode
  TCCR5B = (1 << ICNC5) | (1 << ICES5) | (1 << CS51);    // noise cancel on, positive edge trigger IC & f/8 prescaler
  PRR1 &= ~(1 << PRTIM5);                 // turn OFF power reduction for timer 5 => turn it ON!

  //  SYNC all synchronous timers via Prescaler reset
  //
  GTCCR = (1 << TSM) | (1 << PSRSYNC);    // STOP prescaler and all syncronous timers

  TCNT4 = 0;        // timer4: reset count
  timer4_ov = 0;    // timer4: reser overflow count
  TIFR4 = 0;        // timer4: reset all pending interrupts
  TIMSK4 = (1 << ICIE4) | (1 << TOIE4);   // timer 4: turn on IC capture and overflow interrupts

  TCNT5 = 0;        // timer5: reset count
  timer5_ov = 0;    // timer5: reser overflow count
  TIFR5 = 0;        // timer5: reset all pending interrupts
  TIMSK5 = (1 << ICIE5) | (1 << TOIE5);   // timer 5: turn on IC capture and overflow interrupts

  GTCCR = 0;    // RESTART prescaler and all synchronous timers
  

  //*********************
  // init data buffers
  //
  ClearBuffers();

  //******************
  //  Waiting for GPS
  //
  FlashMode = WaitingForGPS;
  Serial.println("Waiting for GPS...");

  tBegin = millis();
  while(FlashMode != Syncing)
  {
    // wait up to 10 minutes for GPS
    //
    tNow = millis();
    if ((tNow - tBegin) > (10 * 60000))
    {
      Serial.println("Timeout waiting for GPS.");
      FlashMode = FatalError;
      while(1);
    }
  } // end of waiting for GPS 

  //******************
  //  Now wait for sync to end
  //
  tBegin = millis();
  while(FlashMode != Ready)
  {
    // wait up to 10 minutes for Sync
    //
    tNow = millis();
    if ((tNow - tBegin) > (10 * 60000))
    {
      Serial.println("Sync failed.");
      FlashMode = FatalError;
      while(1);
    }
  }

  //*****************
  //  Now start timing...
  //
  FlashMode = PPS;      // start in PPS mode for now

} // end of setup


//=====================================
// ClearBuffers - clear out NMEA and command buffer
//=====================================
void ClearBuffers()
{
  int tmp;

  // reset command stream from PC
  //     clear incomming data buffer from PC
  //
  while( Serial.available() )
  {
    tmp = Serial.read();
  }
  strCommand[0] = 0;  // no command
  Cmd_Next = 0;

  // clear NMEA buffers
  //
  S_BufferHead = -1;
  S_BufferTail = 0;
  S_Count = 0;
  S_Current = -1;
  N_Count = 0;
  N_BufferHead = -1;
  N_BufferTail = 0;
  N_Start = false;

  // clear event buffers
  //
  noInterrupts();
  E_Count = 0;
  E_BufferHead = -1;
  E_BufferTail = 0;
  interrupts();  
  
} // end of ClearBuffers

//==========================================================================
//  LOOP
//
//  Basic algorithm
//	- check for button press
//  - Keep checking for NMEA data until there is none
//  - Then output message buffers (earliest time stamp first)
//  - Then check for incomming commands from PC
//  - Then execute PC commands
//
//============================================================================
void loop()                     // run over and over again
{
  unsigned long tmpTime;
  unsigned long firstTime;
  char pps_string[] = "[00000000]pps 0000\r\n";
  char time_string[] = "[00000000]";
  char c;
  int iCount;
  int iDuration;
  int iPos;
  int iPosEnd;
  String inOption;
  
  byte lastButtonState = LOW;
  unsigned long debounceDuration = 50; 				// millis
  unsigned long lastTimeButtonStateChanged = 0;


  int NextBuffer;     // 0 => nothing to do, 1=> NMEA, 2=> pps, 3=> external


  //******************************************************
  //  check for errors
  //
  if (timer45_error)
  {
    ultohex(pps_string+1,timer45_values);
    pps_string[13] = 'X';
    
    timer45_error = false;    // clear it

    // report it
    //
    for( int i = 0; i < 20; i++ )
    {       
      if ((c=pps_string[i]) == 0)
        break;
  
      Serial.print(c);
    }

    // clear error in string now
    //
    pps_string[13] = ' ';
  }

  //******************************************************
  // check for Button Press (with debouncing)
  //   => if not flashing, initiate a flash
  //
  //******************************************************
  if (millis() - lastTimeButtonStateChanged > debounceDuration)
  {
      byte buttonState = digitalRead(BUTTON_PIN);
      if (buttonState != lastButtonState)
	  {
		  lastTimeButtonStateChanged = millis();
		  lastButtonState = buttonState;
		  if (buttonState == HIGH)
		  {
        // button PRESSED!
        //  start flashing at next PPS
        //
        Pulse_Duration = PPS_Pulse_Duration;
        Pulse_Count = PPS_Pulse_Count;
		  }
    }
  }


  //******************************************************
  //  check for pending NMEA data
  //
  ReadNMEA();

  //******************************************************
  // Output messages
  //
  // Find the buffer entry with the earliest timestamp
  //
  NextBuffer = 0;
  
  // check external buffer first
  //
  if (E_Count > 0)
  {
    firstTime = E_Buffer[E_BufferHead];
    NextBuffer = 3;
  }

  // now check pps
  //
  if (pps_new)
  {
    if (NextBuffer == 0)
    {
      firstTime = pps_time;
      NextBuffer = 2;
    }
    else
    {
      // we have a time, is the pps time earlier?
      //   comparing unsigned, must also watch for time rollover
      //   hopefully this logic works...
      //
      if ((pps_time - firstTime) > 0x80000000)
      {
        // pps time is earlier
        firstTime = pps_time;
        NextBuffer = 2;
      }
    }

  }  // end of pps check

  // check NMEA sentence times
  //
  if (S_Count > 0)
  {
    // check first NMEA sentence
    //  - first - is this sentence complete? 
    //    we only send NMEA sentences AFTER they are complete
    //
    if (S_Buffer[S_BufferHead].Complete)
    {
      tmpTime = S_Buffer[S_BufferHead].RecvTime;
      
      if (NextBuffer == 0)
      {
        firstTime = tmpTime;
        NextBuffer = 1;
      }
      else
      {
        if ((tmpTime - firstTime) > 0x80000000)
        {
          firstTime = tmpTime;
          NextBuffer = 1;
        }
      }
    } // end of check for complete sentence
  } // end of checking NMEA sentences

  //
  // Now we may have a time message to send out
  //
  switch( NextBuffer)
  {
    // output NMEA sentence
    //
    case 1 :
      S_SendFromBuffer();
      break;

    // output pps time
    //
    case 2:
      ultohex(pps_string+1,pps_time);
      if (pps_flash)
      {
        ustohex(pps_string+14, (unsigned short)Pulse_Duration);
      }
      else
      {
        strncpy(pps_string + 14, "0   ",4);
      }
      for( int i = 0; i < 20; i++ )
      {       
        if ((c=pps_string[i]) == 0)
          break;
    
        Serial.print(c);
      }
      pps_new = false;
      break;

    // output external time
    //
    case 3:
      E_SendFromBuffer();
      break;

    // do nothing
    //
    default:
      break;
      
  } // end of outputting message

  //***********************
  //  now check for an incomming command
  //   but we don't execute the command until we see a newline...
  //
  ReadCMD();
 
  //*******************************************************
  //  Execute pending command
  //
  if (Cmd_Next < 0)
  {
    // ECHO command string to USB port
    //
    ultohex(time_string+1,cmd_time);
    for( int i = 0; i < 20; i++ )
    {       
      if ((c=time_string[i]) == 0)
        break;
  
      Serial.print(c);
    }
    
    Serial.print("CMD <");
    Serial.print(strCommand);
    Serial.print(">\r\n");

    // EXECUTE command
    //
    Cmd_Next = 0;

    // Commands
    //   ^flash [Count [Duration]], where Count = number of flashes and Duration = length of flash in microseconds (16 us resolution)
    //
    if (strncmp(strCommand,"^flash",6) == 0)
    {
      // look for command options: Count, then Duration
      //
      if (GetFlashParms( &strCommand[6] ))
      {
        pinMode(4,OUTPUT);    // ENABLE OC0B output to LED       
      }
      
    }
    
  } // end of handling a pending command

} // end of loop()

//=====================================
//  ReadNMEA - get pending NMEA data
//======================================
//
void ReadNMEA()
{
  char c;
  unsigned long cTime;
  int idx;

  // is a char ready?
  //
  while (Serial1.available() > 0)
  {

    // char ready, process it
    //
      
    c = Serial1.read();
  
    // time of this character (from timer 4)
    //
    noInterrupts();
//    cTime = ((unsigned long)timer4_ov << 16) + (unsigned long)TCNT4;
    cTime = GetTicks(CNT4);
    interrupts();
    
    // we have a new character ...
    //   '\n' => end of current NMEA string
    //   '$' => start of new NMEA string
    //   else -> just a char in the NMEA string
    //
    //   on start of NMEA - place [time] in buffer followed by $
    //
    
    if (c == '$')
    {
  
      if (S_Current >= 0)
      {
        Serial.println("");
        Serial.println("ERROR 0");
        Serial.println("");
//        DebugBuffers();
        return;
      }
      
      // START of NMEA string
      //    get new entry in list 
      //
      S_Current = S_GetEntry();
      
      S_Buffer[S_Current].RecvTime = cTime;
      S_Buffer[S_Current].Complete = false;
       
      idx = N_SaveToBuffer(c);
      if (idx < 0)
      {
        // unexpected error
        //
        Serial.println("");
        Serial.println("ERROR 1");
        Serial.println("");
        DebugBuffers();
        return;
      }
      else
      {
        S_Buffer[S_Current].IdxFirst = idx;
      }
    }
    else if ( c == '\n' )
    {
      // end of sentence
      //
      if (S_Current < 0)
      {
        // but we weren't in a sentence?
        //
        Serial.println("");
        Serial.println("ERROR 6");
        Serial.println("");
        // whoops!
        //
        return;
      }
  
      // save the character
      //
      idx = N_SaveToBuffer(c);
      if (idx < 0)
      {
        // unexpected error
        //
        Serial.println("");
        Serial.println("ERROR 2");
        Serial.println("");
        DebugBuffers();
        return;
      }
  
      // finalize the status of the sentence
      //
      S_Buffer[S_Current].Complete = true;
  
      // Now that it is complete... we have no "current" sentence
      //
      S_Current = -1;
    }
    else
    {
      // all other characters
      //   ignore if not in a sentence...
      //
      if (S_Current >= 0)
      {
        // save the character
        //
        idx = N_SaveToBuffer(c);
        if (idx < 0)
        {
          // unexpected error
          //
          Serial.println("");
          Serial.println("ERROR 3");
          Serial.println("");
          DebugBuffers();
          return;
        }
      }
            
  }  // end of handling a char read
  } // end of while loop for character available
} // end of ReadNMEA

//=====================================
//  ReadCMD - get pending command data
//======================================
void ReadCMD()
{

  byte bIn;
  
  // read command from PC
  //  - one byte at a time
  //  - command terminates with a Newline
  //
  while ( Serial.available() > 0 )
  {
    // was a command pending?
    //
    if (Cmd_Next < 0)
    {
      Cmd_Next = 0;     // not any more...
    }
    else if (Cmd_Next >= CommandSize)
    {
      // ran out of room => error
      //
      Cmd_Next = 0;   // reset to new command
    }
    
    // is this the start of a command?
    //
    if (Cmd_Next == 0)
    {

      // get time from timer 4 count
      //
      noInterrupts();
//      cmd_time = ((unsigned long)timer4_ov << 16) + (unsigned long)ICR4;
      cmd_time = GetTicks(CNT4);
      interrupts();
      
    }

    
    // ok -> get the character & save it
    //   but don't save CR or LF
    //
    bIn = Serial.read();
      
    // if \n, terminate command line and don't save the \n
    //
    if (bIn == '\n')
    {
      strCommand[Cmd_Next] = 0;
      Cmd_Next = -1;
      break;
    }
    else if (bIn != '\r')
    {
      // here if NOT \r or \n
      //
      strCommand[ Cmd_Next ] = bIn;
      Cmd_Next++;
    }
    // if \r, do nothing with char
    
  } // end of loop reading incoming data
  
} // end of ReadCMD

//===========================================================================
// N_SaveToBuffer - save a character to the next open spot in the NMEA buffer
//   returns buffer index for this character (-1 => ERROR)
//
//   notes:
//    * if we run out of space, discard previous "first" sentence entirely so
//      all "old" sentences are full sentences.
//
//===========================================================================
int N_SaveToBuffer(char cIn)
{
  int idx;
    
  // Check for full buffer
  //
  if (N_BufferTail == N_BufferHead)
  {
    // the buffer is FULL
    //  we are going to overwrite the first char of the first sentence
    //  so we must now invalidate the first sentence entirely and dump all of its characters
    //  ** check for situation where ONE sentence has filled the entire buffer - should not happen!
    //
    if (S_Count <= 1)
    {
      // oops!  - we have only one sentence in progress
      //  => clear everything and start over
      //
      Serial.println("");
      Serial.print("ERROR 4 :");
      Serial.println(cIn);
      Serial.println("");
      DebugBuffers();

      N_BufferHead = -1;
      N_BufferTail = 0;
      N_Count = 0;
      S_BufferHead = -1;
      S_BufferTail = 0;
      S_Count = 0;
      S_Current = -1;
      return(-1);
    }
    else
    {
      // ok - we do have more than one entry in the sentence list/buffer
      //    Remove the first entry from the sentence list
      //    and set char head pointer to first char in new first sentence
      //
      
      S_DeleteHead(); // delete head entry in sentence list (invalid now)

      N_BufferHead = S_Buffer[S_BufferHead].IdxFirst;   // and make head of char buffer = head of new first sentence
      N_Count = N_BufferTail - N_BufferHead;            // update char count
      if (N_Count < 0)
        N_Count = N_Count + N_BufferSize;
    }
   
    // char Count remains the same...
  }
  else
  {
    // now check for start of buffer
    //
    if (N_Count == 0)
    {
      N_BufferHead = N_BufferTail;    // set head ptr
    }

    // we had a free spot, update the count
    N_Count++;
  }

  // copy char into Buffer
  //
  N_Buffer[N_BufferTail] = cIn;
  idx = N_BufferTail;
  
  // update BufferTail (with wraparound)
  //
  N_BufferTail++;
  if (N_BufferTail == N_BufferSize)
    N_BufferTail = 0;

  return( idx );      // return index for the saved character

} // end of N_SaveToBuffer

//===========================================================================
// N_SendFromBuffer - send ONE sentence from NMEA Buffer to serial output
//      NOTE:
//        * free space in buffer as data is sent
//        * checks for incoming NMEA data while sending characters
//===========================================================================
void N_SendFromBuffer()
{
  char c;
  int cCount = 0;
  
  while( N_Count > 0 )
  {
    
    // grab char
    //
    c = N_Buffer[N_BufferHead];

    // update pointers
    //
    N_Count--;                  // one less char
    if (N_Count == 0)
    {
      // buffer is empty, reset it
      //
      N_BufferHead = -1;
      N_BufferTail = 0;
    }
    else
    {
      N_BufferHead++;
      if (N_BufferHead == N_BufferSize)
        N_BufferHead = 0;
    }
    
    // send the head character
    //
    Serial.print(c);
    
    // check for end of string
    //
    if (c == '\n')
      break;      // all done
   
  } // end of loop through string
  
} // end of N_SendFromBuffer


//===========================================================================
// E_SaveToBuffer - save a time to the next open spot in the event buffer
//
//===========================================================================
void E_SaveToBuffer(unsigned long ul)
{

  // Check for full buffer 
  //   if the tail "catches" the head, bump up the head one spot (delete the first entry in the buffer)
  //
  if (E_BufferTail == E_BufferHead)
  {
    // Buffer is full
    //  - but we are saving to the former head location - move the head forward
    
    E_BufferHead++;
    if (E_BufferHead == E_BufferSize)
      E_BufferHead = 0;
      
    // entry Count remains the same...
  }
  else
  {
    // buffer NOT full
    // now check for start of buffer
    //
    if (E_Count == 0)
    {
      E_BufferHead = E_BufferTail;    // set head ptr
    }

    // we had a free spot, update the count
    E_Count++;
  }
  
  // copy data into Buffer
  //
  E_Buffer[E_BufferTail] = ul;
     
  // update BufferTail (with wraparound)
  //
  E_BufferTail++;
  if (E_BufferTail == E_BufferSize)
    E_BufferTail = 0;
     
} // end of E_SaveToBuffer

//===========================================================================
// E_SendFromBuffer - grab one event time from event buffer and output it to serial port
//    NOTE:
//
//===========================================================================
void E_SendFromBuffer()
{
  unsigned long ul;
  char event_string[] = "[00000000]exp";
  char c;

  // is there anything in the queue?
  //
  if (E_Count == 0)
    return;
     
  // get the first time in the queue
  //
  ul = E_Buffer[E_BufferHead]; 
 
  // update count, ptrs
  //
  E_Count--;
  if (E_Count == 0)
  {
    // buffer is empty, reset it
    E_BufferHead = -1;
    E_BufferTail = 0;
  }
  else
  {
    E_BufferHead++;
    if (E_BufferHead == E_BufferSize)
    {
      E_BufferHead = 0;
    }
  }
  
  // Now send the data
  //
  ultohex(event_string+1,ul);
  for( int i = 0; i < 20; i++ )
  {
    
    if ((c=event_string[i]) == 0)
      break;

    Serial.print(c);
  }
  Serial.print('\r');
  Serial.print('\n');
//  Serial.print(event_string);
  
} // end of E_SendFromBuffer

//===========================================================================
// S_GetEntry - allocate a sentence struct from the next open spot in the buffer and return the index
//
//===========================================================================
int S_GetEntry()
{
  int idx;
  
  // Check for full buffer 
  //   if the tail "catches" the head, bump up the head one spot (delete the first entry in the buffer)
  //
  if (S_BufferTail == S_BufferHead)
  {
    // buffer is full, we are overwriting the previous head entry in the buffer with the new "tail" data
    //
    S_BufferHead++;
    if (S_BufferHead == S_BufferSize)
      S_BufferHead = 0;
      
    // S_Buffer Count remains the same...
  }
  else
  {
    // buffer is NOT full...
    //
    
    // now check for start of buffer
    //
    if (S_Count == 0)
    {
      S_BufferHead = S_BufferTail;  // set head ptr
    }
    
    // we had a free spot, update the count
    S_Count++;

  }

  // save the index for the allocated spot
  //
  idx = S_BufferTail;
     
  // update BufferTail (with wraparound)
  //
  S_BufferTail++;
  if (S_BufferTail == S_BufferSize)
    S_BufferTail = 0;

  // all done, return the index
  //
  return(idx);
  
} // end of S_GetEntry

//===========================================================================
// S_SendFromBuffer - extract first NMEA sentence buffers and output it to serial port
//    NOTE:
//      *** This routine checks for pending incoming NMEA data during the write sequence
//    
//
//===========================================================================
void S_SendFromBuffer()
{
  unsigned long ul;
  char strNMEA[] = "[00000000]";

  // is there anything in the queue?
  //
  if (S_Count == 0)
    return;

  // OK - is it complete? (should be if we called this routine)
  //
  if (S_Buffer[S_BufferHead].Complete)
  {
    
    // output preamble (time)
    //
    ultohex(strNMEA+1,S_Buffer[S_BufferHead].RecvTime);
    Serial.print(strNMEA);

    // output contents of head entry in character buffer
    //  note: IdxFirst for head sentence SHOULD point to N_BufferHead
    //
    if (S_Buffer[S_BufferHead].IdxFirst != N_BufferHead)
    {
      Serial.println("");
      Serial.println("ERROR 5");
      Serial.println("");
      DebugBuffers();
      return;
    }
    
    // send head sentence, free up space, and check ReadNMEA along the way...
    //   NOTE: 
    //    *** N_SendFromBuffer may call ReadNMEA and this may affect S_Buffer!  
    //        => S_Buffer must be "stable" before calling ReadNMEA
    //        => call S_DeleteHead FIRST!
    //
    S_DeleteHead();     // remove this head entry from S_Buffer
    
    N_SendFromBuffer();
   
  }  // end of check for complete sentence
  
} // end of S_SendFromBuffer

//===========================================================================
// S_DeleteHead - remove head entry in NMEA sentence buffer (invalidate it)
//     note:
//      ** this routine does NOT update the NMEA character buffer pointers
//
//===========================================================================
void S_DeleteHead()
{
  // sanity check
  //
  if (S_Count == 0)
    return;

  // mark it
  //
  S_Buffer[S_BufferHead].IdxFirst = -1;
  S_Buffer[S_BufferHead].Complete = false;
  
  // update count and pointers
  //
  S_Count--;
  if (S_Count == 0)
  {
    // buffer is empty, reset it
    //
    S_Current = -1;
    S_BufferHead = -1;
    S_BufferTail = 0;
  }
  else
  {
    S_BufferHead++;
    if (S_BufferHead == S_BufferSize)
      S_BufferHead = 0;
  }
  
} // end of S_DeleteHead


//===========================================================================
// GetFlashParms() - get parameters for Flash command AND sets flash variables
//    return: true if success, false otherwise
//
//===========================================================================
bool GetFlashParms(char *inParms)
{
  String parm = "";
  long parmCount;
  long parmDuration;

  // Do we have any parameters?
  //    search for first non-space char
  //
  while( (*inParms != 0) && isWhitespace(*inParms) )
  {
    inParms++;
  }

  // check for no parameters
  //    default to 5 second pulse
  //
  if (*inParms == 0)
  {
    PPS_Pulse_Count = 5;
    PPS_Pulse_Duration = 5;
    return true;
  }

  //*********************************************
  // Looks like we have at least one parameter - assume it is the count
  // 
  while( (*inParms != 0) && !isWhitespace(*inParms) )
  {
    if ( !isDigit(*inParms) )
    {
      // ERROR
      return(false);
    }

    parm += *inParms;

    inParms++;
  }

  // we now have a potential count parm
  //
  parmCount = parm.toInt();
  if (parmCount == 0)
  {
    return false;       // return error
  }

  //***********************************
  // Look for Duration value
  //

  // find first non-space char - to start duration parm
  //
  while( (*inParms != 0) && isWhitespace(*inParms) )
  {
    inParms++;
  }

  if (*inParms == 0)
  {
    // no Duration parm, use default and leave
    //
    PPS_Pulse_Count = parmCount;
    PPS_Pulse_Duration = 5;
    return true;
  }

  // get duration parm chars
  //
  parm = "";
  while( (*inParms != 0) && !isWhitespace(*inParms) )
  {
    if ( !isDigit(*inParms) )
    {
      // ERROR
      return(false);
    }

    parm += *inParms;

    inParms++;
  }

  // we now have a potential count parm
  //  * must be greater than 0 and less than 900ms
  //
  parmDuration = parm.toInt();
  if ((parmDuration == 0) || (parmDuration > 900000))
  {
    return false;       // return error
  }

  //*********************************
  // OK - we have a count and a duration (in seconds)
  //
  PPS_Pulse_Count = parmCount;
  PPS_Pulse_Duration = parmDuration;
  
}  // end of GetFlashParms

//===========================================================================
// ultohex - convert unsigned long to 8 hex ASCII characters in a character array
//
//===========================================================================
void ultohex(char *dest, unsigned long ul)
{

  char hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

  char *pn;
  unsigned long nibble;

  pn= dest + 7;
  
  for(int i = 0; i < 8; i++)
  {

    // get nibble 
    //
    nibble = (ul & 0xF);
    *pn = hex[nibble];

    // move to next nibble
    //
    ul = ul >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ultohex

//===========================================================================
// ustohex - convert unsigned short to 4 hex ASCII characters in a character array
//
//===========================================================================
void ustohex(char *dest, unsigned short us)
{

  char hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

  char *pn;
  unsigned long nibble;

  pn= dest + 3;
  
  for(int i = 0; i < 4; i++)
  {

    // get nibble 
    //
    nibble = (us & 0xF);
    *pn = hex[nibble];

    // move to next nibble
    //
    us = us >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ustohex



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  INTERRUPT ROUTINES
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


  digitalWrite(LED_PIN,false);
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
// ISR for Timer 4 input capture interrupt - capture 1PPS input 
//==================================================================
ISR( TIMER4_CAPT_vect)
{

  long count4;
  long count5;
  long diff; 
  unsigned long LED_time;

  //******************
  // Check for start/end of LED pulse
  //
  if (Pulse_Duration <= 0)
  {
	  // no more flashes left
	  //
    digitalWrite(LED_PIN,false);            // LED OFF
    LED_time = GetTicks(CNT4);              // time LED turned OFF
    LED_ON = false;
    pps_flash = false;                      // flash OFF at this PPS
  }
  else
  {
    // 
    // Pulse active => LED ON
    //
    digitalWrite(LED_PIN,true);             // LED ON 
    LED_time = GetTicks(CNT4);              // time LED turned ON
    pps_flash = true;

    if (!LED_ON)
    {
      // LED Pulse begins
      //
      LED_ON = true;
    }

    // one less second left in this pulse
    //
    Pulse_Duration--;

  } // end of turning on LED


  //************************
  // Get TIME of PPS event from input capture
  //
  pps_time = GetTicks(IC4);
  pps_new = true;

  //***********************
  // increment pps count
  //
  pps_count++;

  //****************************
  // NOW do error checking
  //
  
  // check to make sure timer 4 and 5 are similar
  //   if they differ by more than 50 us, report an error - NOT sure this always works...
  //
  count4 = long(TCNT4);
  count5 = long(TCNT5);
  diff = count5 - count4;
  if ( (diff > 100) || (diff < -100) )
  {
    
    timer45_error = true;
    timer45_values = ((unsigned long)count5 << 16) + (unsigned long)count4;
    
  } // end of check on timer 4 /5 sync
  
} // end of Timer4 input capture interrupt

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
//  cTime = ((unsigned long)timer5_ov << 16) + (unsigned long)ICR5;
  cTime = GetTicks(IC5);
  E_SaveToBuffer(cTime); 
  
} // end of Timer5 input capture interrupt


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


//**********************************************************************************************************
// NEO 6 GPS routines
//**********************************************************************************************************
#define gpsSerial Serial1
#define gps_OK        0
#define gps_E_RMC     1
#define gps_E_GGA     2
#define gps_E_DTM     3
#define gps_E_PUBX04  4
#define gps_E_CFGTP   5
#define gps_E_CFGDTM  6

//=================================================
//  gpsInit - initialize GPS device
//
//  This version is for the UBX NEO-6 gps module
//    * uses timer4_ov for timeout detection
//    * assumes NEO-6 boots to 9600 baud
//    * initializes NEO-6 to output the following SENTENCES on the serial port
//        $GPRMC
//        $GPGGA
//        $GPDTM
//        $PUBX,04
//
// return 0 on success
//    
//=================================================
int gpsInit()
{
  uint8_t enableRMC[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01};       // Set GPRMC rate on current target
  uint8_t enableGGA[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01};       // Set GPGGA rate on current target
  uint8_t enableDTM[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x0A, 0x01};       // Set GPDTM rate on current target
  uint8_t enablePUBX04[] = {0x06, 0x01, 0x03, 0x00, 0xF1, 0x04, 0x01};    // Set PUBX,04 rate on current target

  uint8_t configTimepulse[] = {0x06, 0x07, 0x14, 0x00,        //   configure timepulse
                          0x40, 0x42, 0x0F, 0x00,             // time interval = 1,000,000 us
                          0xA0, 0x86, 0x01, 0x00,             // pulse length = 100,000 us = 100ms
                          0x01,                               // positive pulse
                          0x00,                               // align to UTC
                          0x00,                               // time pulse only when sync'd to valid GPS
                          0x00,                               // reserved
                          0x00, 0x00,                         // Antenna cable delay
                          0x00, 0x00,                         // Receiver RF group delay
                          0x00, 0x00, 0x00, 0x00              // user time function delay
                          };
                          
  // 9600 NMEA is the default rate for the GPS
  //
  gpsSerial.begin(9600);
  
  //********************************
  //  TURN OFF everything to keep the serial port quiet
  //
  gpsSerial.println("$PUBX,40,RMC,0,0,0,0,0,0*46");   // RMC OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GGA,0,0,0,0,0,0*5A");   // GGA OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,DTM,0,0,0,0,0,0*46");   // DTM OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");   // VTG OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");   // GSA OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GSV,0,0,0,0,0,0*59");   // GSV OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");   // GLL OFF
  gpsSerial.flush(); // wait for it...

  // empty any pending incoming data
  //
  while (gpsSerial.available())
    gpsSerial.read();

  //*********************************
  //  TURN ON sentences that we want
  //    GPRMC
  //    GPGGA
  //    GPDTM
  //    PUBX,04
  //
  ubxSend(enableRMC,sizeof(enableRMC)/sizeof(uint8_t));
  if (!ubxGetAck(enableRMC))
  {
    return gps_E_RMC;
  }

  
  ubxSend(enableGGA,sizeof(enableGGA)/sizeof(uint8_t));
  if (!ubxGetAck(enableGGA))
  {
    return gps_E_GGA;
  }

  ubxSend(enableDTM,sizeof(enableDTM)/sizeof(uint8_t));
  if (!ubxGetAck(enableDTM))
  {
    return gps_E_DTM;
  }

  ubxSend(enablePUBX04,sizeof(enablePUBX04)/sizeof(uint8_t));
  if (!ubxGetAck(enablePUBX04))
  {
    return gps_E_PUBX04;
  }

  // configure timepulse
  //
  ubxSend(configTimepulse,sizeof(configTimepulse)/sizeof(uint8_t));
  if (!ubxGetAck(configTimepulse))
  {
    return gps_E_CFGTP;
  }
 
  return gps_OK;
  
} // end of gpsInit

//=================================================
//  ubxSend
//
//  Send a UBX command to the GPS
//    MSG = byte array containing the UBX command data
//    len = # of bytes in MSG
//=================================================
void ubxSend(uint8_t *MSG, uint32_t len) 
{

  uint32_t CK_A = 0, CK_B = 0;
  uint8_t sum1=0x00, sum2=0x00;
  uint8_t ubxPacket[len+4];


  // UBX prefixes
  //
  ubxPacket[0]=0xB5;
  ubxPacket[1]= 0x62;
  
  for(int i=0; i<len; i++) 
  {
    ubxPacket[i+2]=MSG[i];
  }

  // Calculate checksum
  //
  for(int i=0; i<len; i++)
  {
    CK_A = CK_A + MSG[i];
    CK_B = CK_B + CK_A;
  }
  sum1 = CK_A &0xff;//Mask the checksums to be one byte
  sum2= CK_B &0xff;

  // add the checksum to the end of the UBX packet
  //
  ubxPacket[len+2]=sum1;
  ubxPacket[len+3]=sum2;
 
  // send the UBX command
  //
  gpsSerial.write(ubxPacket,len+4);
  gpsSerial.flush();  // wait for it ...
  
}   // end of sendUBX

//=================================================
//    ubxGetAck - wait for ack after command
//
//=================================================
bool ubxGetAck(uint8_t *MSG)
{
  
  uint8_t b;
  uint8_t ackByteID;
  uint8_t ackPacket[10];
  uint8_t ackReceived[10];
  long startTime;
  long currentTime;
  long waitTime;
  uint32_t CK_A=0, CK_B=0;

  //*********************************
  // Construct the expected ACK packet    
  //
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[0];    // MGS class
  ackPacket[7] = MSG[1];    // MSG id
  ackPacket[8] = 0;     // CK_A
  ackPacket[9] = 0;     // CK_B

  // checksums
  //
  for (uint8_t i=2; i<8; i++) 
  {
    CK_A = CK_A + ackPacket[i];
    CK_B= CK_B + CK_A;
  }

  ackPacket[8]= CK_A &0xff;//Mask the checksums to be one byte
  ackPacket[9]= CK_B &0xff;

  //***********************************************
  // get the 10 byte ack packet (with timeout)
  //  wait for first header byte of ACK (0xB5) - should arrive within timeout period..
  //    
  //
  startTime = (long)timer4_ov;
 
  
  for( int i = 0; i < 10;  )
  {

    // Timeout if no ACK in 500ms
    //
    //
    currentTime = (long)timer4_ov;
    waitTime = currentTime - startTime;
    if (waitTime < 0)
    {
      waitTime += 65536;    // adjust for rollover
    }

    //  timer4_ov runs at 32ms per "tick"
    //
    if (waitTime > (500 / 32))    // about 500 ms
    {
      return false;   // timeout
    }

    // got a byte?
    //   if starting header, save it and continue
    //
    if (gpsSerial.available()) 
    {
      b = gpsSerial.read();
      ackReceived[i] = b;
      if (i != 0)
      {
        i++;    // next byte
      }
      else
      {
        // i == 0
        // we are looking for the first byte of the header
        //  move forward IFF the data bye matches the first header byte value
        if (b == 0xB5)
        {
          // ok, we have the starting header byte and can move forward
          //
          i++;
        }
      } // end of i== 0 check

    } // end of check for data available
    
  } // end of for loop to get the bytes

  //****************************
  //  check the packet
  //
  for( int i = 0; i < 10; i++ )
  {
    if (ackReceived[i] != ackPacket[i])
    {
      return(false);
    }
  }  // end of checking the received packet

  //*********************
  //  all OK
  //
  return( true );
    
} // end of ubxAck

//===========================================================================
// DEBUG routines
//===========================================================================
void SendBuffer(char* cBuffer, int bufferSize)
{
  char c;
  char hex[10] = {0,0,0,0,0,0,0,0,0,0};

  for( int i = 0; i < bufferSize; i++ )
  {
    c = cBuffer[i];

    if (isPrintable(c))
    {
      Serial.print(c);                   // output non-null char to std serial port     
    }
    else if (c == 0)
    {
      Serial.print("<0>");
    }
    else if (c == '\n')
    {
      Serial.print("<n>");
    }
    else
    {
      ultohex(hex,(unsigned long)c);
      Serial.print('<');
      Serial.print(hex+6);
      Serial.print('>');
    }
  }

} // end of SendBuffer

void DebugBuffers()
{

  // S_Buffer
  //
  Serial.print("S_Buffer: Count, Cur, Head, Tail :");
  Serial.print(S_Count);
  Serial.print(", ");
  Serial.print(S_Current);
  Serial.print(", ");
  Serial.print(S_BufferHead);
  Serial.print(", ");
  Serial.print(S_BufferTail);
  Serial.println("");
  for (int i = 0; i < S_BufferSize; i++)
  {
    Serial.print(S_Buffer[i].RecvTime,HEX);
    Serial.print(", ");
    Serial.print(S_Buffer[i].IdxFirst);
    Serial.println("");
  }

  // N_Buffer
  //
  Serial.print("N_Buffer: Count, Head, Tail : ");
  Serial.print(N_Count);
  Serial.print(", ");
  Serial.print(N_BufferHead);
  Serial.print(", ");
  Serial.print(N_BufferTail);
  Serial.println("");
  SendBuffer((char *)N_Buffer,N_BufferSize);
  Serial.println("");
  
}
