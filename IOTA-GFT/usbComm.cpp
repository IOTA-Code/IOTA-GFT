/*
  usbComm

  Routines for parsing and executing commands sent to this flashing device via USB.  

*/

#include <Arduino.h>
#include "gpsComm.h"
#include "iota-gft.h"
#include "ublox.h"
#include "logger.h"
#include "usbComm.h"

//===========================================================================================================
//
//  GLOBAL variables and definitions
//
//===========================================================================================================

//****************************
// Command string from the PC
//
#define CommandSize 100               // max size of a command line
char strCommand[ CommandSize + 1 ];   // null terminated command line
int Cmd_Next;                         // speed optimization ( -1 => full command pending )

char idxToken[10];                    // starting index of tokens in command string - separated by spaces
int lenToken[10];                     // length of the non-blank chars in this token
int tokenCount;                       // # of tokens in string

//
// Flash settings
//
int Flash_Duration_Sec = 5;       // duration of one LED "flash" in seconds
int Pulse_Count = 5;              // # of pulses in EXP sequence


char strChk[4] = "*XX";
char strDONE[] = "[DONE]*06";   // fixed string response
char strParseError[] = "[ERROR: unable to parse command.]*52";

//===========================================================================================================
//
//   MISC FUNCTIONS
//
//===========================================================================================================

//--------------------------------------------------------------------------------------
//  ClearInputBuffer - read/clear pending data from serial port
//    Note: this code causes a delay of at least 100ms
//--------------------------------------------------------------------------------------
void ClearSerialInput()
{


  // dumb but should work to clear random data pending from the PC
  //
  for (int i = 0; i < 10; i++)
  {
    while ( Serial.available() > 0 )
    {
      Serial.read();

    } //end of reading pending data

    delay(10);
  }

} // end of ClearSerialInput

//--------------------------------------------------------------------------------------
//  FindTokens - find non-space tokens in null terminated command string
//--------------------------------------------------------------------------------------
void FindTokens()
{
  int idx;

  // init
  //
  tokenCount = 0;
  idx = 0;

  // sanity check
  //
  if (strCommand[0] == 0)
  {
    // empty command line - just leave now
    //
    return;
  }

  // walk through command string looking for non-blank tokens
  //
  while( strCommand[idx] != 0 )
  {
    // skip all blank chars to find start of a token
    //
    while (strCommand[idx] == ' ')
    {
      idx++;
    }

    // first non-blank char is null => end of string
    //    => no more tokens => leave now
    //
    if (strCommand[idx] == 0)
    {
      return;
    }

    // found a non-blank char in the string => start of new token
    //
    idxToken[tokenCount] = idx;   // save starting index of this token
    tokenCount++;                 // new token => bump count

    // now consume all non-blank chars to find end of this token
    //
    while( strCommand[idx] != ' ')
    {
      if (strCommand[idx] == 0)
      {
        // end of string => end of token, exit the loop
        //
        break;
      }
      idx++;
    }

    // we have reached the end of this token
    // record length of this token
    //
    lenToken[tokenCount-1] = (idx - idxToken[tokenCount-1]);

    // if we have room, find next token
    //
    if (tokenCount == 10)
    {
      // we only have room for 10 tokens, leave now
      //
      return;
    }

  } // end of loop through all chars of string

  // all done
  //
  return;

} // end of FindTokens

//===========================================================================================================
//
//   MANUAL OPERATIONS FUNCTIONS
//
//===========================================================================================================

//=====================================
//  ReadCMD - get input from USB port
//  
//  pick up single line "commands" sent via the USB port
//  after reading \n (newline), sets Cmd_Next = -1 and EXECUTE COMMAND
//
//  COMMANDS:
// general commands
//    status - get current device status
//    device - get device name
//    version - get version info for this device
//    null - ignore this command string if it includes the word "null" - useful for clearing the input buffer
//
// flash commands
//    flash now - start flash sequence at the next PPS or EXP pulse
//    flash duration X - X is seconds in PPS mode or starting number of pulses in EXP mode
//    flash level X - get/set the current flash intensity level [ 0 to 255]
//    flash range [X] -get/set the current range of flash intensity [ 0 to 2]
//    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)
//
//    pulse duration [X] - get/set the flash pulse duration (us)
//    pulse interval [X] - get/set the flash pulse interval = # of exp interrupts between pulse sequences
//
//    led [ON | OFF] - turn the LED on/off right now
//
// logging commands
//    log [on | off ] - enable disable data logging (default = ON)
//
//  OUTPUTS:
//    All commands return two sentences.  Both sentences will be terminated with a CRC code. 
//    The first sentence is an echo of the command received.
//    If the command succeeds, the second sentence will be one of two forms:
//      [DONE] for commands which do not return a value (e.g. commands that set a value)
//      [command: value] where command represents the type of value returned and value is the value returned.
//    If the command fails, the second sentence will be of the form [ERROR: error description]
//
//======================================
void ReadCMD()
{

  byte bIn;
  int idx;
  long lTmp;
  
  // read command from PC
  //  - one byte at a time
  //  - converts all characters to lower case!
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
    
    
    // ok -> get the character & save it
    //   but don't save CR or LF
    //
    bIn = Serial.read();
      
    // if \n, end of command line
    //
    if (bIn == '\n')
    {
      strCommand[Cmd_Next] = 0;   // end of a command
      Cmd_Next = -Cmd_Next;       // set to negative length value
      break;                      // and exit serial read loop to process command
    }
    else if (bIn != '\r')
    {
      // here if NOT \r or \n
      //
      strCommand[ Cmd_Next ] = tolower(bIn);
      Cmd_Next++;
    }
    // if \r, do nothing with char
    
  } // end of loop reading pending data


  //*******************************************************
  //  if we have a full command, execute the command now
  //
  if (Cmd_Next < 0)
  {
    int Cmd_Length = -Cmd_Next;     // length of this command

    // get ready for next command
    //
    Cmd_Next = 0;

    // nothing there?
    //
    if (Cmd_Length == 0)
    {
      return;     // do nothing
    }

    // first check to see if this is a "null" command
    //
    if (strstr(strCommand,"null") != NULL )
    {
      // command does contain "null", ignore this command string
      //
      return;
    }

    // ECHO command string to USB port
    //    
    strcpy(strResponse,"[CMD ");
    strcpy(strResponse+5,strCommand);
    strcat(strResponse,"]");
    SendResponse();

    // check for checksum in form *XX at end of sentence
    //
    if (Cmd_Length >= 3)
    {
      //  if * char in expected position, assume checksum is present and verify it
      //
      if (strCommand[Cmd_Length-3] == '*')
      {
        byte chk;

        // compute checksum of previous chars
        //
        chk = chksum_b(strCommand,Cmd_Length-3);      // compute checksum of command & convert to lowercase
        btohexA(strChk+1, chk);
        strChk[1] = tolower(strChk[1]);
        strChk[2] = tolower(strChk[2]);

        // does it match value in command string?
        //
        if (!(strCommand[Cmd_Length-2] == strChk[1]) || !(strCommand[Cmd_Length-1] == strChk[2]))
        {
          Serial.println("[ERROR: checksum error.]*3F");
          return;
        }

        // checksum passed.  Now remove the checksum
        //
        Cmd_Length -= 3;
        strCommand[Cmd_Length] = 0;   // null-terminate it
        if (Cmd_Length == 0)
        {
          return;     // do nothing on empty command
        }

      }
    }

    // finds tokens in the command line
    //
    FindTokens();

    // if no tokens, ignore the command
    //
    if (tokenCount == 0)
    {
      return;
    }

    //***************
    //  Which command?
    //   first token => type of command
    //

    //-----------------------------
    // MANUAL Operations Commands
    //

    idx = idxToken[0];        // start of first token


    //  * device - get device name
    //
    if (strncmp(strCommand+idx,"device", 6) == 0)
    {
      strcpy(strResponse,"[device: ");
      strcat(strResponse,strDeviceName);
      strcat(strResponse,"]");
      SendResponse();
      return;
    }   // end of version command

    //  * version - get version info
    //
    else if (strncmp(strCommand+idx,"version", 7) == 0)
    {
      strcpy(strResponse,"[version: ");
      strcat(strResponse,strVersion);
      strcat(strResponse,"]");
      SendResponse();
      return;
    }   // end of version command

    //  * mode - get current operating mode
    //
    else if (strncmp(strCommand+idx,"status", 6) == 0)
    {
      strcpy(strResponse,"[mode: ");
      switch(DeviceMode)
      {
        case InitMode :
          strcat(strResponse,"InitMode");
          break;
        case WaitingForGPS :
          strcat(strResponse,"WaitingForGPS");
          break;
        case TimeValid :
          strcat(strResponse,"TimeValid");
          break;
        case Syncing :
          strcat(strResponse,"Syncing");
          break;
        case FatalError :
          strcat(strResponse,"FatalError");
          break;
      }
      strcat(strResponse,"]");
      SendResponse();
      return;

    }   // end of mode command

    //--------------------
    // flash commands
    //    flash now
    //    flash duration [X] - get/set the current PPS flash duration (seconds)
    //    flash level [X] - get/set the current flash intensity level [ 0 to 255]
    //    flash range [X] -get/set the current range of flash intensity [ 0 to 2]
    //    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)
    //
    else if (strncmp(strCommand+idx,"flash", 5) == 0)
    {
      if (tokenCount < 2)
      {
        // should be at least two tokens...
        Serial.println(strParseError);
        return;
      }

      idx = idxToken[1];    // second token

      // Flash now?
      //
      if (strncmp(strCommand+idx,"now",3) == 0)
      {
        // turn on flash now
        //
        Serial.println(strDONE);
        PPS_Flash_Countdown_Sec = Flash_Duration_Sec;
        return;
      } // end of "flash now"

      //  * Flash Duration [X] - get/set the current flash duration 
      //
      else if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get duration value amd return it
          //
          strcpy(strResponse,"[flash duration: ");
          itoa(Flash_Duration_Sec,strResponse+17,10);
          strcat(strResponse,"]");
          SendResponse();
          return;
        }

        // assume we are trying to set the flash duration
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println(strParseError);
          return;
        }

        // set the value
        //
        if (FlashMode == PPS)
        {
          Flash_Duration_Sec = lTmp;          
        }
        else if (FlashMode == EXP)
        {
          Pulse_Count = lTmp;          
        }
        else
        {
          strcpy(strResponse,"[ERROR: unknown flash mode]");
          SendResponse();
        }
        Serial.println(strDONE);
        return;

      } // end of "flash duration "

      //  * Flash level [X] - get/set the current flash level 
      //
      else if (strncmp(strCommand+idx,"level",5) == 0)
      {
        unsigned int sReg;

        if (tokenCount < 3)
        {
          // Get flash level and return it
          //
          strcpy(strResponse,"[flash level: ");
          itoa(flashlevel,strResponse+14,10);
          strcat(strResponse,"]");
          SendResponse();
          return;
        }

        // assume we are trying to set the flash level
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp < 0)
        {
          Serial.println(strParseError);
          return;
        }
        else if ((lTmp < 0) || (lTmp > 255))
        {
          strcpy(strResponse,"[ERROR: flashlevel not in range (0,255)]");
          SendResponse();
        }

        // set the value
        //
        flashlevel = lTmp;

        // if LED LED is ON, change the flash level now
        //

        // disable interrupts to ensure LED state doesn't change before setting flashlevel
        //
        sReg = SREG;
        noInterrupts();
        if (LED_ON)
        {
          OCR2A = OCR2B = flashlevel;
        }
        SREG = sReg;      // re-enable interrrupts

        Serial.println(strDONE);

        return;

      } // end of "flash level "

      //  * Flash range [ X ] - get/set the current flash intensity range (0 .. 2)
      //
      else if (strncmp(strCommand+idx,"range",5) == 0)
      {
        if (tokenCount < 3)
        {
          // Get range value and return it
          //
          strcpy(strResponse,"[flash range: ");
          itoa(flashrange,strResponse+14,10);
          strcat(strResponse,"]");
          SendResponse();
          return;
        }

        // more than two parameters, assume we are setting flash range
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        if (strCommand[idx] == '0')
        {
          flashrange = 0;
          setLEDtoLowRange();
        }
        else if (strCommand[idx] == '1')
        {
          flashrange = 1;
          setLEDtoMidRange();
        }
        else if (strCommand[idx] == '2')
        {
          flashrange = 2;
          setLEDtoHighRange();
        }
        else
        {
          Serial.println(strParseError);
          return;
        }

        Serial.println(strDONE);

        return;

      } // end of "flash range "

      // Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
      //
      else if (strncmp(strCommand+idx,"mode",4) == 0)
      {
        if (tokenCount < 3)
        {
          // Get flash mode and return it
          //
          if (FlashMode == PPS)
          {
            strcpy(strResponse,"[flash mode: PPS]");
          }
          else if (FlashMode == EXP)
          {
            strcpy(strResponse,"[flash mode: EXP]");
          }
          else
          {
            strcpy(strResponse,"[ERROR: Unknown flash mode!]");
          }
          SendResponse();
          return;
        }

        // 3 parameters => assume we are trying to set the flash mode
        //
        idx = idxToken[2];
        if (strncmp(strCommand+idx,"pps",3) == 0)
        {
          FlashMode = PPS;
          Serial.println(strDONE);
        }
        else if (strncmp(strCommand+idx,"exp",3) == 0)
        {
          FlashMode = EXP;
          Serial.println(strDONE);
        }
        else
        {
          Serial.println(strParseError);
        }
        return;

      } // end of "flash mode"


      else    // last catch-all segment of flash command parsing
      {
        strcpy(strResponse,"[ERROR: unknown command.]");
        SendResponse();
        return;
      }


    } // end of flash command logic


    //--------------------
    // pulse commands (for EXP flashing)
    //  * pulse duration [X] - get/set the flash pulse duration (us)
    //  * pulse interval [X] - get/set the flash pulse interval = # of EXP interrupts between pulse sequences
    //
    else if (strncmp(strCommand+idx,"pulse", 5) == 0)
    {

      // all pulse commands are at least 2 tokens 
      //
      if (tokenCount < 2)
      {
        // should be at least three tokens...
        Serial.println(strParseError);
        return;
      }

      idx = idxToken[1];    // second token

      //  * pulse duration [X] - get/set the current value
      //
      if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get pulse duration value (microseconds) and return it
          //
          strcpy(strResponse,"[pulse duration:  ");
          itoa(pulse_duration_us,strResponse+17,10);
          strcat(strResponse,"]");
          SendResponse();
          return;
        }

        // assume we are setting pulse duration
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println(strParseError);
          return;
        }

        // set the value
        //
        pulse_duration_us = lTmp;
        OCR3A_pulse = lTmp/Timer3_us;       // set timer3 duration to pulse duration
        Serial.println(strDONE);

        return;

      } // end of "pulse duration "
      else if (strncmp(strCommand+idx,"interval",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get pulse interval value (milliseconds) and return it
          //
          strcpy(strResponse,"[pulse interval:  ");
          itoa(pulse_interval,strResponse+17,10);
          strcat(strResponse,"]");
          SendResponse();
          return;
        }

        // assume we are setting pulse interval
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println(strParseError);
          return;
        }

        // set the value
        //
        pulse_interval = lTmp;
        Serial.println(strDONE);

        return;

      } // end of "pulse duration "

      else    // last catch-all segment of pulse command parsing
      {
        strcpy(strResponse,"[ERROR: unknown command.]");
        SendResponse();
        return;
      }

    } // end of pulse command logic

    //--------------------
    // LED flash commands
    //    led [on | off ] - turn ON / OFF LED
    //
    else if (strncmp(strCommand+idx,"led", 3) == 0)
    {
      // "led" command
      //
      unsigned long tk_LED;

      // parse second token 
      //
      if (tokenCount < 2)
      {
        // should be at least two tokens...
        Serial.println(strParseError);
        return;
      }

      idx = idxToken[1];    // second token

      // turn LED on
      //  NOTE: if LED already ON, do nothing and no error
      //
      if (strncmp(strCommand+idx,"on",2) == 0)
      {


        // if LED OFF, turn it ON
        //
        if (!LED_ON)
        {
          byte chk;

          // disable interrupts to ensure accurate time for LED ON
          //
          noInterrupts();
          
          // turn on LED
          //
          OCR2A = OCR2B = flashlevel;
          LED_ON = true;

          // log time LED went ON
          //
          tk_LED = GetTicks(CNT4);              // time LED turned ON
          ultohexA(logFlashON + offset_logFlashON,tk_LED);
          chk = chksum_b(logFlashON,chksum_logFlashON-1);   // compute checksum
          btohexA(logFlashON + chksum_logFlashON, chk);
          LogTextWrite(logFlashON,len_logFlashON);

          // interrupts back on again...
          //
          interrupts();
        }

        Serial.println(strDONE);
        return;
      }

      // turn LED off
      // IF LED already OFF, do nothing and no error
      //
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {

        // if LED already OFF, do nothing
        //
        if (LED_ON)
        {
          byte chk;

          // disable interrupts to ensure accurate time for LED OFF
          //
          noInterrupts();

          // turn OFF LED 
          //
          OCR2A = OCR2B = 0;
          LED_ON = false;

          // log time flash went off
          //
          tk_LED = GetTicks(CNT4);              // time LED turned OFF

          ultohexA(logFlashFINAL + offset_logFlashFINAL,tk_LED);
          chk = chksum_b(logFlashFINAL,chksum_logFlashFINAL-1);   // compute checksum
          btohexA(logFlashFINAL + chksum_logFlashFINAL, chk);
          LogTextWrite(logFlashFINAL,len_logFlashFINAL);
          
          // interrupts back on again...
          //
          interrupts();
        }

        Serial.println(strDONE);
        return;
      }

      else    // last catch-all segment of LED command parsing
      {
        strcpy(strResponse,"[ERROR: unknown command.]");
        SendResponse();
        return;
      }
    } // end of LED commands

    //--------------------
    // logging commands
    //    log [on | off ] - enable disable data logging (default = ON)
    //
    else if (strncmp(strCommand+idx,"log", 3) == 0)
    {
      // "log" command
      //

      // parse second token 
      //
      if (tokenCount < 2)
      {
        // should be at least two tokens...
        Serial.println(strParseError);
        return;
      }

      idx = idxToken[1];    // second token

      // log on
      if (strncmp(strCommand+idx,"on",2) == 0)
      {
        // "log ON"
        blnLogEnable = true;    // turn ON logging
        Serial.println(strDONE);
        return;
      }

      // log off
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {
        //  "log OFF"
        //    & flush buffers!
        blnLogEnable = false;   // turn OFF logging
        LogFlushAll();          // flush out pending data
        Serial.println(strDONE);

        return;
      }

      else // end of log command parsing
      {
        Serial.println(strParseError);
        return;
      }

    } // end of log commands

    else    // end of command parsing
    {
      strcpy(strResponse,"[ERROR: unknown command.]");
      SendResponse();
      return;
    } // end of command options

  } // end of handling a pending command


} // end of ReadCMD

