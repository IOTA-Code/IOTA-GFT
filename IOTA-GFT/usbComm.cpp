/*
  usbComm

  Routines for parsing and executing commands sent to this flashing device via USB.  T

  Manual operations allow the user to set/get settings for the general device operation and manually fire off flash sequence. The current operations are:
    * mode - get current device operating mode
    * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb comm port.
    * ? or HELP - list all commands


    * Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
    * Flash Duration [X] - get/set the current flash duration (seconds in PPS mode or pulses in EXP mode)
    * Flash Level [X] - get/set the current flashlevel = percent flash PWM (0 - 100)
    * Flash Now - execute a flash sequence now
 
    * Flash Interval [X] - get/set the interval between flash pulses (ms) for EXP mode
    * Flash Count [X] - get/set the number of flashes in a sequence for EXP mode

*/

#include <Arduino.h>
#include "gpsComm.h"
#include "iota-gft.h"
#include "ublox.h"
#include "logger.h"
#include "usbComm.h"
#include "SdFat.h"

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

  // Flash settings
int Flash_Duration_Sec = 5;       // duration of one LED "flash" in seconds
int Pulse_Count = 5;              // # of pulses in EXP sequence
char strDONE[] = "[DONE]";

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

  byte bIn;

  // dumb but should work to clear random data pending from the PC
  //
  for (int i = 0; i < 10; i++)
  {
    while ( Serial.available() > 0 )
    {
      bIn = Serial.read();

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
  char c;

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
//    echo on | off - enable/disable echo of the GPS NMEA data to the usb port.
//    status - get current device status
//    device - get device name
//    version - get version info for this device
//    null - ignore this command string if it includes the word "null" - useful for clearing the input buffer
//
// flash commands
//    flash now
//    flash duration X - X is seconds in PPS mode or total number of pulses in EXP mode
//    flash level X - X is the percent PWM flash level ( 0 - 100 )
//    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)
//    pulse duration [X] - get/set the flash pulse duration (us)
//    pulse interval [X] - get/set the flash pulse interval (ms)
//
// logging commands
//    log [on | off ] - enable disable data logging (default = ON)
//
//======================================
void ReadCMD()
{

  byte bIn;
  int idx;
  int iTmp;
  long lTmp;
  bool blnTmp;
  char strTmp[50];
  
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
      strCommand[Cmd_Next] = 0;
      Cmd_Next = -1;
      break;
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

    // get ready for next command
    //
    Cmd_Next = 0;

    // first check to see if this is a "null" command
    //
    if (strstr(strCommand,"null") >0 )
    {
      // command does contain "null", ignore this command string
      //
      return;
    }

    // ECHO command string to USB port
    //    
    Serial.print("[CMD ");
    Serial.print(strCommand);
    Serial.print("]\r\n");

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
      Serial.print("[");
      Serial.print(strDeviceName);
      Serial.println("]");
      return;
    }   // end of version command

    //  * version - get version info
    //
    else if (strncmp(strCommand+idx,"version", 7) == 0)
    {
      Serial.print("[");
      Serial.print(strVersion);
      Serial.println("]");
      return;
    }   // end of version command

    //  * mode - get current operating mode
    //
    else if (strncmp(strCommand+idx,"status", 6) == 0)
    {
      Serial.print("[");
      switch(DeviceMode)
      {
        case InitMode :
          Serial.print("InitMode");
          break;
        case WaitingForGPS :
          Serial.print("WaitingForGPS");
          break;
        case TimeValid :
          Serial.print("TimeValid");
          break;
        case Syncing :
          Serial.print("Syncing");
          break;
        case FatalError :
          Serial.print("FatalError");
          break;
      }
      Serial.println("]");      
      return;

    }   // end of mode command

    //  * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb port.
    //
    else if (strncmp(strCommand+idx,"echo", 4) == 0)
    {
      if (tokenCount < 2)
      {
        // should be two tokens...
        Serial.println("[ERROR: unable to parse command.]");
        return;
      }
      idx = idxToken[1];    // second token
      if (strncmp(strCommand+idx,"on",2) == 0)
      {
        // turn on echo of NMEA data
        //
        blnEchoNMEA = true;
        Serial.println(strDONE);
        return;
      }
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {
        // turn OFF echo
        //
        blnEchoNMEA = false;
        Serial.println(strDONE);
        return;
      }
      else
      {
        // format error
        //
        Serial.println("[ERROR: unable to parse command.]");
        return;
      }
    }   // end of echo command



    //--------------------
    // flash commands
    //    flash now
    //    flash duration X - X is seconds in PPS mode
    //    flash level X - X is the percent PWM flash level ( 0 - 100 )
    //    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)
    //    flash time  - list current flash times
    //    TBD flash time [YYYY-MM-DD HH:MM:SS] - set the time for a future flash
    //
    else if (strncmp(strCommand+idx,"flash", 5) == 0)
    {
      if (tokenCount < 2)
      {
        // should be at least two tokens...
        Serial.println("[ERROR: unable to parse command.]");
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
      else if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get duration value
          Serial.print("[flash duration: ");
          Serial.print(Flash_Duration_Sec);
          Serial.println("]");
          return;
        }

        // assume we are setting duration
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println("[ERROR: unable to parse command.]");
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
          Serial.println("[ERROR: unknown flash mode.]");
        }
        Serial.println(strDONE);
        return;

      } // end of "flash duration "

      //  * Flash level [X] - get/set the current flash level 
      else if (strncmp(strCommand+idx,"level",5) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("[flash level: ");
          Serial.print(flashlevel);
          Serial.println("]");
          return;
        }

        // assume we are setting level
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println("[ERROR: unable to parse command.]");
          return;
        }
        else if ((lTmp < 0) || (lTmp > 100))
        {
          Serial.println("[ERROR: flashlevel not in range (0,100).]");
        }

        // set the value
        //
        flashlevel = lTmp;
        OCR2B = map(flashlevel, 0, 100, 0, OCR2Alevel);  		// adjust value in OCR2B register for PWM
        Serial.println(strDONE);

        return;

      } // end of "flash level "

      // Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
      else if (strncmp(strCommand+idx,"mode",4) == 0)
      {
        if (tokenCount < 3)
        {
          // Get duration value
          if (FlashMode == PPS)
          {
            Serial.println("[PPS flash mode.]");
            return;
          }
          else if (FlashMode == EXP)
          {
            Serial.println("[EXP flash mode.]");
            return;
          }
          else
          {
            Serial.println("[ERROR: Unknown flash mode!]");
            return;
          }
          return;
        }

        // 3 parameters => assume we are trying to set the mode
        //
        idx = idxToken[2];
        if (strncmp(strCommand+idx,"pps",3) == 0)
        {
          FlashMode = PPS;
          Serial.println(strDONE);
          return;
        }
        else if (strncmp(strCommand+idx,"exp",3) == 0)
        {
          FlashMode = EXP;
          Serial.println(strDONE);
          return;
        }
        else
        {
          Serial.println("[ERROR: unable to parse command.]");
          return;
        }

      } // end of "flash mode"


      else
      {
        Serial.println("[ERROR: unknown command.]");
        return;
      }


    } // end of flash command logic

    //--------------------
    // pulse commands (for EXP flashing)
    //  * pulse duration [X] - get/set the flash pulse duration (us)
    //  * pulse interval [X] - get/set the flash pulse interval (ms)
    //
    else if (strncmp(strCommand+idx,"pulse", 5) == 0)
    {

      // all pulse commands are at least 2 tokens 
      //
      if (tokenCount < 2)
      {
        // should be at least three tokens...
        Serial.println("[ERROR: unable to parse command.]");
        return;
      }

      idx = idxToken[1];    // second token

      //  * pulse duration [X] - get/set the current value
      if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("[pulse duration (us): ");
          Serial.print(pulse_duration_us);
          Serial.println("]");
          return;
        }

        // assume we are setting duration
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println("[ERROR: unable to parse command.]");
          return;
        }

        // set the value
        //
        pulse_duration_us = lTmp;
        OCR3A_pulse = lTmp/Timer3_us;       // set timer3 duration to pulse duration
        Serial.println(strDONE);

        return;

      } // end of "pulse interval "
      else if (strncmp(strCommand+idx,"interval",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("[pulse interval (ms): ");
          Serial.print(pulse_interval);
          Serial.println("]");
          return;
        }

        // assume we are setting interval
        //
        idx = idxToken[2];

        // try to parse the value
        //  should work without null terminating the token...
        //
        lTmp = atol(strCommand+idx);
        if (lTmp <= 0)
        {
          Serial.println("[ERROR: unable to parse command.]");
          return;
        }

        // set the value
        //
        pulse_interval = lTmp;
        Serial.println(strDONE);

        return;

      } // end of "pulse duration "

    } // end of pulse command logic

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
        Serial.println("[ERROR: unable to parse command.]");
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

      else
      {
        Serial.println("[ERROR unable to parse command.]");
        return;
      }

    } // end of log commands

    else
    {
      Serial.println("[ERROR unknown command.]");
      return;
    } // end of command options

  } // end of handling a pending command


} // end of ReadCMD

