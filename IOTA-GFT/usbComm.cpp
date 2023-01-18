/*
  usbComm

  Routines for parsing and executing commands sent to this flashing device via USB.  These routines are divided into
  three sections: manual operations, event file operations, log file operations.

  Manual operations allow the user to set/get settings for the general device operation and manually fire off flash sequence. The current operations are:
    * mode - get current device operating mode
    * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb comm port.
    * ? or HELP - list all commands

    * Camera [generic, shutter] - get/set the camera type 

    * Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
    * Flash Duration [X] - get/set the current flash duration 
    * Flash Level [X] - get/set the current flashlevel = percent flash PWM (0 - 100)
    * Flash Now - execute a flash sequence now
    * Flash Time [YYYY-MM-DD HH:MM:SS] - set the time for a future flash
    * Flash Time Clear - clear all flash times
    * Flash Time  - list all flash times
 
    * Pulse Duration [X] - get/set the flash pulse duration (us)
    * Pulse Interval [X] - get/set the flash pulse interval (ms)

  Event file operations with set/get settings for future occultation event observations.  The event file specifies the timeframe for timing flashes associated with a future event.
  The event file commands to download the current event file from the device or upload a new event file to the device.
    * event file read
    * event file upload {file contents}

  Log file operations.  If a flash sequence occurs during after the device is powered up, the device creates a log file for recording all timing information while the device remains powered.
  With the log file commands, the user can list the log files, download log files, and delete log files on the device.
    * log file - list any log files
    * log file read filename
    * log file rm filename
    * log file save - savef/flush current log file to SD

*/

#include <Arduino.h>
#include <SdFat.h>
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

//*************************
//  Command mode
//
enum usbMode
{
  SingleLine,         // single line USB command
  EventUpload,        // uploading event file to device
  EventDownload,      // downloading event file from device
  LogFileDownload,    // downloading log file from device
  LogListDownload     // downloading list of log files from device
};
usbMode CurrentMode;      // current operating mode of USB comm

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
int Pulse_Duration_us = 5000;         // duration of one shutter (EXP) pulse in EXP mode
int Pulse_Interval_ms = 1000;         // interval between pulses

//===========================================================================================================
//
//   MISC FUNCTIONS
//
//===========================================================================================================


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
//    mode - get current device operating mode
//
// camera commands
//    camera [generic, shutter ] - get/set camera type
//
// flash commands
//    flash now
//    flash duration X - X is seconds in PPS mode
//    flash level X - X is the percent PWM flash level ( 0 - 100 )
//    flash mode [pps | exp ] - get/set the current flash mode (PPS or EXP)
//    flash time  - list current flash times
//    flash time [YYYY-MM-DD HH:MM:SS] - set the time for a future flash
//
// pulse commands
//    pulse duration [X] - get/set the flash pulse duration (us)
//    pulse interval [X] - get/set the flash pulse interval (ms)
//
// logging commands
//    log [on | off ] - enable disable data logging (default = ON)
//    log serial [on | off] - enable/disable data logging to serial port (default = OFF)
//    log file [on | off] - enable/disable data logging to sd card file (default = ON)
//    log file list - list log files
//    log file read filename - download (echo) log file "filename"
//    log file rm filename - delete log file named "filename"
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
    // ECHO command string to USB port
    //
    
    Serial.print("CMD <");
    Serial.print(strCommand);
    Serial.print(">\r\n");

    // get ready for next command
    //
    Cmd_Next = 0;

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

    //  * mode - get current operating mode
    //
    if (strncmp(strCommand+idx,"mode", 4) == 0)
    {

      switch(DeviceMode)
      {
        case InitMode :
          Serial.println("InitMode");
          break;
        case WaitingForGPS :
          Serial.println("WaitingForGPS");
          break;
        case TimeValid :
          Serial.println("TimeValid");
          break;
        case Syncing :
          Serial.println("Syncing");
          break;
        case FatalError :
          Serial.println("FatalError");
          break;
      }
      return;

    }   // end of mode command

    //  * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb port.
    //
    if (strncmp(strCommand+idx,"echo", 4) == 0)
    {
      if (tokenCount < 2)
      {
        // should be two tokens...
        Serial.println("ERROR: unable to parse command.");
        return;
      }
      idx = idxToken[1];    // second token
      if (strncmp(strCommand+idx,"on",2) == 0)
      {
        // turn on echo of NMEA data
        //
        blnEchoNMEA = true;
        return;
      }
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {
        // turn OFF echo
        //
        blnEchoNMEA = false;
        return;
      }
      else
      {
        // format error
        //
        Serial.println("ERROR: unable to parse command.");
        return;
      }
    }   // end of echo command

    //--------------------
    // camera commands
    //    camera [ generic, shutter ]
    //    
    else if (strncmp(strCommand+idx,"camera", 6) == 0)
    {
      if (tokenCount < 2)
      {
        // return current camera type
       switch(CameraType)
        {
          case generic :
            Serial.println("generic");
            break;
          case shutter :
            Serial.println("shutter");
            break;
          default:
            Serial.println("unknown");
            break;
        }
        return;
      }
    
      idx = idxToken[1];    // second token

      if (strncmp(strCommand+idx,"generic",7) == 0)
      {
        CameraType = generic;
        FlashMode = PPS;
        return;
      }
      else if (strncmp(strCommand+idx,"shutter",7) == 0)
      {
        CameraType = shutter;
        FlashMode = EXP;
        return;
      }
      else
      {
        Serial.println("ERROR: unable to parse command.");
        return;
      }


    } // end of camera command logic


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
        Serial.println("ERROR: unable to parse command.");
        return;
      }

      idx = idxToken[1];    // second token

      // Flash now?
      //
      if (strncmp(strCommand+idx,"now",3) == 0)
      {
        // turn on flash now
        //
        PPS_Flash_Countdown_Sec = Flash_Duration_Sec;
        return;
      } // end of "flash now"

      //  * Flash Duration [X] - get/set the current flash duration 
      else if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get duration value
          Serial.print("flash duration: ");
          Serial.println(Flash_Duration_Sec);
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
          Serial.println("ERROR: unable to parse command.");
          return;
        }

        // set the value
        //
        Flash_Duration_Sec = lTmp;
        return;

      } // end of "flash duration "

      //  * Flash level [X] - get/set the current flash level 
      else if (strncmp(strCommand+idx,"level",5) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("flash level: ");
          Serial.println(flashlevel);
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
          Serial.println("ERROR: unable to parse command.");
          return;
        }
        else if ((lTmp < 0) || (lTmp > 100))
        {
          Serial.println("ERROR: flashlevel not in range (0,100).");
        }

        // set the value
        //
        flashlevel = lTmp;
        OCR2B = map(flashlevel, 0, 100, 0, OCR2Alevel);  		// adjust value in OCR2B register for PWM

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
            Serial.println("PPS flash mode.");
            return;
          }
          else if (FlashMode == EXP)
          {
            Serial.println("EXP flash mode.");
            return;
          }
          else
          {
            Serial.println("ERROR: Unknown flash mode!");
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
          return;
        }
        else if (strncmp(strCommand+idx,"exp",3) == 0)
        {
          FlashMode = EXP;
          return;
        }
        else
        {
          Serial.println("ERROR: unable to parse command.");
          return;
        }

      } // end of "flash mode"

      // flash time ...
      else if (strncmp(strCommand+idx,"time",4) == 0)
      {
        // "flash time" command
        //

        // check for "flash time" with no args => list current flash times
        //
        if (tokenCount < 3)
        {
          char strTime[20] = "YYYY-MM-DD HH:MM:SS";
          int duration = Flash_Duration_Sec;

          Serial.println("Flash Times:");
          for( int i = 0; i < FT_Count; i++)
          {
            sprintf(strTime,"%04d-%02d-%02d %02d-%02d-%02d [%04d (sec)]",
                    FlashTimes[i].yyyy,FlashTimes[i].mon,FlashTimes[i].day,
                    FlashTimes[i].hh,FlashTimes[i].mm,FlashTimes[i].ss,duration);
            Serial.println(strTime);
          }
          Serial.println();
          return;
        }

        // now handle three token flash time commands
        //
        idx = idxToken[2];

        //  flash time clear - clear all flash times
        if (strncmp(strCommand+idx,"clear",5)==0)
        {
          FT_Count = 0;
          return;
        }

        //  * flash time [YYYY-MM-DD HH:MM:SS] - set the time for a future flash
        else 
        {
          // assume this is attempt to set a time
          //

        } // end of flash time X commands

      } // end of flash time commands

      else
      {
        Serial.println("ERROR: unknown command.");
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
        Serial.println("ERROR: unable to parse command.");
        return;
      }

      idx = idxToken[1];    // second token

      //  * pulse duration [X] - get/set the current value
      if (strncmp(strCommand+idx,"duration",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("pulse duration (us): ");
          Serial.println(Pulse_Duration_us);
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
          Serial.println("ERROR: unable to parse command.");
          return;
        }

        // set the value
        //
        Pulse_Duration_us = lTmp;

        return;

      } // end of "pulse interval "
      else if (strncmp(strCommand+idx,"interval",8) == 0)
      {
        if (tokenCount < 3)
        {
          // Get level value
          Serial.print("pulse interval (ms): ");
          Serial.println(Pulse_Interval_ms);
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
          Serial.println("ERROR: unable to parse command.");
          return;
        }

        // set the value
        //
        Pulse_Interval_ms = lTmp;

        return;

      } // end of "pulse duration "

    } // end of pulse command logic

    //--------------------
    // logging commands
    //    log [on | off ] - enable disable data logging (default = ON)
    //    log serial [on | off] - enable/disable data logging to serial port (default = OFF)
    //    log file [on | off] - enable/disable data logging to sd card file (default = ON)
    //    log file save - flush buffers to current log file
    //    log file list - list log files
    //    log file read filename - download (echo) log file "filename"
    //    log file rm filename - delete log file named "filename"
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
        Serial.println("ERROR: unable to parse command.");
        return;
      }

      idx = idxToken[1];    // second token

      // log on
      if (strncmp(strCommand+idx,"on",2) == 0)
      {
        // "log ON"
        blnLogEnable = true;    // turn ON logging
        return;
      }

      // log off
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {
        //  "log OFF"
        //    & flush buffers!
        blnLogEnable = false;   // turn OFF logging
        LogFlushAll();
        return;
      }

      // log serial
      else if (strncmp(strCommand+idx,"serial",4) == 0)
      {

        // "log serial" command, now parse third token
        //
        if (tokenCount < 3)
        {
          // should be at least three tokens...
          Serial.println("ERROR: unable to parse command.");
          return;
        }
        idx = idxToken[2];    // third token - should be ON or OFF
        if (strncmp(strCommand+idx,"on",2) == 0)
        {
          // "log serial ON"
          blnLogToSerial = true;    // turn ON logging
          return;
        }
        else if (strncmp(strCommand+idx,"off",3) == 0)
        {
          //  "log serial OFF"
          //  
          blnLogToSerial = false;   // turn OFF logging
          return;
        }
        else
        {
          // unknown...
          Serial.println("ERROR: unable to parse command.");
          return;
        }

      }
      
      // log file
      else if (strncmp(strCommand+idx,"file",4) == 0)
      {
        // "log file" command, now parse third token
        //
        if (tokenCount < 3)
        {
          // should be at least three tokens...
          Serial.println("ERROR: unable to parse command.");
          return;
        }
        idx = idxToken[2];

        // log file on
        if (strncmp(strCommand+idx,"on",2) == 0)
        {
          // turn on file loggin
          //
          blnLogToFile = true;
          return;
        }

        // log file off
        else if (strncmp(strCommand+idx,"off",3) == 0)
        {
          blnLogToFile = false;   // turn OFF logging to file
          LogFlushAll();
          return;
        }

        // log file save
        else if (strncmp(strCommand+idx,"save",4) == 0)
        {
          // turn off logging during flush
          //
          blnTmp = blnLogToFile;
          blnLogToFile = false;   // turn OFF logging to file
          LogFlushAll();
          blnLogToFile = blnTmp;      /// back to where it was...
          return;
        }

        // log file list
        else if (strncmp(strCommand+idx,"list",4) == 0)
        {        
          // "log file list" => list log files
          //
          // walk through file list and output names for any *.log files
          //
          Serial.println("Log files:");

          rootDir.rewind();   // start at beginning of dir

          // Open next file 
          //
          while (tmpFile.openNext(&rootDir, O_RDONLY))
          {
            tmpFile.getName(strTmp,25);   // get the name of this file

            if (rootDir.getError())
            {
              Serial.println("ERROR: failed listing files in directory.");
              return;
            }

            // check filename
            //
            blnTmp = true;
            iTmp = strlen(strTmp);
            if (tmpFile.isDir())
            {
              blnTmp = false;   // directory => skip it
            }
            else if (iTmp < 4)
            {
              blnTmp = false;   // too short
            }
            else if (strncmp(strTmp+(iTmp-4),".log",4) != 0)
            {
              blnTmp = false;   // not a .log file
            }

            // log file => output file name
            if (blnTmp)
            {
              Serial.println(strTmp);
            }

            // close this file
            //
            tmpFile.close();

          } // end of loop through files in root dir
          Serial.println();
          return;

        }  // list of log files

        // log file read
        else if (strncmp(strCommand+idx,"read",4)==0)
        {
          //  "log file read" - download contents of log file
          //

          // filename present?
          //
          if (idx < 4)
          {
            Serial.println("ERROR: no filename for log file read command.");
            return;
          }
          
          // logging must be disabled first
          //
          if (blnLogEnable)
          {
            Serial.println("ERROR: disable logging before file read.");
            return;
          }

          // read contents of log file & echo to serial port
          //
          idx = idxToken[3];
          strncpy(strTmp,strCommand+idx,lenToken[3]);
          strTmp[lenToken[3]] = 0;    // null terminate filename
          if (!EchoFile(strTmp))
          {
            Serial.println("ERROR: error reading log file.");
            return;
          }
          return;

        } // end of log file read 

        // "log file rm" => delete log file
        //
        else if (strncmp(strCommand+idx,"rm",2)==0)
        {
          // **********TBD***************

          // logging must be disabled
          //

          // filename present?
          //
          if (idx < 4)
          {
            Serial.println("no filename for log file rm command.");
            return;
          }

          // remove the file
          //

          return;
        } // end of log file rm command

        Serial.println("unknown command");
        return;

      } // end of parse token after "log" 
      else
      {
        Serial.println("ERROR: unable to parse command.");
        return;
      }

    } // end of log commands

    else
    {
      Serial.println("unknown command.");
      return;
    } // end of command options

  } // end of handling a pending command


} // end of ReadCMD

