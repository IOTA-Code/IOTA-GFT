/*
  usbComm

  Routines for parsing and executing commands sent to this flashing device via USB.  These routines are divided into
  three sections: manual operations, event file operations, log file operations.

  Manual operations allow the user to set/get settings for the general device operation and manually fire off flash sequence. The current operations are:
    * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb comm port.
    * Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
    * Flash Duration [X] - get/set the current flash duration 
    * Flash Now - execute a flash sequence now
    * Flash Time [YYYY-MM-DD HH:MM:SS] - set the time for a future flash
    * ? or HELP - list these commands

  Event file operations with set/get settings for future occultation event observations.  The event file specifies the timeframe for timing flashes associated with a future event.
  The event file commands to download the current event file from the device or upload a new event file to the device.
    * event file read
    * event file upload {file contents}

  Log file operations.  If a flash sequence occurs during after the device is powered up, the device creates a log file for recording all timing information while the device remains powered.
  With the log file commands, the user can list the log files, download log files, and delete log files on the device.
    * log file ls
    * log file read filename
    * log file rm filename

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

  // PPS mode settings
int PPS_Flash_Duration_Sec = 5;                 // duration of one LED "flash" in seconds

int Flash_Test_Interval = 60;					// # of seconds between flash sequences while emitting test flashes

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
     PPS_Flash_Duration_Sec = 5;
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
    PPS_Flash_Duration_Sec = 5;
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
  PPS_Flash_Duration_Sec = parmDuration;
  
}  // end of GetFlashParms

//=====================================
//  ReadCMD - get input from USB port
//  
//  pick up single line "commands" sent via the USB port
//  after reading \n (newline), sets Cmd_Next = -1 and returns
//
//======================================
void ReadCMD()
{

  byte bIn;
  int idx;
  
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

    //  * echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb port.
    //
    idx = idxToken[0];        // start of first token
    if (strncmp(strCommand+idx,"echo", 4) == 0)
    {
      if (tokenCount < 2)
      {
        // should be two tokens...
        Serial.println("Error parsing command.");
        return;
      }
      idx = idxToken[1];    // second token
      if (strncmp(strCommand+idx,"on",2) == 0)
      {
        // turn on echo of NMEA data
        //
        blnEchoNMEA = true;
      }
      else if (strncmp(strCommand+idx,"off",3) == 0)
      {
        // turn OFF echo
        //
        blnEchoNMEA = false;
      }
      else
      {
        // format error
        //
        Serial.println("Error parsing command.");
        return;
      }
    }   // end of echo command


  } // end of handling a pending command


} // end of ReadCMD

