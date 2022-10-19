/*
  usbComm

  Routines for USB communication (with separate computer).  These routines are divided into
  three sections: manual operations, event file operations, log file operations.

  Manual operations allow the user to set/get settings for the general device operation and manually fire off flash sequence. The current operations are:
    * Echo ON | OFF - enable/disable echo of the GPS NMEA data to the usb comm port.
    * Flash Mode [PPS | EXP ] - get/set the current flash mode (PPS or EXP)
    * Flash Duration [X] - set/set the current flash duration 
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
char strCommand[ CommandSize + 1 ];
int Cmd_Next;                         // speed optimization ( -1 => full command pending )

  // PPS mode settings
int PPS_Flash_Duration_Sec = 5;                 // duration of one LED "flash" in seconds

int Flash_Test_Interval = 60;					// # of seconds between flash sequences while emitting test flashes

//===========================================================================================================
//
//   FUNCTIONS
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
      strCommand[ Cmd_Next ] = bIn;
      Cmd_Next++;
    }
    // if \r, do nothing with char
    
  } // end of loop reading pending data


  //*******************************************************
  //  Execute any pending command
  //
  if (Cmd_Next < 0)
  {
    // ECHO command string to USB port
    //
    
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


} // end of ReadCMD

