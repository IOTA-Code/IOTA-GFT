/*
  settings.cpp

  Routines for parsing the settings file from the SD card.  

  Settings are divided into two categories:
    * Basic operational settings
    * Events

  Basic operational Settings:
    * camera = [ generic | EXP ]  = determines type of flashing sequence
    * flash settings
      * duration, level
    * pulse settings
      * duration, interval

  Events:
  Specifies timing and other parameters for flashing sequences around one or more occultation events.
    * camera
    * flash settings
    * pulse setting
    * logging setting
    * goalpost flash times

  The settings file uses a subset of JSON formatting.  This module provides the routines for parsing the contents of the settings
  file and applying the appropriate settings to the OpenGFT device.  The parsing logic is NOT generalized JSON parsing but is
  very specific to the parameters expected for this device.

*/

#include <Arduino.h>
#include <SdFat.h>
#include "iota-gft.h"
#include "logger.h"
#include "usbComm.h"
#include "SdFat.h"

//===========================================================================================================
//
//  GLOBAL variables and definitions
//
//===========================================================================================================


//===========================================================================================================
//
//   FUNCTIONS
//
//===========================================================================================================

//=====================================
//  dPrint - debug print
//=====================================
void dPrint(char *strLine, int iLen)
{
  // replace LF with CRLF
  //
  strLine[iLen-1] = '\r';
  strLine[iLen] = '\n';
  strLine[iLen+1] = 0;
  // output the line
  //
  Serial.print(strLine);
}

//=====================================
//  getKey - return index and length of key string
//
//  INPUT:
//    strInput - input line to be parsed
//    iStart - starting search position for key string
//    iLen - length of string to parse
//  
//  OUTPUT:
//    return value = -1 for error OR next position after this key (after all delimeters)
//    iKeyBegin - starting position for key string  (string may be an object delimeter { or } )
//    iKeyLen - length of key string
//
//  NOTES:
//    * may return an object delimiter instead of a key value
//    * output strings (index,length) for keys do NOT include quotes
//    * input keys should always be enclosed in quotes
//    * input string whitespace is ignored
//
//=====================================
int getKey(char *strInput, int iStart, int iLen,
                  int *iKeyBegin, int *iKeyLen)
{
  int iPos;       // current position in input string
  int iEnd;       // end position of input string
  char c;
  bool blnFound;

  //***********
  // sanity checks
  //
  if ((iStart < 0) || (iLen <= 0))
  {
    return -1;
  }

  //************
  //  init
  //
  iEnd = iStart + iLen - 1;

  //************
  // find start of Key string = double quote char
  //   note: ignore whitespace 
  //
  blnFound = false;
  while( iPos < iEnd)
  {
    c = strInput[iPos];
    if ( (c == ' ') || (c == '\t'))
    {
      // ignore whitespace
      iPos++;
      continue;
    }
    else if ( c == '/"')
    {
      // found start of key value
      //
      blnFound = true;
      break;
    }
    else if ((c == '{') || (c == '}'))
    {
      // OBJECT delimter as start of key - stop now and return this char
      //
      *iKeyBegin = iPos;
      *iKeyLen = 1;
      iPos++;
      return( iPos );
    }
    else 
    {
      // anything else is a parsing error
      //
      return -1;
    }
  }
  if (!blnFound)
  {
    // parsing error - no key string
    return -1;
  }
  else
  {
      // we did find it...
      //
    iPos++;     // skip this quote delimiter
    *iKeyBegin = iPos;
  }

  // iPos = position of first char in key string
  //
 
  //************
  // find end delimeter for key string
  //
  blnFound = false;
  while( iPos <= iEnd)
  {
    c = strInput[iPos];
    if ( c == '/"')
    {
      // found end of key string
      //
      blnFound = true;
      break;
    }
    else if (( c == ':') || (c == '{') || (c == '}'))
    {
      // parsing error
      //
      return -1;
    }
    
    // next char
    //
    iPos++;
  }  // end of loop

  if (!blnFound)
  {
    // parsing error - no end quote for key string
    return -1;
  }
  else
  {
    // we did find it...
    //
    *iKeyLen = iPos - *iKeyBegin;
    iPos++;                       // move past this delimeter

  }

  //*************
  //  find ':' delimeter
  //
  blnFound = false;
  while( iPos < iEnd)
  {
    c = strInput[iPos];
    if ( (c == ' ') || (c == '\t'))
    {
      // ignore whitespace
      iPos++;
      continue;
    }
    else if ( c == ':')
    {
      // found separator char
      //
      blnFound = true;
    }
    else
    {
      // anything else is a parsing error
      //
      return -1;
    }
   }  // end of loop

  if (!blnFound)
  {
    // parsing error - no : delimeter after key string
    return -1;
  }
  else
  {
    // we did find :
    //
    iPos++;                       // move past the ':'
  }

  // all done - return index for subsequent searching
  //
  return( iPos );

} // end of getKey

//=====================================
//  getValue - return index and length of value string
//
//  INPUT:
//    strInput - input string to be parsed
//    iStart - starting position for parsing value string
//    iLen - length of string to parse
//  
//  OUTPUT:
//    return value = -1 for error OR next position after this value string (after all delimeters)
//
//    iValueBegin - starting position for value string
//    iValueLen - length of value string
//
//  NOTES:
//    * output strings (index,length) for values do NOT include quotes
//    * output strings for object values WILL include "{" and "}" characters
//    * input keys should always be enclosed in quotes
//    * input values may be enclosed in quotes or not
//    * input value starting with a "{" signals the start of a new object as the value for this key.
//    * input string whitespace is ignored
//
//=====================================
int getValue(char *strInput, int iStart, int iLen,
                  int *iValueBegin, int *iValueLen)
{

  int iPos;       // current position in input string
  int iEnd;       // end position of input string
  char c;
  bool blnFound;

  //***********
  // sanity checks
  //
  if ((iStart < 0) || (iLen <= 0))
  {
    return -1;
  }

  //************
  //  init
  //
  iEnd = iStart + iLen - 1;


  //*************
  //  find start of value string
  //   * The value string can contain multiple instances of key, value pairs via an object structure
  //   * objects can nest inside objects as well
  //
  blnFound = false;
  while( iPos < iEnd)
  {
    c = strInput[iPos];
    if ( (c == ' ') || (c == '\t'))
    {
      // ignore whitespace
      iPos++;
      continue;
    }
    else if ( c == '/"')
    {
      // found start of value string
      //
      blnFound = true;
      break;
    }
    else if (c == "{")
    {
      // start of object => value is an object
      // just return this start of object char
      //
      // OBJECT delimter as start of key - stop now and return this char
      //
      *iValueBegin = iPos;
      *iValueLen = 1;
      iPos++;
      return( iPos );
    }
    else
    {
      // anyting else is a parsing error
      //
      return -1;
    }
    

  } // end of loop
  
  if (!blnFound)
  {
    // parsing error - no value string
    return -1;
  }
  else
  {
    // we did find start of value string
    //
    iPos++;     // skip initial delimiter
    *iValueBegin = iPos;

  }

  //*************
  // find end of value string
  //

  blnFound = false;
  while( iPos <= iEnd)
  {
    c = strInput[iPos];
    
    // look for terminating char
    //
    if ( c == '"')
    {
      // found end of value string
      //
      blnFound = true;
      break;
    }
    else if (( c == ':') || (c == '{') || (c == '}'))
    {
      // parsing error
      //
      return -1;
    }
    
    // next char
    //
    iPos++;
  }

  if (!blnFound)
  {
    // parsing error - no end quote for key string
    return -1;
  }
  else
  {
    // found value
    //
    *iValueLen = iPos - *iValueBegin;
    iPos++;                       // move past this delimeter
  }

  return( iPos );

} // end of getValue


//=====================================
//  getSettings - read settings and apply them
//  
//=====================================
int getSettings( char *fpSettings)
{
  int retval;
  int iPos;
  int iStart;
  int iLen;

  // verify the file exists
  //
  if (!sd.exists(fpSettings))
  {
    return(false);
  }

  // Open the file
  //
  if (!tmpFile.open(fpSettings,O_RDONLY))
  {
    return(false);
  }

  // read and parse one line at a time
  //   staring at top "level" of objects
  //
  while( (retval=tmpFile.fgets(strLine,MAXLINE)) > 0)
  {

    // parse this line
    // retval == # of chars READ
    // 

    // get key value
    //
    iPos = getKey(strLine,0,retval,&iStart,&iLen);
    if (iPos < 0)
    {
      // error
      // 
      return(-1);
    }

    // parse by key
    //
    if (strncmp(strLine+iStart,"unit",4) == 0)
    {
      // key = "unit", get the value *** but ignore it for now
      //
      iPos = getValue(strLine,iPos,retval,&iStart,&iLen);
      if (iPos < 0)
      {
        return(-1);
      }

      //*** debug - just print value for now
      //
      dPrint(strLine+iStart,iLen);

    } // end of "unit" key
    else if (strncmp(strLine+iStart,"camera",6) == 0)
    {
      // key = "camera" , get value for camera 
      //
      iPos = getValue(strLine,iPos,retval,&iStart,&iLen);
      if (iPos < 0)
      {
        return(-1);
      }

      //*** debug - just print value for now
      //
      dPrint(strLine+iStart,iLen);

    } // end of "camera"
    
  }  // end of top level loop
  if (retval < 0)
  {
    return(false);    // error
  }

  // done
  //
  tmpFile.close();

  return(true);

} // end of getSettings()

