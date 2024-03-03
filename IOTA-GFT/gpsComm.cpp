/*
  gpsComm

  routines for talking to gps receiver.
  Initialization of receiver
  Reading and parsing NMEA data from GPS receiver

*/

#include <Arduino.h>
#include "gpsComm.h"
#include "iota-gft.h"
#include "ublox.h"
#include "logger.h"


//---------------------------------------
//  GLOBALS
//---------------------------------------
unsigned long tk_NMEAStart;
int n_hh;
int n_mm;
int n_ss;
bool n_blnUTC;
unsigned long tk_GPSRMC;   // time (ticks) for start of current RMC data (if valid)
unsigned long tk_PUBX04;   // time of start of current PUBX04 data (if valid)


// parsed NMEA sentence data
//
struct nmeaRMC gpsRMC;
struct nmeaGGA gpsGGA;
struct nmeaDTM gpsDTM;
struct ubxPUBX04 gpsPUBX04;

#define NMEA_MAX  201    // max length of a nmea sentence

char nmeaTime[11] = "{TTTTTTTT ";     // string version of NMEA time
char nmeaSentence[NMEA_MAX+1];        // current NMEA sentence
char nmeaEnd[4] = "}\r\n";            // end of NMEA in log
int nmeaCount = -1;                   // position of next char in NMEA sentence = # of chars in current sentence, 0 => no current sentence
  
#define MAX_FIELDS 17         // GGA has 17 fields (including the terminating CRLF)
int fieldStart[MAX_FIELDS];   // start position of each field
                              // end of field = (start of next field - 2)                         

//------------------------------------------
//
// Functions
//
//------------------------------------------

//===========================================================================
// d2i - decode two POSITIVE ascii digits to int value
//          * on error or negative value, returns negative value as error
//
//===========================================================================
int d2i(char *src)
{
    int val;

    //  first char
    // 0x20 = ASCII space
    if (*src == 0x20)
    {
      // space for left digit
      //
      val = 0;
      src++;
    }
    else
    {
      // non-space char
      if ((*src < 0x30) || (*src > 0x39))   // 0x30 = '0'
      {
        return -1;
      }
  
      val = (*src - 0x30)*10;
  
      src++;
    }

    // right most digit
    //
    if ((*src < 0x30) || (*src > 0x39))
    {
      return -1;
    }
    val += (*src - 0x30);

    return val;
    
} // end of d2i

//===========================================================================
// h2i - decode two POSITIVE ascii digits to int value
//          * on error or negative value, returns negative value as error
//
//===========================================================================
int h2i(char *src)
{
    int val;

    //  first char
    // 0x20 = ASCII space
    if (*src == 0x20)
    {
      // space for left digit
      //
      val = 0;
      src++;
    }
    else
    {
      // non-space char
      if ((*src >= 0x30) && (*src <= 0x39))   // 0x30 = '0', 0x39 = '9'
      {
        val = (*src - 0x30)*16;
      }
      else if ((*src >= 0x41) && (*src <= 0x46))   // 0x41 = 'A', 0x46 = 'F'
      {
        val = (*src - 0x41 + 10 )*16;
      }
      else
      {
        return -1;
      }
  
      src++;
    }

    // right most digit
    //
    if ((*src >= 0x30) && (*src <= 0x39))   // 0x30 = '0', 0x39 = '9'
    {
      val += (*src - 0x30);
    }
    else if ((*src >= 0x41) && (*src <= 0x46))   // 0x41 = 'A', 0x46 = 'F'
    {
      val += (*src - 0x41 + 10 );
    }
    else
    {
      return -1;
    }

    return val;
    
} // end of h2i

//=============================================================
//  ParseGGA - parse & save the GGA data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//=============================================================
int ParseGGA()
{
  int iStart;
  int iLen;

  gpsGGA.valid = false;

  //******************************
  // field 2 = Latitude
  //
  iStart = fieldStart[2];
  iLen = fieldStart[3] - iStart - 1;
  if (iLen <= 0)
  {
    // sentence not valid, just leave as invalid but no error
    //
    return NMEA_GGA;
  }

  if ((iLen < 5) || (iLen > MAX_LATLONG))
  {
    return NMEA_ERROR_GGA_LAT; 
  }

  for( int i = 0; i < MAX_LATLONG; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.lat[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.lat[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 3 = N/S for latitude
  //
  iStart = fieldStart[3];
  iLen = fieldStart[4] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR_GGA_NSLEN; 
  }
  gpsGGA.NS = (char)nmeaSentence[iStart];  
  if ( ((char)gpsGGA.NS != 'N') && ((char)gpsGGA.NS != 'n') && 
          ((char)gpsGGA.NS != 'S') && ((char)gpsGGA.NS != 's') )
  {
    return NMEA_ERROR_GGA_NS;
  }

  //******************************
  // field 4 = Longitude
  //
  iStart = fieldStart[4];
  iLen = fieldStart[5] - iStart - 1;

  if ((iLen < 5) || (iLen > MAX_LATLONG))
  {
    return NMEA_ERROR_GGA_LONG; 
  }

  for( int i = 0; i < MAX_LATLONG; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.lng[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.lng[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 5 = E/W for Longitude
  //
  iStart = fieldStart[5];
  iLen = fieldStart[6] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR_GGA_EWLEN; 
  }
  gpsGGA.EW = (char)nmeaSentence[iStart];
  if ( ((char)gpsGGA.EW != 'E') && ((char)gpsGGA.EW != 'e') && 
          ((char)gpsGGA.EW != 'W') && ((char)gpsGGA.EW != 'w') )
  {
    return NMEA_ERROR_GGA_EW;
  }

  //******************************
  // field 9 = MSL altitude
  //
  iStart = fieldStart[9];
  iLen = fieldStart[10] - iStart - 1;

  if ((iLen < 1) || (iLen > MAX_ALT))
  {
    return NMEA_ERROR_GGA_ALT; 
  }

  for( int i = 0; i < MAX_ALT; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.alt[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.alt[i] = ' ';  // pad with spaces on the end
    }
  }
  gpsGGA.alt_len = iLen;

  //**************************
  // field 10 = units for altitude
  //
  iStart = fieldStart[10];
  iLen = fieldStart[11] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR_GGA_ALTULEN; 
  }
  gpsGGA.alt_units = (char)nmeaSentence[iStart];
  if ((gpsGGA.alt_units != 'M') && (gpsGGA.alt_units != 'm'))      // must be "m"
  {
    return NMEA_ERROR_GGA_ALTU;
  }

  //******************************
  // field 11 = geoid separation
  //
  iStart = fieldStart[11];
  iLen = fieldStart[12] - iStart - 1;

  if ((iLen < 1) || (iLen > MAX_ALT))
  {
    return NMEA_ERROR_GGA_GEOID; 
  }

  for( int i = 0; i < MAX_ALT; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.geoid_sep[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.geoid_sep[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 12 = units for altitude
  //
  iStart = fieldStart[12];
  iLen = fieldStart[13] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR_GGA_GULEN; 
  }
  gpsGGA.sep_units = (char)nmeaSentence[iStart];
  if ((gpsGGA.sep_units != 'M') && (gpsGGA.sep_units != 'm'))      // must be "m"
  {
    return NMEA_ERROR_GGA_GU;
  }
  
  //**********
  // all done
  //
  gpsGGA.valid = true;
  
  return NMEA_GGA;
  
} // end of parseGGA

//=============================================================
//  ParseRMC - parse & save the RMC data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//=============================================================
int ParseRMC()
{
  int iStart;
  int iLen;
  int iTmp;

  gpsRMC.valid = false;

  //**************
  // field 2 - status indicator
  //
  iStart = fieldStart[2];
  iLen = fieldStart[3] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR_RMC_STATUSLEN; 
  }
  gpsRMC.status = (char)nmeaSentence[iStart];     // status char

  if (gpsRMC.status != 'A')
  {
    return NMEA_RMC;   // RMC sentence but data not valid
  }

  //*****
  //  sentence is marked as valid, all remaining fields should be present
  //

  //******************************
  // field 1 = HH:MM:SS time
  //
  iStart = fieldStart[1];
  iLen = fieldStart[2] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR_RMC_TIMELEN; 
  }

  //*** protect from interrupts ***
  //  hh,mm,ss from the RMC sentence may be used by ISR for the PPS signal
  //  we should keep these changes "atomic"
  //
  noInterrupts();
  iTmp = d2i( &nmeaSentence[iStart]);
  if (iTmp < 0)
  {
    interrupts();
    return NMEA_ERROR_RMC_HH;
  }
  gpsRMC.hh = iTmp;
  
  iTmp = d2i( &nmeaSentence[iStart+2]);
  if (iTmp < 0)
  {
    interrupts();
    return NMEA_ERROR_RMC_MM;
  }
  gpsRMC.mm = iTmp;
  
  iTmp = d2i( &nmeaSentence[iStart+4]);
  if (iTmp < 0)
  {
    interrupts();
    return NMEA_ERROR_RMC_SS;
  }
  gpsRMC.ss = iTmp;

  interrupts();
  
  //****************************
  // field 9 = ddmmyy Date 
  //
  iStart = fieldStart[9];
  iLen = fieldStart[10] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR_RMC_DAYLEN; 
  }
  
  iTmp = d2i( &nmeaSentence[iStart]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_RMC_DAY;
  }
  gpsRMC.day = iTmp;

  iTmp = d2i( &nmeaSentence[iStart+2]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_RMC_MON;
  }
  gpsRMC.mon = iTmp;

  iTmp = d2i( &nmeaSentence[iStart+4]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_RMC_YR;
  }
  gpsRMC.yr = iTmp;
  
  //**********
  // all done
  //
  gpsRMC.valid = true;
  
  return NMEA_RMC;
  
} // end of parseRMC

//=============================================================
//  ParseDTM - parse & save data from the DTM sentence
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//=============================================================
int ParseDTM()
{
  int iStart;
  int iLen;

  //**************
  // field 1 - datum code
  //
  iStart = fieldStart[1];
  iLen = fieldStart[2] - iStart - 1;

  if (iLen != 3)
  {
    return NMEA_ERROR_DTM;
  }

  for (int i = 0; i< 3; i++)
  {
    gpsDTM.local_datum[i] = nmeaSentence[iStart+i];
  }  
  
  gpsDTM.valid = true;
  return NMEA_DTM;
}

//=============================================================
//  ParsePUBX04 - parse & save the PUBX04 data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//=============================================================
int ParsePUBX04()
{
  int iStart;
  int iLen;
  int iTmp;

  gpsPUBX04.valid = false;

  //******************************
  // field 2 = hhmmss.ss
  //
  iStart = fieldStart[2];
  iLen = fieldStart[3] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR_PUBX04_TIMELEN; 
  }

  iTmp = d2i( &nmeaSentence[iStart]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_PUBX04_HH;
  }
  gpsPUBX04.hh = iTmp;

  iTmp = d2i( &nmeaSentence[iStart+2]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_PUBX04_MM;
  }
  gpsPUBX04.mm = iTmp;

  iTmp = d2i( &nmeaSentence[iStart+4]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_PUBX04_SS;
  }
  gpsPUBX04.ss = iTmp;
  
  //******************************
  // field 6 = LEAP seconds
  //
  iStart = fieldStart[6];
  iLen = fieldStart[7] - iStart - 1;

  // this field should always be two digits with an optional 'D' at the end
  //
  if ((iLen < 2) || (iLen > 3))
  {
    return NMEA_ERROR_PUBX04_OFFSETLEN; 
  }
  gpsPUBX04.cLeap[0] = nmeaSentence[iStart];
  gpsPUBX04.cLeap[1] = nmeaSentence[iStart+1];

  // decode the two digit leap second count
  //
  iTmp = d2i(&nmeaSentence[iStart]);
  if (iTmp < 0)
  {
    return NMEA_ERROR_PUBX04_OFFSETPARSE;
  }
  gpsPUBX04.usLeapSec = (uint8_t)iTmp;

  // is this terminated with a 'D' to indicate no almanac yet?
  //
  if (iLen == 2)
  {
    // no terminating 'D' => alamanac is up to date and time is UTC
    //
    gpsPUBX04.blnLeapValid = true;
    gpsPUBX04.cLeap[2] = 0x20;        // space at end
  }
  else
  {
    // terminatinting D => using firmware default leap seconds
    //
    if (nmeaSentence[iStart + 2] == 'D')
    {
      gpsPUBX04.blnLeapValid = false;
      gpsPUBX04.cLeap[2] = 0x44;        // 'D'
    }
    else
    {
      return NMEA_ERROR_PUBX04_NOD;
    }
  }
  
  //**********
  // all done
  //
  gpsPUBX04.valid = true;
  
  return NMEA_PUBX04;
  
} // end of parsePUBX04


//=============================================================
//  ParseNMEA - parse the current NMEA sentence
//    currently supports the following sentences
//        RMC, GGA, and PUBX,04
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    nmeaCount = # of chars in sentence (including terminating CRLF)
//
//  RETURNS:
//    negative value for errors
//    positive => sentence type
//=============================================================
int ParseNMEA()
{
  int iField;       // current field #
  int fieldCount;   // # of fields in sentence (including CRLF)
  int iPos;         // current position in nmea string

  int fStart;
  int fLen;
  
  //********
  // find the start,end of each field
  //
  iField = 0;
  fieldStart[iField] = 0;
  iPos = 1;                 // next char to be tested

  while (iPos < nmeaCount)
  {
    // start of a new field?
    //
    if ((char)nmeaSentence[iPos] == ',')
    {
      //  end of this field inside the sentence
      //
      iField++;           // new field start
      iPos++;             // start with position past ','
      if (iPos >= nmeaCount)
      {
        return NMEA_ERROR_SENTEND;     // no start of next field => unexpected end of sentence
      }
      fieldStart[iField] = iPos;

    }
    else if ((char)nmeaSentence[iPos] == '*')
    {
      // * => end of actual sentence data, next two chars are the checksum, then terminating CRLF
      //      last field with be the two checksum characters
      //
      byte chksum = (byte)nmeaSentence[1];
      for( int i = 2; i < iPos; i++)
      {
        chksum = chksum ^ (byte)nmeaSentence[i];
      }

      if ((int)chksum != (h2i(nmeaSentence + iPos+1)))
      {
        // checksum didn't match
        //
        return NMEA_ERROR_CHKSUM_WRONG;
      }

      iPos++;         // -> one past the * 
      iField++;       // last field = checksum character
      if ( (iPos+2) > nmeaCount)
      {
        return NMEA_ERROR_CHKSUM_MISSING;     // not enough room for two checksum characters
      }
      fieldStart[iField] = iPos;

    }
    else if ((char)nmeaSentence[iPos] == '\r')
    {
      // CR => end of sentence and end of this field
      //
      iPos++;             // -> one past the CR = LF
      iField++;           // last field = CRLF
      fieldStart[iField] = iPos;
      break;
    }
    else
    {
      // in a field, move to next char
      //
      iPos++;
    }
  } // loop through all chars in sentence

  //  done scanning for fields
  //   iPos should point to the terminating LF now
  //   iField = field # of last field in sentence (CRLF)
  //
  if ((char)nmeaSentence[iPos] != '\n')
  {
    return NMEA_ERROR_NOCRLF;       // terminated loop without finding CRLF
  }
  fieldCount = iField + 1;
  if (fieldCount < 3)
  {
    return NMEA_ERROR_TOOSHORT;     // all sentences have at least three fields
  }
  
  //************
  // found the fields, now parse sentence data
  //  
  fStart = fieldStart[0];                 // start of message ID field
  fLen = fieldStart[1] - fStart - 1;    // length of message ID field

  if ( ((char)nmeaSentence[fStart] == '$') &&
        ((char)nmeaSentence[fStart+1] == 'G') &&
        ((char)nmeaSentence[fStart+2] == 'P') )
  {
    // this is a standard NMEA sentence starting with $GP
    //
    if (fLen < 6)
    {
      // unknown message - just return no error for now
      return NMEA_UNKNOWN;
    }
    
    if ( ((char)nmeaSentence[fStart+3] == 'R') &&
          ((char)nmeaSentence[fStart+4] == 'M') &&
          ((char)nmeaSentence[fStart+5] == 'C') )
    {
      return ParseRMC();
    }
    else if ( ((char)nmeaSentence[fStart+3] == 'G') &&
          ((char)nmeaSentence[fStart+4] == 'G') &&
          ((char)nmeaSentence[fStart+5] == 'A') )
    {
      return ParseGGA();
    }
    else if ( ((char)nmeaSentence[fStart+3] == 'D') &&
          ((char)nmeaSentence[fStart+4] == 'T') &&
          ((char)nmeaSentence[fStart+5] == 'M') )
    {
      return ParseDTM();
    }
    else
    {
        return NMEA_UNKNOWN;
    } // end of if/else block parsing standard NMEA sentences
  }
  else if ( ((char)nmeaSentence[fStart] == '$') &&
          ((char)nmeaSentence[fStart+1] == 'P') &&
          ((char)nmeaSentence[fStart+2] == 'U') &&
          ((char)nmeaSentence[fStart+3] == 'B') &&
          ((char)nmeaSentence[fStart+4] == 'X'))
  {
    // this is a UBX proprietary sentence
    //  check field 1 for the sentence type
    //
    fStart = fieldStart[1];
    fLen = fieldStart[2] - fStart - 1;
    if ( ((char)nmeaSentence[fStart] == '0') &&
          ((char)nmeaSentence[fStart+1] == '4') )
    {
      // PUBX,04 sentence
      //
      return ParsePUBX04();
    } 
    else
    {
      // unknown PUBX sentence => do nothing
      //
      return NMEA_UNKNOWN;
      
    }// end of check for PUBX sentence type
  }
  else
  {
    // unknown sentence type
    //
    //  DO NOTHING and no error
    return NMEA_UNKNOWN;
  }
  
} // end of ParseNMEA

//=============================================================
//  ReadGPS - gather and parse any pending serial data from GPS
//
//  Outputs:
//    nmeaSentence[] - current NMEA sentence placed in this array
//    nmeaCount = position of next char in array
//    tk_NMEAStart = timer value when $ char of sentence is received
//    
//    * after reading full sentence (\n), parse data from sentence into 
//      global variables for the sentence
//
//
//  Returns:
//    returns non-zero iff error parsing data
//=============================================================
int ReadGPS()
{
  char c;
  int resultParse;
  long internalSec;
  long ubxSec;
  int ErrorFound;

  //******
  //  INIT
  //
  ErrorFound = 0;

  //************
  // Read/process all currently available characters
  //  nmeaCount == 0 => not in a sentence now
  //

  while (gpsSerial.available() > 0)
  {
    // get the char
    //
    c = gpsSerial.read();

    // Watch for beginning/ending of a sentence
    //

    if (c == '$')
    {
      // start of a sentence
      //
      noInterrupts();
      tk_NMEAStart = GetTicks(CNT4);
      interrupts();

      // save the time at the start of the string
      //
      ultohexA(nmeaTime+1,tk_NMEAStart);
  
      // save the current hh:mm:ss time
      //
      noInterrupts();
      n_blnUTC = time_UTC;
      n_hh = sec_hh;
      n_mm = sec_mm;
      n_ss = sec_ss;
      interrupts();
      
      // are we in the right place?
      //
      if (nmeaCount > 0)
      {
        // oops! - currently in a sentence, should not be here
        //
        nmeaCount = 0;
        return err_gps_nmeaCount;
      }

      //  ok, save the start char and get ready for next one
      //
      nmeaSentence[0] = (uint8_t)c;
      nmeaCount++;
      
    }
    else
    {
      // we are somewhere IN a sentence
      //

      // are we in the right place?
      //
      if (nmeaCount <= 0)
      {
        // oops! - not in a sentence now
        //
        return err_gps_nmeaCount;
      }

      // ok, save the end of the sentence and call the parser
      //
      nmeaSentence[nmeaCount] = (uint8_t)c;
      nmeaCount++;                      

      // too many?
      //
      if (nmeaCount >= NMEA_MAX)
      {
        nmeaCount = 0;      // drop all of it 
        continue;
      }

      // if we just ended a sentence, call the parser
      //
      if (c == '\n')
      {
        // terminate the sentence with zero
        //
        nmeaSentence[nmeaCount] = 0;

        // log this sentence to buffer 
        //   NOTE: interrupts are enabled so this sentence could be "interrupted"
        //          by a PPS or EXP log event
        //
        LogTextWrite(nmeaTime,10);
        LogTextWrite(nmeaSentence,nmeaCount-2);
        LogTextWrite(nmeaEnd,3);

        // call the parser
        //
        resultParse = ParseNMEA();
        if (resultParse < 0)
        {
          nmeaCount = 0;   // restart
          return (err_gps_parseNMEA * 256) + -resultParse;   // exit on parsing error
        }

        // save time of RMC and PUBX04 sentences
        //
        if (resultParse == NMEA_RMC)
        {
          
          // NMEA_RMC sentence...
          //
          tk_GPSRMC = tk_NMEAStart;     // save start time
          
        }
        else if (resultParse == NMEA_PUBX04)
        {
          tk_PUBX04 = tk_NMEAStart;     // save start time

          // turn on mode report
          //
          blnReportMode = true;

          // update current or default GPS-UTC offset value
          //
          if (gpsPUBX04.blnLeapValid)
          {
            offsetUTC_Current = gpsPUBX04.usLeapSec;
          }
          else
          {
            offsetUTC_Default = gpsPUBX04.usLeapSec;
          }
          
          // now check PUBX04 time against internal time to catch errors...
          //
          if ( DeviceMode == TimeValid )
          {
            
            internalSec = (long)n_hh*3600 + (long)n_mm*60 + (long)n_ss;   // internal seconds of the day
            
            if (!n_blnUTC)
            {
              // internal time is GPS
              //
              ubxSec = (long)gpsPUBX04.hh*3600 + (long)gpsPUBX04.mm * 60 + (long)gpsPUBX04.ss;
              if (gpsPUBX04.blnLeapValid )
              {
                ubxSec += offsetUTC_Current;      // move to GPS time
              }
              else
              { 
                ubxSec += offsetUTC_Default;      // move to GPS time
              }
              if (ubxSec >= 86400)
              {
                ubxSec -= 86400;      // fixup 24hr overflow
              }
              
              if (ubxSec != internalSec)
              {
                ErrorFound = err_gps_ubxTime;
              }            
              
            }
            else
            {
              // internal time is UTC
              //   UBX time should be UTC also
              //
              ubxSec = (long)gpsPUBX04.hh*3600 + (long)gpsPUBX04.mm * 60 + (long)gpsPUBX04.ss;
              if (ubxSec != internalSec)
              {
                ErrorFound = err_gps_utcMatch;
              }
            
            } // end of checking internal time for UTC
          
            // Did we find an error?
            //
            if (ErrorFound > 0)
            {
              // try to re-sync
              //
              DeviceMode = Syncing;
              TimeSync = SYNC_SECONDS;

              noInterrupts();                 // clear the ave interval computation
              tk_pps_interval_total = 0;
              tk_pps_interval_count = 0;
              interrupts();

            }
              
          } // end of RMC check against internal time
          
        } // end of processing for RMC,PUBX04 data
        
        // looking for new sentence now...
        //
        nmeaCount = 0;
      
      } // found the end of a sentence
      
    }  // end of check for start/non-start char
    
  } // end of loop through available characters
  
  return ErrorFound;
  
} // end of ReadGPS


//=============================================================
//  Init - initialize the GPS communications
//=============================================================
bool gpsCommInit()
{

  int retVal;

  // initialize NMEA data 
  //
  gpsRMC.valid = false;
  gpsGGA.valid = false;
  gpsDTM.valid = false;
  gpsPUBX04.valid = false;

  //*********************
  //  Init GPS
  //
  retVal = ubxInit();
  if (retVal > 0)
  {
    Serial.print("[ERROR initializing GPS module : ");
    Serial.print(retVal);
    Serial.println("]");
    return( false );
  }

  return( true );

} // end of init




