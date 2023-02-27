/*
  Logger

  Routines for logging timing data
  
  * Open file on SD card for logging
  * write text or data into logging buffer
  * output buffer contents to file and/or Serial (USB)
  
  NOTES:
		* Dedicated_SPI mode
		* exFAT format on SD card
		* preallocates contiguous file on SD card before starting write
		* V2 SdFat - writing to preallocated file uses multi-block write commands "under the hood"?

*/

#include <Arduino.h>
#include <SPI.h>
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <SdFat.h>
#include <sdios.h>
#include "gpsComm.h"
#include "logger.h"
#include "iota-gft.h"

//---------------------------------------
//  GLOBALS
//---------------------------------------

//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = SS;

// Misc
//
bool blnLogEnable = false;        // enable logging
bool blnLogEXP = true;            // log EXP events
bool blnLogToFile = false;        // log to file
bool blnLogToSerial = true;       // echo log to serial port
bool bln_SD_OK = false;           // Is SD card avaiable?
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in bytes.
// The program creates a contiguous file with MAX_FILE_SIZE_MiB bytes.
// The file will be truncated if logging is stopped early.
//const uint32_t MAX_FILE_SIZE_MiB = 100;  // 100 MiB file.
const uint32_t MAX_FILE_SIZE_MiB = 1;  // testing ... 1 MiB file.
const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

// log file name
//   YYYY-MM-DD_HH-MM-SS.log
//
char fileName[25] = "YYYY-MM-DD_HH-MM-SS.log";
bool blnFileOpen = false;

//------------------------
// SdFat config
//

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
//
#define SPI_CLOCK SD_SCK_MHZ(50)

// dedicated SPI
//
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)

// sd objects
//
SdExFat sd;

ExFatFile logFile;    // log file
ExFatFile rootDir;    // root directory
ExFatFile tmpFile;    // misc file pointer

//------------------------------------------------------------------------------
// SD write Buffer definitions.
//    * FIFO queue of data blocks
//
const uint8_t FIFO_DIM = 6;   // # of blocks in FIFO

const uint16_t BLOCK_SIZE = sizeof(block_t);

block_t fifoBuffer[FIFO_DIM];      // allocate buffer space

// fifo variables
//    fifoCount = # of fifo blocks (always >= 1)
//    fifoHead = index of last (most recent) fifo block with data.
//    fifoTail = index of first (earliest) fifo block with data
//
volatile uint16_t fifoCount = 1; // volatile - shared, ISR and background.
volatile uint16_t fifoHead = 0;  // Only accessed by buffer writer.
volatile uint16_t fifoTail = 0;  // Only accessed by sd writer.

// Pointer to current data block (head block)
//
block_t* curBlock = &fifoBuffer[0];

// total overrun count
uint16_t fifoOverrun = 0;

// stats
uint32_t maxLatencyUsec = 0;
uint32_t maxFifoUse = 0;
uint32_t bytesWritten = 0;
uint32_t isrCount = 0;

//-------------------------------------------------------------
//  data structs
//

// time record - Basic time record - text values
//
struct data_t {
  uint8_t flags;
  uint8_t hextime[8];   // unsigned long micros() time in hex
  uint8_t comma;
};

// Line from file
//
char strLine[MAXLINE+1];

// crc value
char strCRC[12]={'[','C','R','C',' ',0,0,0,0,']',0};
int offset_CRC = 5;

//=========================================================
//
// Functions
//
//=========================================================

//------------------------------------------
//  errorHalt - print error message and halt
//-----------------------------------------
void errorHalt(char *strMessage)
{
  Serial.println(strMessage);
  while(1);             // hang!
}

//------------------------------------------
// crc_update_string - add zero terminated string data to running CRC value
//  INPUTS:
//    crc_current = current CRC value
//    strInput = byte array to include in CRC
//  RETURN:
//    returns updated CRC value
//------------------------------------------
uint16_t crc_update_string( uint16_t crc_current, uint8_t *strInput)
{
  uint8_t cInput;

  // init
  //
  uint8_t sum1 = (uint8_t) crc_current;
  uint8_t sum2 = (uint8_t) (crc_current >> 8);

  // walk though string until null value
  //
  while( *strInput != 0)
  {
    // update CRC sums
    //
    sum1 = (sum1 + *strInput) % 255;
    sum2 = (sum2 + sum1) % 255;
    // next character
    strInput++;
  }

  // done
  //
  return (sum2 << 8) | sum1;

} // update CRC value

//---------------------------------------------
// LogTextWrite - write text string into logging fifo
// 
// Inputs:
//  strIn - pointer to first char 
//  iCount - # of characters to copy into fifo
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool LogTextWrite(char *strIn, int iCount)
{
  // is logging enabled?
  //
  if (!blnLogEnable || (!blnLogToFile && !blnLogToSerial))
  {
    return true;      // no error, just ignore the request
  }

  // copy char bytes into the current fifo buffer
  //
  for ( ; iCount > 0; iCount--)
  {    

    // check for block full / overrun
    //  curBlock->count = # of bytes in block = index for next char
    //
    if (curBlock->count > 511)
    {
      // this block is full

      // check for overrun
      //
      if (fifoCount >= FIFO_DIM)
      {
        // no more buffers => overrun!
        //    do not save this char
        //
        curBlock->overrun++;
        fifoOverrun++;
        return(false);
      }

      // no overrun => move head block to next block in buffer
      //
      fifoCount++;
      fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
      curBlock = &fifoBuffer[fifoHead];
      curBlock->count = 0;
      curBlock->overrun = 0;
      if (fifoCount >maxFifoUse) {
        maxFifoUse = fifoCount;
      }

    } // end of block full...
    
    // copy char into fifo buffer data block
    //
    curBlock->data[curBlock->count] = *strIn;
    strIn++;            
    curBlock->count++;

    
  } // end of loop through input chars

  // all done
  //
  return(true);
  
} // end of LogTextWrite

//---------------------------------------------
//  LogFlushFull - write out ONE pending FULL data blocks to SD card
//      true iff block written
//---------------------------------------------
bool LogFlushFull()
{
  uint32_t m;
  block_t* pBlock;

  //************
  //  if full block available, write it to the SD card
  //
  if (fifoCount <= 1)
  {
    // just using one fifo block, no full blocks
    //
    return(false);
  }
  else
  {
    // fifoCount > 1 => we have a full block ready
    //
    pBlock = &fifoBuffer[fifoTail];

    // write the log buffer data to SD card and/or Serial
    //

    if (blnLogToFile)
    {
      // writing log buffer to file on SD card ...
      //

      // make sure we have room in the file
      //
      if (logFile.curPosition() >= MAX_FILE_SIZE) {
        // File full => report error and stop
        Serial.println("[ERROR Log file full!]");
      }

      // Write tail block data to SD.
      //
      m = micros();
      if (logFile.write(pBlock->data, 512) != 512) {
        Serial.println("[ERROR write data failed]");
      }
      m = micros() - m;
      if (m > maxLatencyUsec) {
        maxLatencyUsec = m;
      }

    } // end of writing to file
 
    if (blnLogToSerial)
    {
      // writing log buffer to serial port
      Serial.write(pBlock->data, 512);
    }

    bytesWritten += 512;

    // Initialize empty block & reset tail pointer
    //
    pBlock->count = 0;
    pBlock->overrun = 0;
    fifoTail = fifoTail < (FIFO_DIM - 1) ? fifoTail + 1 : 0;

    // block written, decrement count
    //
    fifoCount--;

    return(true);

  } // end of fifo_count > 1 loop

} // end of logFlushFull

//---------------------------------------------
//  LogFlushAll - flush ALL pending FIFO data
//    NOTE: only call this routine AFTER stopping all writes into the FIFO buffer
//          (i.e. turn OFF logging)
//
//---------------------------------------------
bool LogFlushAll()
{
  uint32_t m;

  // first write out any pending FULL data blocks
  //
  LogFlushFull();

  // Does the head block contain any data?
  //
  if (curBlock->count > 0)
  {
    
    if (blnLogToFile)
    {
      // writing log buffer to file on SD card ...
      //

      // make sure we have room in the file
      //
      if (logFile.curPosition() >= MAX_FILE_SIZE) {
        // File full => report error and stop
        Serial.println("[ERROR: Log file full!]");
      }

      m = micros();
      if (logFile.write(curBlock->data, curBlock->count) != curBlock->count) {
        Serial.println("[ERROR write data failed]");
      }
      m = micros() - m;
      if (m > maxLatencyUsec) {
        maxLatencyUsec = m;
      }

    } // end of writing to file
 
    if (blnLogToSerial)
    {
      // writing log buffer to serial port
      Serial.write(curBlock->data, curBlock->count);
    }
    // write out data from this partial block
    //
   
    bytesWritten += curBlock->count;
    curBlock->count = 0;                // head block now empty
  }

  return(true);

} // end of LogFlushAll

//---------------------------------------------
//  LogFlushToFile - flush pending data to SD file
//
//---------------------------------------------
void LogFlushToFile()
{

  // ignore if not logging to SD file
  //
  if (!blnLogToFile)
  {
    return;
  }

  // flush the SD output buffer to the SD card file.
  //
  logFile.flush();

}  // end of LogFlushToFile

//---------------------------------------------
// LogInit - create a new log file and clear buffers
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool LogInit()
{

  //************
  // initialize sdFat
  //
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("[ERROR initializing SD!]");
    return(false);
  }

  //***************
  //  Root directory object
  //
  if (!rootDir.open("/"))
  {
    Serial.println("[ERROR root dir.open failed]");
    return(false);
  }


  //*****************
  // Initialize fifo buffers
  //   turn off interrupts - just in case...
  //
  noInterrupts();
  fifoCount = 1;    // first block in use
  fifoHead = 0;
  fifoTail = 0;
  curBlock = &fifoBuffer[0];

  for( int i=0; i < FIFO_DIM; i++)
  {
    fifoBuffer[i].count = 0;
    fifoBuffer[i].overrun = 0;
  }
  interrupts();

  //****************
  // no file yet...
  //
  blnFileOpen = false;

  return(true);

} // end of LogInit

//---------------------------------------------
// LogFileOpen - create a new log file and clear buffers
//
//  note: if a log file is already open, close it first
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool LogFileOpen()
{
  bool blnLogExists;

  uint16_t yr;
  uint8_t mon;
  uint8_t day;
  uint8_t hh;
  uint8_t mm;
  uint8_t ss;


  //***********************
  //  if file already open, close it now
  //
  if (blnFileOpen)
  {
    logFile.close();
    Serial.print("[Closed current logfile <");
    Serial.print(fileName);
    Serial.println(">]");
    blnFileOpen = false;
  }

  //***********************
  //  if RMC data valid
  //    * get the current date/time
  //    * set filename based on this date/time
  //
  if (gpsRMC.valid)
  {
    yr = gpsRMC.yr + 2000;
    mon = gpsRMC.mon;
    day = gpsRMC.day;
    hh = gpsRMC.hh;
    mm = gpsRMC.mm;
    ss = gpsRMC.ss;

    sprintf(fileName,"%04d-%02d-%02d_%02d-%02d-%02d.log", yr, mon, day,hh,mm,ss);
  }

  //******************
  //  if this file already exists, delete it
  //
  blnLogExists = sd.exists(fileName);
  if (blnLogExists)
  {
    Serial.print(F("[Removing existing logfile...]"));
    sd.remove(fileName);
  }

  //******************
  //  create a new log file
  //
  Serial.print("[Opening log file on SD card: <");
  Serial.print(fileName);
  Serial.println(">]");
  if (!logFile.open(fileName, O_RDWR | O_CREAT)) {
    Serial.println("[ERROR open file failed]");
    return(false);   // dont start!
  }

// skip pre-allocate for now...
//
#if 0  
  Serial.print("[Allocating: ");
  Serial.print(MAX_FILE_SIZE_MiB);
  Serial.println(" MiB]");
  if (!logFile.preAllocate(MAX_FILE_SIZE)) {
    Serial.println("[ERROR preAllocate failed]");
    return(false);   // don't start!
  }
#endif
  blnFileOpen = true;

  //*******************
  //  set file date/time from RMC sentence
  //
  if (gpsRMC.valid)
  {
    if (!logFile.timestamp(T_ACCESS|T_CREATE|T_WRITE, yr, mon, day, hh, mm, ss))
    {
      Serial.println("[ERROR File timestamp failed]");
      return(false);   // don't start!
    }
  } // end of setting time of file

  //*****************
  // Initialize fifo buffers
  //   turn off interrupts - just in case...
  //
  noInterrupts();
  fifoCount = 1;    // first block in use
  fifoHead = 0;
  fifoTail = 0;
  curBlock = &fifoBuffer[0];

  for( int i=0; i < FIFO_DIM; i++)
  {
    fifoBuffer[i].count = 0;
    fifoBuffer[i].overrun = 0;
  }
  interrupts();

  return(true);

} // end of LogFileOpen

//---------------------------------------------
// EchoFile - echo contents of file to serial port
//  Inputs:
//    fname = null terminated name of file to read
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool EchoFile(char* fname)
{
  int retval;
  uint16_t crc_val = 0;

  // verify the file exists
  //
  if (!sd.exists(fname))
  {
    Serial.print("[ERROR file <");
    Serial.print(fname);
    Serial.println("> does not exist.]");
    return(false);
  }

  // Open the file
  //
  if (!tmpFile.open(fname,O_RDONLY))
  {
    Serial.println("[ERROR opening file.]");
    return(false);
  }

  // Mark start of file with "{" on a line
  //
  Serial.println("{");

  // read each line and echo to the serial port
  //
  while( (retval=tmpFile.fgets(strLine,MAXLINE)) > 0)
  {
    // replace LF with CRLF
    //
    strLine[retval-1] = '\r';
    strLine[retval] = '\n';
    strLine[retval+1] = 0;

    // output the line
    //
    Serial.print(strLine);

    // update the CRC value
    //
    crc_val = crc_update_string(crc_val,(uint8_t *)strLine);

  }
  if (retval < 0)
  {
    return(false);    // error
  }

  // mark end of file
  //
  Serial.println("}");

  // now output the crc_value
  //   [XXXX]
  //
  ustohexA(strCRC+offset_CRC,crc_val);
  Serial.println(strCRC);

  // done
  //
  tmpFile.close();

  return(true);

} // end of EchoFile