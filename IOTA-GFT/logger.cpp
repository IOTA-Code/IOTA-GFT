/*
  Logger

  Routines for logging timing data
  
  * Open file on SD card for logging
  * write text or data into logging buffer
  * output buffer contents
  
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

//---------------------------------------
//  GLOBALS
//---------------------------------------

//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = SS;

//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in bytes.
// The program creates a contiguous file with MAX_FILE_SIZE_MiB bytes.
// The file will be truncated if logging is stopped early.
const uint32_t MAX_FILE_SIZE_MiB = 100;  // 100 MiB file.
const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

// log file name
//
char fileName[10] = "test.txt";

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

ExFatFile logFile;


//------------------------------------------------------------------------------
// SD write Buffer definitions.
//    * FIFO queue of data blocks
//
const uint8_t FIFO_DIM = 8;   // # of blocks in FIFO

// data block definition
//
struct block_t {
  uint8_t overrun;      // non-zero => overrun while trying to add to this block's data
  uint16_t count;       // # of bytes of data used in this block
  uint8_t data[512];
};
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

// prototype time string
//
char strTime[] = "XXXXXXXX c\r\n";


//------------------------------------------
//
// Functions
//
//------------------------------------------

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
//  LogFlushFull - write out pending FULL data blocks to SD card
//---------------------------------------------
bool LogFlushFull()
{
  uint32_t m;
  block_t* pBlock;


  //************
  //  if full block available, write them to the SD card
  //
  if (fifoCount > 1)
  {
      // fifoCount > 1 => we have a full block ready
      //
      pBlock = &fifoBuffer[fifoTail];

      // is tail block full?
      //
      if (pBlock->count < 512)
      {
        // no -> done
        return;
      }
      
#if 0
      // Write tail block data to SD.
      //
      m = micros();
      if (logFile.write(pBlock->data, 512) != 512) {
        errorHalt("write data failed");
      }
      m = micros() - m;
      if (m > maxLatencyUsec) {
        maxLatencyUsec = m;
      }
#else
      Serial.write(pBlock->data,512);
#endif      
      bytesWritten += 512;

      // Initialize empty block & reset tail pointer
      //
      pBlock->count = 0;
      pBlock->overrun = 0;
      fifoTail = fifoTail < (FIFO_DIM - 1) ? fifoTail + 1 : 0;

      fifoCount--;

#if 0
      if (logFile.curPosition() >= MAX_FILE_SIZE) {
        // File full => report error and stop
        errorHalt("Error: Log file full! - halting.");
      }
#endif

  } // end of while loop
  
} // end of logFlushFull

//---------------------------------------------
//  LogFlushAll - flush ALL pending FIFO data
//    NOTE: only call this routine AFTER stopping all writes into the FIFO buffer
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
    // write out data from this block
    //
#if 0
    m = micros();
    if (logFile.write(curBlock->data, curBlock->count) != curBlock->count) {
      errorHalt("write data failed");
    }
    m = micros() - m;
    if (m > maxLatencyUsec) {
      maxLatencyUsec = m;
    }
#else
    Serial.write(curBlock->data,curBlock->count);
#endif    
    bytesWritten += curBlock->count;
    curBlock->count = 0;                // this block now empty
  }
  
  return(true);

} // end of LogFlushAll


//---------------------------------------------
// LogInit - initialize logging 
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool LogInit()
{

  //*****************
  // Initialize fifo buffers
  //
  fifoCount = 1;    // first block in use
  fifoHead = 0;
  fifoTail = 0;

  return(true);

} // end of LogInit
