/*
  Logger

  Routines for logging timing data
  
  * write text or data into logging buffer
  * output buffer contents to Serial (USB)


*/

#include <Arduino.h>
#include "gpsComm.h"
#include "logger.h"
#include "iota-gft.h"

//---------------------------------------
//  GLOBALS
//---------------------------------------


// Misc
//
bool blnLogEnable = false;        // enable logging
bool blnLogEXP = true;            // log EXP events
bool blnLogToSerial = true;       // echo log to serial port


//------------------------------------------------------------------------------
// Buffer definitions.
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
volatile uint16_t fifoTail = 0;  // Only accessed by log writer.

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
  if (!blnLogEnable || !blnLogToSerial)
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
//  LogFlushFull - write out ONE pending FULL data blocks 
//      true iff block written
//---------------------------------------------
bool LogFlushFull()
{
  block_t* pBlock;

  //************
  //  if full block available, output it
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

    // write the log buffer data 
    //
 
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


  // first write out all pending FULL data blocks
  //
  while (LogFlushFull());

  // Does the head block contain any data?
  //
  if (curBlock->count > 0)
  {
 
    // write out data from this partial block
    //
    if (blnLogToSerial)
    {
      // writing log buffer to serial port
      Serial.write(curBlock->data, curBlock->count);
    }
   
    bytesWritten += curBlock->count;
    curBlock->count = 0;                // head block now empty
  }

  return(true);

} // end of LogFlushAll


//---------------------------------------------
// LogInit -  clear buffers
//
// Returns:
//  True if no issue, False if overrun
//---------------------------------------------
bool LogInit()
{

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

} // end of LogInit

