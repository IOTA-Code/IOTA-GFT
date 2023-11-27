/*
	logger.h 
	
	includes for logger.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__logger_h)
  #define __logger_h

  //********
  //  debug
  //************
extern volatile uint16_t fifoCount; // volatile - shared, ISR and background.
extern volatile uint16_t fifoHead;  // Only accessed by buffer writer.
extern volatile uint16_t fifoTail;  // Only accessed by sd writer.

// control variables
//
extern bool blnLogEnable;   // true => logging is enabled
extern bool blnLogEXP;      // log EXP events
extern bool blnLogToSerial;   // flase => serial port logging is enabled

// data block definition
//
extern const uint8_t FIFO_DIM;   // # of blocks in FIFO

struct block_t {
  uint8_t overrun;      // non-zero => overrun while trying to add to this block's data
  uint16_t count;       // # of bytes of data used in this block
  uint8_t data[512];
};
extern const uint16_t BLOCK_SIZE;

extern block_t fifoBuffer[];      // allocate buffer space


  //******************
  // function prototypes
  //
  extern bool LogTextWrite(char *, int);
  extern bool LogFlushFull();
  extern bool LogFlushAll();
  extern bool LogInit();


#endif