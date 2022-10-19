/*
	usbComm.h 
	
	includes for usbComm.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__usbComm_h)
  #define __usbComm_h

  // PPS mode settings
extern int PPS_Flash_Duration_Sec;          // duration of one LED "flash" in seconds
extern int Flash_Test_Interval;					    // # of seconds between flash sequences while emitting test flashes
  

  //****************
  //   usbComm function prototypes
  //****************
extern void ReadCMD();


#endif // end of block containing entire include file

