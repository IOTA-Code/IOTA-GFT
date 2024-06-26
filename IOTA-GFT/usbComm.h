/*
	usbComm.h 
	
	includes for usbComm.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__usbComm_h)
  #define __usbComm_h

  // PPS mode settings
extern int Flash_Duration_Sec;              // duration of one LED "flash" in seconds
extern int Pulse_Count;

  //****************
  //   usbComm function prototypes
  //****************
extern void ReadCMD();
extern void ClearSerialInput();

#endif // end of block containing entire include file

