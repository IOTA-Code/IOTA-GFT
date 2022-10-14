/*
	ublox.h 
	
	includes for ublox.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__ublox_h)
  #define __ublox_h

  //++++++++++++++++++++++++++++++++++++++++
  // Definitions
  //++++++++++++++++++++++++++++++++++++++++
  #define gpsSerial Serial1

  #define INIT_TP 0             // init timepulse?

  #define gps_OK        0
  #define gps_E_RMC     1
  #define gps_E_GGA     2
  #define gps_E_DTM     3
  #define gps_E_PUBX04  4
  #define gps_E_CFGTP   5
  #define gps_E_CFGDTM  6

  //++++++++++++++++++++++++++++++++++++++++
  // function prototypes
  //++++++++++++++++++++++++++++++++++++++++

  extern int ubxInit();
  extern void ubxSend(uint8_t *, uint32_t) ;
  extern bool ubxGetAck(uint8_t *);

#endif // end of block containing entire include file
