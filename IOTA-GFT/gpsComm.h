/*
	gpsComm.h 
	
	includes for gpsComm.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__gpsComm_h)
  #define __gpsComm_h

  //*************************
  // NMEA return codes
  //*************************
  //#define NMEA_ERROR    0
  #define NMEA_UNKNOWN  1
  #define NMEA_RMC      2
  #define NMEA_GGA      3
  #define NMEA_DTM      4
  #define NMEA_PUBX04   5

  #define NMEA_ERROR_MISC	-1

  #define NMEA_ERROR_GGA_LAT -10
  #define NMEA_ERROR_GGA_NSLEN -11
  #define NMEA_ERROR_GGA_NS -12
  #define NMEA_ERROR_GGA_LONG -13
  #define NMEA_ERROR_GGA_EWLEN -14
  #define NMEA_ERROR_GGA_EW -15
  #define NMEA_ERROR_GGA_ALT -16
  #define NMEA_ERROR_GGA_ALTULEN -17
  #define NMEA_ERROR_GGA_ALTU -18
  #define NMEA_ERROR_GGA_GEOID -19
  #define NMEA_ERROR_GGA_GULEN -20
  #define NMEA_ERROR_GGA_GU -21

  #define NMEA_ERROR_RMC_STATUSLEN -30
  #define NMEA_ERROR_RMC_TIMELEN -31
  #define NMEA_ERROR_RMC_HH -32
  #define NMEA_ERROR_RMC_MM -33
  #define NMEA_ERROR_RMC_SS -34
  #define NMEA_ERROR_RMC -35
  #define NMEA_ERROR_RMC_DAYLEN -36
  #define NMEA_ERROR_RMC_DAY -37
  #define NMEA_ERROR_RMC_MON -38
  #define NMEA_ERROR_RMC_YR -39

  #define NMEA_ERROR_DTM -50

  #define NMEA_ERROR_PUBX04_TIMELEN -60
  #define NMEA_ERROR_PUBX04_HH -61
  #define NMEA_ERROR_PUBX04_MM -62
  #define NMEA_ERROR_PUBX04_SS -63
  #define NMEA_ERROR_PUBX04_OFFSETLEN -64
  #define NMEA_ERROR_PUBX04_OFFSETPARSE -65
  #define NMEA_ERROR_PUBX04_NOD -66
  
  #define NMEA_ERROR_SENTEND -67
  #define NMEA_ERROR_CHKSUM_WRONG -68
  #define NMEA_ERROR_CHKSUM_MISSING -69
  #define NMEA_ERROR_NOCRLF -70
  #define NMEA_ERROR_TOOSHORT -71

  //*************************
  // NMEA sentence data structures
  //*************************
  struct nmeaRMC {
    bool valid;
    char status;          // A  => valid fix
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t yr;
    uint8_t mon;
    uint8_t day;
  };

  #define MAX_LATLONG 12
  #define MAX_ALT 10
  struct nmeaGGA {
    bool valid;
    uint8_t lat[MAX_LATLONG];     // latitude
    uint8_t NS;                   // North / South indicator for latitude
    uint8_t lng[MAX_LATLONG];     // longitude
    uint8_t EW;                   // East / West indicator for longitude
    uint8_t alt[MAX_ALT];         // MSL altitude
    uint8_t alt_len;              // length of alt field
    uint8_t alt_units;            // units for altitude (should be m for meters)
    uint8_t geoid_sep[MAX_ALT];   // geoid separation (N)
    uint8_t sep_units;            // units for geoid separation 
  };
 

  struct nmeaDTM {
    bool valid;
    uint8_t local_datum[3];
  };
 
  struct ubxPUBX04 {
    bool valid;
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t usLeapSec;      // current leap seconds
    bool  blnLeapValid;     // true => leap seconds is up to date, false => using firmware default value
    uint8_t cLeap[3];       // leap seconds field from sentence
  };

  // 
  // parsed NMEA sentence data
  //
  extern struct nmeaRMC gpsRMC;
  extern struct nmeaGGA gpsGGA;
  extern struct nmeaDTM gpsDTM;
  extern struct ubxPUBX04 gpsPUBX04;

  // Misc externals
  //
  extern unsigned long tk_GPSRMC;   // time (ticks) for start of current RMC data (if valid)
  extern unsigned long tk_PUBX04;   // time of start of current PUBX04 data (if valid)
  

  //****************
  //   gpsComm functions
  //****************
  extern bool gpsCommInit();
  extern int ReadGPS();

#endif // end of block containing entire include file


