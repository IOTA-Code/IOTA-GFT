/*
	logger.h 
	
	includes for logger.cpp
	
*/

//  make sure this include is invoked only once
//
#if !defined(__logger_h)
  #define __logger_h

  //******************
  // function prototypes
  //
  extern bool LogTextWrite(char *, int);
  extern bool LogFlushFull();
  extern bool LogFlushAll();
  extern bool LogInit();
  
#endif