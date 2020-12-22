#ifndef _GERNAL_MACROS_H_
#define _GERNAL_MACROS_H_

#include "Arduino.h"
				
				// // M A C R O S 
 
#define TIMESTAMP Serial.print(micros()); Serial.print("\t");

// Use in periodically called function calls to prevent high frequency access
// Only call in functions that are statically declared or declared only once; 
// these macros use static variables
#define ACCESS_TIMER(__ACCESS_PERIOD__)   			{static uint32_t update_time = 0;                                     \
																								 if ((uint32_t)(micros() - update_time) < __ACCESS_PERIOD__) return 0;\
																								 update_time = micros();                                              \
																								}
#define ACCESS_TIMER_NR(__ACCESS_PERIOD__)			{static uint32_t update_time = 0;                                     \
																								 if ((uint32_t)(micros() - update_time) < __ACCESS_PERIOD__) return;  \
																								 update_time = micros();                                              \
																								}
#define ACCESS_TIMER_BLOCKING(__ACCESS_PERIOD__){static uint32_t update_time = 0;                                     \
																								 while ((uint32_t)(micros() - update_time) < __ACCESS_PERIOD__) ;     \
																								 update_time = micros();                                              \
																								}
																								 
// Serial Printing Shortcuts																								 
#define Print   Serial.print                                                     
#define Printl  Serial.println  
#define Pline   Printl
#define Pspace(_ARG_)  {Print(_ARG_);Print(" ");}
#define Tab     Serial.print("\t");
#define Line    Serial.println("");
#define Ptab(_ARG_) {Print(_ARG_);Tab;}    


#endif




