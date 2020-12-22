#include "ThePod.h"
#include "Orca.h"
#include "UARTs.h"
#include "Actuator.h"


UART2_Class UART2;   
UART1_Class UART1; 

Orca ThePod::orcas [ ThePod::NUMBER_OF_ORCAS ] = {
    { "O0", 	UART1,  -1, 0 },   // the first four use UART1 to compile but should be changed to UART0 when this is implemented 
    { "O1", 	UART1,  -1, 1 },
    { "O2", 	UART1,  -1, 2 },
    { "O3", 	UART1,  -1, 3 },
    { "O4", 	UART1,  28, 0 },
    { "O5", 	UART1,  27, 1 },
    { "O6", 	UART1,  28, 2 },   
    { "O7", 	UART1,  27, 3 },
    { "O8", 	UART2,  14,	0 },
    { "O9", 	UART2,  2,	1 },
    { "O10", 	UART2, 	14,	2 },
    { "O11", 	UART2, 	2,	3 } 
  };

 

uint8_t CRCFast::table [256];
