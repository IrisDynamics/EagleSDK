#pragma once 

#define ORCA_II_rev_0




#if defined ( ORCA_II_rev_0 ) 


//#define UART_BAUD_RATE				250000
#define UART_BAUD_RATE						500000

	// TODO: Tune these properly 
	//
	
	// Use new Pod app to find the right tune 
#if UART_BAUD_RATE == 500000
	// These are for how long Servo waits after sending a message before sending another 
	#define Four2Four		205
	#define Four2Seven		300
	#define Four2Twelve		700
	#define Twelve2Twelve	700  
	#define EEPROM_wait		200000
#elif UART_BAUD_RATE == 250000   
	// These are for how long Servo waits after sending a message before sending another 	
	#define Four2Four		600///300//203 // 183 //175 //203 // 225 //191
	#define Four2Seven		900//390//263 // 243 // 285
	#define Four2Twelve		1300//500//363 // 343 //405//393
	#define Twelve2Twelve	1000  
	#define EEPROM_wait		50000
#endif 


#define CONNECTIVITY_CHECK_PERIOD			10

#define SHAFT_TRAVEL						150000

#define CRC_PERIOD    						1000

#define CRC_MIN_REPORTING_SUM				2    // 5% loss at 2000 Frame rate 


#define MAX_FORCE							510
#define OUTPUT_FORCE_FILTER_ALPHA 			1 // .0625
#define POSITION_ALPHA						.3
#define SPEED_ALPHA							.003

#define UNRESPONSIVE_ERROR_COUNT			10

#define SENSOR_SAMPLES_AFTER_FORCE_ERROR    2
#define SENSOR_SAMPLES_AFTER_POS_ERROR      0

#define EXTENDED_FRAME_PERIOD               10000 // 500000

#define TEMP_LOW_TEST 	 					5 	
#define TEMP_HIGH_TEST 		 				250
#define CURRENT_LOW_TEST 	 				28 - 16
#define CURRENT_HIGH_TEST 					28 + 16
#define VCC_LOW_TEST 		 				0
#define VCC_HIGH_TEST 						250

#define TEST_BRIDGE_PWM						80
#define TEST_BRIDGE_DELAY					12
#define TEST_BRIDGE_CURRENT					30/4

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

#define LOWEST_POSITION_POLARITY			1
#define HIGHEST_POSITION_POLARITY			4

#define LOWEST_MOTOR_WINDING				1
#define HIGHEST_MOTOR_WINDING				8

#define LOWEST_EXPONENT						100
#define HIGHEST_EXPONENT					150

#define LOWEST_TEMPCODE1					39
#define HIGHEST_TEMPCODE1					60

#define LOWEST_TEMPCODE2					40
#define HIGHEST_TEMPCODE2					200

#define LOWEST_TEMP_THROTTLE				0
#define HIGHEST_TEMP_THROTTLE				150

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

#define CURRENT_CONST						0.017//0.015932 (find accurate value with resistors)

#define LOWEST_RESISTANCE					1
#define HIGHEST_RESISTANCE					1.2

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

#define LOWEST_CURRENT 					1
#define HIGHEST_CURRENT 				40


#define LOWEST_HULL_B					120
#define HIGHEST_HULL_B					136

#define LOWEST_HULL_A					120
#define HIGHEST_HULL_A					136

#define LOWEST_CURRENT_BM				LOWEST_CURRENT
#define HIGHEST_CURRENT_BM				HIGHEST_CURRENT

#define LOWEST_CURRENT_BP				LOWEST_CURRENT
#define HIGHEST_CURRENT_BP				HIGHEST_CURRENT

#define LOWEST_CURRENT_AM				LOWEST_CURRENT
#define HIGHEST_CURRENT_AM				HIGHEST_CURRENT

#define LOWEST_CURRENT_AP				LOWEST_CURRENT
#define HIGHEST_CURRENT_AP				HIGHEST_CURRENT

#define LOWEST_RANGE_HULL_B				0
#define HIGHEST_RANGE_HULL_B			255

#define LOWEST_RANGE_HULL_A				0
#define HIGHEST_RANGE_HULL_A			255

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//




#define VOLTAGE_CONVERSION					(4./1023.*2.048*(270+4700)/270.)//.1043

#define FREQ_CONVERSION						20
#endif 