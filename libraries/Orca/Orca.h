#pragma once
#include <General_Macros.h>
#include "OrcaStates.h"
#include "UARTs.h"  
#include "Config.h"


class Orca {

  public:

	const char * name;
  
    ORCA_UART & uart; 	
    
    const int reset_pin = -1;
	const int address = -1;
		
    Orca( 
		const char * n, 
		ORCA_UART & _uart, 
		int rst_pin, 
		int addy
		) :
	  name(n), 
      uart ( _uart ), 
	  reset_pin( rst_pin ),
		address ( addy )	  
	{
      pinMode ( reset_pin, OUTPUT );     
      digitalWrite ( reset_pin, LOW);     
	}	
	
	// High level Frame Control
    // todo int begin( int verbose = 0);     // Enable drivers and start servo stream 
    // todo void servo  ( int force ) ;  // Force valid from -1023 to 1024
	// todo sleep ( int verbose = 0 );  // Make drivers safe and cease Vcc power draw
	
	// Reset pin is positive logic due to optoisolator circuit
    void reset() { 
      digitalWrite ( reset_pin, HIGH);  delay(100);  digitalWrite ( reset_pin, LOW ); delay(100);
    }
	
	void disable(){
		digitalWrite (reset_pin, HIGH); 
	}
	
	void reenable(){
		digitalWrite (reset_pin, LOW);
	}
	
    float getPos()    	{	return Information.phase_position;   }
    uint8_t getStatus() {	return Information.status;   	}
	float getVoltage () {	return Information.voltage;     }	
	float getTemp () 	{	return Information.temp;		}
	float getOrcaFreq() {	return Information.freq;		}		
	int getForce () 	{	return Input.force;				}

	
	// Simple State Commands 
    int broadcast = 0;   // use this flag to have this orca talk to all orcas on its UART. Only this orca will respond
	//when talking to orca record what broadcast is then set to 0, do your thing then, set broadcast to the previous value
    
	// OrcaTX.cpp
	int reset_state  ( int verbose = 0 );
    int enable_state ( int verbose = 0 );
    int enable ( int verbose = 0 ) { return enable_state( verbose ); };
    int pos_ctrl_state ( unsigned int position, int verbose = 0);
    int pos_ctrl_setup_state (  unsigned int position, uint8_t p_gain, uint8_t i_gain, uint8_t sat , int save_l = 0, int save_r = 0, int save_tune = 0,  int verbose = 0);
    int servo_state  ( int force, int verbose = 0 );
    int extended_servo_state  ( int force, int verbose = 0 );
    int info_state   ( int page, int verbose = 0 );
    int geterror_state ( int page, int get_errors, int verbose = 0 );
    int pwm_state    ( int,int,int,int,int verbose = 0 );
    int config_state ( int page, int, int, int, int, int, int, int verbose = 0) ;
    int zero_sensors ( int save, int blind, int verbose = 0 );
	int range_sensors( int verbose = 0 );  // saves max detected range of hall sensor readings to EEPROM ( so post-reset position reporting is glitch-free )
	int test_state1  ( int ap,int am, int bp, int bm, int K11, int K12, int K21, int K22, int verbose = 0 ); 
	int testbed_study	 ( int target_phase, int target_current, int target_study, int verbose = 0 );  
	int sensor_calibration  ( int ap,int am, int bp, int bm, int K11, int K12, int K21, int K22, int verbose = 0 ); 
	
	
	int send_and_wait( int stateID, int input1 = 0, int input2 = 0, int input3 = 0, int input4 = 0, int input5 = 0, int input6 = 0, int input7 = 0, int input8 = 0, int input9 = 0, int input10 = 0, int verbose = 0);
	

	// Procedures 
	// All procedures should return a 1 on success
	//           0 on timeout 
	//          -1 on fail 
    int init (int verbose = 0);	
	int config ( int m, int s, int ex, int t1, int t2, int th );	
	//int calibrate (); 
	
	
	// For the following, use Config.h definitions to decide pass and fail 	
    int handshake( int verbose = 0  );
	int testConfig  ( int verbose = 0  );
	int testOffsets ( int verbose = 0  );
	// todo testCalibration	
	int testSensors( int verbose = 0  );
	int testBridges( int verbose = 0  );
	int testRes	   ( float& ram, float& rap, float& rbm, float& rbp, int verbose = 0 );
	int testLocation( int verbose = 0  );
    int clearErrors( int verbose = 0  ); // zeros the Error ID counters on all Error pages

	// ** Polling for Data ** // 
    // Returns true once when a frame has been received and then false until another has been received;
	// Should be called frequently 
	// Will parse the feedback which may take some time	
	// It is good practice to check this function before sending to the Orca for the first time 
	// to catch anything it has said 
    int new_feedback ( int verbose = 0 ) {		
      int ret = uart.InputBuffers[address].new_frame;      
	  if ( ret ) {
		Errors.unresponsive = 0;
		rx_since_tx ++; 
		last_rx = micros();
		parseFeedback( verbose ); //parse frame
		uart.InputBuffers[address].new_frame = 0;
	  }
      return ret;  
    }
	uint8_t rx_since_tx = 0;
	uint32_t last_rx = 0;
	
	
	void parseFeedback( int verbose );
	//   ^^ uses vv
	inline void parseServo();
	inline void parseExtendedServo(); 
	inline void parsePWM();
	inline void parseSensorPage();
	inline void parseConfigPage(int);	
	inline void parseOffsetPage();
	inline void parseCalibPage1();
	inline void parseCalibPage2();
	inline void parseErrors(int);	
	inline void parseTestInfo();	
	inline void parseForceStudy();	
	inline void parsePositionStudy();	
	inline void parseStudySelection();
	inline void parseFredInfo();
	
    void log_raw () { // Prints the last feedback frame to the USB port
      Pspace(name); Pspace(OrcaState::name(uart.InputBuffers[address].run_command));
      for (int i = 0; i < OrcaState::response_size(uart.InputBuffers[address].run_command)-2; i++) Pspace(uart.InputBuffers[address].data[i]);
      Printl("");
    }

	
	class { 
		public: 
		int force; 
	} Input;
		
	class {
		public: 
		int reset = 0;
		int temp = 0;
		uint8_t flags = 0;
	} Warnings ;
	class { 
		public:
		float phase_position  = -1;       
		uint8_t status        = 0xFF;       
		uint32_t voltage         = -1;     		// units are mV 
		uint32_t temp            = -1;				
		uint32_t freq            = -1;
		uint8_t bridge_checks = 0;	
		// |||  will be set by last command and is based on how long this 
		// vvv  orca will take to response to the last issued command 
		uint32_t min_package_period;  
		int last_packet = -1;
	} Information; 

 	class {
		
		public: 
		uint8_t position_polarity;
		uint8_t motor_winding;	
		uint8_t Exponent;
		uint8_t TempCode1 ;
		uint8_t TempCode2;
		uint8_t TempThrottle;
		uint8_t BuildIDhigh;
		uint8_t BuildIDlow;
		uint16_t BuildID;
		uint8_t FirmwareID;
		uint32_t SerialNum;
		uint8_t max_duty =2;
		uint8_t max_current;
		//uint8_t new_data = 0;
		uint8_t new_data_1 = 0;
		uint8_t new_data_2 = 0;
		const char * log ( const char * name ) {
			if (new_data_1){
				new_data_1 = 0;
				const char * ret =  ( (String( name ) 		+ "<" +
									 position_polarity 	+ "><" +
									 motor_winding 		+ "><" +
									 Exponent 			+ "><" +
									 TempCode1 			+ "><" +
									 TempCode2 			+ "><" +
									 TempThrottle 		+ "><" +
									 BuildIDhigh 		+ "><" +
									 BuildIDlow 		+ "><" +
									 FirmwareID 		+ ">" ).c_str() ) ;
				//Print(name); Printl(" logging Config");
				return ret;
			}
			if (new_data_2) {
				new_data_2 = 0;
				const char * ret =  ( (String( name ) 	+ "<" +
									SerialNum		+ "><" +
									max_duty 			+ "><" +
									max_current 		+ ">").c_str() ) ;

				//Print(name); Printl(" logging Config");
				return ret;
			}
			return "";
		}
	} Config;
	
	class {
		public: 
		uint8_t RangeHallB;
		uint8_t RangeHallA;
		uint8_t ZeroHallB;
		uint8_t ZeroHallA;
		uint8_t ZeroCurrentBm;
		uint8_t ZeroCurrentBp;
		uint8_t ZeroCurrentAm;
		uint8_t ZeroCurrentAp;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 		+ "." +
									 RangeHallB 		+ "." +
									 RangeHallA 		+ "." +
									 ZeroHallB 			+ "." +
									 ZeroHallA 			+ "." +
									 ZeroCurrentBm 		+ "." +
									 ZeroCurrentBp 		+ "." +
									 ZeroCurrentAm 		+ "." +
									 ZeroCurrentAp 		+ "." ).c_str() ) ;
			//Print(name); Printl(" logging Offsets");
			return ret;
		}	
	} Offsets;
	
	class {
		public: 
	} Calibrations;
	
	class {	
		public:
		uint32_t unresponsive = UNRESPONSIVE_ERROR_COUNT;
		uint8_t err11 = 0;	uint8_t err12 = 0;	uint8_t err13 = 0;	uint8_t err14 = 0;	uint8_t err15 = 0;
		uint8_t err16 = 0;	uint8_t err17 = 0;	uint8_t err18 = 0;	uint8_t err19 = 0;	uint8_t err21 = 0;
		uint8_t err22 = 0;	uint8_t err23 = 0;	uint8_t err24 = 0;	uint8_t err25 = 0;	uint8_t err26 = 0;
		uint8_t err27 = 0;	uint8_t err28 = 0;	uint8_t err29 = 0;
		uint8_t new_data_1 = 0;
		uint8_t new_data_2 = 0;
		int missing_packet = -1;
		
		const char * log ( const char * name ) {
			if (new_data_1) {  
				new_data_1 = 0; 
				const char * ret =  ( (String( name ) 	+ "-" +
										 err11 			+ "-" +
										 err12 			+ "-" +
										 err13 			+ "-" +
										 err14 			+ "-" +
										 err15 			+ "-" +
										 err16 			+ "-" +
										 err17 			+ "-" +
										 err18 			+ "-" +
										 err19 		).c_str() ) ;
				PrintWarnings( name ); 
				//Print(name); Printl(" logging errors1");
				return ret;
			}   
			if (new_data_2) {
				new_data_2 = 0;
				const char * ret =  ( (String( name ) 	+ "=" +
										 err21 			+ "=" +
										 err22 			+ "=" +
										 err23 			+ "=" +
										 err24 			+ "=" +
										 err25 			+ "=" +
										 err26 			+ "=" +
										 err27 			+ "=" +
										 err28 			+ "=" +
										 err29 		).c_str() ) ;
				PrintErrors( name );
				//Print(name); Printl(" logging errors2");
				return ret;			
			}
			return "";
		}
		void PrintErrors( const char * name ) {
			if ( err21 ) { Print( name );Printl(" UART Timeout from active state"); }
			if ( err22 ) { Print( name );Printl(" Over Temperature"); }
			if ( err23 ) { Print( name );Printl(" Low Voltage"); }
			if ( err24 ) { Print( name );Printl(" High Voltage"); }
			if ( err25 ) { Print( name );Printl(" No Shaft"); }
			if ( err26 ) { Print( name );Printl(" Bad Shaft image");			}
		}
		
		
		void PrintWarnings ( const char * name ) {
			//if ( err11 > 1 )  { Print( name ); Print(" UART CRC match errors: "); Printl(err11);   }
			//if ( err12 > 1 )  { Print( name ); Print(" UART Invalid Frame Format: "); Printl(err12); } 
			//if ( err13 < 245 ) 	Printl("");	
			//if ( err14 > 0 ) 	Printl("Hall Sensor A Saturation");
			//if ( err15 > 0 )	Printl("Hall Sensor B Saturation");
			//if ( err16 < 150 ){ Print( name ); Print (" temperature throttling max duty to "); Printl (err16); }
		}
	} Errors;
	
	class {
		
		public: 
		uint16_t currentA;
		uint16_t currentB;
		uint16_t hallA;
		uint16_t hallB;
		int8_t AA;
		int8_t BA ;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 		+ "," +
									 currentA+ "," +
									 currentB+ "," +
									 hallA+ "," +
									 hallB+ ",").c_str() ) ;
			//Print(name); Printl(" logging Config");
			return ret;
		}
	} Test;	
	
	class {
		public:
		uint8_t type;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 		+ "," +
									 type+ ",").c_str() ) ;
			return ret;
		}
	} Study;
	
	class {
		
		public: 
		uint16_t CurrentA;
		uint16_t CurrentB;
		uint8_t  DutyA;
		uint8_t  DutyB;
		uint16_t Temp;
		uint16_t Voltage ;
		uint16_t Position;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 	+ " " +
									 CurrentA	+ " " +
									 CurrentB	+ " " +
									 DutyA 		+ " " +
									 DutyB		+ " " +
									 Temp 		+ " " +
									 Voltage 	+ " " +
									 Position	+ " " ).c_str() ) ;
			//Print(name); Printl(" logging Config");
			return ret;
		}
	} ForceStudy;
	
	class {
		
		public: 
		uint8_t CurrentA;
		uint8_t CurrentB;
		uint8_t HallA 	;
		uint8_t HallB	;
		uint8_t Temp 	;
		uint8_t Voltage ;
		uint8_t Position;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 	+ " " +
									 CurrentA	+ " " +
									 CurrentB	+ " " +
									 HallA 		+ " " +
									 HallB		+ " " +
									 Temp 		+ " " +
									 Voltage 	+ " " +
									 Position	+ " " ).c_str() ) ;
			//Print(name); Printl(" logging Config");
			return ret;
		}
	} PosStudy;
	
	class {
		public:
		uint16_t Phase;
		uint16_t  CurrentA;
		uint8_t  CurrentAp;
		uint8_t  CurrentAm;
		uint16_t  CurrentB;
		uint8_t  VCC;
		uint8_t  Therm;	
		
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 	+ "/t" +
									 Phase 			+ "/t" +
									 CurrentA 		+ "/t" +
									 CurrentB 		+ "/t" +
									 VCC 		+ "/t" +
									 Therm ).c_str() ) ;
			//Print(name); Printl(" logging sensors");
			return ret;
		}
		
	} PWM;	
	
	class {
		public:
		uint8_t current[4];
		uint8_t hall[2];
		uint8_t temp;
		uint8_t vcc        = 0xFF; 		
		
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) + "|" +
									 current[0] + "|" +
									 current[1] + "|" +
									 current[2] + "|" +
									 current[3] + "|" +
									 hall[0] + "|" +
									 hall[1] + "|" +
									 temp + "|" +
									 vcc ).c_str() ) ;
			//Print(name); Printl(" logging sensors");
			return ret;
		}
		
	} Sensors;
	
	class {
		
		public: 
		uint16_t currentA;
		uint16_t currentB;
		uint16_t hallA;
		uint16_t hallB;
		uint8_t new_data = 0;
		const char * log ( const char * name ) {
			new_data = 0;
			const char * ret =  ( (String( name ) 		+ "," +
									 currentA+ "," +
									 currentB+ "," +
									 hallA+ "," +
									 hallB+ ",").c_str() ) ;
			//Print(name); Printl(" logging Config");
			return ret;
		}
	} Fred;	
	

	
	static float adc_to_temp [256]; 

	protected:
	int send2( int ) ;
	int send4( int, uint8_t, uint8_t ) ;
	int send12( int, uint8_t b1 = 0, uint8_t b2 = 0, uint8_t b3 = 0, uint8_t b4 = 0, uint8_t b5 = 0, uint8_t b6 = 0, uint8_t b7 = 0, uint8_t b8 = 0, uint8_t b9 = 0, uint8_t b10 = 0) ;
	
};

extern Orca Orcas[12]; 

