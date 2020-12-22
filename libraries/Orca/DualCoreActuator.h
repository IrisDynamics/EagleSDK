#pragma once 

#include "Actuator.h"
#include <IIRFilter.h>


//#define ERROR_CHECKING_PERIOD    200
#define NUM_CORES 2
#define NUM_ADDRESSES 4


class DualCoreActuator : public Actuator {
	public: 
	
	
		int suppress_even = 0;
		int suppress_odd  = 0;

	
		// These will convert each core's incremental position output into 
		// an integrated and normalized output to be used with force effects
		// and for user applications 
		IncrementalPosition pos_a, pos_b;
		
		// Orca cores changed from references to pointer array 
		Orca * orcas [NUM_CORES];      

		IIRFilter OutputForce;    // We can filter the forces sent to the actuators to prevent us from sending highly noisy effects 
			
		
		const char * my_name;		
	
		
		DualCoreActuator( const char * name, Orca *o_a, Orca *o_b ) : 
			Actuator ( name ), 
			pos_a (1,4095), pos_b (1,4095),
			OutputForce ( OUTPUT_FORCE_FILTER_ALPHA, 0 )
		{	
			my_name = name; 
			orcas[0] = o_a;
			orcas[1] = o_b;
			for (int i = 0; i < NUM_ADDRESSES; i ++ ) {
				States.dc[i] = 1;
				States.reset[i] = 0;
				States.temp[i] = 0;
			}
		}
		
		
		// ** Init ** // 
		// This functions checks all components of the Actuator that 
		// it can without human interaction. 
		// This function should be quick and its performance should 
		// be specified when a stable version is obtained. 
		void initialize ( int verbose, int & ra, int & rb ) ;
		void reset () {
			orcas[0]->reset();
			orcas[1]->reset();
			resetPosition();
		}

		// ** Translating Orca to Human ** // 
		// Here are some functions to get information from the Orca 
		// printed to your serial terminal in English 
		int getErrors ( int verbose = 0 );
		int clearErrors ( ) ;  // todo 
		
		int config ( int m1, int s1, int m2, int s2, int ex, int t1, int tf, int th, int verbose );   // in DualCoreActuator.cpp
		
		// Todo: remove 
		void resetPosition() {
			combined_position.reset( 50000 + pos_a.get() + pos_b.get() );
		}
		
		// ** Servo ** // 
		//  This function takes a force command and takes care of the rest. 
		//  Deals with flow control, error detection, logging and resetting
		void servo ( int force ) ;
		int poll_receive () ;
		void sleep ( int verbose = 0);  // attempt to put drivers to sleep 
		void enable ( int verbose = 0 );

		int new_data() {
			int ret = my_new_data; 
			my_new_data = 0;
			return ret; 
		}
		int my_new_data = 0;
		
	protected: 
		
		
		
		// ** Error Levels ** //
		//  Error levels of both cores is tracked. When there is a change in 
		//  levels, a number of information requests are sent to the core.
		//  An attempt to clear the errors is made following the information.
		void getErrors( Orca * orca, int verobse = 0 );
		void report_errors ( Orca * orca, int verbose );   // used to report errors and warnings recieved from drivers 
		void report_connectivity ( int verbose = 0 );
		uint32_t last_connectivity_update = 0;
		
		// These are various error and other state markers 
		class {
			public: 
			int dc		[NUM_ADDRESSES];   // 1 = dc'd, 0 = connected 
			int reset	[NUM_ADDRESSES];   // 1 = was reset, 0 = wasnt reset 
			int temp	[NUM_ADDRESSES];   // 1 = throttling, 0 = no throttling 
			int errors	[NUM_ADDRESSES];   // 0 = no errors, 1 = force errors, 2 = position errors, 3 = invalid
			int last_address = 0 ;  // address of the Lask Orca communicated with 
		} States;
		

		// These tokens are used to schedule certain packages to be sent to the cores 
		// See broadcast() function to see how these are prioritized 
		class {
			public:
			int extended_servo 	[NUM_ADDRESSES];
			int info		 	[NUM_ADDRESSES];
			int get_errors	 	[NUM_ADDRESSES];
			int get_sensors		[NUM_ADDRESSES];
			int clear_errors 	[NUM_ADDRESSES];
		} PackageTokens;
	
		void generate_tokens () ;
		uint32_t last_extended_frame = 0;
		
		// called by servo()
		void broadcast  ( int force );	
		// called by broadcast() 
		void send_package ( Orca * orca, int force );
		uint32_t last_send_time = 0;
		uint32_t send_delay = 0;  
		
		
		// This should be called following recieving a new data packets. 
		// This is called by poll_recieve() 
		// This will integrate the position information, calculate speeds
		// and other derivative information. 
		void perform_math ( int what_data ) ;
	
		// ** Error Logging ** // 
		//  Calling the following function (which is called after determining 
		//  error levels ), checks the cores' data buffers and logs any new 
		//  information. Raising a data buffer's "new_data" flag will cause 
		//  it to be logged.
		void log_all_new ();
		void log_all_new ( Orca * orca );
		
		

		
		
		
		
		
		//uint32_t last_error_check  = 0;
		//uint32_t error_check_period = ERROR_CHECKING_PERIOD; 

/*
		int core_a_dc = 0;
		int core_b_dc = 0;
		int core_a_error_level = 0;
		int core_b_error_level = 0;
		
		
		int core_a_reset = 0;
		int core_b_reset = 0;
		
		void new_temp_warning ( Orca & orca, int verbose = 0 ); 
		void temp_warning_reset ( Orca & orca, int verbose = 0 ); 
		int core_a_temp = 0;
		int core_b_temp = 0;
		
		
		void error_cleared( Orca & orca ) ;
		void new_force_error( Orca & orca, int & info_token, int & get_token, int & sensor_token, int & reset_token );
		void new_position_error( Orca & orca, int & get_token, int & sensor_token, int & reset_token );
		void new_setup_error( Orca & orca ) ;
*/ 
		// int  // tokens:   inc or set these to fire priority messages to the orcas 
			// a_extended_token,	a_info_token, 	a_errors_token, 	a_sensors_token,	a_reset_errors_token, 
			// b_extended_token, 	b_info_token, 	b_errors_token, 	b_sensors_token,	b_reset_errors_token;		
		
		// ** Incoming Communications ** // 
		//  All information that is successfully passed through the UARTs frame 
		//  and CRC checking, will be distributed to the Orca objects that this 
		//  class has access to. This class will use those data buffers to perform
		//  math to calculate the speed, acceleration and other dynamics. 
		//  Poll receive is called from the servo function and will perform math 
		//  whenever new data for one of its cores is detected. 
		// int poll_receive () ;
	
	
	
};



extern DualCoreActuator actuator, actuator2;