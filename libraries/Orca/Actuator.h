#pragma once 

#include <IrisControls.h>
#include <Logger.h>
#include <Motion.h>
#include <Spring.h>
#include <Damping.h>
#include <ConstantForce.h>
#include <Periodic.h>
#include "ThePod.h"
#include "Config.h"

#include <IIRFilter.h>



/**
	Actuator objects are responsible for providing an easy-to-use C++ interface for the Iris Actuators 
	
	These objects combine the information they recieve from their various drivers into a device-state 
	
	Calling servo ( force ) will transmit to the drivers a request for forces. 
	
	Calling poll_recieve () will check the drivers' UARTs for successfully recieved frames from the Drivers,
	and will parse the information into appropriate data buffers for later or immediate access. 
	
	These two functions can and should be called in rapid succession as often as possible. 
	
	servo function should automatically observe good flow control when transmitting frames so as to keep the 
	common Rx line clear of collisions. 
	
	Actuators typically "do math" on the data they recieve from the Drivers as soon as they recieve it, and 
	leave the results in the mathResults buffer. 
	
	buffers in Orcas ( drivers ) and Actuators all include a "new_data" flag which are set whenever parsing 
	information from the Orca. These flags can be cleared to ensure data is only used once. 
	
	
*/
class  Actuator  { 

	public: 
	
		const char * name; 
		DataLogger	error_log;
		AbsolutePosition  combined_position;
		Derivative  combined_speed;
		IIRFilter temp_filtered; 
		
		
		
		// TODO: User EEPROM Settings 
		class {
			public:
			int travel = SHAFT_TRAVEL;         // Todo: load from EEPROM 
			unsigned int max_force = 0; 
			int stop_width = 0; 
			int stop_force = 0; 
		} Settings;
		
		Actuator( const char * _name ) : 	
			name ( _name ),
			error_log ( _name ), 
			combined_position ( POSITION_ALPHA, 4095 ),
			combined_speed   ( SPEED_ALPHA ),
			temp_filtered( 0.08, 25 )									
		{	 }		
		

		uint32_t tx_events = 0;
		uint32_t rx_events = 0;
		uint32_t tx_events_cnt = 0;
		uint32_t rx_events_cnt = 0;		
		
		uint32_t user_sending_period = 0; 		// manipulate away - this is in microseconds. Values too small will be ignored 
		
		int error_level = 0;  // populated by the servo command 
		int warnings = 0;
		//virtual void initialize( int verbose = 0 ) = 0; 
		virtual void servo ( int force ) = 0;         // transmit forces, flow control, detect errors, auto reconnect 
		virtual int poll_receive () = 0;             // check for information from orca, populate information buffers  
		//virtual void determine_error_level () = 0;    // consider driver error flags and determine the actuators state 

		// TODO 
		virtual float getPosition() 	{ return mathResult.position; }
		virtual void resetPosition () = 0;            // reset the poition of every driver 
		
		virtual float getTemp    () 	{ temp_filtered.update( mathResult.temp ); return temp_filtered.get(); }
		virtual int getVoltage () 	{ return mathResult.voltage; }		// units are mV 
		virtual int getTxFreq  () 	{ return mathResult.tx_freq; }
		virtual int getRxFreq  () 	{ return mathResult.rx_freq; }
		virtual int getOrcaFreq  () { return mathResult.orca_freq; }		

		int end_stops( int force ) {
			
			// if ( mathResult.position > Settings.travel + Settings.stop_width ) {
				// mathResult.max_force = 0;
				// mathResult.min_force = 0;
			// }
			// else if ( mathResult.position < Settings.stop_width ) {
				// mathResult.min_force =  Settings.stop_force - ( (float)Settings.stop_force + (float )Settings.max_force ) * 
							// (  mathResult.position /  Settings.stop_width );	
				// mathResult.max_force = Settings.max_force;
			// }
			// else  if ( mathResult.position > Settings.travel - Settings.stop_width ) {
				// mathResult.max_force = -(float)Settings.stop_force + ( (float)Settings.max_force +  (float)Settings.stop_force ) * 
							// ( ( Settings.travel - mathResult.position ) / Settings.stop_width );
				// mathResult.min_force = -Settings.max_force;
			// }
			// else { 
				mathResult.max_force = Settings.max_force;
				mathResult.min_force = -Settings.max_force;
			// } 
			
			// Pspace(name);Pspace(mathResult.min_force);Pline(mathResult.max_force);delay(100);
			return constrain ( force , mathResult.min_force, mathResult.max_force );
		}
		
		
		class {
			public:
			int force = 0; 
		} Input;
		class {
			public: 
			int force = 0;
			float position = 0;
			float speed = 0;
			float voltage = 0;
			float temp = 25;
			float orca_freq = 0; 
			int tx_freq = 0;
			int rx_freq = 0; 
			int max_force = 255, min_force = -255;
			
			int are_fresh() {
				int ret = is_fresh; 
				is_fresh = 0; 
				return ret;
			}			
			int is_fresh = 0; 
		} mathResult;      // a place where math results are stored 


	protected: 
		uint32_t extended_frame_period = EXTENDED_FRAME_PERIOD; 
		uint32_t min_sending_period = OrcaState::tx_periods(OrcaState::enable_cmd);
		
	// Run at around 100 Hz

};
