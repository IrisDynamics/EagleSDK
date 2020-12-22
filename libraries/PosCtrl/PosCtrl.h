/**
* @file PosCntl.h
* @author Kyle Hagen <khagen@irisdynamics.com> 
* @version 1.1 
* 
* 
* Copyright 2019 Iris Dynamics Ltd. All rights reserved. 
* 
* 
* @mainpage Introduction 
* 
* The position controller library enables proportional plus integral plus damping control 
* of a generic device. 
* 
* The IrisControls library is used to implement a GUI page which can be displayed or hidden 
* as usual with setup and shutdown calls. 

@Revision Notes 
r1.1	Oct 2020
Made controller output forces in mN 
Removed accumulations and decay rate settings for i action
Made gains use real units 
Retuned max gains 

*/


#pragma once 



#define K20_POSITION_CONTROLLER 

#if defined (K20_POSITION_CONTROLLER) 

#include "IrisControls.h"
#include "Exposed.h"
#include "Settings.h"
#define TIME  uint32_t 

#define SLIDER  ExposedIO 
#define LABEL   ExposedLabel
#define BUTTON  ExposedButton 


#define FORCE_RANGE     (169000)				// mN
#define MAX_PGAIN		8000					// mN/mm
#define MAX_IGAIN  		65000    				// 2mN/mm-s
#define MAX_DGAIN 		 1000    // force over speed 

#endif 



/**
* The position controller object is responsible for handing the closed loop position control 
* for a single actuator. 
* 
* The controller objects includes a GUI server using the IrisControlsAPI for slider inputs 
* and outputs, which allow tuning the controller at runtime when used with the IrisControls application. 
* 
* Position Feedback is provided via reference to the object: ie this object watches a position variable 
* thats been stored and updates elsewhere 
* 
* Position Input is passed to the obect in the setpoint() function and valid arguments are stored here. 
* Position inputs time out after a delay, so even if a constant position is required, the setpoint 
* function must be called regularily or the controller will timeout and forces will cease. 
* 
*/
class PosCtrl : public IrisControlsAPI  {
public:   

	/// The nubmer of the bytes that saved settings for this object takes 
	static const uint32_t eeprom_size = 24;  // bytes 
	static const uint32_t my_timeout_period = 100; // milliseconds 
	
	const char * my_name; 


	PosCtrl 
	( 
		const char * _name,
		float & fb_ref,
		float & _speed_ref,
		float range,
		uint32_t settings_address
	) :
		my_name(_name),
		gui 
		{
			{_name},

			// Results Section 
			{"__Setpoint___(um)", 0, (int)range*1000, -1},
			{"__Feedback___(um)", 0, (int)range*1000, -1},
			{"____Force____(mN)", -FORCE_RANGE, FORCE_RANGE, 0},

			// Proportional Section 
			{"Proportional"},
			{"___Gain______(N/m)", 0, MAX_PGAIN, 0},
			{"Saturation____(mN)", 0, FORCE_RANGE, 0},
			{"__Action______(mN)", -FORCE_RANGE, FORCE_RANGE, 0},

			// Integral Section 
			{"Integral"},
			{"___Gain____(2N/m-s)", 0, MAX_IGAIN, 0},
			{"Saturation_____(mN)", 0, FORCE_RANGE, 0},
			{"Target_width___(um)", 0, 10000, 0},
			{"__Action_______(mN)", -FORCE_RANGE, FORCE_RANGE, 0},

			// Damping Section 
			{"Damping"},
			{"___Gain____(N-s/m)", 0, MAX_DGAIN, 0},
			{"Saturation____(mN)", 0, FORCE_RANGE, 0},		
			{"__Action______(mN)", -FORCE_RANGE, FORCE_RANGE, 0},	

			// Buttons 
			{"Save_Settings", -1},
			{"Load_Settings", -1},
		},
		eeprom_start(settings_address),
		pos_feedback( fb_ref ),
		speed_feedback(_speed_ref)

	{
		set_range(range);  
	}


	struct GUI {
		LABEL title;
		SLIDER setpoint; 
		SLIDER feedback; 
		SLIDER result; 

		LABEL ptitle;
		SLIDER pgain; 
		SLIDER psat; 
		SLIDER paction; 

		LABEL ititle; 
		SLIDER igain; 
		SLIDER isat; 
		SLIDER target_width;
		SLIDER iaction; 

		LABEL dtitle; 
		SLIDER dgain; 
		SLIDER dsat; 
		SLIDER daction;

		BUTTON save; 
		BUTTON load; 

		TIME last_fast_update;
		TIME last_slow_update; 
		int is_active; 
	} gui; 



	void setup();
	void shutdown();
	void run_gui();

	void save_settings ();
	void load_settings ();

	enum State {
		RESET,
		ACTIVE,
		TIMEOUT
	};


	int setpoint( float target ) {
		if ( target <= my_range && target >=0 ) {
			my_target = target;
			refresh_timeout(); 
			return 1;
		}
		return 0;
	}

	/**
	* Returns the sum of the controllers' force 
	*/
	float get_force() {
		if ( is_enabled() && !is_timed_out() )	return constrain( p_force + i_force + d_force, -FORCE_RANGE, FORCE_RANGE); 
		return 0;
	}

	void run_control() {

		run_p();
		run_i();
		run_d(); 
		
	}
	
	void run_p();	
	void run_i();
	void run_d();
	

	/**
	 *  @brief Sets the travel range that the position controller will control 
	 *  
	 *  @param [in] range the travel range in mm 
	 */
	void set_range ( float range ) {
		if (range < 0) range = 0; 
		my_range = range; 
		gui.setpoint.newUpperBound(range*1000); 
		gui.feedback.newUpperBound(range*1000); 
	}

	/**
	* This function will need to be implemented differently depending on the time functions available 
	*/
	void refresh_timeout () {
		my_last_update_time = millis();
	}

	int is_timed_out() { 
		if ( (uint32_t)(millis() - my_last_update_time) > my_timeout_period ) return 1;
		return 0; 
	}

	int parse ( char * command, char * args) {
		START_PARSING
		FINISH_PARSING 
	}
	
	void enable() {
		enabled = 1; 
	}
	void disable() {
		enabled = 0; 
	}
	int is_enabled() {
		return enabled;
	}


protected: 

	State state = RESET; 

	int enabled = 0;
	uint32_t my_last_update_time = 0; 

	/// The location in EEPROM of the saved settings 
	const uint32_t eeprom_start; 

	/// The last received position target (from setpoint()) 
	float my_target;
	/// reference to the actuator's calculated position value 
	float &pos_feedback; 
	/// reference to the actuator's calculated speed value 
	float &speed_feedback; 	
	/// The range of acceptable positions 
	float my_range; 
	/// The accumulated errors for integral control 
	float my_integral; 



	/// The output of the damping effect 
	float d_force; 


	/**
	* Stores the gain setting and converts it into the actual gain used in calculating paction 
	*/
	void set_pgain( uint16_t gain ) {
		p_gain_setting = gain;
		p_gain = -p_gain_setting;
	}

	/// The gain of the proportional controller 
	uint16_t p_gain_setting; 
	float p_gain;
	/// The saturation of the proportional controller 
	float p_sat; 
	/// The output of the proportional control
	float p_force; 



	/**
	* Stores the gain setting and converts it into the actual gain used in calculating paction 
	*/
	void set_igain( uint16_t gain ) {
		i_gain_setting = gain;
		i_gain = 2 * i_gain_setting;
		calc_max_i_error();
	}

	/**
	* Stores the saturation setting and calculates the maximum error accumulation 
	*/
	void set_isat( float sat) {
		i_sat = sat;
		calc_max_i_error();
	}

	/**
	* When the gain or saturation force is changed, this should be called to limit 
	* the error accumulation 
	*/
	void calc_max_i_error() {
		i_error_max = i_sat / i_gain; 
	}    

	/**
	*  @brief Converts the saved setting and sets the target width local working variable 
	*  
	*  @param [in] tw the target width in um 
	*  
	*  @details More details
	*/
	void set_target_width( uint16_t tw ) {
		target_width_setting = tw; 
		target_width = tw / 1000.;
	}

	/// The integrer representation of the gain integral gain 
	uint16_t i_gain_setting; 
	/// The floating point representation of the integral gain 
	float i_gain;
	/// The floating point representation of the integral saturation force
	float i_sat;
	/// The maximum error accumulation to reach the saturation force 
	float i_error_max; 
	/// The integer representation of the accumulation rate 
	uint16_t accu_rate_setting; 
	/// The floating point representation of the accumulation rate in persentage per second 
	float accu_rate;
	/// The integer representation of the delay rate 
	uint16_t decay_rate_setting; 
	/// The floating point representation of the delay rate in persentage per second 
	float decay_rate;
	/// The number of um on either side of the target that is ignored by the i controller 
	uint16_t target_width_setting; 	
	/// The number of pixels on either side of 
	float target_width; 	

	/// The accumulated error 
	float i_error; 
	/// The millisecond time the last integral control was called 
	uint32_t last_i_calc;

	/// The output of the integral control
	float i_force;
	
	
	
	/**
	* Stores the gain setting and converts it into the actual gain used in calculating paction 
	*/
	void set_dgain( uint16_t gain ) {
		d_gain_setting = gain;
		d_gain = -d_gain_setting;
	}

	/**
	* Stores the saturation setting and calculates the maximum error accumulation 
	*/
	void set_dsat( uint16_t sat) {
		d_sat = sat;
	}
	
	
	/// The integer representation of the damping gain 
	uint16_t d_gain_setting; 
	/// The floating point representation of the dampng gain (transforms speed to force) 
	float d_gain;
	/// The floating point representation of the damping saturation force
	float d_sat;	
	

	TIME last_command_received; 


};

