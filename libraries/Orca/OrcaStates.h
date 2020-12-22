#pragma once 
#include "Config.h"


class OrcaState {
  public:

    enum OrcaStateIDs {
      reset_cmd                     = 0,     
      enable_cmd                    = 1,    
	  pos_ctrl_cmd					= 2,
	  pos_ctrl_setup_cmd			= 3,
      servo_cmd                     = 4,
      info_cmd                      = 5,              
      geterror_cmd                  = 6,      
	  extended_servo_cmd            = 7,
	  test_state1					= 8,
	  testbed_study					= 9,
      pwm_cmd                       = 11,       
      config_cmd                    = 12,                                        
      zero_cmd                      = 13, 
	  range_sensors_cmd				= 14,
	  sensor_calibration			= 15,
      
        page_0    = 0,         // 
        page_1    = 1,         // Configuration settings (temp, windings config) 
        page_2    = 2,         // Sensor calibration (zeros and peaks) 
        page_3    = 3,         // Sensor-Winding calibration :: local phase A and B 
        page_4    = 4,         // Sensor-Winding calibration :: remote phase C and D 
        page_5    = 5,         // Supervisor task list 
		page_6    = 6,		   // configuration page 1
    
    };
	
  
    static int response_size(int index) {
      switch ( index ) {
        case reset_cmd:     return 2;
        case enable_cmd:    return 2;
        case pos_ctrl_cmd:    		return 9;
        case pos_ctrl_setup_cmd:    return 9;
		case test_state1:	return 12; 
		case testbed_study:	return 12; 
        case servo_cmd:     return 4;
        case info_cmd:      return 12;
        case geterror_cmd:  return 12;
		case extended_servo_cmd: return 7;
        case pwm_cmd:       return 12;
        case config_cmd:    return 12;
        case zero_cmd:      return 12;
		case range_sensors_cmd:	return 12;
		case sensor_calibration:		return 12;
        default:            return -1;
      }
    }
    static const char * name( int index ) { 
      switch ( index ) {
        case reset_cmd:     return "sleep";
        case enable_cmd:    return "enable";
        case pos_ctrl_cmd:    		return "position_control";
        case pos_ctrl_setup_cmd:    return "position_control_setup";
		case test_state1: 	return "test_state1";
		case testbed_study: return "test_bed_study";
        case servo_cmd:     return "force_servo";
        case info_cmd:      return "get_info";
        case geterror_cmd:  return "get_errors";
		case extended_servo_cmd: return "extended_servo";
        case pwm_cmd:       return "pwm_control";
        case config_cmd:    return "config_eeprom";
        case zero_cmd:      return "zero_sensors";
		case range_sensors_cmd:	return "range_sensors";
		case sensor_calibration:		return "sensor calibration";
        default:            return "unsupported_command";
      }
    }
	
// See Config.h for these values. 
    static int tx_periods ( int index ) {
      switch ( index ) {
        case reset_cmd:     return Four2Four;
        case enable_cmd:    return Four2Four;
        case pos_ctrl_cmd:    return Four2Seven;
        case pos_ctrl_setup_cmd:    return Four2Seven;
		case test_state1:   return Twelve2Twelve;
		case testbed_study:	return Twelve2Twelve;
        case servo_cmd:     return Four2Four; //215;
        case info_cmd:      return Four2Twelve;
        case geterror_cmd:  return Four2Twelve;
		case extended_servo_cmd: return Four2Seven; //263;
        case pwm_cmd:       return Four2Twelve;
        case config_cmd:    return EEPROM_wait;
        case zero_cmd:      return EEPROM_wait;
		case range_sensors_cmd:	return EEPROM_wait;
		case sensor_calibration:		return Twelve2Twelve;
        default:            return -1;
      }      
    }
};
 
