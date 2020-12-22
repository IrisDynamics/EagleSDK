
#include "ThePod.h"
	
    void ThePod::talk ( ) {
	  
      orcas [ target.update() ] . broadcast = broadcast.toggled_on();
      switch ( run_command ) {		  
		case OrcaState::test_state1: 
// <<<<<<< HEAD
			// orcas [ target.update() ]. test_state1 ( in1.update(),in2.update(),in3.update(), 0 );
// =======
			orcas [ target.update() ]. test_state1 ( in1.update(),in2.update(),in3.update(), in4.update(), in5.update(), in6.update(), in7.update(),in8.update(), 0 );
			break;
		case OrcaState::testbed_study: 
			orcas [ target.update() ]. testbed_study ( in1.update(),in2.update(),in3.update(), 0 );
//>>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
			break;
        case OrcaState::reset_cmd:     orcas [ target.update() ] . reset_state( );         
			break;
        case OrcaState::enable_cmd:    orcas [ target.update() ] . enable_state( );        
			break;
        case OrcaState::pos_ctrl_cmd:    orcas [ target.update() ] . pos_ctrl_state( in1.update() * 8 );        
			break;
        case OrcaState::pos_ctrl_setup_cmd:    orcas [ target.update() ] .  pos_ctrl_setup_state( in1.update() * 8, in2.update(), in3.update()	,in4.update(), in6.update(), in7.update(), in10.update() );        
			break;
        case OrcaState::servo_cmd:     orcas [ target.update() ] . servo_state( ( in1.update() - in2.update() ) ); 
			break;
        case OrcaState::extended_servo_cmd:     orcas [ target.update() ] . extended_servo_state( ( in1.update() - in2.update() ) ); 
			break;
        case OrcaState::info_cmd:      orcas [ target.update() ] . info_state( in1.update() );        
			break;
        case OrcaState::geterror_cmd:  orcas [ target.update() ] . geterror_state( in1.update(), in2.update() );        
			break;
        case OrcaState::zero_cmd:      orcas [ target.update() ] . zero_sensors( in5.update(), in10.update() );        
			break;
        case OrcaState::config_cmd:    orcas [ target.update() ] . config_state ( in1.update(), in2.update(), in3.update(), in4.update(), in5.update(), in6.update(), in7.update() ); 
			break;
        case OrcaState::pwm_cmd:       orcas [ target.update() ] . pwm_state( in1.update(), in2.update(), in3.update(), in4.update() );           
			break;     
		case OrcaState::sensor_calibration:	   orcas [ target.update() ]. sensor_calibration ( in1.update(),in2.update(),in3.update(), in4.update(), in5.update(), in6.update(), in7.update(),in8.update(), 0 );
			break;
      }
    }

    int ThePod::listen ( uint32_t wait_period ) {
      uint32_t t_start = micros();      
      while ( !orcas [ target.update() ] . new_feedback() && micros() - t_start < wait_period );  // wait for a response or a timeout
      if ( micros() - t_start > wait_period ) return 0;
	  return 1;      
    }
