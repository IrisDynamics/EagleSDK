#pragma once 

#include <IrisControls.h>
#include "DualCoreActuator.h" 




class ActuatorFeedbackApp : public IrisControlsAPI {
	public: 
		DualCoreActuator & actuator;
		
		DataLogger	& error_log;
				
		///////////////////////////////////////////////////////////// App Stuff 
		ExposedStatus title2;
		ExposedOutputBar	force, 	position, 	speed, temp;
		ExposedOutputValue firmware, tx_freq, rx_freq, orca_freq;
		ExposedOutputBar 	voltage;
		ExposedStatus buffer, buffer2;
		
		
		ActuatorFeedbackApp ( DualCoreActuator & act ) : 							 
			actuator 	( act ) ,
			error_log 	( act.error_log ),
			title2 		( act.name ), 
			force 		( "(10b)___Force_Command", -MAX_FORCE, MAX_FORCE, 0 ),
			position 	( "(um)______Position___", 0, act.Settings.travel, 0),
			speed		( "(mm/s)_____Speed_____", -1000, 1000, 0),
			temp 		( "(deg_C)_____Temp_____", 25, 45, 0 ),
			firmware 	( "Version", -1),
			tx_freq  	( "(Hz)__Tx_Frequency___", 0 ),
			rx_freq  	( "(Hz)__Rx_Frequency___", 0 ),
			orca_freq 	( "(Hz)Driver_Frequency_", 0 ),
			voltage 	( "(mV)______Voltage____", 20000, 28000, 0),
			buffer      ( "" ),
			buffer2     ( "" )
			{	}
		
		
		void run () {			
			
			///////////////////////////////////////////////  20 ms gate
			if ( millis() - last_gate_e > 16 ) {
				last_gate_e = millis(); 
				
				voltage.update( actuator.mathResult.voltage );
				position.update ( actuator.mathResult.position *1000);
				speed.update ( actuator.mathResult.speed );
				force.update	( actuator.mathResult.force ); 

			//}				
			///////////////////////////////////////////  100 ms gate
			//if ( millis() - last_gate_f > 100 ) {
				
				last_gate_f = millis();

				temp.update( actuator.getTemp() );
				orca_freq.update( actuator.mathResult.orca_freq );
				rx_freq.update( );
				tx_freq.update( );
				
				if  ( actuator.error_level != last_error_level ) {
					last_error_level = actuator.error_level;
					switch (actuator.error_level) {
						// case 0: title2.update( (String(actuator.name) + "").c_str() ); break; 
						// case 3: title2.update( (String(actuator.name) + ":_setup_error").c_str() ); break; 
						// case 1: title2.update( (String(actuator.name) + ":_force_error").c_str() ); break; 
						// case 2: title2.update( (String(actuator.name) + ":_position_error").c_str() ); break; 	
						// case 10: title2.update( (String(actuator.name) + ":_single_driver_active").c_str() ); break; 	
						// case 11: title2.update( (String(actuator.name) + ":_single_driver_active_with_setup_error").c_str() ); break; 	
						// case 12: title2.update( (String(actuator.name) + ":_single_driver_active_with_force_error").c_str() ); break; 	
						// case 13: title2.update( (String(actuator.name) + ":_single_driver_active_with_pos_error").c_str() ); break; 	
						// case 90: title2.update( (String(actuator.name) + ":_drivers_unresponsive").c_str() ); break; 						
					}
				}
			}
			/////////////////////////////////////////////  1000 ms gate
			if ( millis() - last_gate_g > 1000 ) {
				last_gate_g = millis();
				//Serial.send_now();
				rx_freq.update( actuator.rx_events );
				tx_freq.update( actuator.tx_events );
				actuator.tx_events = actuator.tx_events_cnt;
				actuator.rx_events = actuator.rx_events_cnt;
				actuator.tx_events_cnt = 0;
				actuator.rx_events_cnt = 0;
				
			}		
		}
		uint32_t last_gate_e = 0;
		uint32_t last_gate_f = 0;
		uint32_t last_gate_g = 0;
		
				
		void setup() {
			
			buffer2.show();
			error_log.add();			
			//start_uart_error_logs();

			rx_freq.show();
			tx_freq.show();
			orca_freq.show();
			temp.show();
			voltage.show();
			
			actuator.orcas[0]->send_and_wait ( OrcaState::info_cmd, 0 );  // get orca a config 
			actuator.orcas[1]->send_and_wait ( OrcaState::info_cmd, 0 );  // get orca b config 
			//temp.newUpperBound( 45 );
			//Orca::adc_to_temp[ min (actuator.orcas[0]->.Config.TempCode2, actuator.orcas[0]->.Config.TempCode2) ] );
			
			force.show();			
			position.show();		
			speed.show();
			firmware.show();
			title2.update( actuator.name );
			title2.show();			
			buffer.show();
			firmware.update( actuator.orcas[0]->Config.FirmwareID + actuator.orcas[1]->Config.FirmwareID );
			//delay(400);
		}
		
		void shutdown() {
			
			//error_log.close();
			//effects.shutdown();
		
			
			// Feedback
			speed.hide();
			firmware.hide();
			tx_freq.hide();
			rx_freq.hide();
			orca_freq.hide();
			voltage.hide();
			temp.hide();
			force.hide();			
			position.hide();
			title2.hide();		
			buffer.hide();
			buffer2.hide();
			//delay(400);
		}
	
		protected: 		
			int last_error_level = -1;
			
};


class ActuatorControlApp : public IrisControlsAPI {
	public: 
		DualCoreActuator & actuator; 
		
		ExposedLabel title; 
        ExposedIO center, gain;   
        SpringEffect spring;		
		
		ExposedIO damper_slider;
		DampingEffect             damper;
		
        ExposedIO force_slider;
        ConstantForce force;
		
        ExposedIO magnitude, frequency; 
        SineEffect  sine;
        
		ExposedIO 			freq;		
		ExposedButton test_servo , test_init, get_errors;
		
		ExposedLabel buffer, buffer2;
		
		ActuatorControlApp ( DualCoreActuator & act ) : 
			actuator 	( act ),
			//title 		( (String(act.name) + "_test_controls").c_str() ),
			title 		( "--------------------------------" ),
			center		( "Spring_center", act.Settings.travel*0.1, act.Settings.travel*0.9, 0), 
			gain		( "Spring_constant", 0, 75, 0),
			spring  	( act.mathResult.position, 0) ,
			damper_slider ( "Damping_constant", 0, 150, 0 ), 
			damper 		( act.mathResult.speed, 0) ,
            force_slider( "Force", -50, 50, 0),   
			magnitude	( "Sine_magnitude", 0, 150, 0),
			frequency	( "Sine_frequency", 1, 70000, 1000),
			freq  		( "Max_Tx_Frequency", 0, 10000, 10000 ),
			test_servo 	( "Test_servo", 0 ), 
			test_init 	( "Test_init", -1 ),  
			get_errors 	( "Get_errors", -1 ),
			buffer		( "" ) ,
			buffer2		( "" ) 
		{	}
	

	    int last_freq = 0;
		
		int getForce() {
			return spring.getForce() + force.getForce() + sine.getForce() + damper.getForce();
		}
		
        void run () {			
			
			if ( actuator.new_data() ) {
				spring.k = -gain.update() / 1000.;
				spring.center = center.update();
				sine.amplitude ( magnitude.update() );
				spring.update();
				damper.adjustK( -damper_slider.update() / 100000. );
				damper.update();
				force.update( force_slider.update() );
				if (last_freq != frequency.update()) {     
					sine.frequency(frequency.get() / 1000.);
					last_freq = frequency.get();
				}
			}
			
			sine.update();
			
			if (!IrisControlsAPI::isConnected()) 
				actuator.servo ( 0 );
			//if ( test_servo.toggled_on() ) {
			else actuator.servo ( getForce() );
			//}
			// else {
				// actuator.servo ( 0 );
			// }
						///////////////////////////////////////////  100 ms gate
			if ( millis() - last_gate_f > 100 ) {	
				last_gate_f = millis();								
				

				if ( test_servo.pressed() )  {
					magnitude.set(0);
					gain.set(0);
					force_slider.set(0);					
				}

				if ( get_errors.pressed() ) { 
					actuator.getErrors(1);
				}
				
				if ( test_init.pressed() ) {
					int a, b;
					actuator.initialize(1,a,b);
				}
				
				if ( freq.update() ) {
					actuator.user_sending_period = constrain ( 1000000 / freq.update(), 100, 100000000 );
				}
			}
		}
		uint32_t last_gate_f = 0; 		
		
	    void setup() {
			buffer.show();
			damper_slider.show();
			center.set(center.max/2+center.min/2);
			center.show();    gain.show();    
			force_slider.show();
			magnitude.show();
			magnitude.set(0);
			frequency.show();
			frequency.update();
			sine.start();
			freq.show();
			//title.name = (String (actuator.name) + "_test_controls").c_str();
			title.update ( (String (actuator.name) + "_test_controls").c_str() );
			title.show();		
			buffer2.show();
			
			// Buttons 
			//test_servo.show(); 
			test_init.show();
			//get_errors.show();
			test_servo.update(0);
        };
		
        void shutdown() {
			buffer.hide();
			damper_slider.hide();
			buffer2.hide();
			title.hide();
			center.hide();    gain.hide();    
			spring.k = 0;
			force_slider.hide();
			magnitude.hide();
			frequency.hide();
			sine.stop();			
			// Buttons 
			//test_servo.hide(); 
			test_init.hide();
			//get_errors.hide();
			freq.hide();
        };
		
};