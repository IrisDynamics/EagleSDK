#include "DualCoreActuator.h"


		void DualCoreActuator::initialize ( int verbose, int & result_a, int & result_b ) {		
				
			if ( verbose ) { 
			//Printl("___"); Print("<b>"); 
			Print ( my_name ); Print("</b>"); Printl(" initializing..."); }
			
			error_log.write ("Starting_init...");
			
			uint32_t t_error_check = millis(); 
			while ( millis() - t_error_check > 20 ) servo(0);
			delay(10);  // wait for servo to finish
			//uint32_t t_start = micros(); 
			
			
			int result;
			
			if (suppress_even) {
				Printl("...supressing flux processor P");
				result_a = -1;
				orcas[0]->disable();
			}			
			else {
				result = orcas[0]-> init( verbose == 2 ); 
				//uint32_t t_elapsed = micros() - t_start; 
				
				if ( result > 0 ) { 
					String s = orcas[0]->name;
					s += "initialized_successfully";
					error_log.write(s.c_str());
				}
				else {
					error_log.write ( (String (orcas[0]->name) + "_failed_to_initialize").c_str());
					error_log.write ( (String ("bridge_checks:_") + orcas[0]->Information.bridge_checks).c_str() );
				}
				
				if ( verbose ) { 
					if ( result > 0 ) { Printl("...flux processor P initialized."); } // P took "); Print(t_elapsed); Printl(" us"); }
					else {  Print("...init Flux Processor P failed: "); Printl(result);}
					//Printl("---");
				}
				
				result_a = result;
			}
			
			// delay(50);
			// if ( verbose ) { Print("<b>"); Print ( my_name );Print("</b>");  Printl(":: init Flux Processor Q..."); }			
			
			//t_start = micros(); 
			
			if (suppress_odd) {
				Printl("...supressing flux processor Q");
				result_b = -1;
				orcas[1]->disable();
			}
				else {
				result = orcas[1]-> init( verbose == 2 ); 
				//t_elapsed = micros() - t_start; 	
				
				if ( result > 0 ) { 
					String s = orcas[1]->name;
					s += "_initialized_successfully";
					error_log.write(s.c_str());
				}
				else {
					error_log.write ( (String (orcas[1]->name) + "_failed_to_initialize").c_str());
					error_log.write ( (String ("bridge_checks:_") + orcas[1]->Information.bridge_checks).c_str() );
				}

				if ( verbose ) { 
					if ( result > 0 ) { Printl("...flux processor Q initialzied."); } 
					//if ( result > 0 ) { Print("...init Flux Processor Q took "); Print(t_elapsed); Printl(" us"); }
					else {  Print("...init Flux Processor Q failed: "); Printl(result);}
				}	
				
				result_b = result;
				
			}
			
			
			log_all_new();
			t_error_check = millis(); 
			while ( millis() - t_error_check > 20 ) servo(0);
			
			error_log.write ("...finish_init");
			
			//if ( verbose ) 
				//Printl("===");
			
			
		}
		
		// This function both checks to see if another frame should be sent, and polls to see if new data has come back from the actuator 
		
		
		void DualCoreActuator::servo ( int force ) {
			
			//force = constrain ( force, -Settings.max_force, Settings.max_force );
			Input.force = force;
			
			my_new_data += poll_receive();		// will filter Input.Force into OutputForce
			force = OutputForce.get();
			mathResult.force = end_stops( force );	
			
			
			if ( micros() - last_send_time > send_delay ) {	
				broadcast( mathResult.force );			
				last_send_time = micros();
				send_delay = max ( user_sending_period, min_sending_period );
			}			
		}
		
		
		void DualCoreActuator::broadcast ( int force ) {
			
			generate_tokens () ;  // gated function that generates periodic tokens  
	
			if ( States.last_address == orcas[0]->address ) {			
				States.last_address = orcas[1]->address;
				if (suppress_odd) return;
				send_package( orcas[1], (force+1)/2 );
				tx_events_cnt++;	
			}
			else { // ( States.last_address == orcas[1]->address ) {			
				States.last_address = orcas[0]->address;
				if (suppress_even) return;
				send_package( orcas[0], (force)/2 );
				tx_events_cnt++;	
			}			
		}
		
		void DualCoreActuator::send_package ( Orca * orca, int force ) {	
				
			if ( orca->Errors.unresponsive ) {
				orca->broadcast = 0;
				orca->enable_state ( ); }

			else if ( PackageTokens.info[orca->address] ) {	
				PackageTokens.info[orca->address] = 0; 
				orca->broadcast = 0;
				orca->geterror_state( 0, 0 ); 	
			}
				
			else if ( PackageTokens.get_errors[orca->address] ) {
				PackageTokens.get_errors[orca->address] = 0; 
				orca->broadcast = 0;
				orca->geterror_state( 1, 0 ); 	
			}
				
			else if ( PackageTokens.get_sensors[orca->address] ) {
				PackageTokens.get_sensors[orca->address] --; 
				orca->broadcast = 0;
				orca->info_state( 5 ); 		
			}				

			else if ( PackageTokens.clear_errors[orca->address] ) {
				PackageTokens.clear_errors[orca->address] = 0; 
				orca->broadcast = 0;
				orca->geterror_state ( 2, 1 ); 
			}

			else if ( PackageTokens.extended_servo[orca->address] ) {
				PackageTokens.extended_servo[orca->address] = 0; 
				orca->broadcast = 0;
				orca->extended_servo_state ( force ); 
			}					

			else {
				orca->broadcast = 0; 		
				orca->servo_state ( force );  
			}

			min_sending_period = orca->Information.min_package_period; // Prevents servo from sending until at least this amount of time has passed 		
		}
		
		
		// Periodic Token Generator 
		void DualCoreActuator::generate_tokens () {			
			if ( micros() - last_extended_frame > extended_frame_period ) {
				last_extended_frame = micros();
				PackageTokens.extended_servo[2] = 1;
				PackageTokens.extended_servo[3] = 1;				
			}			
			// TODO: periodic other package schedules? 
		}
		
		int DualCoreActuator::poll_receive () { 
			
			int ret = 0;
			int new_orca_a = orcas[0]->new_feedback();
			rx_events_cnt += new_orca_a;
			if ( new_orca_a )  {
				
				OutputForce.update( Input.force ); 
				
				if ( orcas[0]->Information.status == OrcaState::extended_servo_cmd ) {
					perform_math( 2 ); 
					perform_math( 0 ); 
				}
				else 
					perform_math( 0 ); 	
				ret += new_orca_a;
				report_errors ( orcas[0], 1 ); /////////////////////////////////// Verbose here 
			}		
			
			int new_orca_b = orcas[1]->new_feedback();
			rx_events_cnt += new_orca_b;
			if ( new_orca_b )  {
				
				OutputForce.update( Input.force ); 
				
				if ( orcas[1]->Information.status == OrcaState::extended_servo_cmd ) {
					perform_math( 2 ); 
					perform_math( 1 ); 
				}
				else 
					perform_math( 1 ); 	
				ret += new_orca_b;
				report_errors ( orcas[1], 1 ); /////////////////////////////////// Verbose here 
			}	
			report_connectivity ( 1 ); /////////////////////////////////// Verbose here 			
			return ret; 
		}
		

		
		// Use orcaInfoA and orcaInfoB to generate mathResult 
		void DualCoreActuator::perform_math ( int what_data ) {
			switch ( what_data ) {
				case 0:
					if ( orcas[0]->Information.phase_position >= 0 ) {
						pos_a.update ( orcas[0]->Information.phase_position );
						if ( orcas[1]->Information.phase_position >= 0 ) 
							combined_position.update ( pos_a.get() + pos_b.get() ) ;
						else 
							combined_position.update ( pos_a.get() ) ;
						mathResult.position = combined_position.corrected * 50.4 / 8192;
						combined_speed.update(mathResult.position);
						mathResult.speed = combined_speed.filtered;
						mathResult.is_fresh ++;
					}
				break; 
				case 1:	
					if ( orcas[1]->Information.phase_position >= 0 ) 
						pos_b.update ( orcas[1]->Information.phase_position );
						if ( orcas[0]->Information.phase_position >= 0 ) 
							combined_position.update ( pos_a.get() + pos_b.get() ) ;
						else 
							combined_position.update ( pos_b.get() ) ;
						mathResult.position = combined_position.corrected * 50.4 / 8192;		
						combined_speed.update(mathResult.position);
						mathResult.is_fresh ++;
				break;			
				case 2:
				
					if (suppress_even) {
						mathResult.temp 		=	orcas[1]->Information.temp;
						mathResult.voltage 		=	orcas[1]->Information.voltage;
						mathResult.orca_freq 	=	orcas[1]->Information.freq;
						
					}
					else if (suppress_odd) {
						mathResult.temp 		=	orcas[0]->Information.temp;
						mathResult.voltage 		=	orcas[0]->Information.voltage;
						mathResult.orca_freq 	=	orcas[0]->Information.freq;
					}
					else {
						mathResult.temp 		=	(orcas[0]->Information.temp  
												+ 	orcas[1]->Information.temp) / 2;  // todo : linearize 
						mathResult.voltage 		=	(orcas[0]->Information.voltage  
												+ 	orcas[1]->Information.voltage) / 2; 
						mathResult.orca_freq 	=	orcas[0]->Information.freq  
												+	orcas[1]->Information.freq; 						
					}
				
					//perform_math ( 0 );
				break; 
				// case 3:
					// mathResult.temp 	= orcas[0]->Information.temp / 2 + orcas[1]->Information.temp / 2;  // todo : linearize 
					// mathResult.voltage =  ( orcas[0]->Information.voltage / 2 + orcas[1]->Information.voltage / 2); 
					// mathResult.orca_freq =  ( orcas[0]->Information.freq  + orcas[1]->Information.freq ); 
					// //perform_math ( 1 );

				break;			
			}			
		}
		
		void DualCoreActuator::sleep( int verbose ) {
			if ( verbose ) { Print(name); Printl(" sleeping"); } 
			delay(5);
			poll_receive();		
			orcas[0]->reset_state();
			delay(2);
			orcas[1]->reset_state();
			delay(2);
			poll_receive();			
		}
		
		void DualCoreActuator::enable ( int verbose ) {
			if ( verbose ) { Print(name); Printl(" enable"); }
			delay(5);
			poll_receive();
			orcas[0]->enable_state();
			delay(2);
			orcas[1]->enable_state();
			delay(2);
			poll_receive();
		}		
		
		