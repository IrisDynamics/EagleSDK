#include "DualCoreActuator.h"


		//  lost connection 
		//  was reset 
		//  temp throttle 
		//  force error:    (comms timeout, critical temp, low voltage, high voltage)
		//  position error: (bad shaft image)
		//  setup error: 
		
		
		// how to notify token scheduler in a general sense? 
		// need a way to programmically link orca addersses to tokens 

	void DualCoreActuator::report_connectivity ( int verbose ) {					
	
		if ( millis() - last_connectivity_update < CONNECTIVITY_CHECK_PERIOD ) return;
		last_connectivity_update = millis();
		
		for ( int index = 0; index < NUM_CORES; index++) {
	
			//////////////////////////////////////////////////////////////////  Disconnection / Reconnection Reporting 
			if ( orcas[index]->Errors.unresponsive >= UNRESPONSIVE_ERROR_COUNT) {
				if ( !States.dc[index] ) {	 
					error_log.write( ( String (orcas[index]->name) + "_lost_connection_" + orcas[index]->Errors.missing_packet).c_str() );
					if ( verbose )  { Print(orcas[index]->name); Print(" lost connection "); Printl(orcas[index]->Errors.missing_packet); } 
				}
				States.dc[index] = 1;		
			}
			else {			
				if ( States.dc[index] ) {	 
					error_log.write( ( String (orcas[index]->name) + "_established_connection_" + orcas[index]->Errors.missing_packet).c_str() );
					if ( verbose )  { Print(orcas[index]->name); Print(" established connection "); Printl(orcas[index]->Errors.missing_packet); } 
				}
				States.dc[index] = 0;		
			}	
		}
	}
	
		
	void DualCoreActuator::report_errors ( Orca * orca, int verbose ) {					
		
		//////////////////////////////////////////////////////////////////  Reset Reporting 
		if (  orca->Warnings.reset != States.reset[orca->address] ) {			
			States.reset[orca->address] = orca->Warnings.reset;
			if (  orca->Warnings.reset ) {
				error_log.write ( (String ( orca->name) + "_detected_reset").c_str() );
				if ( verbose ) { Print( orca->name); Printl(" was reset"); }
				PackageTokens.clear_errors[orca->address] = 1;	
			}
		}
		
		//////////////////////////////////////////////////////////////////  Temp Throttling Reporting 
		if (  orca->Warnings.temp != States.temp[orca->address]  ) {			
			States.temp[orca->address]  =  orca->Warnings.temp;
			if (  orca->Warnings.temp ) {
				error_log.write ( (String ( orca->name) + "_temp_throttling").c_str() );
				if ( verbose ) { Print( orca->name); Printl(" temp high"); }				
				//new_temp_warning(orca , verbose ); 
			}
			else {				
				error_log.write ( (String ( orca->name) + "_temp_normal").c_str() );
				if ( verbose ) { Print( orca->name); Printl(" temp normal"); }				
				//temp_warning_reset( orca, verbose ) ;
			}
		}				
		
		// //////////////////////////////////////////////////////////////////  Critical Error Reporting 
		// if (  orca->uart.InputBuffers[ orca->address ].error_level != error ) {
			// error =  orca->uart.InputBuffers[ orca->address].error_level;
			// switch (  orca->uart.InputBuffers[ orca->address].error_level ) {
				// case 0: 
					// //error_cleared( orca ); 
					// break;
				// case 1: 
					// //new_force_error( orca , a_info_token, a_errors_token, a_sensors_token, a_reset_errors_token); 
					// break; 
				// case 2: 
					// //new_position_error( orca , a_errors_token, a_sensors_token, a_reset_errors_token ); 
					// break; 
				// case 3: 
					// //new_setup_error( orca ); 
					// break;
			// }
		// }		

		// error_level = max ( orca->uart.InputBuffers[orca->address].error_level , 
							// orca->uart.InputBuffers[orca->address].error_level );
		warnings = orca->Warnings.flags | orca->Warnings.flags;
		
		log_all_new();  
	}
	

	void DualCoreActuator::log_all_new () {
		log_all_new ( orcas[0] );
		log_all_new ( orcas[1] );
	}

	void DualCoreActuator::log_all_new ( Orca * orca ) {
		if ( 	 orca->Sensors.new_data 	) 	{ 
			error_log.write(  orca->Sensors.log( orca->name ) ); 
			//delay(5); 
		}
		if ( 	 orca->Errors.new_data_1 	|| 
				 orca->Errors.new_data_2 	) 	{
			error_log.write( orca->Errors.log( orca->name ) );	
			//delay(1);
		}
				
		if ( 	 orca->Config.new_data_1  ||
				 orca->Config.new_data_2 	) 	{
			error_log.write( orca->Config.log( orca->name ) ); delay(1);
			//delay(1);
		}
		if ( 	 orca->Offsets.new_data 	) {
			error_log.write( orca->Offsets.log( orca->name ) ); delay(1);
			//delay(1);
		}
	}		