#include "Orca.h"

/////////////////////////////////////////////////////////////////////////////////   ZERO SENSOR STATE 
int Orca::handshake( int verbose ) {  // Enables the HBridge Drivers
  
	reset_state( ); delay (1);
	
	int nfb = new_feedback();   // Parse the info state - all other frames were discarded 
	if ( nfb < 1 ) {   // but they will have been counted, so we can check that we've received them all 
		Printl("handshake communication problems"); 
		return 0;
	}
	
	else {
		if ( verbose ) Printl("handshake completed with success"); 
		return 1;
	}
}

int Orca::testConfig ( int verbose ) {//this is Spencer///////////////<-      
														// Hey buddy! Welcome!! ~//
	//verbose = 1;
	info_state( 0 ); delay (2);
	
	int nfb = new_feedback();   // Parse the info state - all other frames were discarded 
	if ( nfb < 1 ) {   // but they will have been counted, so we can check that we've received them all 
		Printl("test config communication problems");
		return 0;
	}
	else {  	                                // info was sent back and should have been handled by parseConfig()
	
		int flags = 0;		
		if ( 	Config.position_polarity	>= LOWEST_POSITION_POLARITY && 
				Config.position_polarity 	<= HIGHEST_POSITION_POLARITY ) flags |= 0x1;
		if ( 	Config.motor_winding 		>= LOWEST_MOTOR_WINDING && 
				Config.motor_winding 		<= HIGHEST_MOTOR_WINDING ) flags |= 0x2;
		if ( 	Config.Exponent 			>= LOWEST_MOTOR_WINDING && 
				Config.Exponent 			<= HIGHEST_MOTOR_WINDING ) flags |= 0x3;
		if ( 	Config.TempCode1 			>= LOWEST_TEMPCODE1 && 
				Config.TempCode1 			<= HIGHEST_TEMPCODE1 ) flags |= 0x4;
		if ( 	Config.TempCode2 			>= LOWEST_TEMPCODE2 && 
				Config.TempCode2 			<= HIGHEST_TEMPCODE2 ) flags |= 0x5;
		if ( 	Config.TempThrottle 		>= LOWEST_TEMP_THROTTLE && 
				Config.TempThrottle 		<= HIGHEST_TEMP_THROTTLE ) flags |= 0x6;
		//etc...
		
		int ret = 1;
		 
		if ( !(flags & 0x1) ) { if ( verbose ) {Print("test config: position_polarity invalid: "); 	Printl(Config.position_polarity); 	} ret = -1; } 
		if ( !(flags & 0x2) ) { if ( verbose ) {Print("test config: motor_winding invalid: "); 		Printl(Config.motor_winding); 		} ret = -1; } 
		if ( !(flags & 0x3) ) { if ( verbose ) {Print("test config: Exponent invalid: "); 			Printl(Config.Exponent); 			} ret = -1; } 
		if ( !(flags & 0x4) ) { if ( verbose ) {Print("test config: TempCode1 invalid: "); 			Printl(Config.TempCode1); 			} ret = -1; } 
		if ( !(flags & 0x5) ) { if ( verbose ) {Print("test config: TempCode2 invalid: "); 			Printl(Config.TempCode2); 			} ret = -1; } 
		if ( !(flags & 0x6) ) { if ( verbose ) {Print("test config: TempThrottle invalid: "); 		Printl(Config.TempThrottle); 		} ret = -1; } 

		if ( verbose ) {
			if ( ret > 0) Printl("test config completed with success");
			else 		Printl("test config completed with errors");
		}
		return ret; 
	}
}

int Orca::testRes (  float& rAM, float& rAP, float& rBM, float& rBP, int verbose ) {//this is Spencer///////////////<-      
														// Hey buddy! Welcome!! ~//
	//verbose = 1;
	int ret = 1;
	int flags = 0;	
	int pwm_delay = 10;

	info_state( 5 ); delay (2);
	int nfb = new_feedback();
	if ( nfb < 1 ) { Printl("test resistance communication problems"); return 0; }
    send_and_wait( OrcaState::enable_cmd);
	for (int phase =0; phase<4; phase++){
		switch (phase){
			case 0 :send_and_wait( OrcaState::pwm_cmd, TEST_BRIDGE_PWM,  0, 0, 0 ); delay (pwm_delay);
					send_and_wait( OrcaState::pwm_cmd, TEST_BRIDGE_PWM,  0, 0, 0 );
					rAP = ( PWM.VCC * ( TEST_BRIDGE_PWM /255.)) / (PWM.CurrentA * CURRENT_CONST);
					//Print("IAP_"); Printl(PWM.CurrentA);
					break;
			case 1 : send_and_wait( OrcaState::pwm_cmd, 0,TEST_BRIDGE_PWM, 0, 0 ); delay (pwm_delay);
					send_and_wait( OrcaState::pwm_cmd, 0,TEST_BRIDGE_PWM, 0, 0 );
					rAM = ( PWM.VCC * ( TEST_BRIDGE_PWM /255.)) / (PWM.CurrentA * CURRENT_CONST);
					//Print("IAM_"); Printl(PWM.CurrentA);
					break;
			case 2:send_and_wait( OrcaState::pwm_cmd, 0, 0,TEST_BRIDGE_PWM, 0 ); delay (pwm_delay);
					send_and_wait( OrcaState::pwm_cmd, 0,0,TEST_BRIDGE_PWM,  0 );
					rBP = ( PWM.VCC * ( TEST_BRIDGE_PWM /255.)) / (PWM.CurrentB * CURRENT_CONST);
					//Print("IBP_"); Printl(PWM.CurrentB);
					break;
			case 3:send_and_wait( OrcaState::pwm_cmd, 0,0, 0, TEST_BRIDGE_PWM); delay (pwm_delay);
					send_and_wait( OrcaState::pwm_cmd, 0, 0, 0,TEST_BRIDGE_PWM );
					rBM = ( PWM.VCC * ( TEST_BRIDGE_PWM /255.)) / (PWM.CurrentB * CURRENT_CONST);
					//Print("IBM_"); Printl(PWM.CurrentB);
					break;
		}
	}
	if ( 	( rAP ) 	>= LOWEST_RESISTANCE && 
			( rAP ) 	<= HIGHEST_RESISTANCE ) flags |= 0x1;
	if (   !(flags & 0x1) ) { if ( verbose ) {Print("test resistance Ap: "); 	Printl(rAP); 	} ret = -1; } 

	if ( 	( rAM ) 	>= LOWEST_RESISTANCE && 
			( rAM ) 	<= HIGHEST_RESISTANCE ) flags |= 0x2;
	if (   !(flags & 0x2) ) { if ( verbose ) {Print("test resistance Am: ");  Printl(rAM); 	} ret = -1; } 

	if ( 	( rBP ) 	>= LOWEST_RESISTANCE && 
			( rBP) 		<= HIGHEST_RESISTANCE ) flags |= 0x3;
	if (   !(flags & 0x3) ) { if ( verbose ) {Print("test resistance Bp: "); Printl(rBP); 	} ret = -1; } 

	if ( 	( rBM )		>= LOWEST_RESISTANCE && 
			( rBM ) 	<= HIGHEST_RESISTANCE ) flags |= 0x4;
	if (   !(flags & 0x4) ) { if ( verbose ) {Print("test resistance Bm: "); Printl(rBM); 	} ret = -1; } 

	pwm_state( 0, 0, 0, 0 ); delay (4);
	if ( verbose ) {
		if ( ret > 0) Printl("test resistance completed with success");
		else 		Printl("test resistance completed with errors");

	}
	return ret; 
}

int Orca::testOffsets ( int verbose ){
	info_state( 1 ); delay (2);

	int nfb = new_feedback();   // Parse the info state - all other frames were discarded 
	if ( nfb < 1 ) {   // but they will have been counted, so we can check that we've received them all 
		Printl("test offsets communication problem");
		return 0;
	}
	else {  	                                // info was sent back and should have been handled by parseConfig()
	
		int flags = 0;		
		if ( 	Offsets.RangeHallB		>= LOWEST_RANGE_HULL_B && 
				Offsets.RangeHallB 		<= HIGHEST_RANGE_HULL_B ) 		flags |= 0x1;
		if ( 	Offsets.RangeHallA 		>= LOWEST_RANGE_HULL_A && 
				Offsets.RangeHallA 		<= HIGHEST_RANGE_HULL_A ) 		flags |= 0x2;
		if ( 	Offsets.ZeroHallB 		>= LOWEST_HULL_B && 
				Offsets.ZeroHallB 		<= HIGHEST_HULL_B ) 			flags |= 0x3;
		if ( 	Offsets.ZeroHallA 		>= LOWEST_HULL_A && 
				Offsets.ZeroHallA 		<= HIGHEST_HULL_A ) 			flags |= 0x4;
		if ( 	Offsets.ZeroCurrentBm 	>= LOWEST_CURRENT_BM && 
				Offsets.ZeroCurrentBm 	<= HIGHEST_CURRENT_BM ) 		flags |= 0x5;
		if ( 	Offsets.ZeroCurrentBp 	>= LOWEST_CURRENT_BP && 
				Offsets.ZeroCurrentBp 	<= HIGHEST_CURRENT_BP ) 		flags |= 0x6;
		if ( 	Offsets.ZeroCurrentAm 	>= LOWEST_CURRENT_AM && 
				Offsets.ZeroCurrentAm 	<= HIGHEST_CURRENT_AM ) 		flags |= 0x7;
		if ( 	Offsets.ZeroCurrentAp 	>= LOWEST_CURRENT_AP && 
				Offsets.ZeroCurrentAp 	<= HIGHEST_CURRENT_AP ) 		flags |= 0x8;
		//etc...
		
		int ret = 1;
		if ( !(flags & 0x1) ) { if (verbose) {Print("test offsets: RangeHallB invalid: ");	Printl(Offsets.RangeHallB); 		} ret = -1; } 
		if ( !(flags & 0x2) ) { if (verbose) {Print("test offsets: RangeHallA invalid: "); 	Printl(Offsets.RangeHallA); 		} ret = -1; } 
		if ( !(flags & 0x3) ) { if (verbose) {Print("test offsets: ZeroHallB invalid: "); 	Printl(Offsets.ZeroHallB); 			} ret = -1; } 
		if ( !(flags & 0x4) ) { if (verbose) {Print("test offsets: ZeroHallA invalid: "); 	Printl(Offsets.ZeroHallA); 			} ret = -1; } 
		if ( !(flags & 0x5) ) { if (verbose) {Print("test offsets: ZeroCurrentBm invalid: "); Printl(Offsets.ZeroCurrentBm); 	} ret = -1; } 
		if ( !(flags & 0x6) ) { if (verbose) {Print("test offsets: ZeroCurrentBp invalid: "); Printl(Offsets.ZeroCurrentBp); 	} ret = -1; } 
		if ( !(flags & 0x7) ) { if (verbose) {Print("test offsets: ZeroCurrentAm invalid: "); Printl(Offsets.ZeroCurrentAm); 	} ret = -1; } 
		if ( !(flags & 0x8) ) { if (verbose) {Print("test offsets: ZeroCurrentAp invalid: "); Printl(Offsets.ZeroCurrentAp); 	} ret = -1; } 

		if ( verbose )  {
			if ( ret > 0) Printl("test offsets completed with success");
			else Printl("test offsets completed with errors");
		}
		return ret; 
	}
	
}



// Returns a 1 on "test passed"
//           0 on timeout 
//          -1 on fail 
int Orca::testSensors( int verbose  ) {  // Enables the HBridge Drivers

	// Here we toggle the HBridge directions to cause the driver to engage each side of the bridge,
	// and in turn, switch the current sensor channel. This will ensure we get a recent picture 
	// of the current sensors for the upcoming sensor test. This is needed because some driver 
	// firmwares do not automatically check the "noisy" edge. 
	enable_state( );			delay(1);
	pwm_state ( 0,0,0,0 );  	delay(1);
	pwm_state ( 13,0,13,0 );  	delay(1);  // engage the hbridge, and the forward current sensor
	pwm_state ( 0,0,0,0 );  	delay(10); // allow any current to settle 
	pwm_state ( 0,13,0,13 );  	delay(1);  // reverse the hbridge and engage the reverse sensor 
	pwm_state ( 0,0,0,0 );  	delay(10);
	pwm_state ( 13,0,13,0 );  	delay(1);  // engage the hbridge, and the forward current sensor
	pwm_state ( 0,0,0,0 );  	delay(10); // allow any current to settle 
	pwm_state ( 0,13,0,13 );  	delay(1);  // reverse the hbridge and engage the reverse sensor 
	pwm_state ( 0,0,0,0 );  	delay(10); 
	info_state( 5 );			delay(10);
	int nfb = new_feedback();   // Parse the info state - all other frames were discarded 
	if ( nfb < 7 ) {   // but they will have been counted, so we can check that we've received them all 
		Print("test sensors: communication problem: "); Printl(nfb);
		return 0;
	}
	else {                                      // test the response 	
		int flags = 0;
		if ( Sensors.temp > TEMP_LOW_TEST && Sensors.temp < TEMP_HIGH_TEST ) flags |= 0x1;
		if ( Sensors.current[0] > CURRENT_LOW_TEST && Sensors.current[0] < CURRENT_HIGH_TEST ) flags |= 0x2;
		if ( Sensors.current[1] > CURRENT_LOW_TEST && Sensors.current[1] < CURRENT_HIGH_TEST ) flags |= 0x4;
		if ( Sensors.current[2] > CURRENT_LOW_TEST && Sensors.current[2] < CURRENT_HIGH_TEST ) flags |= 0x8;
		if ( Sensors.current[3] > CURRENT_LOW_TEST && Sensors.current[3] < CURRENT_HIGH_TEST ) flags |= 0x10;
		if ( Sensors.vcc < VCC_HIGH_TEST && Sensors.vcc >= VCC_LOW_TEST  ) flags |= 0x20;
		
		int ret = 1;
		if ( verbose ) {
			if ( !(flags & 0x1) ) { Print("test sensors: temp out of bounds: "); 		Printl(Sensors.temp); 		ret = -1; }
			if ( !(flags & 0x2) ) { Print("test sensors: current Ap out of bounds: "); 	Printl(Sensors.current[0]); ret = -1; } 
			if ( !(flags & 0x4) ) { Print("test sensors: current Am out of bounds: "); 	Printl(Sensors.current[1]); ret = -1; } 
			if ( !(flags & 0x8) ) { Print("test sensors: current Bp out of bounds: "); 	Printl(Sensors.current[2]); ret = -1; }
			if ( !(flags & 0x10) ) { Print("test sensors: current Bm out of bounds: "); 	Printl(Sensors.current[3]); ret = -1; }
			if ( !(flags & 0x20) ) { Print("test sensors: voltage out of bounds: "); 	Printl(Sensors.vcc); 		ret = -1; }
		}	
		
		if ( !verbose ) return ret; 
		if ( flags == 0b00111111) 
			Printl("test sensors completed with success");
		else   					 
			Printl("test sensors completed with errors"); 
		return ret;
	}	
}


int Orca::testBridges( int verbose  ) {
	int ret = 1;
	int checks[4][4] = { 	{TEST_BRIDGE_PWM,0,0,0},
							{0,TEST_BRIDGE_PWM,0,0},
							{0,0,TEST_BRIDGE_PWM,0},
							{0,0,0,TEST_BRIDGE_PWM}	};
							
	//enable_state();
	
	send_and_wait( OrcaState::enable_cmd );
	
	Information.bridge_checks = 0x0F;
	for (int i = 0; i < 4; i++) {
		//enable_state();	
		
		//pwm_state( checks[i][0], checks[i][1], checks[i][2], checks[i][3] );
		send_and_wait ( OrcaState::pwm_cmd, checks[i][0], checks[i][1], checks[i][2], checks[i][3] ); 
		delay(TEST_BRIDGE_DELAY);
		
	//	new_feedback();
	//	new_feedback();

		send_and_wait ( OrcaState::pwm_cmd, 0,0,0,0 ); 
//		delay( 1 );
//		new_feedback();
		
		if ( uart.InputBuffers[address].run_command != OrcaState::pwm_cmd ) {
			if ( verbose ) { Print ("test bridges: invalid response from "); Pspace(name); Printl(uart.InputBuffers[address].run_command); }
			return -2;
		}		
		
		switch ( i ) {
			case 0: if ( uart.InputBuffers[address].data[2] < TEST_BRIDGE_CURRENT ) { if (verbose) { Print("test bridges: bridge A+ fail "); Printl(uart.InputBuffers[address].data[2]); } 
									Information.bridge_checks &= ~0x1; ret = -1;  } break;
			case 1: if ( uart.InputBuffers[address].data[2] < TEST_BRIDGE_CURRENT ) { if (verbose) { Print("test bridges: bridge A- fail "); Printl(uart.InputBuffers[address].data[2]); } 
									Information.bridge_checks &= ~0x2; ret = -1; } break;
			case 2: if ( uart.InputBuffers[address].data[5] < TEST_BRIDGE_CURRENT ) { if (verbose) { Print("test bridges: bridge B+ fail "); Printl(uart.InputBuffers[address].data[5]); } 
									Information.bridge_checks &= ~0x4; ret = -1; } break;
			case 3: if ( uart.InputBuffers[address].data[5] < TEST_BRIDGE_CURRENT ) { if (verbose) { Print("test bridges: bridge B- fail "); Printl(uart.InputBuffers[address].data[5]); } 
									Information.bridge_checks &= ~0x8; ret = -1; } break;
		}						
		delay(TEST_BRIDGE_DELAY);
	}
	reset_state(); 
	delay(2);
	new_feedback();

	if ( !verbose ) return ret; 
	if ( ret == 1) 	Printl("test bridges completed with success"); 
	else 			Printl("test bridges completed with errors"); 
	return ret;
}


int Orca::testLocation( int verbose) {
	reset_state( ); delay (1);
	int ret = 1;
	int nfb = new_feedback();   
	if ( nfb < 1 ) {   
		Printl("locater communication problem "); 
		return 0;
	}
	else {
		broadcast = 1;
		disable();
		if(send_and_wait( OrcaState::reset_cmd )) { 
			ret = -1;
			if (verbose) { Print("Orca in wrong position");}
		}
		reset();
	}
	if ( !verbose ) return ret; 
	if ( ret == 1) 	Printl("test location completed with success"); 
	else 			Printl("test location completed with errors"); 
	return ret;
}