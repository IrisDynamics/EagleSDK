#include "Orca.h" 


void Orca::parseFeedback( int verbose ) {
	//cli();
	
	
	if ( uart.InputBuffers[address].locked != 0 ) { 
		Print(name); Printl(" parser called while uart is receiving frame"); 
		delay(20);
		return;
	}
	
	uart.InputBuffers[address].locked = 1; 
	
	// TODO: checkout Orca Buffers 
	
	Information.status 		= uart.InputBuffers[address].run_command; 
	if (verbose) { Pspace( name ); Pspace ( Information.status ); Pline( uart.InputBuffers[address].error_level );}
	
	switch ( uart.InputBuffers[address].run_command ) {
		case OrcaState::reset_cmd: 
		case OrcaState::enable_cmd: break;
		case OrcaState::servo_cmd: 				parseServo(); 			break;
		case OrcaState::extended_servo_cmd: 	parseExtendedServo(); 	break;
		case OrcaState::pwm_cmd:				parsePWM();				break;
		case OrcaState::info_cmd:  
			switch ( uart.InputBuffers[address].data[0] ) { // page 
			case OrcaState::page_0: parseConfigPage(0); 	break;
			case OrcaState::page_1: parseOffsetPage();  break;
			case OrcaState::page_2: break;  // todo parseABCalibrations
			case OrcaState::page_3: parseTestInfo();	break;   // todo parseCDCalibrations 
			case OrcaState::page_4: break;
			case OrcaState::page_5: parseSensorPage(); 	break;
			case OrcaState::page_6: parseConfigPage(1); break;
			}
			break;
		case OrcaState::geterror_cmd:  
			switch ( uart.InputBuffers[address].data[0] ) { // page 
			case OrcaState::page_0: parseErrors(0); break;
			case OrcaState::page_1: parseErrors(1); break; 
			}
			break;
		case OrcaState::config_cmd: break;
			switch (uart.InputBuffers[address].data[0] ) {
			case OrcaState::page_0:		parseConfigPage(0);break;
			case OrcaState::page_1:		parseConfigPage(1);break;	
			}
		case OrcaState::test_state1: 	parseTestInfo(); break;
		case OrcaState::testbed_study: parseStudySelection(); 
			switch(Study.type){
				case 0: parseForceStudy();break;
				case 1: parsePositionStudy();break;
			}
		case OrcaState::sensor_calibration:	parseFredInfo();break;
		default:  return;
		
	}

	// TODO: put back Orca buffers 
	uart.InputBuffers[address].locked = 0;
	
	//sei();
}


void Orca::parsePWM() {
// <<<<<<< HEAD
	// PWM.Phase 			= (uart.InputBuffers[address].data[0] << 8) + uart.InputBuffers[address].data[1];  //  Get Range of Hall B
	// PWM.CurrentA 		= uart.InputBuffers[address].data[2];  //  Get Range of Hall A
	// PWM.CurrentAp 		= uart.InputBuffers[address].data[3];  //  Get Zero of Hall B
	// PWM.CurrentAm 		= uart.InputBuffers[address].data[4];  //  Get Zero of Hall A
	// PWM.CurrentB 		= uart.InputBuffers[address].data[5];  //  Get Zero of Current B-
	// PWM.CurrentBp 		= uart.InputBuffers[address].data[6];  //  Get Zero of Current B+
	// PWM.CurrentBm 		= uart.InputBuffers[address].data[7];  //  Get Zero of Current A-
	// PWM.Therm		 	= uart.InputBuffers[address].data[8];  //  Get Zero of Current A+
// =======
	PWM.Phase 			= (uart.InputBuffers[address].data[0] << 8) + uart.InputBuffers[address].data[1];  
	PWM.CurrentA 		= uart.InputBuffers[address].data[2] + (uart.InputBuffers[address].data[3]<<8); 
	PWM.CurrentB 		= uart.InputBuffers[address].data[5] + (uart.InputBuffers[address].data[4]<<8);  
	PWM.VCC		 		= VOLTAGE_CONVERSION *uart.InputBuffers[address].data[6];  
	PWM.Therm		 	= uart.InputBuffers[address].data[8];  
// >>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
	PWM.new_data++;
}

void Orca::parseServo() {
	//if ( uart.InputBuffers[address].data[0] & 0x80 ) Information.phase_position = -1;
	//else 
		Information.phase_position = uart.InputBuffers[address].data[1] + ((uart.InputBuffers[address].data[0] & 0x0F) <<8);
}
void Orca::parseExtendedServo() {	
	Warnings.flags 			= (uart.InputBuffers[address].data[0] >> 4);
	Warnings.reset          = (uart.InputBuffers[address].data[0] >> 4) & 0x01;
	Warnings.temp			= (uart.InputBuffers[address].data[0] >> 4) & 0x02;
	Information.phase_position = uart.InputBuffers[address].data[1] + ((uart.InputBuffers[address].data[0] & 0x0F) <<8);
	Information.temp 		= adc_to_temp[ uart.InputBuffers[address].data[2] ]; 
	Information.voltage 	= 4716 * uart.InputBuffers[address].data[3] / 32;		// converts the APIs voltage to millivolts 
	Information.freq 		= 20 * uart.InputBuffers[address].data[4]; 
}

void Orca::parseOffsetPage(){
	// See OrcaBRAINs Wiki - "Get Info Responses, Page 1" for reference 
	//Printl("parseOffsetPage in Parsing.cpp");
	Offsets.RangeHallB 		= uart.InputBuffers[address].data[1];  //  Get Range of Hall B
	Offsets.RangeHallA 		= uart.InputBuffers[address].data[2];  //  Get Range of Hall A
	Offsets.ZeroHallB 		= uart.InputBuffers[address].data[3];  //  Get Zero of Hall B
	Offsets.ZeroHallA 		= uart.InputBuffers[address].data[4];  //  Get Zero of Hall A
	Offsets.ZeroCurrentBm 	= uart.InputBuffers[address].data[5];  //  Get Zero of Current B-
	Offsets.ZeroCurrentBp 	= uart.InputBuffers[address].data[6];  //  Get Zero of Current B+
	Offsets.ZeroCurrentAm 	= uart.InputBuffers[address].data[7];  //  Get Zero of Current A-
	Offsets.ZeroCurrentAp 	= uart.InputBuffers[address].data[8];  //  Get Zero of Current A+
	Offsets.new_data++;
	
}
void Orca::parseConfigPage(int page) {
	// See OrcaBRAINs Wiki - "Get Info Responses, Page 0" for reference 
	//Printl("parseConfigPage in Parsing.cpp"); 
	if (page == 0) {
// <<<<<<< HEAD
		// Config.position_polarity 	= uart.InputBuffers[address].data[1];  // position polarity
		// Config.TempThrottle 		= uart.InputBuffers[address].data[2];  // temp throttle size
		// Config.TempCode2 			= uart.InputBuffers[address].data[3];  // ADC_t-critical
		// Config.TempCode1 			= uart.InputBuffers[address].data[4];  // ADC_t-start-throttle
		// Config.Exponent 			= uart.InputBuffers[address].data[5];  // winding polarity
		// Config.motor_winding 		= uart.InputBuffers[address].data[6];  // winding exponent
		// Config.BuildIDhigh 			= uart.InputBuffers[address].data[7];  // firmware build ID (high)
		// Config.BuildIDlow 			= uart.InputBuffers[address].data[8];  // firmware build ID (low)
		// Config.FirmwareID 			= uart.InputBuffers[address].data[9];  // firmware ID
		// Config.new_data_1++;
	// }
	// if (page == 1) {
		// Config.max_duty 			= uart.InputBuffers[address].data[1];
		// Config.max_current			= uart.InputBuffers[address].data[2]; 
// =======
		Config.position_polarity 	= uart.InputBuffers[address].data[1];  // position polarity
		Config.TempThrottle 		= uart.InputBuffers[address].data[2];  // temp throttle size
		Config.TempCode2 			= uart.InputBuffers[address].data[3];  // ADC_t-critical
		Config.TempCode1 			= uart.InputBuffers[address].data[4];  // ADC_t-start-throttle
		Config.Exponent 			= uart.InputBuffers[address].data[5];  // winding polarity
		Config.motor_winding 		= uart.InputBuffers[address].data[6];  // winding exponent
		Config.BuildIDhigh 			= uart.InputBuffers[address].data[7];  // firmware build ID (high)
		Config.BuildIDlow 			= uart.InputBuffers[address].data[8];  // firmware build ID (low)
		Config.BuildID				= uart.InputBuffers[address].data[8] + (uart.InputBuffers[address].data[7]<<8);
		Config.FirmwareID 			= uart.InputBuffers[address].data[9];  // firmware ID
		Config.new_data_1++;
	}
	if (page == 1) {
		Config.SerialNum			= (uart.InputBuffers[address].data[3]<<16) + (uart.InputBuffers[address].data[2]<<8) + uart.InputBuffers[address].data[1];
		Config.max_duty 			= uart.InputBuffers[address].data[5];
		Config.max_current			= uart.InputBuffers[address].data[6]; 
//>>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
		Config.new_data_2++;
	}	
}

// See OrcaBRAINs Wiki - "Get Info Responses, Page 5" for reference 
// Parsed by Kyle on July 7th, 2018
void Orca::parseSensorPage() {
	Sensors.current[3] 	= uart.InputBuffers[address].data[1];
	Sensors.current[1] 	= uart.InputBuffers[address].data[2];
	Sensors.current[2] 	= uart.InputBuffers[address].data[3];
	Sensors.current[0] 	= uart.InputBuffers[address].data[8];
	Sensors.hall[0] 	= uart.InputBuffers[address].data[5];
	Sensors.hall[1] 	= uart.InputBuffers[address].data[4];
	Sensors.temp 		= uart.InputBuffers[address].data[6];
	Sensors.vcc  		= uart.InputBuffers[address].data[7];
	Information.temp 	= adc_to_temp[ uart.InputBuffers[address].data[6] ]; 
	Information.voltage = VOLTAGE_CONVERSION * uart.InputBuffers[address].data[7]; 
	Sensors.new_data ++;
}

void Orca::parseErrors(int page) {
	if( page == 0 ){								// Error						Self-Reseting
		Errors.err11 = uart.InputBuffers[address].data[1];  	// UART CRC match errors 		No
		Errors.err12 = uart.InputBuffers[address].data[2];  	// UART Invalid Frame Format	No
		Errors.err13 = uart.InputBuffers[address].data[3];  	// Undefined
		Errors.err14 = uart.InputBuffers[address].data[4];  	// Hall Sensor A Saturation		No
		Errors.err15 = uart.InputBuffers[address].data[5];  	// Hall Sensor B Saturation		No
		Errors.err16 = uart.InputBuffers[address].data[6];  	// Temperature throttling 		Yes
		Errors.err17 = uart.InputBuffers[address].data[7];  	// Undefined
		Errors.err18 = uart.InputBuffers[address].data[8];  	// Undefined
		Errors.err19 = uart.InputBuffers[address].data[9];  	// Undefined
		Errors.new_data_1 ++;
	}
	if( page == 1 ){
		Errors.err21 = uart.InputBuffers[address].data[1];  	// Communication timeout 		Yes
		Errors.err22 = uart.InputBuffers[address].data[2];  	// Temperature is too high		Yes
		Errors.err23 = uart.InputBuffers[address].data[3];  	// Voltage is too low			Yes
		Errors.err24 = uart.InputBuffers[address].data[4];  	// Voltage is too high	 		No
		Errors.err25 = uart.InputBuffers[address].data[5];  	// No shaft				 		Yes
		Errors.err26 = uart.InputBuffers[address].data[6];  	// Position error		 		Yes
		Errors.err27 = uart.InputBuffers[address].data[7];  	// Undefined
		Errors.err28 = uart.InputBuffers[address].data[8];  	// Undefined
		Errors.err29 = uart.InputBuffers[address].data[9];  	// Undefined
		Errors.	new_data_2 ++;
	}
}

void Orca::parseTestInfo(){
// <<<<<<< HEAD
	// Test.int1 = uart.InputBuffers[address].data[1];
	// Test.int2 = uart.InputBuffers[address].data[2];
	// Test.int3 = uart.InputBuffers[address].data[3];
	// Test.int4 = uart.InputBuffers[address].data[4];
// =======
	Test.currentA =  uart.InputBuffers[address].data[0]+((uart.InputBuffers[address].data[1] & 0x03) << 8);
	Test.currentB = uart.InputBuffers[address].data[2]+ ((uart.InputBuffers[address].data[3] & 0x03) << 8);
	Test.hallA = uart.InputBuffers[address].data[5]+ ((uart.InputBuffers[address].data[6] & 0x03) << 8);
	Test.hallB = uart.InputBuffers[address].data[7]+ ((uart.InputBuffers[address].data[8] & 0x03) << 8);
	Test.AA = uart.InputBuffers[address].data[4];
	Test.BA = uart.InputBuffers[address].data[9];
//>>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
	Test.new_data++;
}

void Orca::parseStudySelection(){
	Study.type = uart.InputBuffers[address].data[9]>>7;
	Study.new_data++;
}

void Orca::parseForceStudy(){
// <<<<<<< HEAD
	// ForceStudy.CurrentA 	= 	 uart.InputBuffers[address].data[1] 		+ ((uart.InputBuffers[address].data[2] & 0x03) << 8);
	// ForceStudy.CurrentB 	=   (uart.InputBuffers[address].data[2] >> 6) 	+ ((uart.InputBuffers[address].data[3] ) 		>> 2);
	// ForceStudy.DutyA 		= 	 uart.InputBuffers[address].data[4];//straight read
	// ForceStudy.DutyB		= 	 uart.InputBuffers[address].data[5];//straight read
	// ForceStudy.Position 	= 	(uart.InputBuffers[address].data[6]) 		+  (uart.InputBuffers[address].data[7]			 << 8);
	// ForceStudy.Temp 		= 	 uart.InputBuffers[address].data[8] 		+ ((uart.InputBuffers[address].data[9]  & 0x03) << 8);
	// ForceStudy.Voltage 		=   (uart.InputBuffers[address].data[9] >> 6) 	+ ((uart.InputBuffers[address].data[10] & 0x03) >> 2);
	// ForceStudy.new_data++;
// }
// void Orca::parsePositionStudy(){
	// PosStudy.CurrentA 	= 	 uart.InputBuffers[address].data[1] 			+ ((uart.InputBuffers[address].data[2] & 0x03) << 8);
	// PosStudy.CurrentB 	=   (uart.InputBuffers[address].data[2] >> 6) 		+ ((uart.InputBuffers[address].data[3] ) 		>> 2);
	// PosStudy.Temp 		= 	 uart.InputBuffers[address].data[4] 			+ ((uart.InputBuffers[address].data[5] & 0x03) << 8);
	// PosStudy.Voltage 	=   (uart.InputBuffers[address].data[5] >> 6)		+ ((uart.InputBuffers[address].data[6] & 0x03)	>> 2);
	// PosStudy.HallA 		=  	(uart.InputBuffers[address].data[7] 			+ ((uart.InputBuffers[address].data[8] & 0x03) << 8));
	// PosStudy.HallB		= 	(uart.InputBuffers[address].data[8] >> 2)		+ ((uart.InputBuffers[address].data[9] & 0x0F  << 6));
	// PosStudy.Position 	= 	(uart.InputBuffers[address].data[9] >> 4)		+ (uart.InputBuffers[address].data[10] & 0x7F	<< 4);
// =======
	ForceStudy.CurrentA 	= 	 uart.InputBuffers[address].data[0] 		+ ((uart.InputBuffers[address].data[1] & 0x03) << 8);
	ForceStudy.CurrentB 	=   (uart.InputBuffers[address].data[1] >> 6) 	+ ((uart.InputBuffers[address].data[2] ) 		<< 2);
	ForceStudy.DutyA 		= 	 uart.InputBuffers[address].data[3];//straight read
	ForceStudy.DutyB		= 	 uart.InputBuffers[address].data[4];//straight read
	ForceStudy.Position 	= 	(uart.InputBuffers[address].data[5]) 		+  (uart.InputBuffers[address].data[6]			 << 8);
	ForceStudy.Temp 		= 	 uart.InputBuffers[address].data[7] 		+ ((uart.InputBuffers[address].data[8] & 0x03) << 8);
	ForceStudy.Voltage 		=   (uart.InputBuffers[address].data[8] >> 4) 	+ ((uart.InputBuffers[address].data[9] ) << 4);
	ForceStudy.new_data++;
}
void Orca::parsePositionStudy(){
	PosStudy.CurrentA 	= 	 uart.InputBuffers[address].data[0] 			+ ((uart.InputBuffers[address].data[1] & 0x03) << 8);
	PosStudy.CurrentB 	=   (uart.InputBuffers[address].data[1] >> 6) 		+ ((uart.InputBuffers[address].data[2] ) 		<< 2);
	PosStudy.Temp 		= 	 uart.InputBuffers[address].data[3] 			+ ((uart.InputBuffers[address].data[4] & 0x03) << 8);
	PosStudy.Voltage 	=   (uart.InputBuffers[address].data[4] >> 6)		+ ((uart.InputBuffers[address].data[5] & 0xFF)	<< 2);
	PosStudy.HallA 		=  	(uart.InputBuffers[address].data[6] 			+ ((uart.InputBuffers[address].data[7] & 0x03) << 8));
	PosStudy.HallB		= 	(uart.InputBuffers[address].data[7] >> 2)		+ ((uart.InputBuffers[address].data[8] & 0x0F) << 6);
	PosStudy.Position 	= 	(uart.InputBuffers[address].data[8] >> 4)		+ ((uart.InputBuffers[address].data[9] & 0x7F)	<< 4);
//>>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
	PosStudy.new_data++;
}

void Orca::parseFredInfo(){
	Fred.currentA =  uart.InputBuffers[address].data[0]+((uart.InputBuffers[address].data[1] & 0x03) << 8);
	Fred.currentB = uart.InputBuffers[address].data[2]+ ((uart.InputBuffers[address].data[3] & 0x03) << 8);
	Fred.hallA = uart.InputBuffers[address].data[5]+ ((uart.InputBuffers[address].data[6] & 0x03) << 8);
	Fred.hallB = uart.InputBuffers[address].data[7]+ ((uart.InputBuffers[address].data[8] & 0x03) << 8);
	Fred.new_data++;
}
