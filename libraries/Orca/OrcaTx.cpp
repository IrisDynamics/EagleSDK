#include "Orca.h"
/////////////////////////////////////////////////////////////////////////////////   INIT ORCA 
//  This will fail when temp or vcc sensors fall out of their safe ranges
int Orca::init( int verbose ) {  // Enables the HBridge Drivers
	//float ram, rap,rbm,rbp;
	broadcast = 0; 

#define INIT_WORK	
	if ( handshake	 ( verbose ) < 1 ) return -1;
	if ( testConfig  ( verbose ) < 1 ) return -2; 
	if ( testOffsets ( verbose ) < 1 ) return -3; 
	//if ( testCalibrations ( verbose ) < 1 ) return -1; 
	if ( testSensors ( verbose ) < 1 ) return -5;
	if ( testBridges ( verbose ) < 1 ) return -6;
	//if ( testRes 	 (ram, rap, rbm, rbp, verbose ) < 1 ) return -4;
	//if ( testLocation(verbose ) < 1 ) return -7;
	send_and_wait( OrcaState::enable_cmd );
	delay(10);
	new_feedback();
	
	return 1;

}


inline int Orca::send2( int cmd) {
	
	//Printl("sending2");
	Information.min_package_period = OrcaState::tx_periods(cmd);
	
	if ( rx_since_tx < 1 ) {
		Errors.unresponsive++; 
		Errors.missing_packet = Information.last_packet;	
	}	
	else Errors.unresponsive = 0; 
	rx_since_tx = 0;
	

  TxFrame * frame = uart.get_next_tx_frame(); 
  frame -> load ( TxHeader::get (broadcast ? TxHeader::broacast: TxHeader::no_broacast, address, 0, cmd ) );
                                  
  uart.load_next_frame();                                                                                                                       
  uart.send_all_frames();
  
  Information.last_packet = cmd ;
	return 1;
  
}

inline int Orca::send4( int cmd, uint8_t b1, uint8_t b2 ) {
	
	//Printl("sending4");
	Information.min_package_period = OrcaState::tx_periods(cmd);
	
	if ( rx_since_tx < 1 ) {
		Errors.unresponsive++; 
		Errors.missing_packet = Information.last_packet;	
		//return 0;
	}	
	else Errors.unresponsive = 0;
	rx_since_tx = 0;
	//return 1;
	

  TxFrame * frame = uart.get_next_tx_frame(); 
  frame -> load ( TxHeader::get (broadcast ? TxHeader::broacast: TxHeader::no_broacast, address, TxHeader::request_response, cmd ) , b1, b2 );
                                  
  uart.load_next_frame();                                                                                                                       
  uart.send_all_frames();
  
  Information.last_packet =  cmd ;
  return 1;
  
}
inline int Orca::send12( int cmd, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8, uint8_t b9, uint8_t b10  ) {
	
	// Printl("sending12");
	// Printl(b1);
	// Printl(b2);
	// Printl(b3);
	// Printl(b4);
	// Printl(b5);
	// Printl(b6);
	// Printl(b7);
	// Printl(b8);
	// Printl(b9);
	// Printl(b10);
	Information.min_package_period = OrcaState::tx_periods(cmd);
	
	if ( rx_since_tx < 1 ) {
		Errors.unresponsive++; 
		Errors.missing_packet = Information.last_packet;	
		//return 0;
	}	
	else Errors.unresponsive = 0; 
	rx_since_tx = 0;
	//return 1;
	

  TxFrame * frame = uart.get_next_tx_frame(); 
  frame -> load ( TxHeader::get (broadcast ? TxHeader::broacast: TxHeader::no_broacast, address, TxHeader::request_response, cmd ) 
		, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10 );
                                  
  uart.load_next_frame();                                                                                                                       
  uart.send_all_frames();
  
  Information.last_packet =  cmd ;
  return 1;
  
}


/////////////////////////////////////////////////////////////////////////////////   RESET STATE 
int Orca::reset_state( int verbose ) {
  
  if ( verbose ) { Print( "Reset State " ); Pspace(name); Printl(address); }
  send2( OrcaState::reset_cmd ) ;  
  return 1;
  
}

/////////////////////////////////////////////////////////////////////////////////   ENABLE STATE 
int Orca::enable_state( int verbose ) {  // Enables the HBridge Drivers
  if ( verbose ) { Print( "Enabling " ); Pline(name);  }      
  
  send2( OrcaState::enable_cmd ) ;  
  return 1;
}

/////////////////////////////////////////////////////////////////////////////////   SERVO STATE 
int Orca::servo_state( int force, int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( "Servo state " ); Pspace(name); Printl(address); }      

	Input.force = force; 
	uint8_t byte1, byte2;
	if ( force > 0 ) {
		byte2 = force & 0xFF;
		byte1 = 0x04 | ( ( force & 0x300 ) >> 8 );
	}
	else {
		force = -force;
		byte2 = force & 0xFF;
		byte1 = ( force & 0x300 ) >> 8;	}
	
	
	send4( OrcaState::servo_cmd, byte1, byte2 );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   SERVO STATE 
int Orca::pos_ctrl_state( unsigned int position, int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( "Positon Control  state " ); Pspace(name); Printl(address); }      

	uint8_t byte2 = position;
	uint8_t byte1 = ( ( position  ) >> 8 );
	
	send4( OrcaState::pos_ctrl_cmd, byte1, byte2 );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   SERVO STATE 
int Orca::pos_ctrl_setup_state( unsigned int position, uint8_t pgain, uint8_t i_gain, uint8_t sat, int save_l , int save_r , int save_tune , int verbose) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( "Positon Control setup state " ); Pspace(name); Printl(address); }      

	
	uint8_t byte2 = position;
	uint8_t byte1 = ( ( position ) >> 8 );
	
	send12( OrcaState::pos_ctrl_setup_cmd, byte1, byte2, pgain, i_gain, sat, 0, save_l, save_r, 0, save_tune );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   EXTENDED SERVO STATE 
int Orca::extended_servo_state( int force, int verbose ) {  // Enables the HBridge Drivers

  if ( verbose ) { Print( "Extended Servo state " ); Pspace(name); Printl(address); }      
  Input.force = force; 
	
	
	uint8_t byte1, byte2;
	if ( force > 0 ) {
		byte2 = force & 0xFF;
		byte1 = 0x04 | ( ( force & 0x300 ) >> 8 );
	}
	else {
		force = -force;
		byte2 = force & 0xFF;
		byte1 = ( force & 0x300 ) >> 8;
	}

	
	send4 (OrcaState::extended_servo_cmd, byte1, byte2);
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////   INFO STATE 
int Orca::info_state( int page, int verbose ) {
	if ( verbose ) { Pspace(name); Pspace(address); Print( "Info State " ); Pline(page); }
	send4 ( OrcaState::info_cmd, page, 0 );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   TEST BED STUDY STATE 
int Orca::testbed_study(int target_phase, int target_current, int target_study, int verbose ) {
	if ( verbose ) { Pspace(name); Pspace(address); Print( "Test Bed Study State " ); }
	send12 ( OrcaState::testbed_study, target_phase, target_current, target_study, 0,0,0,0,0,0,0 );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   GETERROR STATE 
int Orca::geterror_state( int page, int reset_errors, int verbose ) {
  if ( verbose ) { Pspace(name); Pspace(address); Print( "GetError State " ); Pline(page); }
	send4 ( OrcaState::geterror_cmd, page, reset_errors );
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////   PWM STATE 
int Orca::pwm_state( int ap, int am, int bp, int bm, int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( " Manual PWM "); Pspace(name); Printl(address); }
	send12 ( OrcaState::pwm_cmd, ap, am, bp, bm );
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////   CONFIG STATE 
int Orca::config_state( int page, int c1, int c2, int c3, int c4, int c5, int c6, int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( " Configure "); Pspace(name); Printl(address); }     
	send12 ( OrcaState::config_cmd, page, c1, c2, c3, c4, c5, c6 );
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////   ZERO SENSOR STATE 
int Orca::zero_sensors( int blind, int save, int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( " Zeroing Sensor "); Pspace(name); Printl(address); }    	
	send12 ( OrcaState::zero_cmd, 0,0,0,0,blind,0,0,0,0,save*10 );
	return 1;

}
/////////////////////////////////////////////////////////////////////////////////   ZERO SENSOR STATE 
int Orca::range_sensors( int verbose ) {  // Enables the HBridge Drivers
	if ( verbose ) { Print( " Ranging Sensor "); Pspace(name); Printl(address); }    	
	send12 ( OrcaState::range_sensors_cmd, 0,0,0,0,0,0,0,0,0,0 );
	return 1;

}

#define Test_state
///////////////////////////////////////////////////////////////////////////////////  Test STATE 
int Orca::test_state1 ( int ap, int am, int bp, int bm, int K11, int K12, int K21, int K22, int verbose ) {
	if ( verbose ) { 
		Print( " Test State "); Pspace(name); Printl(address); 
			
	}    	
	send12 ( OrcaState::test_state1, ap, am,bp,bm,K11, K12, K21, K22,0 );
	return 1;
}

///////////////////////////////////////////////////////////////////////////////////  SENSOR CALIBRATION STATE
int Orca::sensor_calibration ( int ap, int am, int bp, int bm, int K11, int K12, int K21, int K22, int verbose ) {
	if ( verbose ) { 
		Print( " Sensor Calibration "); Pspace(name); Printl(address); 
			
	}    	
	send12 ( OrcaState::sensor_calibration, ap, am,bp,bm,K11, K12, K21, K22,0 );
	return 1;
}




/////////////////////////////////////////////////////////////////////////////////
int Orca::send_and_wait ( 	int stateID, 
							int input1, 
							int input2, 
							int input3, 
							int input4, 
							int input5, 
							int input6, 
							int input7, 
							int input8, 
							int input9, 
							int input10,
							int verbose ){
	int bump = broadcast;
	broadcast = 0;
	
	switch ( stateID ){
		case OrcaState::test_state1: 
				test_state1( input1, input2,  input3, input4, input5, input6,input7, input8, verbose ); 
				break;
		case OrcaState::config_cmd: 
				config_state( input1, input2, input3, input4, input5, input6, input7 );
				break; 
		case OrcaState::info_cmd:
				info_state( input1 );				
				break;
		case OrcaState::testbed_study:
				testbed_study( input1, input2 , input3, verbose );				
				break;	
		case OrcaState::geterror_cmd:
				geterror_state( input1, 0 );
				break;		
        case OrcaState::reset_cmd:     
				reset_state( );
				break;		
        case OrcaState::enable_cmd:    
				enable_state( );
				break;		
        case OrcaState::servo_cmd:     
				servo_state( input1, input2 );
				break;		
		case OrcaState::extended_servo_cmd: 
				extended_servo_state( input1, input2 );
				break;		
        case OrcaState::pwm_cmd:
				pwm_state( input1, input2, input3, input4 );
				break;		       
        case OrcaState::zero_cmd:      
				zero_sensors( input1, input2 );
				break;		
		case OrcaState::range_sensors_cmd:
				range_sensors();
				break;
		case OrcaState::sensor_calibration: 
				sensor_calibration( input1, input2,  input3, input4, input5, input6,input7, input8, verbose ); 
				break;	
		default: 
			Printl("Not programmed yet");
			return -1;
	}		
		
	delayMicroseconds(2*OrcaState::tx_periods(  stateID  ));
	broadcast = bump;
	//new_feedback();	
	return new_feedback();			
}

/////////////////////////////////////////////////////////////////////////////////

// This function should go in the actuator class and should be two functions: 

// void getErrors( int verbose )    // called when a button is pressed, or automatically in some cases 
// {
	// getErrors( orca_a , verbose );
	// getErrors( orca_b , verbose ); 
// }

// void getErrors( Orca & orca ) {
	// int nfb = new_feedback();
	// if ( nfb < 1 ) { 
		// Printl("test offsets communication problem");
		// return 0;
	// }
	// else {
	
		// uint16_t flags = 0;		
		// if ( Errors.err11 	!= 0 )	flags |= 0x01; 
		// if ( Errors.err12	!= 0 )	flags |= 0x02;
		// if ( Errors.err13	!= 0 )	flags |= 0x03;  
		// if ( Errors.err14	!= 0 )	flags |= 0x04;
		// if ( Errors.err15	!= 0 )	flags |= 0x05;
		// if ( Errors.err16	!= 0 )	flags |= 0x06;
		// if ( Errors.err17	!= 0 )	flags |= 0x07;
		// if ( Errors.err18	!= 0 )	flags |= 0x08;
		// if ( Errors.err19 	!= 0 )	flags |= 0x09;
		// if ( Warnings.flags != 0 )	flags |= 0x10;
		// if ( Warnings.reset != 0 )	flags |= 0x11;
		// if ( Warnings.temp	!= 0 )	flags |= 0x12;
		
		// //etc...
		
		// int ret = 1;
			
		// if ( verbose ) {
			// if ( !(flags & 0x01) )  { Printl("UART CRC match errors");	 			ret = -1; } 
			// if ( !(flags & 0x02) )  { Printl("UART Invalid Frame Format"); 	 		ret = -1; } 
			// if ( !(flags & 0x03) )  { Printl("Undefined"); 	 						ret = -1; } 
			// if ( !(flags & 0x04) )  { Printl("Hall Sensor A Saturation");			ret = -1; } 
			// if ( !(flags & 0x05) )  { Printl("Hall Sensor B Saturation");			ret = -1; } 
			// if ( !(flags & 0x06) )  { Printl("Temperature throttling"); 				ret = -1; } 
			// if ( !(flags & 0x07) )  { Printl("Undefined"); 							ret = -1; } 
			// if ( !(flags & 0x08) )  { Printl("Undefined"); 							ret = -1; } 
			// if ( !(flags & 0x09) )  { Printl("Undefined"); 							ret = -1; } 
			// if ( !(flags & 0x10) ) { Printl("test offsets: ZeroCurrentAp invalid: ");	ret = -1; } 
			// if ( !(flags & 0x11) ) { Printl("test offsets: ZeroCurrentAp invalid: ");	ret = -1; } 
			// if ( !(flags & 0x12) ) { Printl("test offsets: ZeroCurrentAp invalid: ");	ret = -1; } 
		// }

	// }
// }
// int Orca::error_push( int verbose ){ 

	// geterror_state( 1, 0 ,verbose );
	// delay(1);

	
		// }
	// if ( verbose && ret > 0) Printl("test offsets completed with success");
		// return ret; 
	// }




// Sends and checks the configuration page 0 
//todo sends and checks teh configuration page 1
int Orca::config( int m, int s, int ex, int t1, int t2, int th ) {
	delay(10);
	send_and_wait ( OrcaState::config_cmd, 0, s, m, ex, t1, t2, th ); 
	send_and_wait ( OrcaState::info_cmd, 0 ); 
	if (	0
		|| 	Config.motor_winding 		!= m
		|| 	Config.position_polarity  	!= s
		||	Config.TempCode1 	 		!= t1 
		|| 	Config.TempCode2 	 		!= t2
		|| 	Config.Exponent 	 		!= ex
		|| 	Config.TempThrottle  		!= th
		// etc 
		) 
		return 0; 
	return 1; 
}


float Orca::adc_to_temp[256] = {			
-40	,	//	0
-29.9497094	,	//	1
-19.56046109	,	//	2
-13.03737574	,	//	3
-8.183633262	,	//	4
-4.277266114	,	//	5
-0.986253183	,	//	6
1.870928083	,	//	7
4.404839432	,	//	8
6.688030909	,	//	9
8.770787086	,	//	10
10.68945455	,	//	11
12.47119396	,	//	12
14.13685789	,	//	13
15.70281874	,	//	14
17.18217658	,	//	15
18.5855838	,	//	16
19.92182428	,	//	17
21.19823002	,	//	18
22.42098724	,	//	19
23.59536554	,	//	20
24.72589211	,	//	21
25.81648637	,	//	22
26.87056515	,	//	23
27.89112604	,	//	24
28.88081399	,	//	25
29.8419752	,	//	26
30.776701	,	//	27
31.68686392	,	//	28
32.57414759	,	//	29
33.44007166	,	//	30
34.28601276	,	//	31
35.1132221	,	//	32
35.92284059	,	//	33
36.71591158	,	//	34
37.4933919	,	//	35
38.25616137	,	//	36
39.00503096	,	//	37
39.74075003	,	//	38
40.46401246	,	//	39
41.1754622	,	//	40
41.87569807	,	//	41
42.56527795	,	//	42
43.2447226	,	//	43
43.91451893	,	//	44
44.57512303	,	//	45
45.22696278	,	//	46
45.87044021	,	//	47
46.50593368	,	//	48
47.13379974	,	//	49
47.75437487	,	//	50
48.36797706	,	//	51
48.97490719	,	//	52
49.57545033	,	//	53
50.1698769	,	//	54
50.75844371	,	//	55
51.34139497	,	//	56
51.91896312	,	//	57
52.49136969	,	//	58
53.05882598	,	//	59
53.62153382	,	//	60
54.17968613	,	//	61
54.73346752	,	//	62
55.28305483	,	//	63
55.82861761	,	//	64
56.37031859	,	//	65
56.90831409	,	//	66
57.4427544	,	//	67
57.97378417	,	//	68
58.50154274	,	//	69
59.02616443	,	//	70
59.54777887	,	//	71
60.06651124	,	//	72
60.58248255	,	//	73
61.09580988	,	//	74
61.60660659	,	//	75
62.11498251	,	//	76
62.62104421	,	//	77
63.1248951	,	//	78
63.62663566	,	//	79
64.12636358	,	//	80
64.62417394	,	//	81
65.1201593	,	//	82
65.61440992	,	//	83
66.10701383	,	//	84
66.59805696	,	//	85
67.08762329	,	//	86
67.57579492	,	//	87
68.06265222	,	//	88
68.5482739	,	//	89
69.0327371	,	//	90
69.51611751	,	//	91
69.99848945	,	//	92
70.47992593	,	//	93
70.96049877	,	//	94
71.44027862	,	//	95
71.91933511	,	//	96
72.39773687	,	//	97
72.87555158	,	//	98
73.3528461	,	//	99
73.82968648	,	//	100
74.30613806	,	//	101
74.78226549	,	//	102
75.25813281	,	//	103
75.73380351	,	//	104
76.2093406	,	//	105
76.68480661	,	//	106
77.1602637	,	//	107
77.63577368	,	//	108
78.11139807	,	//	109
78.58719814	,	//	110
79.06323498	,	//	111
79.53956952	,	//	112
80.0162626	,	//	113
80.49337501	,	//	114
80.97096753	,	//	115
81.44910098	,	//	116
81.92783627	,	//	117
82.40723445	,	//	118
82.88735673	,	//	119
83.36826458	,	//	120
83.8500197	,	//	121
84.33268412	,	//	122
84.81632026	,	//	123
85.30099092	,	//	124
85.78675938	,	//	125
86.2736894	,	//	126
86.76184531	,	//	127
87.25129207	,	//	128
87.74209524	,	//	129
88.23432114	,	//	130
88.72803681	,	//	131
89.22331013	,	//	132
89.72020983	,	//	133
90.21880556	,	//	134
90.71916797	,	//	135
91.22136873	,	//	136
91.72548062	,	//	137
92.23157757	,	//	138
92.73973477	,	//	139
93.25002866	,	//	140
93.76253709	,	//	141
94.2773393	,	//	142
94.79451607	,	//	143
95.31414976	,	//	144
95.83632439	,	//	145
96.36112574	,	//	146
96.88864141	,	//	147
97.41896094	,	//	148
97.95217587	,	//	149
98.48837987	,	//	150
99.02766884	,	//	151
99.57014098	,	//	152
100.1158969	,	//	153
100.6650399	,	//	154
101.2176758	,	//	155
101.7739133	,	//	156
102.333864	,	//	157
102.8976426	,	//	158
103.4653671	,	//	159
104.0371587	,	//	160
104.6131423	,	//	161
105.1934465	,	//	162
105.7782039	,	//	163
106.367551	,	//	164
106.9616286	,	//	165
107.5605822	,	//	166
108.164562	,	//	167
108.7737231	,	//	168
109.3882258	,	//	169
110.0082361	,	//	170
110.6339256	,	//	171
111.265472	,	//	172
111.9030597	,	//	173
112.5468794	,	//	174
113.1971294	,	//	175
113.8540152	,	//	176
114.5177503	,	//	177
115.1885566	,	//	178
115.8666649	,	//	179
116.5523153	,	//	180
117.2457577	,	//	181
117.9472526	,	//	182
118.6570715	,	//	183
119.3754977	,	//	184
120.1028269	,	//	185
120.8393681	,	//	186
121.5854443	,	//	187
122.3413933	,	//	188
123.1075692	,	//	189
123.8843425	,	//	190
124.6721023	,	//	191
125.4712564	,	//	192
126.2822336	,	//	193
127.1054845	,	//	194
127.9414833	,	//	195
128.7907294	,	//	196
129.6537492	,	//	197
130.5310982	,	//	198
131.423363	,	//	199
132.3311641	,	//	200
133.255158	,	//	201
134.1960404	,	//	202
135.1545492	,	//	203
136.1314683	,	//	204
137.1276308	,	//	205
138.143924	,	//	206
139.1812938	,	//	207
140.2407495	,	//	208
141.3233706	,	//	209
142.4303123	,	//	210
143.5628134	,	//	211
144.7222042	,	//	212
145.9099157	,	//	213
147.1274899	,	//	214
148.3765914	,	//	215
149.6590207	,	//	216
150.9767292	,	//	217
152.3318361	,	//	218
153.7266481	,	//	219
155.1636822	,	//	220
156.6456909	,	//	221
158.1756927	,	//	222
159.7570069	,	//	223
161.3932941	,	//	224
163.0886039	,	//	225
164.8474312	,	//	226
166.674783	,	//	227
168.5762573	,	//	228
170.5581391	,	//	229
172.6275148	,	//	230
174.7924124	,	//	231
177.0619725	,	//	232
179.4466605	,	//	233
181.9585297	,	//	234
184.6115537	,	//	235
187.4220486	,	//	236
190.4092169	,	//	237
193.5958554	,	//	238
197.009293	,	//	239
200.6826499	,	//	240
204.6565594	,	//	241
208.9815724	,	//	242
213.7215898	,	//	243
218.9588962	,	//	244
224.8017688	,	//	245
231.3964037	,	//	246
238.9464303	,	//	247
247.7465621	,	//	248
258.2445408	,	//	249
271.1651477	,	//	250
287.7879657	,	//	251
310.6770238	,	//	252
346.1398365	,	//	253
417.5355683	,	//	254
500	};	//	255

