#pragma once
#include <IrisControls.h>
#include <Exposed.h>
#include "Orca.h"
#include "Programmer.h"


extern AppChain * FastApps;


// The Pod Class is intended to maintain control over all the Orcas connected to the Eagle
// V0.1 The Pod is a simple array of orcas along with an administrator App that allows
//      detection, testing, debugging and configuration or any member of the Pod.



// This class extends the IrisControls API for use with the IrisControls USB Windows App
class ThePod : public IrisControlsAPI {
  public:

    uint32_t response_timeout = 10000;

    enum { NUMBER_OF_ORCAS = 12 };
    static Orca orcas [ NUMBER_OF_ORCAS ];

    ThePod() :
      // buttons
      program ( "Flash_Orca", -1 ),
      //stream  ( "Stream"    ,  0 ),
      send    ( "Send"      ,  0 ),
      reset   ( "Reset"     ,  -1 ),
      broadcast ( "Broadcast" , 0 ), 
      // IO bars
      target  ( "Target_Orca", 0, NUMBER_OF_ORCAS - 1, NUMBER_OF_ORCAS - 1 ),
      frequency ( "Stream_Frequency", 0, 6000, 1500  ),
	  wait_period ( "Timeout_Period", 0, 1000, 300  ), 
      //memory  ( "History_Depth" , 0, 1000, 0 ),
      command ( "Run_Command" , 0, 15, 0 ),
      in1    ( "In_1",  0 ,255, 0 ),
      in2     ( "In_2", 0 ,255, 0 ),
      in3     ( "In_3", 0 ,255, 0 ),
      in4     ( "In_4", 0 ,255, 0 ),
      in5     ( "In_5", 0 ,255, 0 ),
      in6     ( "In_6", 0 ,255, 0 ),
      in7     ( "In_7", 0 ,255, 0 ),
      in8     ( "In_8", 0 ,255, 0 ),
      in9     ( "In_9", 0 ,255, 0 ),
      in10    ( "In_10", 0 ,255, 0 ),
      // Status
      command_name ( "" ),
      comm_settings ( "Communications_Settings" ),
      data0   ( "Feedback_0" , 0, 255, -1 ),
      data1   ( "Feedback_1" , 0, 255, -1 ),
      data2   ( "Feedback_2" , 0, 255, 2 ),
      data3   ( "Feedback_3" , 0, 255, -1 ),
      data4   ( "Feedback_4" , 0, 255, -1 ),
      data5   ( "Feedback_5" , 0, 255, -1 ),
      data6   ( "Feedback_6" , 0, 255, -1 ),
      data7   ( "Feedback_7" , 0, 255, -1 ),
      data8   ( "Feedback_8" , 0, 255, -1 ),
      data9   ( "Feedback_9" , 0, 255, -1 ),      
      orca_status ( "Status", -1 ),
      orca_error  ( "Error_Level", -1 ),
	  tx_hz ( "Tx_Hz", 0 ),
	  rx_hz ( "Rx_Hz", 0 ),
	  missed_hz ( "No_Response_Hz", 0 )
    {} // constructor

    ExposedButton
      program,
      //stream,
      send,
      reset,
      broadcast;
    // TODO: disable comms timeout
    // Todo: standard actions
    ExposedIO
      target,
      frequency,
	  wait_period,
      //memory,
      command,
      in1,in2,in3,in4,in5,in6,in7,in8,in9,in10;
    //ExposedStatus
    ExposedLabel
      command_name,
      comm_settings;
    ExposedOutputBar
      data0, data1, data2, data3, data4, data5, data6, data7, data8, data9;
    ExposedOutputValue 
      orca_status,
      orca_error,
	  tx_hz ,
	  rx_hz ,
	  missed_hz;


	 
    void run() {
		
		static uint32_t tx_events = 0, rx_events = 0, silence = 0, last_send = 0;

		if ( program.pressed() ) {   
			Programmer::program ( orcas[ target.update() ] , -1 );
			command.set(0); // change the state to reset for the new firmware build
			return;
		}
		
		if (reset.pressed() ) {
			Print("Resetting "); Print(orcas[ target.update() ]. name ); Print(" reset pin: "); Printl(orcas[ target.update() ] . reset_pin);
			orcas[ target.update() ] . reset(); 
		}


		uint32_t send_period = 1000000. / frequency.update();

		if ( send.toggled_on() ) {
			
			if ( micros() - last_send > send_period ) {
				last_send = micros();
				tx_events++;
				run_command = command.update();				
				talk( );   				
				if ( listen ( wait_period.update() ) == 1 ) {
					rx_events++;
				}
				else {
					silence++;
				}			
			}
			else {
				listen(0);
			}
		}
		else {
		  listen(0);
		}

	  static float usb_available = 0;
	  usb_available = min ( usb_available, Serial.availableForWrite());
	  //usb_available = usb_available * 0.9999 + Serial.availableForWrite() * 0.0001;
        
      ACCESS_TIMER_NR ( 2000 )

	  Orca & orca = orcas [ target.update() ];
      
		orca_status	.update ( 	orca. uart.InputBuffers[orca.address]. run_command );
		orca_error	.update ( 	orcas [ target.update() ]. uart.InputBuffers[orca.address]. error_level);	  
		data0	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [0] ); 
		data1	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [1] );
		data2	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [2] );
		data3	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [3] );
		data4	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [4] );
		data5	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [5] );
		data6	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [6] );
		data7	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [7] );
		data8	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [8] );
		data9	.update( orcas [ target.update() ]. uart.InputBuffers[orca.address]. data [9] );

	  tx_hz.update();
	  rx_hz.update();
	  missed_hz.update();

      static int last_run_command = -1;
      if ( command.update() != last_run_command ) {
        last_run_command = command.get();      
        switch (command.get()) {
          case OrcaState::reset_cmd  : 			refresh_outputs(OrcaState::name( command.get() ) ); break;
          case OrcaState::enable_cmd : 			refresh_outputs(OrcaState::name( command.get() )); break;
          case OrcaState::pos_ctrl_cmd : 		refresh_outputs(OrcaState::name( command.get() ) , 255 ); break;
          case OrcaState::pos_ctrl_setup_cmd : 	refresh_outputs(OrcaState::name( command.get() ) , 255, 255, 255, 255, 0, 1, 1, 0, 0, 1); break;
		  case OrcaState::test_state1: 			refresh_outputs( OrcaState::name( command.get() ), 255, 128, 1 ); break;
		  case OrcaState::testbed_study: refresh_outputs( OrcaState::name( command.get() ), 3, 255,1 ); break;
          case OrcaState::servo_cmd : 			refresh_outputs(OrcaState::name( command.get()) , 255, 255 ); break;
          case OrcaState::info_cmd   : 			refresh_outputs(OrcaState::name( command.get() ), 6); break;
          case OrcaState::geterror_cmd : 		refresh_outputs(OrcaState::name( command.get() ), 2, 1); break;
          case OrcaState::pwm_cmd    : 			refresh_outputs(OrcaState::name( command.get() ) , 255, 255, 255, 255); break;
          case OrcaState::config_cmd   : 		refresh_outputs(OrcaState::name( command.get() ) , 1, 4, 8, 255, 255, 255, 255, 0, 0, 0); break;
          case OrcaState::zero_cmd   : 			refresh_outputs(OrcaState::name( command.get() ) , 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); break;
		  case OrcaState::sensor_calibration	  : refresh_outputs( OrcaState::name( command.get() ), 255, 255, 255, 255); break;

          default: refresh_outputs(OrcaState::name( command.get() )); break;
        }
      }
	  
	  ACCESS_TIMER_NR( 1000000 ) 
	  
	  tx_hz.update(tx_events);
	  tx_events = 0;
	  rx_hz.update(rx_events);
	  rx_events = 0;
	  missed_hz.update(silence);
	  silence = 0;
	  
	  
	  
    }

    void refresh_outputs(const char * name = "", int a=0, int b=0, int c=0, int d=0, int e=0, int f=0, int g=0, int h=0, int i=0, int j=0) {
 
      command_name.update(name);      
      in1.newUpperBound(a);in2.newUpperBound(b);in3.newUpperBound(c);in4.newUpperBound(d);in5.newUpperBound(e);
      in6.newUpperBound(f);in7.newUpperBound(g);in8.newUpperBound(h);in9.newUpperBound(i);in10.newUpperBound(j);
      if (a) in1.enable(1); else in1.enable(0);  in1.update();
      if (b) in2.enable(1); else in2.enable(0);  in2.update();
      if (c) in3.enable(1); else in3.enable(0);  in3.update();
      if (d) in4.enable(1); else in4.enable(0);  in4.update();
      if (e) in5.enable(1); else in5.enable(0);  in5.update();
      if (f) in6.enable(1); else in6.enable(0);  in6.update();
      if (g) in7.enable(1); else in7.enable(0);  in7.update();
      if (h) in8.enable(1); else in8.enable(0);  in8.update();
      if (i) in9.enable(1); else in9.enable(0);  in9.update();
      if (j) in10.enable(1); else in10.enable(0);  in10.update();
    }

    void setup() {
      // Outputs
	  
	  IrisControlsAPI::background( 255,255,255,255 );
	  IrisControlsAPI::text( 0,0,0,255 );
	  
	  tx_hz.show();
	  rx_hz.show();
	  missed_hz.show();
      orca_status.show();   
      orca_error.show();     
      data0.show();data1.show();data2.show();data3.show();data4.show();data5.show();data6.show();data7.show();data8.show();data9.show();
      // Buttons 
      program.show();
      //stream.show();
      send.show();
      reset.show();
      broadcast.show();
      // Inputs
      // Communication Settings 
      target.show();      
      frequency.show();
	  wait_period.show();
      //memory.show();
      comm_settings.show();
      // State Controls      
      command.show();      
      in1.show();in2.show();in3.show();in4.show();in5.show();in6.show();in7.show();in8.show();in9.show();in10.show(); 
      refresh_outputs();      

    }
    void shutdown() {
	  
	  tx_hz.hide();
	  rx_hz.hide();
	  missed_hz.hide();
      comm_settings.hide();
      program.hide();
      target.hide();
      broadcast.hide();
	  wait_period.hide();

      //stream.hide();
      frequency.hide();

      send.hide();
      reset.hide();
      //memory.hide();

      command.hide();
      command_name.hide();

      orca_status.hide();      orca_error.hide();     

      in1.hide();in2.hide();in3.hide();in4.hide();in5.hide();in6.hide();in7.hide();in8.hide();in9.hide();in10.hide();       
      data0.hide();
      data1.hide();
      data2.hide();
      data3.hide();
      data4.hide();
      data5.hide();
      data6.hide();
      data7.hide();
      data8.hide();
      data9.hide();
    }


  protected:

    int run_command = 0;
    void talk ( ) ;    
    int listen ( uint32_t wait_period  );

};

