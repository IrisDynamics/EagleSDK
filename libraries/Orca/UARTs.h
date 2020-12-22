#pragma once 
#include <Arduino.h>
#include "UARTFrames.h" 
#include "OrcaStates.h"
#include "Config.h"

#include <Logger.h>
// Adapted with lots of help from from Paul's Teensy core libraries class; Much love, buddy 

//Removeme
#include <General_Macros.h>



#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV2(baud) (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS * 2) + ((baud) >> 1)) / (baud))
#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest


class OrcaInputBuffer {
	public:
	volatile uint8_t locked = 0;
	volatile uint32_t missed_frames = 0;
	volatile uint8_t new_frame;
	volatile uint8_t run_command;
	volatile uint8_t error_level;
	uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};   
};


class ORCA_UART {
  public: 

    int number; 


	///////////////// In development 
	enum {
		UNUSED ,
		RECEIVING ,
		PARSING
	};		
	char buffer_semephore = UNUSED;
	////////////////////////////////////
	
    enum {
      CRC_MISS = 0,
    };
    int errors [ 5 ]; 
    
	DataLogger	error_log;

    // Find all this assignment in Singletons.h
    // volatile uint8_t &orca_a_data_flag, &orca_b_data_flag, &orca_x_data_flag, &orca_y_data_flag;
    // volatile uint8_t &orca_rc_a, &orca_rc_b, &orca_rc_x, &orca_rc_y;  // References to Orca object's run command 
    // volatile uint8_t &orca_error_a, &orca_error_b, &orca_error_x, &orca_error_y;
    
	// uint8_t (&orca_data_a)[10];                              // Referenece to Orca object's data buffers 
    // uint8_t (&orca_data_b)[10]; 
    // uint8_t (&orca_data_x)[10]; 
    // uint8_t (&orca_data_y)[10]; 
	
	
	// replaces above 
	OrcaInputBuffer InputBuffers[4];  
	
	
     ORCA_UART ( const char * name, int num ) : 
                  number(num), 
				  error_log( name )
               {
				CRCFast::build_table();   				
			   }   
    // ORCA_UART ( const char * name, int num, 
                // volatile uint8_t &df_a, volatile uint8_t &df_b, volatile uint8_t &df_x, volatile uint8_t &df_y, 
                // volatile uint8_t &rc_a, volatile uint8_t &rc_b, volatile uint8_t &rc_x, volatile uint8_t &rc_y, 
                // volatile uint8_t &er_a, volatile uint8_t &er_b, volatile uint8_t &er_x, volatile uint8_t &er_y,   // error levels 
                // uint8_t (&data_a)[10], uint8_t (&data_b)[10], uint8_t (&data_x)[10], uint8_t (&data_y)[10],
                // int ch_pin ) : 
                  // number(num), 
				  // error_log( name ),
                  // orca_a_data_flag (df_a),   orca_b_data_flag (df_b),   orca_x_data_flag (df_x),   orca_y_data_flag (df_y), 
                  // orca_rc_a        (rc_a),   orca_rc_b        (rc_b),   orca_rc_x        (rc_x),   orca_rc_y        (rc_y), 
                  // orca_error_a     (er_a),   orca_error_b     (er_b),   orca_error_x     (er_x),   orca_error_y     (er_y), 
                  // orca_data_a      (data_a), orca_data_b      (data_b), orca_data_x      (data_x), orca_data_y      (data_y), 
                  // channel_pin ( ch_pin ) 
               // {
				// CRCFast::build_table();   				
			   // }

    virtual void begin    ( int baud, int verbose = 0 ) = 0;
    virtual void isr () = 0;    
    virtual void send_all_frames () = 0;
    
    int get_baud () { return baudrate; }
    
    RxFrame * getFrame () ;   // checks for and receives a pointer to a published frame 
    // Call this function to get a pointer to the next 
    TxFrame * get_next_tx_frame ( int verbose = 0 ) {      
      if ( tx_buffer_available () ) {               
        if ( verbose ) { Print(name); Print( "assigning Tx Frame ("); Print( tx_buffer_available() - 1 ); Printl(" remaining)"); }
        return tx_frames[ tx_buffer_peek () ];
      }
      if ( verbose ) { Print(name); Printl(" assign Tx Buffer fail: buffer full"); }
      return 0;
    }
    void load_next_frame ( int verbose = 0 ) {
      tx_buffer_inc ();
    }
    
    // Buffer API 
    virtual int is_sending() = 0;
	
    int tx_buffer_available () {
      return TX_BUFFER_SIZE - ( ( head - tail ) & 0xF );
    }
    int tx_buffer_empty () {
      return  (head == tail) ;
    }    
    
  protected: 
    int baudrate = 0; 
    const char * name = ""; 
    int channel_pin = -1; 

    static const int TX_BUFFER_SIZE = 16;
    //static const int RX_BUFFER_SIZE = 10;
    
    TxFrame * tx_frames [ TX_BUFFER_SIZE ];
    //RxFrame * rx_frames = 0;

    volatile uint8_t head = 0, tail = 0;

    uint8_t tx_buffer_index() {      
      return (tail+1) & 0xF;
    }
    uint8_t tx_buffer_peek( ) {      
      return ((head+1) & 0x0F);
    }
    uint8_t tx_buffer_inc( int verbose = 0 ) {      
      head += 1;
      if (verbose) {Print(name); Print("tx buffer head ="); Printl(head);}
      return (head & 0x0F);
    }    
    uint8_t tx_buffer_dec ( int verbose = 0 ) {     
      tail += 1; 
      if (verbose) {Print(name); Print("tx buffer tail ="); Printl(tail);}
      return (tail & 0x0F);
    }

// Interrupt Functions: 

    volatile uint32_t last_byte_time = 0;
    volatile uint8_t current_size = 0, finished_frame_size = 0;
    volatile uint8_t current_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

    //	const int verbose_frame_creation = 0;
    void finish_frame() {
      if ( current_size > 0 ) {
        for (int i = current_size; i < 12; i++) current_data[i] = 0;
        //if ( verbose_frame_creation ) Printl("finishing frame");
        if ( CRCFast::generate( current_data, current_size - 1)  == current_data[ current_size - 1 ] ) {
          //if ( verbose_frame_creation ) Printl("CRC Match");
          deliver_frame();          
		  log_crc(0);  // so log will be send after crcs are encountered and minimum reporting time has passed
        }
        else {
		  log_crc(1);
		  //Printl("CRC Miss");
          //errors [ CRC_MISS ] ++;
        }
        current_size = 0;     
      }
    }
    void start_new_frame( uint8_t data ) {
	
      //if ( verbose_frame_creation ) { Print("starting frame: "); Printl(data); }
      current_data [ current_size++ ] = data; 
      int index = current_data [ 0 ] & 0xF ;
      finished_frame_size = OrcaState::response_size( index );
    }
    void add_to_frame( uint8_t data ) {
      //if ( verbose_frame_creation ) {Print("adding to frame4: "); Printl(data); }
      current_data [ current_size++ ] = data; 
    }

    void deliver_frame() {
      //if ( verbose_frame_creation ) {
        // switch ( ( current_data [ 0 ] & 0x30 ) >> 4 ) {
          // case 0: Printl("delivering message to orca a"); break;
          // case 1: Printl("delivering message to orca b"); break;
          // case 2: Printl("delivering message to orca x"); break;
          // case 3: Printl("delivering message to orca y"); break;
        // }
      // }
      // if ( verbose_frame_creation ) {
        // Print("status: "); Printl(current_data [ 0 ] & 0xF );
        // Print("errors: "); Printl((current_data [ 0 ] & 0xC0)>>6 );
      // }
// <<<<<<< HEAD
	  
      // switch ( ( current_data [ 0 ] & 0x30 ) >> 4 ) {  // address
        // case 0: 
          // orca_a_data_flag ++;
          // orca_rc_a    = current_data [ 0 ] & 0xF; 
          // orca_error_a = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_a[i] = current_data[i+1];		  
          // break;
        // case 1: 
          // orca_b_data_flag ++;
          // orca_rc_b = current_data [ 0 ] & 0xF; 
          // orca_error_b = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_b[i] = current_data[i+1];
          // break;
        // case 2: 
          // orca_x_data_flag ++;
          // orca_rc_x = current_data [ 0 ] & 0xF; 
          // orca_error_x = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_x[i] = current_data[i+1];
          // break;
        // case 3: 
          // orca_y_data_flag ++;	
          // orca_rc_y = current_data [ 0 ] & 0xF; 
          // orca_error_y = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_y[i] = current_data[i+1];
          // break;
      // }   


		uint8_t addy = ( current_data [ 0 ] & 0x30 ) >> 4;   // 0 - 3 		
		if ( InputBuffers[ addy ] . locked ) {
			InputBuffers[ addy ] . missed_frames ++;
			return;
		}
		InputBuffers[ addy ] . new_frame ++;
		InputBuffers[ addy ] . run_command = current_data [ 0 ] & 0xF; 
		InputBuffers[ addy ] . error_level = (current_data [ 0 ] & 0xC0) >> 6;
		for (int i = 0; i < 10; i++) 
			InputBuffers[ addy ] . data[ i ] = current_data[i+1];		
	
// =======
      // switch ( ( current_data [ 0 ] & 0x30 ) >> 4 ) {
        // case 0: 
          // orca_a_data_flag ++;
          // orca_rc_a    = current_data [ 0 ] & 0xF; 
          // orca_error_a = (current_data [ 0 ] & 0xC0) >> 6;
		  
          // for (int i = 0; i < 10; i++) orca_data_a[i] = current_data[i+1];
		  
          // break;
        // case 1: 
          // orca_b_data_flag ++;
          // orca_rc_b = current_data [ 0 ] & 0xF; 
          // orca_error_b = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_b[i] = current_data[i+1];
          // break;
        // case 2: 
          // orca_x_data_flag ++;
          // orca_rc_x = current_data [ 0 ] & 0xF; 
          // orca_error_x = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_x[i] = current_data[i+1];
          // break;
        // case 3: 
          // orca_y_data_flag ++;	
          // orca_rc_y = current_data [ 0 ] & 0xF; 
          // orca_error_y = (current_data [ 0 ] & 0xC0) >> 6;
          // for (int i = 0; i < 10; i++) orca_data_y[i] = current_data[i+1];
          // break;
      // }      
// >>>>>>> 2121daab6275c21a34f990ca1d88b5e7abd91388
    }
	
	uint32_t crc_record_start = 0 ;

	void log_crc( int new_errors ) {
		
		errors [ CRC_MISS ] += new_errors;
		if ( new_errors && !crc_record_start ) { crc_record_start = millis(); return; }
		if ( !crc_record_start || millis() - crc_record_start < CRC_PERIOD ) return;
		crc_record_start = 0;
		if ( errors [ CRC_MISS ] < CRC_MIN_REPORTING_SUM ) {
			errors [ CRC_MISS ] = 0;
			return;
		}
		Pspace( error_log.name() ); Print( errors [ CRC_MISS ] ); Printl(" CRC errors");
		error_log.write ( ( String("\nCRC_errors:_") + errors [ CRC_MISS ]).c_str() );
		errors [ CRC_MISS ] = 0;
	}
    
    
};

#define C2_ENABLE          UART_C2_TE | UART_C2_RE | UART_C2_RIE
#define C2_TX_ACTIVE      C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING  C2_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE    C2_ENABLE


class UART2_Class : public ORCA_UART {
  public: 

    // UART2_Class ( int ch_pin, 
                  // volatile uint8_t &df_a, volatile uint8_t &df_b, volatile uint8_t &df_x, volatile uint8_t &df_y,   // data flags  
                  // volatile uint8_t &rc_a, volatile uint8_t &rc_b, volatile uint8_t &rc_x, volatile uint8_t &rc_y,   // run commands (status)
                  // volatile uint8_t &er_a, volatile uint8_t &er_b, volatile uint8_t &er_x, volatile uint8_t &er_y,   // error levels 
                  // uint8_t (&data_a)[10], uint8_t (&data_b)[10], uint8_t (&data_x)[10], uint8_t (&data_y)[10] ) 
                    // : ORCA_UART( "UART2", 2, 
                                // df_a, df_b, df_x, df_y, 
                                // rc_a, rc_b, rc_x, rc_y, 
                                // er_a, er_b, er_x, er_y,                                 
                                // data_a, data_b, data_x, data_y, 
                                // ch_pin ) {
      // pinMode( channel_pin, OUTPUT );       
      // digitalWrite ( channel_pin, 0 ); // TODO: #ChangingChannels      
      // for (int i = 0 ; i < TX_BUFFER_SIZE ; i ++ ) tx_frames[i] = new TxFrame;
	  // begin( UART_BAUD_RATE );
    // } 
	
	UART2_Class () : ORCA_UART( "UART2", 2 ) { 
      for (int i = 0 ; i < TX_BUFFER_SIZE ; i ++ ) tx_frames[i] = new TxFrame;
	  begin( UART_BAUD_RATE );
    } 
	
    void begin ( int baud, int verbose = 0) {
        baudrate = baud; 
        SIM_SCGC4 |= SIM_SCGC4_UART2; // turn on clock
        CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
        CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

        uint32_t divisor = BAUD2DIV3(baud); 
          
        UART2_BDH = (divisor >> 13) & 0x1F; 
        UART2_BDL = (divisor >> 5) & 0xFF;
        UART2_C4 = divisor & 0x1F;
        UART2_C1 = 0;        
        UART2_C1 = UART_C1_M;   // 9 bit 
        UART2_PFIFO = 0; 
        UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE; 
        NVIC_SET_PRIORITY(IRQ_UART2_STATUS, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ  (IRQ_UART2_STATUS);
    }


    void send_all_frames () {
      UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TIE;      
    }

	int is_sending () {
      if ( tx_buffer_empty() && ( UART2_S1 & UART_S1_TDRE ) ) return 0;
      return 1;	
	}


    volatile uint16_t data;


    const volatile char isr_debugging = 0;
    void isr () {
      
		//////////////////////////////////////////// RECEIVING 
      if (UART2_S1 & UART_S1_RDRF) {        
        data = UART2_D;  // clear flag 
        if ( (UART2_C3 & 0x80) ) {  // Start of New Frame (End of Last Frame) 
//          if ( isr_debugging ) Print("r 1 "); 
          finish_frame();
          start_new_frame( data ); 
        }          
        else {
//          if ( isr_debugging ) Print("r 0 "); 
          add_to_frame( data );           
          if ( current_size == finished_frame_size ) finish_frame();
        }        
//        if ( isr_debugging ) Printl(data);
      }

      uint8_t c;
      c = UART2_C2;

	  //////////////////////////////////////////////////// SENDING
      if ( ( c & UART_C2_TIE ) && ( UART2_S1 & UART_S1_TDRE ) ) {     // transmit request was enabled and transmitter ready
        if ( tx_buffer_empty() ) UART2_C2 = C2_TX_COMPLETING;            // no frames loaded : enable transmit finished request
        else if ( tx_frames [ tx_buffer_index() ] -> is_empty() ) {   // current frame is finished 
          // TODO: Interframe delay 
          tx_buffer_dec ();                                           // move the frame buffer along
          if ( tx_buffer_empty() ) UART2_C2 = C2_TX_COMPLETING;       // if the buffer is now empty: enable transmit finished request
          else {                                                      // otherwise, send the first byte of the next frame           
            UART2_C3 |= 0x40;                                         // set header flag
            uint8_t data = tx_frames [ tx_buffer_index() ] -> pop();       
//            if ( isr_debugging )  {Print("1 "); Printl(data);}
            UART2_D = data;
          }
        }         
        else {                                                        // frame is loaded and not finished : send next byte 
          if ( tx_frames [ tx_buffer_index() ] -> index == 0 ) {      // frame is new
            UART2_C3 |= 0x40; 
//            if ( isr_debugging ) Print("1 "); 
          }            
          else {
            UART2_C3 &= ~0x40;
//            if ( isr_debugging ) Print("0 ");            
          }
          uint8_t data = tx_frames [ tx_buffer_index() ] -> pop();       
//          if ( isr_debugging ) Printl(data);   
          UART2_D = data;                             
        }        
      }

      if ( ( c & UART_C2_TCIE ) && ( UART2_S1 & UART_S1_TC ) ) {
        //transmitting = 0;
        UART2_C2 = C2_TX_INACTIVE;
      }
      

   }

  protected:    
    const char * name = "UART2"; 

};


class UART1_Class : public ORCA_UART {
  public: 

    // UART1_Class ( int ch_pin, 
                  // volatile uint8_t &df_a, volatile uint8_t &df_b, volatile uint8_t &df_x, volatile uint8_t &df_y,   // data flags  
                  // volatile uint8_t &rc_a, volatile uint8_t &rc_b, volatile uint8_t &rc_x, volatile uint8_t &rc_y,   // run commands (status)
                  // volatile uint8_t &er_a, volatile uint8_t &er_b, volatile uint8_t &er_x, volatile uint8_t &er_y,   // error levels 
                  // uint8_t (&data_a)[10], uint8_t (&data_b)[10], uint8_t (&data_x)[10], uint8_t (&data_y)[10] ) 
                    // : ORCA_UART( "UART1", 1, 
                                // df_a, df_b, df_x, df_y, 
                                // rc_a, rc_b, rc_x, rc_y, 
                                // er_a, er_b, er_x, er_y,                                 
                                // data_a, data_b, data_x, data_y, 
                                // ch_pin ) {
      // pinMode( channel_pin, OUTPUT );       
      // digitalWrite ( channel_pin, 0 ); // TODO: #ChangingChannels      
      // for (int i = 0 ; i < TX_BUFFER_SIZE ; i ++ ) tx_frames[i] = new TxFrame;
	  // begin( UART_BAUD_RATE );
    // } 
	
    UART1_Class () : ORCA_UART( "UART1", 1 ) {  
      for (int i = 0 ; i < TX_BUFFER_SIZE ; i ++ ) tx_frames[i] = new TxFrame;
	  begin( UART_BAUD_RATE );
    } 
	
    void begin ( int baud, int verbose = 0) {
        baudrate = baud; 
        SIM_SCGC4 |= SIM_SCGC4_UART1; // turn on clock
        CORE_PIN9_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
        CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

        uint32_t divisor = BAUD2DIV(baud); 
          
        UART1_BDH = (divisor >> 13) & 0x1F; 
        UART1_BDL = (divisor >> 5) & 0xFF;
        UART1_C4 = divisor & 0x1F;
        UART1_C1 = 0;        
        UART1_C1 = UART_C1_M;   // 9 bit 
        UART1_PFIFO = 0; 
        UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE; 
        NVIC_SET_PRIORITY(IRQ_UART1_STATUS, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ  (IRQ_UART1_STATUS);
    }


    void send_all_frames () {
      UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TIE;      
    }

	int is_sending () {
      if ( tx_buffer_empty() && ( UART1_S1 & UART_S1_TDRE ) ) return 0;
      return 1;	
	}


    volatile uint16_t data;


    const int isr_debugging = 0;
    void isr () {
      
		//////////////////////////////////////////// RECEIVING 
      if (UART1_S1 & UART_S1_RDRF) {        
        data = UART1_D;  // clear flag 
        if ( (UART1_C3 & 0x80) ) {  // Start of New Frame (End of Last Frame) 
//          if ( isr_debugging ) Print("r 1 "); 
          finish_frame();
          start_new_frame( data ); 
        }          
        else {
//          if ( isr_debugging ) Print("r 0 "); 
          add_to_frame( data );           
          if ( current_size == finished_frame_size ) finish_frame();
        }        
//        if ( isr_debugging ) Printl(data);
      }

      uint8_t c;
      c = UART1_C2;

	  //////////////////////////////////////////////////// SENDING
      if ( ( c & UART_C2_TIE ) && ( UART1_S1 & UART_S1_TDRE ) ) {     // transmit request was enabled and transmitter ready
        if ( tx_buffer_empty() ) UART1_C2 = C2_TX_COMPLETING;            // no frames loaded : enable transmit finished request
        else if ( tx_frames [ tx_buffer_index() ] -> is_empty() ) {   // current frame is finished 
          // TODO: Interframe delay 
          tx_buffer_dec ();                                           // move the frame buffer along
          if ( tx_buffer_empty() ) UART1_C2 = C2_TX_COMPLETING;       // if the buffer is now empty: enable transmit finished request
          else {                                                      // otherwise, send the first byte of the next frame           
            UART1_C3 |= 0x40;                                         // set header flag
            uint8_t data = tx_frames [ tx_buffer_index() ] -> pop();       
//            if ( isr_debugging )  {Print("1 "); Printl(data);}
            UART1_D = data;
          }
        }         
        else {                                                        // frame is loaded and not finished : send next byte 
          if ( tx_frames [ tx_buffer_index() ] -> index == 0 ) {      // frame is new
            UART1_C3 |= 0x40; 
//            if ( isr_debugging ) Print("1 "); 
          }            
          else {
            UART1_C3 &= ~0x40;
//            if ( isr_debugging ) Print("0 ");            
          }
          uint8_t data = tx_frames [ tx_buffer_index() ] -> pop();       
//          if ( isr_debugging ) Printl(data);   
          UART1_D = data;                             
        }        
      }

      if ( ( c & UART_C2_TCIE ) && ( UART1_S1 & UART_S1_TC ) ) {
        //transmitting = 0;
        UART1_C2 = C2_TX_INACTIVE;
      }
      

   }

  protected:    
    const char * name = "UART1"; 

};

// Basic Polling-Based Implementation 
class MK500_UART2 {
  public: 
        
	// TODO Channel Pin 
    static void begin ( int baud, int ch_pin = -1, int verbose = 0) {
		
        SIM_SCGC4 |= SIM_SCGC4_UART2; // turn on clock
        CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
        CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

        uint32_t divisor = BAUD2DIV3(baud); 
          
        UART2_BDH = (divisor >> 13) & 0x1F; 
        UART2_BDL = (divisor >> 5) & 0xFF;
        UART2_C4 = divisor & 0x1F;
        UART2_C1 = 0;
        UART2_PFIFO = 0; // non hardware fifo :(         
        UART2_C2 = UART_C2_TE | UART_C2_RE ;
                
    }    
    static void write ( uint8_t data ) {
      while ( !(UART2_S1 & UART_S1_TC) ) ; // wait to be complete       
      UART2_D = data;
    }

    static int available () {
      return UART2_S1 & UART_S1_RDRF; 
    } 

    static uint8_t read ( ) {
      uint8_t data = UART2_D;
      return data;
    }
	
  protected:    
    static constexpr const  char * name = "UART 2 (Programming Mode)";     
};

class MK500_UART1 {
  public: 
        
	// TODO Channel Pin 
    static void begin ( int baud, int verbose = 0) {
		
        SIM_SCGC4 |= SIM_SCGC4_UART1; // turn on clock
        CORE_PIN9_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
        CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

        // uint32_t divisor = BAUD2DIV3(baud);   // wrong 
        uint32_t divisor = BAUD2DIV(baud); 
          
        UART1_BDH = (divisor >> 13) & 0x1F; 
        UART1_BDL = (divisor >> 5) & 0xFF;
        UART1_C4 = divisor & 0x1F;
        UART1_C1 = 0;
        UART1_PFIFO = 0; // non hardware fifo :(         
        UART1_C2 = UART_C2_TE | UART_C2_RE ;
                
    }    
    static void write ( uint8_t data ) {
      while ( !(UART1_S1 & UART_S1_TC) ) ; // wait to be complete       
      UART1_D = data;
    }

    static int available () {
      return UART1_S1 & UART_S1_RDRF; 
    } 

    static uint8_t read ( ) {
      uint8_t data = UART1_D;
      return data;
    }
	
  protected:    
    static constexpr const  char * name = "UART 2 (Programming Mode)";     
};

extern UART1_Class UART1;
extern UART2_Class UART2;


