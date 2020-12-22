#pragma once 
#include <IrisControls.h>
#include "Orca.h" 
#include "UARTs.h"



class Programmer {
  public: 

    // Program the passed orca at the baud rate. 
    // If no programming commands are recieved ( from AVRdude or similar ),
    //   and the timeout is non-negative, this function will stop blocking 
    //   and return.
    // Timeout in millis()
    static int program ( Orca& orca,  uint32_t timeout = 10000, int baud = 113636, uint32_t its_working = 1000 ) {
      Print("Beginning Programming Mode for ");Printl(orca.name);
      Print("Baud rate  : "); Printl(baud);
      Print("Reset Pin  : ");   Printl(orca.reset_pin);
      Print("UART Number: ");   Printl(orca.uart.number);
      Serial.send_now();
	  digitalWrite(2,HIGH);
	  digitalWrite(14,HIGH);
	  
	  
	  delay  (20);
	  //delay(1500);
      
	  int prior_baud; 
	  switch( orca.uart.number ) {
		//case 0: prior_baud = UART0.get_baud(); break; 
		case 1: prior_baud = UART1.get_baud(); break;
		case 2: prior_baud = UART2.get_baud(); break;		
		default: prior_baud = 0;
	  };
      
      setup ( orca.reset_pin, orca.uart.number,  baud );      
      
      IrisControlsAPI::disconnect();
      delay(250);                                  // Solves issue where API characters get sent to UART       
      while (Serial.available()) Serial.read();    // ^

      uint32_t t_start = millis();
      uint32_t this_timeout = (uint32_t)timeout;
      while ( timeout < 0 || millis() - t_start < this_timeout ) {
        int result = usb_to_ttl ( orca.reset_pin, orca.uart.number );
        if (result >= 2) {
          t_start = millis();
          this_timeout = its_working;
          timeout = 0;  // break when done 
        }
      }
      shutdown( orca.reset_pin, orca.uart.number, prior_baud );       
      delay(250);                                  // Solves issue where API characters get sent to UART       
      while (Serial.available()) Serial.read();    // ^
      return 1; 
    }

  protected: 
    static void setup ( int reset_pin, int uart_number, int baud ) {
      Serial.begin( baud ); 
      switch ( uart_number ) {
        case 2: MK500_UART2::begin( baud ); break; 
        case 1: MK500_UART1::begin( baud ); break; 
        //case 1: ProgrammingUART1.begin( baud ); break;
        //case 3: HardwareSerial3.begin( baud ); break;
        default: break; 
      }      
    }
    static void shutdown ( int reset_pin, int uart_number, int baud ) {
      switch ( uart_number ) {
        case 2: UART2.begin( baud, 9 ); break; 
        case 1: UART1.begin( baud, 9 ); break; 
        //case 2: HardwareSerial2.begin( baud ); break;
        //case 3: HardwareSerial3.begin( baud ); break;
        default: break;         
      }            
    }



    static int usb_to_ttl ( int reset_pin, int uart_number ) {
      unsigned char c, dtr;
      static unsigned char prev_dtr = 0;
      int ret = 0;
      switch ( uart_number ) {
        case 2: 
          if (Serial.available()) {
            c = Serial.read();
            MK500_UART2::write(c);    
            //Print("Writing ");Printl(c);
          }        
          if ( MK500_UART2::available() ){      
            c = MK500_UART2::read();
            Serial.write(c);
            ret += 3;
          }
          break;         
		case 1: 
          if (Serial.available()) {
            c = Serial.read();
            MK500_UART1::write(c);    
            //Print("Writing ");Printl(c);
          }        
          if ( MK500_UART1::available() ){      
            c = MK500_UART1::read();
            Serial.write(c);
            ret += 3;
          }
          break; 
        default: return 0; 
      }
      dtr = Serial.dtr();
      if (dtr && !prev_dtr) {
// Works 
        digitalWrite ( reset_pin, HIGH );
        delay(2);
        digitalWrite ( reset_pin, LOW );
        ret += 1;
      }
      prev_dtr = dtr;

      return ret;
    }  // End of usb_to_ttl 

    
};

