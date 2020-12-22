#pragma once 

#include <General_Macros.h>

#define CRC_POLYNOMIAL                0xD5// 

/*
The following CRCFast class was taken from 
https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
and can be found here: 
Barr, Michael. "Slow and Steady Never Lost the Race," Embedded Systems Programming, January 2000, pp. 37-46.

Big thank you for this! 
*/
class CRCFast {

  public:
    static uint8_t table [256];
    
    static uint8_t generate (volatile uint8_t const message[], int nBytes) {
      uint8_t data;
      uint8_t remainder = 0;
      
      // Divide the message by the polynomial, a byte at a time.       
      for (int byte = 0; byte < nBytes; ++byte)
      {
          data = message[byte] ^ remainder;
          remainder = table[data] ^ (remainder << 8);
      }
    
      // The final remainder is the CRC.
      return (remainder);
    }
    
    static  void build_table() {
      uint8_t remainder;
    
      for (int dividend = 0; dividend < 256; ++dividend)
      {          
        remainder = dividend;
        
        // Perform modulo-2 division, a bit at a time.    
        for (uint8_t bit = 8; bit > 0; --bit)
        {  
          if (remainder & 0x80)
            remainder = (remainder << 1) ^ CRC_POLYNOMIAL;
          else 
            remainder = (remainder << 1);
         }
         
         //Store the result into the table.     
         table [dividend] = remainder;
      }
    }  
};

class TxHeader {
  public:   
    enum {
      broacast     = 1,
      no_broacast  = 0,
      request_response = 1,
      no_response      = 0,
    };
    static uint8_t get( int bdcst, int address, int type, int command ) {
      return ( ( bdcst ? 1:0 )   << 7 ) +
             ( ( address & 0x03) << 5 ) +
             ( ( type ?  1:0 )   << 4 ) + 
             (   command & 0x0F       ) ;
    }
};

class RxHeader {
  public:   
    static uint8_t get( int address, int type, int command ) { 
     return 
             ( ( 1 )             << 7 ) +
             ( ( address & 0x03) << 5 ) +
             ( ( type ?  1:0 )   << 4 ) + 
             (   command & 0x0F       ) ; 
    }
};


class TxFrame {
  public: 
    
    void load (uint8_t h) {
      size = 2; 
      index = 0;
      data[0] = h;
      data[1] = CRCFast::generate(data, 1); 
      //Print("crc: ");Printl(data[1]);
    }
    
    void load (uint8_t h, uint8_t d1, uint8_t d2) {
      size = 4; 
      index = 0;
      data[0] = h;
      data[1] = d1; 
      data[2] = d2; 
      data[3] = CRCFast::generate(data, 3); 
      //Print("crc: ");Printl(data[3]);
    }
    
    void load (uint8_t h, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, 
                          uint8_t d6, uint8_t d7, uint8_t d8, uint8_t d9, uint8_t d10 ) {
      size = 12; 
      index = 0;
      data[0] = h;
      data[1] = d1;   data[2] = d2;    data[3] = d3;    data[4] = d4;    data[5] = d5; 
      data[6] = d6;   data[7] = d7;    data[8] = d8;    data[9] = d9;    data[10] = d10; 
      data[11] = CRCFast::generate(data, 11); 
      //Print("crc: ");Printl(data[11]);
    }
    uint8_t pop () { if ( !is_empty() ) return data[ index++ ]; return 0;} // get next data 
    int is_empty () { return index >= size ? 1 : 0; } // check if all data has been sent 

    volatile int channel = 0; 
    volatile int size = 0;
    volatile int index = 0;
    volatile uint8_t data[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; 
};  


// A data comes in to the UART with the 9th bit set, implying it is a new frame 
// If a previous frame was being built, it is checked for validity and on a crc match, sent to orca indicated by the header 
// A new frame is created, and the header is decoded to predict the frame length. 

class RxFrame {
  public: 
    
    RxFrame( int ch, uint8_t first_byte ) : channel(ch) {    
      size = read_frame_size ( first_byte );    
      data = new uint8_t [size];
      push ( first_byte );      
    }
    ~RxFrame () { delete [] data; }
    
    void push ( uint8_t new_byte ) { // add a byte  
      if ( !full() ) data[ index++ ] = new_byte; 
      else index++;  // so we can show if we got overstuffed 
    } 
    int full () { return index == size ? 1 : 0; } // check if all data has been sent 
    int crc_check () {
      if ( data [ index-1 ] == CRCFast::generate ( data, index - 1 ) ) return 1;
      return 0;  
    }
    // Call this on a full frame 
    int check_valid () { 
      return ( full() && crc_check() );  // check for full before CRC check or risk array out of bounds  
    }

    uint8_t read_frame_size ( uint8_t header_byte ) {
      switch ( ( header_byte & 0xC ) ) {
        case 0x0C: return 12; 
          case 0x08: return 12;
            case 0x04: return 4;
              case 0x00: return 2;
      }
    }

    
    
    static int getAddy        ( uint8_t header_data ) { return ( header_data & 0x30 ) >> 4; }
    static int getCommand     ( uint8_t header_data ) { return   header_data & 0x0F;  }
    static int getErrorLevel  ( uint8_t header_data ) { return ( header_data & 0xC0 ) >> 6; }

    int channel; 
    int size;
    int index = 0;
    uint8_t * data; 
};
