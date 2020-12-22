#pragma once

/** USB Serial Parsing 
 *  Breaks recieved serial strings into commands and arguments. 
 *  Calls the application-specfic parsers (which should be implemented in the application code) 
 *  Then parses some general commands 
 */


#include <string.h>
#include <stdio.h>
#include "Arduino.h"
#include <General_Macros.h>


#define START_PARSING if (0) {
#define COMMAND_IS  } else if( strcmp( command,
#define THEN_DO     ) == 0 ) {
#define FINISH_PARSING } else return 0; return 1;

#define MAXCHAR 1000
    
class Comms {

  public:  
    int verbose = 1;
    int echo = 0; 
    
    virtual int  check(void);
    
    int  parseline( char* input ) {
      parsepos = 0;
      if (echo) Serial.print("> ");      
	  if (echo) Serial.println(input);
	  
      char* command = parse_stringUntilWhitespace( input );
	  
      parsepos++; // skip one char, ie a space
    
			int ret = parseSpecificCommands (command, input) +
						parseGeneralCommands  (command, input);
								
      if (!ret) print_help(command, input);  			
      //Serial.flush();
			return ret;
    }

    virtual int parseSpecificCommands(char* command, char* args);
    
    int   parse_int( char* input ){
      int value = 0;
      int charsread;
      sscanf( input + parsepos, "%d%n", &value, &charsread );
      parsepos += charsread;
      return value;
    };
    
    float parse_float( char* input ) {
      float value = 0;
      int charsread;
      sscanf( input + parsepos, "%f%n", &value, &charsread );
      parsepos += charsread;
      return value;
    };    

    int parseGeneralCommands(char* command, char* args) {
      START_PARSING
      COMMAND_IS "hi"    THEN_DO  Printl("Hey, what's up?");
      COMMAND_IS "hello" THEN_DO  Printl("Hello, how do you do?");
      COMMAND_IS "hello"   THEN_DO  Serial.println("Oh hey there!");    
      COMMAND_IS "yo"      THEN_DO  Serial.println("Yo yo homie!");      
      COMMAND_IS "hi"      THEN_DO  Serial.println("Hi :)");
      COMMAND_IS "america" THEN_DO  Serial.println("Fuck Yeah!");
      COMMAND_IS "canada"  THEN_DO  Serial.println("Heck Yeah!");
      COMMAND_IS "yay"     THEN_DO  Serial.println("Im happy you're happy =D");     
      COMMAND_IS "analog_write" THEN_DO
        int pin = parse_int(args);
        int value = parse_int(args);
        analogWrite(pin,value);
      COMMAND_IS "digital_write" THEN_DO
        int pin = parse_int(args);
        int value = parse_int(args);
        digitalWrite(pin,value);  
      COMMAND_IS "on_time" THEN_DO 
        Print(millis() / (24*60*60*1000)); Ptab(" days");
        Print((millis() / (60*60*1000))% 24); Ptab(" hours");
        Print((millis() / (60*1000))   % 60 ); Ptab(" minutes");
        Print((millis() / (1000))      % 60); Printl(" seconds");       
      COMMAND_IS "!@34"          THEN_DO  
		Printl("<br>");
		Serial.send_now();
        __asm__ volatile ("bkpt");
      COMMAND_IS "reset"          THEN_DO 
		Printl("<br>");
		Serial.send_now();
		delay(10); SCB_AIRCR = 0x05FA0004; // software reset
      FINISH_PARSING
    }
    
    virtual void  print_help(char* command, char* args)  {if (verbose) {Serial.print("Invalid Command: "); Pline(args);}}
    virtual void  print_about() {Serial.println("Using Generalized Comms Class");}
    
  protected:
    char line[MAXCHAR+1]; // +1 for \0
    char tmpbuf[MAXCHAR+1]; // +1 for \0    
    int parsepos;
    int linelen = 0;


    
    char* parse_stringUntilWhitespace( char* input ) {
				
		
		
      int charsread;
      sscanf( input + parsepos, "%s%n", tmpbuf, &charsread );
      parsepos += charsread;
      return tmpbuf;
    };    
	
};
		
		
class USBComms : public Comms
{  
	public: 
		int sending_in_progress = 0;
		
		int check() {
			//if (!Serial) return;  
			static uint32_t t_start = millis();
			if ( sending_in_progress && millis() - t_start < 250 ) {}    // timeout to try and fix problem with the app not repopulating sliders after reconnect 
			else 
				linelen = 0;    
			while (Serial.available()) {				
				
				line[ linelen++ ] = Serial.read();				
				line[ linelen ] = '\0';
				if ( linelen == MAXCHAR ) {
					Serial.println("Line too long");
					line[ linelen = 0 ] = '\0';
					tmpbuf[ 0 ] = '\0';
					return 0;
				}
			}   			
			
			
			if (line[0] == '\n' || line[1] == '\n') {				
				return 0;    		
			}				
			else 			
			if (linelen) {						
		
				if (line[linelen-1] != 10 && line[linelen-1] != 13 ) {  // last command was not fully received 
					if (!sending_in_progress) t_start = millis(); 
					sending_in_progress = 1; 
					return 0;				
					// Old:
					// sending_in_progress = 1; 
					// t_start = millis();
					// return 0;				
				}
				sending_in_progress = 0;
				return parseline( line );
			}
			else return 0;
		 }
		 
		int parseSpecificCommands(char* command, char* args);
};


//extern USBComms USB;



