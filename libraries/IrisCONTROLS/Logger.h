#pragma once 

#include <IrisControls.h>

class DataLogger {
	public: 
		
		DataLogger( const char * name ) {
			my_name = name; 
			my_id = id++; 
		}	
		~DataLogger(  ) {
			Printl("deleting data logger");
		}			
		
		void add () {			
			IrisControlsAPI::addLogger( my_id, my_name ); 
		}
		
		void write (const char * string) { 
			IrisControlsAPI::writeToLog( my_id, string );
		}
		
		const char * name () { return my_name; }
		
	protected: 
		static int id; 
		int my_id;
		const char * my_name; 
	
};