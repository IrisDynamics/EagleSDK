#include "DualCoreActuator.h"


int DualCoreActuator::config ( int m1, int s1, int m2, int s2, int ex, int t1, int tf, int th, int verbose ) { 

	if (verbose) { Print(name); Printl(" reconfiguring..."); }
	error_log.write ("Reconfiguring_drivers..."); 
	
	int r;
	
	r = orcas[0]->config ( s1, m1, ex, t1, tf, th ); 
	if ( r != 1 ){ error_log.write ("DriverA_failed_to_configure"); 
		if (verbose) { Print(name); Printl(" DriverA_failed_to_configure"); }}
		
	r = orcas[1]->config ( s2, m2, ex, t1, tf, th ); 
	if ( r != 1 ){ error_log.write ("DriverB_failed_to_configure"); 
		if (verbose) { Print(name); Printl(" DriverB_failed_to_configure"); }}
	
	error_log.write ("...finished_reconfiguring_drivers"); 	
	if (verbose) { Print("..."); Print(name); Printl(" done reconfiguring"); }

	return 1;
} 



int DualCoreActuator::getErrors ( int verbose ) {		
	
	
	getErrors( orcas[0], 1 );
	getErrors( orcas[1], 1 );
	return 1;
	
}		

void DualCoreActuator::getErrors( Orca * orca, int verbose ) {
	orca->send_and_wait (OrcaState::geterror_cmd, 1 );  // results in log entry 
	if ( verbose ) {
		orca->Errors.PrintErrors ( orca->name ) ;
	}
	
}
		