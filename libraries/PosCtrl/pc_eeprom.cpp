/**
*  @file pc_eeprom.cpp
*  @brief Saved settings imported from and exported to EEPROM 
*  @author Kyle Hagen <khagen@irisdynamics.com> 
*  @version 1.0 
* 
* 
*  Copyright 2019 Iris Dynamics Ltd. All rights reserved. 
*/

#include "PosCtrl.h"





/**
* Converts all settings to 16 bits integers and saved to eeprom, starting at the EEPROM base address 
* for the controller.
*/
void PosCtrl::save_settings () {      

	Print(my_name); Printl("Saving Settings"); 

	SavedSettings::save(eeprom_start   + 0, p_gain_setting & 0xFF );
	SavedSettings::save(eeprom_start   + 1, (p_gain_setting >>8) );

	uint16_t p_sat_storage = p_sat; 
	SavedSettings::save(eeprom_start   + 2, p_sat_storage & 0xFF );
	SavedSettings::save(eeprom_start   + 3, (p_sat_storage>>8) );

	SavedSettings::save(eeprom_start   + 4, i_gain_setting & 0xFF );
	SavedSettings::save(eeprom_start   + 5, (i_gain_setting >>8) );     

	uint16_t i_sat_storage = i_sat; 
	SavedSettings::save(eeprom_start   + 6, i_sat_storage & 0xFF );
	SavedSettings::save(eeprom_start   + 7, (i_sat_storage>>8) );        

	SavedSettings::save(eeprom_start   + 12, target_width_setting & 0xFF );
	SavedSettings::save(eeprom_start   + 13, (target_width_setting >>8) );  	
	
	SavedSettings::save(eeprom_start   + 14, d_gain_setting & 0xFF );
	SavedSettings::save(eeprom_start   + 15, (d_gain_setting >>8) );

	uint16_t d_sat_storage = d_sat; 
	SavedSettings::save(eeprom_start   + 16, d_sat_storage & 0xFF );
	SavedSettings::save(eeprom_start   + 17, (d_sat_storage>>8) );	
	
	SavedSettings::save(eeprom_start   + 22, 0x21 ); 
	SavedSettings::save(eeprom_start   + 23, 0x53 );
}

/**
* Retreives all data from the EEPROM starting at the passed base address and converts 
* from 16 bit ints to the appropriate formats. 
*/
void PosCtrl::load_settings () { 

	Print(my_name); Printl(" loading settings..."); 
	
	uint16_t valid 
		= ( SavedSettings::get(eeprom_start   + 22 ) + (SavedSettings::get(eeprom_start   + 23 ) << 8) );
	
	if ( valid != 0x5321 ) {
		Printl("...invalid settings");
		return;
	}
	

	p_gain_setting   
		= ( SavedSettings::get(eeprom_start   + 0 ) + (SavedSettings::get(eeprom_start   + 1 ) << 8) );
	set_pgain(p_gain_setting);
	gui.pgain.set(p_gain_setting);
	Serial.send_now();

	uint16_t p_sat_storage   
		= ( SavedSettings::get(eeprom_start   + 2 ) + (SavedSettings::get(eeprom_start   + 3 ) << 8) );
	p_sat = p_sat_storage; 
	gui.psat.set(p_sat); 
	Serial.send_now();     

	i_gain_setting   
		= ( SavedSettings::get(eeprom_start   + 4 ) + (SavedSettings::get(eeprom_start   + 5 ) << 8) );
	set_igain(i_gain_setting);
	gui.igain.set(i_gain_setting);
	Serial.send_now();     

	uint16_t i_sat_storage   
		= ( SavedSettings::get(eeprom_start   + 6 ) + (SavedSettings::get(eeprom_start   + 7) << 8) );
	set_isat (i_sat_storage); 
	gui.isat.set(i_sat); 
	Serial.send_now();     
	
	target_width_setting
		= ( SavedSettings::get(eeprom_start   + 12 ) + (SavedSettings::get(eeprom_start   + 13 ) << 8) );
	set_target_width( target_width_setting ); 
	gui.target_width.set(target_width_setting);
	Serial.send_now();   

	d_gain_setting   
		= ( SavedSettings::get(eeprom_start   + 14 ) + (SavedSettings::get(eeprom_start   + 15 ) << 8) );
	set_dgain(d_gain_setting);
	gui.dgain.set(d_gain_setting);
	Serial.send_now();

	uint16_t d_sat_storage   
		= ( SavedSettings::get(eeprom_start   + 16 ) + (SavedSettings::get(eeprom_start   + 17 ) << 8) );
	d_sat = d_sat_storage; 
	gui.dsat.set(d_sat); 
	Serial.send_now();   
	Printl("...done");

}
