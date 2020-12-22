/**
*  @file pc_gui.cpp
*  @brief IrisControls gui implementation 
*  
* @author Kyle Hagen <khagen@irisdynamics.com> 
* @version 1.0 
* 
* 
* Copyright 2019 Iris Dynamics Ltd. All rights reserved. 
*/


#include "PosCtrl.h"



/**
*  @brief Iris Controls gui setup function 
*  
*  @details Sets up all sliders and buttons 
*/
void PosCtrl::setup() {

	gui.daction		.show();	gui.daction.enable(0);
	gui.dsat		.show();
	gui.dgain 		.show();
	gui.dtitle		.show(); 
		
	gui.iaction   	.show();    gui.iaction.enable(0);
	gui.target_width.show();
	gui.isat      	.show();
	gui.igain     	.show();
	gui.ititle    	.show();

	gui.paction   	.show();    gui.paction.enable(0);
	gui.psat      	.show();
	gui.pgain     	.show();
	gui.ptitle    	.show();      

	gui.result    	.show();   	gui.result.enable(0);
	gui.setpoint  	.show();	gui.setpoint  .enable(0);
	gui.feedback  	.show();   	gui.feedback  .enable(0);

	gui.title     	.show();

	gui.save      	.show();
	gui.load      	.show();

	gui.is_active = 1; 
}

/**
*  @brief Iris Controls gui cleanup function 
*  
*  @details Removes all gui sliders and buttons 
*/
void PosCtrl::shutdown() {
	gui.daction		.hide();
	gui.dsat		.hide();
	gui.dgain 		.hide();
	gui.dtitle		.hide(); 

	gui.iaction   	.hide();   
	gui.target_width.hide();
	gui.isat      	.hide();
	gui.igain     	.hide();
	gui.ititle    	.hide();

	gui.paction   	.hide();   
	gui.psat      	.hide();
	gui.pgain     	.hide();
	gui.ptitle    	.hide();   

	gui.result    	.hide();   
	gui.setpoint  	.hide();
	gui.feedback  	.hide();   

	gui.title     	.hide();

	gui.save      	.hide();
	gui.load      	.hide();
	gui.is_active = 0; 
}    


/**
*  @brief Iris Controls gui behavior 
*  
*  @details Gets called at regular intervals. Reads user interaction and updates information 
*/
void PosCtrl::run_gui() {

	uint32_t tnow = millis(); 
	if ((uint32_t)(tnow - gui.last_fast_update) < 16) return; 
	gui.last_fast_update = tnow; 

// Manual override via the gui for testing 
//	setpoint( gui.setpoint.update());

	gui.setpoint.set(my_target*1000);




	gui.feedback.set((int)(pos_feedback*1000));
	gui.paction.set(p_force);
	gui.iaction.set(i_force);
	gui.daction.set(d_force);
	gui.result.set(get_force());

	if (tnow - gui.last_slow_update < 100) return; 
	gui.last_slow_update = tnow; 

	p_sat = gui.psat.update();
	set_target_width(gui.target_width.update());
	set_isat(gui.isat.update());
	set_pgain(gui.pgain.update());
	set_igain(gui.igain.update());
	
	set_dgain(gui.dgain.update());
	d_sat = gui.dsat.update();

	if (gui.save.pressed()) {
	save_settings();
	}

	if (gui.load.pressed()) {
	load_settings();
	}

}