/**
 *  @file pc_run.cpp
 *  @brief Position control algorithms 
 *  @author Kyle Hagen <khagen@irisdynamics.com> 
 *  @version 1.0 
 * 
 * 
 *  Copyright 2019 Iris Dynamics Ltd. All rights reserved. 
 */
 
 #include "PosCtrl.h"
 
 
 
/**
 *  @brief Computes the proportional action 
 *  
 *  @details Uses the local saturation and gain variables combined with the 
 *  calculated position error to arrive at a force. 
 *  
 *  Force is saved locally. 
 */ 
void PosCtrl::run_p(){
  int force = ( pos_feedback - my_target ) * p_gain; 
  if (force > p_sat) force = p_sat; 
  if (force < -p_sat) force = -p_sat; 
  p_force = force; 
}

/**
 *  @brief Computers the integral action 
 *  
 *  @details Uses the local saturation, gain, and accumulation/decay rates 
 *  combined with the calculated position error and the historical error 
 *  to arrive at a force. 
 *  
 *  Force is saved locally. 
 */
void PosCtrl::run_i() {

	uint32_t t_elapsed = micros() - last_i_calc;
	last_i_calc = micros();

	float error_now = my_target - pos_feedback ; 

	if ( error_now > 0  ) {
		if ( error_now < target_width ) {  // in target zone 
			error_now = 0;
		}
		else 
			error_now -= target_width;
		} 
	else {	// error now is negative 
		if ( error_now > -target_width ) {
			error_now = 0;		
		}
		else error_now += target_width;
	}

	i_error += ( error_now * t_elapsed / 1000000. );

	if ( i_error > i_error_max )  i_error = i_error_max; 
	if ( i_error < -i_error_max ) i_error = -i_error_max; 

	i_force = i_error * i_gain;
}

/**
 *  @brief Calculates a damping force 
 *  
 *  @details Uses the local gain combined with the shaft speed to arrive 
 *  at a force. 
 */
void PosCtrl::run_d() {

	float force = d_gain * speed_feedback; 
	if ( force > d_sat ) d_force = d_sat; 
	else if ( force < -d_sat ) d_force = -d_sat; 
	else d_force = force; 
  
}













