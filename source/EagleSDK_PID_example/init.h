/**
 * @file init.h
 * @author Kyle 
 * @brief functions to initialize and calibrate the rev5 OrcaBRAINS firmware for this application. 
 * 
 * These functions should not be modified. 
 */


#define EXPONENT          135
#define TEMP1             93
#define TEMP2             103
#define POWER_THROTTLE    150


/**
 * @brief invokes the zero calibration routine in both Orca drivers for an actuator. 
 */
void recalibrate ( DualCoreActuator & act )
{  
    Printl("<b><br>===================<br>Recalibrating Actuator...</b>");
    delay(100);       
    act.sleep();
    act.enable();
    act.orcas[0]->send_and_wait( OrcaState::zero_cmd, 1,1);
    act.orcas[1]->send_and_wait( OrcaState::zero_cmd, 1,1);    
    delay(100);         
    Printl("...done");
}


// These are the configureation codes required when intiiazing DCA-series actuators for forward or reverse operation. 
#define FORWARD_MOTOR 4,5,1,2
#define REVERSE_MOTOR 1,8,2,1


/**
 * @brief attempts to handshake and configure an actuator according to the values defined in Tuning.h 
 */
int initialize_actuator ( DualCoreActuator & act, int winding_a, int winding_b, int sensor_a, int sensor_b ) {
  int result_a, result_b;
  act.initialize( 1 , result_a, result_b );

  int comm_fails = 0;
  if ( result_a == -1 ) {  //////////////// Handshake fails
    Printl("driver P communications failure");
    comm_fails ++;
  }

  if ( result_b == -1 ) {  //////////////// Handshake fails
    Printl("driver Q communications failure");
    comm_fails ++;
  }

  if ( comm_fails == 2 ) {
    Printl("Actuator unresponsive: check communications cable, power cables, and fuse.");
  }
  else if ( result_a == -2 || result_b == -2 ) {  /////////// Test Config fails
    Printl("bad configuration: run reconfiguration");           
      act.config ( sensor_a, winding_a, sensor_b, winding_b, EXPONENT, TEMP1, TEMP2, POWER_THROTTLE, 1 );  
  }
  else if ( result_a == -3 || result_b == -3 ) {  /////////// Test Offsets fails    
    Printl("bad calibration: running calibration...");
    recalibrate ( act );
  }
  else if ( result_a == -5 || result_b == -5 ) {  /////////// Test Sensors fails
    Printl("bad sensors readings: check shaft");
  }
  else if ( result_a == -6 || result_b == -6 ) {  /////////// Test HBridge fails
    Printl("driver check failed: check power supply");
  }
  else {              //////////////////////////////////// Setup Actuator Settings 

    if (  result_a != -1 && 
            (                
            act.orcas[0]->Config.motor_winding != winding_a ||
            act.orcas[0]->Config.position_polarity != sensor_a ||
            act.orcas[0]->Config.TempCode1 != TEMP1 || 
            act.orcas[0]->Config.TempCode2 != TEMP2 || 
            act.orcas[0]->Config.TempThrottle != POWER_THROTTLE || 
            act.orcas[0]->Config.Exponent != EXPONENT 
            ) 
          ) 
      { 
           
      Printl("Orca A configuration incorrect");          
      act.config ( sensor_a, winding_a, sensor_b, winding_b, EXPONENT, TEMP1, TEMP2, POWER_THROTTLE, 1 );  
      //act.sleep(1);  
      
      } 

    if (  result_b != -1 &&
            (
            act.orcas[1]->Config.motor_winding != winding_b ||
            act.orcas[1]->Config.position_polarity != sensor_b || 
            act.orcas[1]->Config.TempCode1 != TEMP1 || 
            act.orcas[1]->Config.TempCode2 != TEMP2 || 
            act.orcas[1]->Config.TempThrottle != POWER_THROTTLE || 
            act.orcas[1]->Config.Exponent != EXPONENT 
            )
          ) {
            
      Printl("Orca B configuration incorrect"); 
      act.config ( sensor_a, winding_a, sensor_b, winding_b, EXPONENT, TEMP1, TEMP2, POWER_THROTTLE, 1 );  
      //act.sleep(1);  
      
    }
  }



  if ( result_a >0 && result_b > 0 ) 
    return 1; 
  else 
    return 0;

}


int initialize_actuator_forward ( DualCoreActuator & act ) {
  return initialize_actuator ( act, FORWARD_MOTOR ); 
}

int initialize_actuator_backward ( DualCoreActuator & act ) {
  return initialize_actuator ( act, REVERSE_MOTOR ); 
}

