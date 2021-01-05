/**
@mainpage Introduction

!! CAUTION !! 
This firmware will cause the actuators to create forces and motion. 
Be very careful at all times.  
Ensure the shaft and motor are mounted in a safe location and not in danger of hitting people or property.  

Note that force effects in this example are generated in the libraries/Orca/ActuatorApp.h file in the ActuatorControlApp object

This firmware is meant to provide basic handshaking and communication with dual core actuators running OrcaBRAINS r5.2.3 firmware using the RS422-based OrcaAPI. 
This firmware further provides 3 basic effects: a spring effect, a vibration effect, and a damping effect. 

## Versions 
1.0.0 - Jan 2021 - KH 
  Initial version 
  
*/

#pragma GCC optimize ("O3")

#include <IrisControls.h>   // Used to communicate with the IrisControls application 
const char * IrisControlsAPI::name = "EagleSDK";    // Feel free to change this...it will dictate what shows up on the IrisControls window
#include <IIRFilter.h>  // Used for low pass filtering filtering
#include <FFB.h>        // Not used in this firmware but required by another library to compile 
#include <Motion.h>     // Position and Speed calculations. Used by Actuator libraries.
#include <Orca.h>       // Communication and management of the 'Orca' embedded motor drivers 
#include <ThePod.h>     // An IrisControls GUI page for manually talking to or programming any connected Orca
#include <DualCoreActuator.h> // abstraction of an actuator using two Orca objects 
#include <ActuatorApp.h>      // IrisControls elements that display runtime info like position and temperature, as well as some demostration effects like springs and dampers 
#include <Settings.h>   // used to access the EEPROM 
#include "EEPROM.h"
#include "Config.h"           // deals with settings saved in the EEPROM 
#include "init.h"             // contains functions used to initialize the motors


#if ACTUATOR_CONTROL_APP_VERSION < 11 
Compiler_Error__Update_the_SDK_libraries 
#endif 

/////////////////////////
//       Globals 
///////////////////////


ThePod thePod;   // storage of the Orca Driver objects, and a diagnostic GUI for programming and implementing the API literally 

// DualCoreActuators contain two Orca elements, and extend the actuator class by merging the data obtained from both cores to a single set of data 
DualCoreActuator  actuator0 ( "Actuator_0", &ThePod::orcas[10], &ThePod::orcas[11] );  // uses RJ3 port (Orcas 8-11) and looks for addresses 2 and 3 (Orcas 10, 11) 
DualCoreActuator  actuator1 ( "Actuator_1", &ThePod::orcas[6], &ThePod::orcas[7] );  // uses RJ2 port 


// ActuatorFeedbackApps display information about an actuator using the IrisControls app. 
ActuatorFeedbackApp actuator0_info ( actuator0 ); 
ActuatorFeedbackApp actuator1_info ( actuator1 ); 
// ActuatorControlApps display basic controls that allow springs and vibrations to be demonstrated 
ActuatorControlApp actuator0_ctrl ( actuator0 ); 
ActuatorControlApp actuator1_ctrl ( actuator1 ); 
  
// ConfigureApp lists the eeprom save settings state and allows saving new values using IrisControls 
EEPROM_config eeprom_config;  

// The following are some buttons to be displayed on IrisControls which enable general nativation of the firmware's state. 
ExposedButton zero_position_button ("Zero_Position", -1); 
ExposedButton enabled_button ("Enable", 0 ); 
ExposedButton config_button ("Config_Menu", -1 ); 
  
// A state variable used by a simple state machine in the loop() function 
enum IrisControls_State { 
  not_connected,
  default_view, 
  pod_view, 
  config_view,
} ic_state = not_connected;

/**
 * @brief Arduino setup function. 
 * 
 * This is primarily used to retrieve values from the EEPROM. 
 * The main loop checks for a connection to IrisControls app, which is when the actuators are initialized and enabled. 
 * That code could be moved here if connection to IrisControls is not to be required 
 */
void setup() {
  IrisControlsAPI::timeout = 2000; // 2 seconds of inactivity from the app will cause ::isConnected() to return false    
  if (SavedSettings::get ( EEPROM_ID::eeprom_version ) != EEPROM_VERSION ) {
    eeprom_config.factory_reset();  
  }
  
  actuator0.Settings.travel = ACTUATOR_STROKE_IN_MM;
  actuator0.Settings.max_force = constrain( (int)SavedSettings::get( EEPROM_ID::max_force_command ), -MAX_ACTUATOR_FORCE_COMMAND, MAX_ACTUATOR_FORCE_COMMAND );
  
  actuator1.Settings.travel = ACTUATOR_STROKE_IN_MM;
  actuator1.Settings.max_force = constrain( (int)SavedSettings::get( EEPROM_ID::max_force_command ), -MAX_ACTUATOR_FORCE_COMMAND, MAX_ACTUATOR_FORCE_COMMAND );
  
}


/**
 * @brief Arduino loop function 
 * 
 * This is called repeatedly. 
 * The IrisControlsAPI is polled every loop to read incoming data on the USB serial 
 * New connections to an IrisControls app result in reinitializing the actuator and populating the default GUI elements. 
 * The state of the IrisControls GUI is tracked; forces are only sent to the motor in certain states
 * Occasionally (once every 16 ms), the GUI buttons are polled. 
 */
void loop() {
  
  IrisControlsAPI::check();  // parses any incomming serial data from the USB 

// Example of detecting a new conntection to the IrisControls app

  if (!IrisControlsAPI::wasConnected() && IrisControlsAPI::isConnected()) {
    Printl("New connection detected...resetting app");
    Print("}my_name_is ");   Print(IrisControlsAPI::name);   Print("\r");
    initialize_actuator_forward(actuator0);     
    initialize_actuator_forward(actuator1);     
    actuator0.enable();
    actuator1.enable();
    
    reset_iriscontrols(); // loads the default view 
  }  
  
// Examples of ways to get the actuator data 
  float position  = actuator0.getPosition();
  float speed     = actuator0.mathResult.speed;
  float temp      = actuator0.getTemp();
  float voltage   = actuator0.getVoltage();

// An example state machine that will not send forces to the actuators unless IrisControls is connected and the enable button is checked. 
  
  if ( !IrisControlsAPI::isConnected() ) ic_state = not_connected;
  switch ( ic_state )  {
    case not_connected: 
      actuator0.servo( 0 );     
      actuator1.servo( 0 );   
      break;
    case pod_view: 
      thePod.run();   // driver diagnostics, literal API comms, firmware flashing algorithm  
      break;
    case default_view: 
      actuator0_info.run();
      actuator1_info.run();
      actuator0_ctrl.run();
      actuator1_ctrl.run();
      if (enabled_button.toggled_on()) {
        actuator0.servo( actuator0_ctrl.getForce() ); 
        actuator1.servo( actuator1_ctrl.getForce() ); 
      }
      else { 
        actuator0.servo( 0 );     
        actuator1.servo( 0 );     
      }
      break; 
    case config_view: 
      eeprom_config.run();
      if (eeprom_config.exit_request) toggle_config();
      break; 
  } 

// Example of checking and updating the IrisControls elements peridocially 

  static uint32_t last_iriscontrols_update; 
  if ( millis() - last_iriscontrols_update > 16  && IrisControlsAPI::isConnected() ) {
    last_iriscontrols_update = millis();         
    
    if (zero_position_button.pressed() ) {
      actuator0.resetPosition();
      actuator1.resetPosition();
    }
    if (config_button.pressed() ) toggle_config();
  }
}

/**
 * @breif This function is called anytime serial data becomes available from the USB. 
 * 
 * First the data is passed to the IrisControls parser.  
 * Next the data is passed to this application specific parser. 
 * Additional serial parsers could be added here as well. 
 */
int USBComms::parseSpecificCommands(char* command, char* args) {
  
  int ret = 
    IrisControlsAPI::parse(command, args) 
    ||  serialParse(command, args) 
    ;
  return ret; 

}

/**
 * @brief this parser contains special text commands for this application 
 * 
 * Custom serial commands can be added here by following the existing format. 
 */
int serialParse(char* command, char * args) {  
  START_PARSING

  // Used to program actuator0 and actuator1
  COMMAND_IS "p11" THEN_DO    
    Programmer::program (ThePod::orcas[11]);
  COMMAND_IS "p10" THEN_DO    
    Programmer::program (ThePod::orcas[10]);
  COMMAND_IS "p7" THEN_DO    
    Programmer::program (ThePod::orcas[7]);
  COMMAND_IS "p6" THEN_DO    
    Programmer::program (ThePod::orcas[6]);

  // Advanced manual packet sender for the actuators 
  COMMAND_IS "pod" THEN_DO    
    if ( ic_state == pod_view ) reset_iriscontrols();
    else {      
      clearApp();
      thePod.setup();     
      IrisControlsAPI::resize(); 
      ic_state = pod_view;
    }

  // Opens/Closes the config menu
  COMMAND_IS "config" THEN_DO 
    toggle_config();

  COMMAND_IS "recal" THEN_DO 
    Printl("Recalibrating actuator 1 and 2. Note that you must now power cycle those actuators.");
    recalibrate ( actuator0 );
    recalibrate ( actuator1 );    

  FINISH_PARSING
}


/**
 * @brief hide all IrisControls elements
 * 
 * Serial.send_now() calls block and make sure the output buffer is cleared to prevent it overflowing 
 */
void clearApp() {  

  Serial.send_now();
  
  thePod.shutdown();  
  actuator1_info.shutdown();
  actuator0_info.shutdown();  
  actuator0_ctrl.shutdown();
  actuator1_ctrl.shutdown();
  eeprom_config.shutdown();
  Serial.send_now();
  
  config_button.hide();
  enabled_button.hide();
  zero_position_button.hide();
  Serial.send_now();
  
}

/// Clear the screen (assuming clearApp() is comprehensive) and set up the defaul view 
void reset_iriscontrols() {  
  clearApp();
  IrisControlsAPI::background ( 100,100,100,25 ); 
  IrisControlsAPI::text ( 220,220,220,255 );
  Serial.send_now();
  
  actuator0_info.setup();
  actuator1_info.setup();
  actuator0_ctrl.setup();
  actuator1_ctrl.setup();
//  actuator2_info.setup();  
  Serial.send_now();
  
  zero_position_button.show();
  enabled_button.show();
  config_button.show();
  Serial.send_now();
  
  ic_state = default_view;
}


/// toggle between the config and default view 
void toggle_config () { 
  if ( ic_state == config_view ) { 
    reset_iriscontrols ();
    ic_state = default_view; 
    setup(); // apply new saved settings 
  }
  else { 
    clearApp();
    eeprom_config.setup();
    ic_state = config_view;   
  }    
}

