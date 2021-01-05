/**
@mainpage Introduction

This is intended to demonstrate how to create and interact with GUI elements on the IrisControls application. No actuator control or communications are present here. 
Parsing serial commands from the GUI is also demonstrated. 

## Versions 
1.0.0 - Jan 2021 - KH 
  Initial version 
  
*/
#pragma GCC optimize ("O3")

#include <IrisControls.h>   // Used to communicate with the IrisControls application 
#include <Exposed.h>        // Exposed library contains the GUI elements (ie allows variables and functions to be exposed to the GUI)
const char * IrisControlsAPI::name = "EagleSDK_GUI_example";    // Feel free to change this...it will dictate what shows up on the IrisControls window
#include "normalize.h"        // contains a static function to translate between mN of force and motor API commands 


/////////////////////////
//       Globals 
///////////////////////

// The following are all the types of GUI elements supported by the SDK
ExposedIO 
  example_io_slider ("io_slider_name", 0, 144, -1);   // arguements are display name, minimum slider value, maximum slider value, initial slider value 

ExposedOutputBar 
  example_fb_slider ("fb_slider_name", 0, 288, -2);   // arguements are display name, minimum slider value, maximum slider value, initial slider value 

ExposedOutputValue
  example_fb_value ("fb_value_name", -1);

ExposedButton 
  example_button_1 ("button_1_name", 0),    // the first argument is the name, the second is the toggle state of the button. 0 is untoggled, 1 is toggled
  example_button_2 ("button_2_name", -1);   // -1 for the toggle state creates a button that cannot be toggled 

ExposedStatus   // places text or space on the right side of the screen 
  example_status ("Example_status_text"),   
  fb_buffers[10] { 
    {""},{""},{""},{""},{""},{""},{""},{""},{""},{""}
  };

ExposedLabel  // places text or space on the left side of the screen 
  example_label ("Example_label_text"),
  io_buffers[10] { 
    {""},{""},{""},{""},{""},{""},{""},{""},{""},{""}
  };




/**
 * @brief Arduino setup function.  
 */
void setup() {
  IrisControlsAPI::timeout = 2000; // 2 seconds of inactivity from the app will cause ::isConnected() to return false    
}

/**
 * @brief Arduino loop function 
 */
void loop() {
  
  IrisControlsAPI::check();  // parses any incomming serial data from the USB 

// Example of detecting a new conntection to the IrisControls app

  if (!IrisControlsAPI::wasConnected() && IrisControlsAPI::isConnected()) {
    Printl("New connection detected...");   // Print, Printl, Pspace, Ptab functions will all write to the IrisControls console 
    Print("}my_name_is ");   Print(IrisControlsAPI::name);   Print("\r");

    // Here is where the initial state of the GUI is set up 
    for (int i = 0; i<5; i++) { 
      fb_buffers[i].show();
      io_buffers[i].show();
    }
    example_io_slider.show(); 
    example_fb_slider.show(); 
    example_fb_value.show();
    example_button_1.show(); 
    example_button_2.show(); 
    example_status.show();
    example_label.show();
    
    for (int i = 5; i<10; i++) { 
      fb_buffers[i].show();
      io_buffers[i].show();
    }
    
  }  

// Example of checking and updating the IrisControls elements peridocially 

  static uint32_t last_iriscontrols_update; 
  if ( millis() - last_iriscontrols_update > 16  && IrisControlsAPI::isConnected() ) {
    last_iriscontrols_update = millis(); 

    if (example_button_1.pressed()) { 
      Printl("Button 1 pressed");
    }

    if (example_button_2.pressed()) {
      Printl("Button 2 pressed"); 
    }

    static int button1_state = 0; 
    if ( example_button_1.toggled_on() != button1_state ) { 
      button1_state = example_button_1.toggled_on();
      example_button_1.toggled_on() ? Printl("Button 1 is toggled on"): Printl("Button 1 is toggled off"); 
      if (button1_state) { 
        example_status.update("A_<b>bold</b>_status"); 
      }
      else { 
        example_status.update("A_<font_color=#00EE00>colored</font>_status");         
      }
    }

    int slider_value = example_io_slider.update();    // read io slider
    example_fb_slider.update(slider_value); // write fb slider
    example_fb_value.update(slider_value);
    
  }
}

/**
 * @brief This function is called anytime serial data becomes available from the USB. 
 * For custom serial commands, the serialParse function can be modified, but this function should be left alone. 
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

  COMMAND_IS "test_0" THEN_DO    
    Printl("test passed!");

  COMMAND_IS "test_1" THEN_DO 
    int a = IrisControlsAPI::USB.parse_int(args);
    float b = IrisControlsAPI::USB.parse_float(args);
    Print("test passed: arg 1 is ");
    Print(a);
    Print("; arg 2 is ");
    Printl(b);


  FINISH_PARSING
}

