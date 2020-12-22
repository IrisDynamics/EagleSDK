/**
 * @file Config.h
 * @author Kyle 
 * @brief a config object using the EEPROM and IrisControlsAPI to provide a graphical config window for permanently saved values. 
 */
#pragma once

#include <PosCtrl.h>

#define EEPROM_VERSION 240
   
#define ACTUATOR_STROKE_IN_MM         150
#define MAX_ACTUATOR_FORCE_COMMAND    510


namespace EEPROM_ID { 
  
enum {  
  
  max_force_command, 
  
  actuator0_k,
  actuator1_k,
  actuator2_k,

  pctrl_settings_0,
  pctrl_settings_1 = pctrl_settings_0 + PosCtrl::eeprom_size,
  pctrl_settings_2 = pctrl_settings_1 + PosCtrl::eeprom_size,

  
  eeprom_version,    
  eeprom_end,
};

}
      

class EEPROM_config {

  public:

    int exit_request = 0;

    ExposedButton save;
    ExposedButton dflt;
    ExposedButton exit_app;
    
    EEPROM_config() :
      save ("Save_Settings", -1) ,
      dflt ("Default_Settings", -1),
      exit_app ("Exit_Config", -1) 
    {
      Setting * p;
      list = new Setting("EEPROM_VERSION", EEPROM_ID::eeprom_version, 0 , 1000 );
      p = list;
      //                            Setting Name                     Setting EEPROM address         Min Value     Max Value 
      p = (p->next = new Setting("Max_Force_Command",             EEPROM_ID::max_force_command,     0 ,     MAX_ACTUATOR_FORCE_COMMAND ));   
      p = (p->next = new Setting("Actuator_0_Force_Constant",       EEPROM_ID::actuator0_k,         2000,   3000));
      p = (p->next = new Setting("Actuator_1_Force_Constant",       EEPROM_ID::actuator1_k,         2000,   3000));
      p = (p->next = new Setting("Actuator_2_Force_Constant",       EEPROM_ID::actuator2_k,         2000,   3000));

    }

    void run() {

      if ( save.pressed() ) {      
        for (Setting * p = list->next; p; p = p->next) p->save();
      }
      
      if ( dflt.pressed() ) {
        factory_reset();
        shutdown();
        setup();
      }

      if ( exit_app.pressed() ) {
        exit_request = 1; 
      }


      ACCESS_TIMER_NR(20000);
      for (Setting * p = list; p; p = p->next) p->run();

            
    }

    void setup() {
      save.show();
      dflt.show();
      exit_app.show();      
      for (Setting * p = list->next; p; p = p->next) p->setup();  // initial setting is list->next to avoid showing the eeprom version 
      exit_request = 0;
    }
    void shutdown() {
      save.hide();
      dflt.hide();
      exit_app.hide();
      for (Setting * p = list; p; p = p->next) p->shutdown();
      exit_request = 0;
    }

    static void factory_reset() { 
  
      Printl("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Printl("! NOTICE:                           ");
      Printl("! Saved settings restored           "); 
      Printl("! to factory settings     ");
      Printl("! Use \"config\" to        ");
      Printl("! view changes            ");
      Printl("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      for (int i = 0; i < 2048; i++) SavedSettings::save( i, 0 );
    
      SavedSettings::save( EEPROM_ID::eeprom_version,EEPROM_VERSION );    
       
      SavedSettings::save( EEPROM_ID::max_force_command,   300 );
      SavedSettings::save( EEPROM_ID::actuator0_k,   2458 );
      SavedSettings::save( EEPROM_ID::actuator1_k,   2458 );
      SavedSettings::save( EEPROM_ID::actuator2_k,   2458 );

    }

  protected:

    // Saved Setting mini-apps
    class Setting: IrisControlsAPI {
      public:
        const char * name;
        int max, min, id;
        Setting( const char * n, int _id, int _min, int _max) {
          max = _max;
          min = _min;
          id  = _id;
          name = n;
        }
        void setup() {
          addControl(name, id, min, max, sliders[id] = SavedSettings::get(id));
          addBar(name, id, min, max, sliders[id] = SavedSettings::get(id));
        }
        void shutdown() {
          removeControl(id);
          removeFeedback(id);
        }
        void save() {
          SavedSettings::save(id, sliders[id]);
        }
        void run() {
          setFeedback(id,   (int32_t)SavedSettings::get(id));
        }
        Setting * next = 0;        //linked list
    };

    Setting * list = 0;
};


