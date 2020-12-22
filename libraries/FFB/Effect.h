#pragma once 

/**
 * Templates for 1D and 2D Feedback and Hapitc Effects
 */

#include <General_Macros.h>


class Effect {
  public: 

    int enabled = 1;
    
//    Effect() {}
    virtual ~Effect() {}

    uint16_t saturation = 0xFFFF;
    
    virtual void start(uint32_t dur = 0) {
      start_time = millis();
      duration = dur;      
      enabled = 1; 
    }
    virtual void stop() {
      start_time = 0;
      enabled = 0;
    }
    virtual void update() {}
    virtual void update(float v) {}
    virtual int getForce() {if (!enabled) return 0; return constrain(force,-saturation, saturation);}
    
    virtual void set (int newforce) {force = newforce;}
    
  protected: 
    int force = 0;
    uint32_t start_time, last_update, duration;    
    
};

class Effect_2D : public Effect{
  public: 
    
		virtual ~Effect_2D(){};
		int getForce() {return abs(x_force) + abs(y_force);}		
    virtual int getXforce() {return constrain(x_force,-saturation, saturation);}
    virtual int getYforce() {return constrain(y_force,-saturation, saturation);}
    
  protected: 
    int x_force = 0, y_force = 0;   
};


