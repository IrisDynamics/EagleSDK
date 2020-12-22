#include "Effect.h"

#pragma once 

class ConstantForce : public Effect {
  public: 
    void update ( int new_force ) { 
      force = new_force;
      last_update = millis();       
    }
    int getForce() {
      //Printl("--Getting Constant Force");  // TODO remove 
      if (millis() - last_update < nofade_duration) return force;       
      else if (millis() - last_update < nofade_duration + rampdown_duration) {
		  return force * (1. - (millis() - last_update - nofade_duration) / (float)rampdown_duration);      
	  }
      else return 0;
    }
    void setNoFadeDuration(int ms) {
//      ms = min (ms, 500); 
//      nofade_duration = ms;
    }
    void setRampdownDuration(int ms) {
//      ms = min (ms, 1500); 
//      rampdown_duration = ms;
    }
  protected: 
    int force; 
    uint32_t last_update = 0, nofade_duration = 100, rampdown_duration = 1000;
};