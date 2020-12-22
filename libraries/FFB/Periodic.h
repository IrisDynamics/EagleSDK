#pragma once 

#include "Effect.h"
#define EFFECT_SAMPLES      3003
#define EFFECT_SAMPLE_SIZE  1024
class PeriodicEffect : public Effect { 
  public: 
    
    virtual ~PeriodicEffect() {
    }
    
    void start(uint32_t dur = 0) {
      start_time = micros();
      duration = dur;      
      enabled = 1;      
    }
    
    void update () {
      if (!enabled) {
        force = 0;
        return; 
      }
      uint32_t elapsed = micros() - start_time;
      if (duration && elapsed > duration*1000) {
        Printl("Stopping Sine");
        stop();
        enabled = 0;
        return;
      }

      phase = getPhase();
      //phase = (uint64_t)(EFFECT_SAMPLES-1) * ((elapsed - phase_adjust) % period) / period;      
      force = (int32_t) _amplitude * midi[(uint32_t)(phase*(EFFECT_SAMPLES-1))] / (EFFECT_SAMPLE_SIZE-1);      
    }
    
    void frequency (float new_hertz) {
      setPeriod(1000000 / new_hertz); 
    }
      
    
    void setPeriod (uint32_t new_period) {  // microseoncd
      if (period == new_period) return;   
      float old_phase = getPhase();
      period = new_period;
      setPhase(old_phase);
    }
    
    void setPhase (float new_phase) {     
      while(new_phase > 1) new_phase -= 1;
      while(new_phase < 1) new_phase += 1;
      uint32_t elapsed = micros() - start_time;     
      float old_phase = (float)(elapsed % period) / period;
      phase = (float)((elapsed) % period) / period;  
      uint32_t usec_adjust = (new_phase - old_phase) * period;
      start_time -= usec_adjust;
      elapsed = micros() - start_time;      
    }
    
    uint32_t getPeriod() {
      return period;
    }
    
    float getPhase() {
      uint32_t elapsed = micros() - start_time;
      return (float)((elapsed) % period) / period;    
    }
    
    void amplitude (int16_t new_amplitude) {  // 10 bit input 
      _amplitude = new_amplitude; //constrain(new_amplitude, -2047, 2047);
    }
    int16_t amplitude() { return _amplitude;}
  
  
  protected:
    float phase = 0; 
    int16_t _amplitude = 0; 
    uint32_t period = 1;
    int32_t phase_adjust = 0; 
    int16_t * midi = 0; 
};


class SineEffect : public PeriodicEffect {
  public:   
    SineEffect() {
      midi = sine;
      if (is_setup) return; 
      for (int i = 0; i < EFFECT_SAMPLES; i++) {
        sine[i] = (EFFECT_SAMPLE_SIZE-1) * sin(6.283 * i / (float)(EFFECT_SAMPLES-1));
      }
      is_setup = 1;
    }
    ~SineEffect() {}
  protected: 
    static int is_setup;
    static int16_t sine[EFFECT_SAMPLES];
};

class SawtoothDownEffect : public PeriodicEffect {
  public: 
    SawtoothDownEffect() {
      midi = sawtoothdown;
      if (is_setup) return;
      for (int i = 0; i < EFFECT_SAMPLES; i++) {
        sawtoothdown[i] = EFFECT_SAMPLE_SIZE -  2 * EFFECT_SAMPLE_SIZE  * ((float)i / EFFECT_SAMPLES);
      }
      is_setup = 1;
    }   
    ~SawtoothDownEffect() {}
  protected: 
    static int is_setup;
    static int16_t sawtoothdown[EFFECT_SAMPLES];
};

class SquareWaveEffect : public PeriodicEffect {
  public: 
    SquareWaveEffect() {
      midi = square;
      if (is_setup) return;
      for (int i = 0; i < EFFECT_SAMPLES; i++) {
        square[i] = ( i < EFFECT_SAMPLES / 2 ) ? (EFFECT_SAMPLE_SIZE-1): -(EFFECT_SAMPLE_SIZE-1);  
      }
      is_setup = 1;
    }   
    ~SquareWaveEffect() {}
  protected: 
    static int is_setup;
    static int16_t square[EFFECT_SAMPLES];
};
