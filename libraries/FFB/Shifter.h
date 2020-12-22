#pragma once 

#include "Spring.h"
#include "Gate.h"
#include "Damping.h"
#include "Periodic.h"

    
class ShifterEffect : public Effect_2D {
  public: 

    Effect * X_axis, * Y_axis;
		
		float & xpos, & ypos, & xspeed, & yspeed, & xaccel, & yaccel;

    // ================================================= Device Output 
    enum State {
      neutral,
      first,
      second,
      third,
      fourth,
      fifth,
      reverse
    };
    int state = neutral;    

    //  ================================================  Sim Data Input 
    int gear = neutral; 
    float rpm = 0;
    float clutch = 0; 
    float mps = 0;
    float long_accel = 0, 
          lat_accel = 0, 
          vert_accel = 0;

    // =================================================== Static Parameters 
    uint16_t  gate_start      		= 120;//200; //125;
    uint16_t  gate_width        	= 1800;   
    uint16_t  y_gate_height_1     = 200;//80; //55; 
    uint16_t  y_gate_height_2     = 180;//120; //75; 
    float neutral_spring_k        = 0.07;
    int neutral_spring_sat        = 120;    
    float mass_k                  = 0.000;
    float damping_k               = 0;//-0.00100;
    uint16_t  wall_dz             = 150; //250;
    uint16_t  wall_inset          = 200;//200;//430; //100; //200;//430;
    float     wall_slope          = .5;//.3;//.25;
    uint16_t  gear_location_y     = 650;

    // ==================================================== Grind Parameters 
    float grind_wall_k            = 0.15;//0.22;   // Resistance to going into gear when unclutched 
    uint16_t grind_wall_start     = 20;   // 12 bit height from center where above force begins 
    float grind_ramp              = 2.5;  // Transfer function between grind wall force and grind magnitude
    float synchro_grind_start     = 150;
    uint16_t synchro_wall_sat     = 191;    
    float first_ratio   = 5.09;  //
    float second_ratio  = 2.99;  //   Gear 
    float third_ratio   = 2.04;  // 
    float fourth_ratio  = 1.59;  //   Ratios 
    float fifth_ratio   = 1.3;   //   
    float sixth_ratio   = 1.;    //     
    float engaged_spring_k = 0.2;
    uint16_t engaged_spring_sat = 190;

    // ============================================== Engine Vibrations Parameters       
    uint8_t engine_amplitude_a = 20;
    uint8_t engine_amplitude_b = 32;
    uint8_t engine_harmonic = 2;    
    uint16_t engine_ramp_a = 2000;      
    uint16_t engine_ramp_b = 6000;      

    // ============================================================   G Forces     
    float gforce_fz = 14.;
    float gforce_fy = 2.;
    float gforce_fx = 3.8;

    ShifterEffect(float & _xpos, float & _ypos, float & _xspeed, float & _yspeed, float & _xaccel, float & _yaccel) :
			xpos(_xpos), ypos(_ypos), xspeed(_xspeed), yspeed(_yspeed), xaccel(_xaccel), yaccel(_yaccel)		
		{
      setup();
    }    
    ~ShifterEffect() {
      cleanUp();    
    }


    void setup() {
      cleanUp();  			

      X_axis = new Effect;
      Y_axis = new Effect;
      X_axis->enabled = 1;
      Y_axis->enabled = 1;

      // ==================================================== Shift Lever Kinetic Force Corrections
      Mass   = new DampingEffect_2D(xspeed, yspeed, mass_k, mass_k);
      Damper = new DampingEffect_2D(xaccel, yaccel, damping_k, damping_k);
      
      // =================================================== Static Shifter Forces - Walls and Gates and Springs 
      NeutralSpring = new SpringEffect(xpos, neutral_spring_k);
      NeutralSpring->saturation = neutral_spring_sat;
      NeutralSpring->center     = 2048;      
      LeftWallSpring   = new SpringEffect(xpos, wall_slope);
      CenterWallSpring = new SpringEffect(xpos, wall_slope);
      RightWallSpring  = new SpringEffect(xpos, wall_slope);
      LeftWallSpring->null_zone   = wall_dz;            
      CenterWallSpring->null_zone = wall_dz;
      RightWallSpring->null_zone  = wall_dz;
      LeftWallSpring->center     =  wall_inset;
      CenterWallSpring->center   =  2048;
      RightWallSpring->center    =  4095 - wall_inset;      
      
      TopYGate = new GateEffect(
												ypos, 
                        (int)(2048 - gate_start - gate_width * 1.0),
                        (int)(2048 - gate_start - gate_width * 0.65),
                        (int)(2048 - gate_start - gate_width * 0.45),
                        (int)(2048 - gate_start - gate_width * 0.1),
                        (int)(2048 - gate_start), 
                        (int)  + y_gate_height_1,
                        (int)  + y_gate_height_2);                        
      BotYGate = new GateEffect(
												ypos, 
                        (int)(2048 + gate_start), 
                        (int)(2048 + gate_start + gate_width * .10),
                        (int)(2048 + gate_start + gate_width * .45),
                        (int)(2048 + gate_start + gate_width * .65),
                        (int)(2048 + gate_start + gate_width * 1.0),
                        (int)  + y_gate_height_2,
                        (int)  + y_gate_height_1);                        

      // ======================================================== Gear Engaged Force
      EngagedSpring = new SpringEffect_2D(xpos, ypos, engaged_spring_k,engaged_spring_k);    
      EngagedSpring->saturation = engaged_spring_sat;     
      EngagedSpring->adjustNullZone( 0 );                            
            
      // ==================================================== Grind Effects Setup
      SynchroWall = new SpringEffect (ypos, grind_wall_k);
      SynchroWall->null_zone = grind_wall_start;
      SynchroWall->center = 2048;   
      SynchroWall->saturation = synchro_wall_sat;         
      SynchroGrind = new SawtoothDownEffect ();    
      SynchroGrind->frequency( 10 );
      SynchroGrind->amplitude(  0  );
      SynchroGrind->start();

      // ===================================================== Engine Vibrations  
      EngineVibration = new SineEffect();      
      EngineVibration->start();     

    }
        
    void update() {
      
      state = advanceState(xpos, ypos);  
      //gear = state;    

      // Conditional Forces 
      if (gear) {
      //if (state) {
        EngagedSpring->moveCenter(gear_locations[gear][0],gear_locations[gear][1]);      
        EngagedSpring->adjustK(0.5 * clutch * .25 , clutch * .25);  
        EngagedSpring->update();        
      }
      else {
        SynchroWall->k = grind_wall_k * clutch;
        SynchroWall->update();   
             
        if (SynchroWall->getForce() > synchro_grind_start) 
          SynchroGrind->amplitude(  (SynchroWall->getForce() - synchro_grind_start) * grind_ramp );
        else if (SynchroWall->getForce() < -synchro_grind_start) 
          SynchroGrind->amplitude(  (SynchroWall->getForce() + synchro_grind_start) * grind_ramp );
        else 
          SynchroGrind->amplitude( 0 );
        switch (state) {
          case first:   SynchroGrind->frequency( first_ratio   * rpm / 100. );        
          case second:  SynchroGrind->frequency( second_ratio  * rpm / 100. );        
          case third:   SynchroGrind->frequency( third_ratio   * rpm / 100. );                  
          case fourth:  SynchroGrind->frequency( fourth_ratio  * rpm / 100. );        
          case fifth:   SynchroGrind->frequency( fifth_ratio   * rpm / 100. );        
          case reverse: SynchroGrind->frequency( sixth_ratio   * rpm / 100. );        
        }
        SynchroGrind->update();
      }     
      
      EngineVibration->update();
      
      // =============================== Static Forces 
      NeutralSpring->update();
      TopYGate->update();
      BotYGate->update();

      Damper->update();
      Mass->update();      
      
      x_force = 0;
      y_force = 0;

      // ==================== Wall Selector 
      switch (state) {
        case neutral:                                                                         break;
        case first:    
        case second:   LeftWallSpring->update();   x_force += LeftWallSpring->getForce();  break;
        case third: 
        case fourth:   CenterWallSpring->update(); x_force += CenterWallSpring->getForce(); break;
        case fifth: 
        case reverse:  RightWallSpring->update();  x_force += RightWallSpring->getForce();   break;
        default: break;
      }

      // ============================================= Add em up 
      //y_force += (vert_accel - 9.8) * gforce_fz;
      //y_force += long_accel * -gforce_fy;      
      y_force += SynchroWall->getForce();    
      y_force += SynchroGrind->getForce();
      y_force += EngagedSpring->getYforce();
      //y_force += Damper->getYforce();   
      //y_force += Mass->getYforce();     
      y_force += TopYGate->getForce();
      y_force += BotYGate->getForce();
      //y_force += EngineVibration->getForce();        

      //x_force += lat_accel * gforce_fx;
      x_force += EngineVibration->getForce();        
      x_force += EngagedSpring->getXforce();      
      //x_force += Damper->getXforce();
      //x_force += Mass->getXforce();       
      x_force += NeutralSpring->getForce();

			
			
      X_axis->set(-x_force);      
      Y_axis->set(-y_force);
      
			
			
      Joystick.buttons(0);      
      Joystick.button(state+1,1);
      pinMode(16, INPUT_PULLUP);
      Joystick.button(9, !digitalRead(16));      

  

      ACCESS_TIMER_NR(16000)
            // ===================================================== EngineVibrations 
      EngineVibration->frequency (engine_harmonic * rpm / 120.);
      if (rpm < engine_ramp_a)      EngineVibration->amplitude(engine_amplitude_a);
      else if (rpm < engine_ramp_b) EngineVibration->amplitude( (rpm - engine_ramp_a) / (float)(engine_ramp_b - engine_ramp_a) * (engine_amplitude_b - engine_amplitude_a) + engine_amplitude_a);
      else                          EngineVibration->amplitude(engine_amplitude_b);

    }

  protected: 

    // Effect Pointers 
    SpringEffect_2D  * EngagedSpring      = 0;    
    SpringEffect     * NeutralSpring      = 0;
    SpringEffect     * LeftWallSpring     = 0, 
                     * RightWallSpring    = 0, 
                     * CenterWallSpring   = 0;
    GateEffect       * TopYGate           = 0, 
                     * BotYGate           = 0;
    DampingEffect_2D * Damper             = 0,
                     * Mass               = 0;                     
    PeriodicEffect   * SynchroGrind     	= 0;
    SpringEffect     * SynchroWall      	= 0;
    PeriodicEffect   * EngineVibration    = 0;
    
    void cleanUp() {
      if (EngagedSpring)      delete EngagedSpring;    
      if (NeutralSpring)      delete NeutralSpring;
      if (LeftWallSpring)     delete LeftWallSpring; 
      if (RightWallSpring)    delete RightWallSpring; 
      if (CenterWallSpring)   delete CenterWallSpring;
      if (TopYGate)           delete TopYGate; 
      if (BotYGate)           delete BotYGate;
      if (Damper)             delete Damper;
      if (Mass)               delete Mass;                     
      if (SynchroGrind)       delete SynchroGrind;
      if (SynchroWall)        delete SynchroWall;
      if (EngineVibration)    delete EngineVibration; 
      EngagedSpring      = 0;    
      NeutralSpring      = 0;
      LeftWallSpring     = 0, 
      RightWallSpring    = 0, 
      CenterWallSpring   = 0;
      TopYGate           = 0, 
      BotYGate           = 0;
      //Damper             = 0,
      //Mass               = 0;                     
      SynchroGrind     = 0;
      SynchroWall      = 0;
      EngineVibration  = 0;
    }
    
    int gear_locations[7][2] = {
      {2048, 2048},      
      {wall_inset/2 , 2048 - (gate_width + gate_start)},   
      {wall_inset/2 , 2048 + gate_width + gate_start},   
                            {2048 , 2048 - (gate_width + gate_start)},    
                            {2048 , 2048 + gate_width + gate_start},    
                                                  {4095 - wall_inset/2 , 2048 - (gate_width + gate_start)},
                                                  {4095 - wall_inset/2 , 2048 + gate_width + gate_start}
    };
    
    int advanceState(float xpos, float ypos) { 
      //if (manual_override) return manual_override;
      if (ypos < gear_location_y * 2 ) {
        if (state != neutral) return state;
        if (xpos < wall_inset * 1.8)      return first;
        if (xpos < 4095-wall_inset * 1.8) return third;
        else                 return fifth;
      }
      else if (ypos > 4095 - gear_location_y * 2 ) {
        if (state != neutral) return state;
        if (xpos < wall_inset * 1.8) return second;
        if (xpos < 4095-wall_inset * 1.8) return fourth;
        else                 return reverse;        
      }
      else return neutral;      
    }
//    int advanceState(float xpos, float ypos) { 
//      //if (manual_override) return manual_override;
//      if (ypos < gear_location_y * 2 ) {
//        if (state != neutral) return state;
//        if (xpos < 1*4095/3) return first;
//        if (xpos < 2*4095/3) return third;
//        else                 return fifth;
//      }
//      else if (ypos > 4095 - gear_location_y * 2 ) {
//        if (state != neutral) return state;
//        if (xpos < 1*4095/3) return second;
//        if (xpos < 2*4095/3) return fourth;
//        else                 return reverse;        
//      }
//      else return neutral;      
//    }
};
