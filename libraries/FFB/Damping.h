#pragma once 
#include "effect.h"

// =================================================== DAMPING EFFECT 
class DampingEffect : public Effect {
  public:

    float k; 
		float dummy;

    DampingEffect(float & _speed, float k_ = 0): speed(_speed) {
      k = k_;
    }		
    DampingEffect() : speed(dummy) {
      k = 0;
    }
    virtual ~DampingEffect() {};
    
    void update() {
      //Printl("--Updating Damping");  // TODO remove 
      force = speed * k;
    }    
		
    void update( float v) {
      //Printl("--Updating Damping");  // TODO remove 
      force = v * k;
    }    
		
    
    void adjustK(float newk) {
      k = newk;
    }
  protected: 
    float & speed;
};

class DampingEffect_2D : public Effect_2D {
  public:

    DampingEffect_2D(float & xderiv, float & yderiv, float kx, float ky) {
      XDamp = new DampingEffect(xderiv, kx);
      YDamp = new DampingEffect(yderiv, ky);
    }
    DampingEffect_2D(float kx, float ky) {
      XDamp = new DampingEffect(kx);
      YDamp = new DampingEffect(ky);
    }		
		~DampingEffect_2D() {
			delete XDamp;
			delete YDamp;
		}
    
    void update() {
      XDamp->update();
      YDamp->update();
      
      x_force = XDamp->getForce();
      y_force = YDamp->getForce();
    }
    
    void update(float v1, float v2) {
      XDamp->update(v1);
      YDamp->update(v2);
      
      x_force = XDamp->getForce();
      y_force = YDamp->getForce();
    }
		
    void adjustK (float new_k) {
      XDamp->k = new_k;
      YDamp->k = new_k;
    }

  protected: 
    DampingEffect * XDamp, * YDamp;
    
};
