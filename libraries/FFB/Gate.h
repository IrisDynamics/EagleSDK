#pragma once 
#include "Effect.h"


class GateEffect : public Effect {
  public: 
    int p1,p2,p3,p4,p5;
    int height_1, height_2;
		float & pos;
		float dummy; 

    GateEffect(float & _pos, int p1_, int p2_, int p3_, int p4_, int p5_, int h1, int h2) :
			pos(_pos) {
      p1 = p1_;
      p2 = p2_;
      p3 = p3_;
      p4 = p4_;
      p5 = p5_;
      height_1 = h1;
      height_2 = h2;
    }

		
    GateEffect(int p1_, int p2_, int p3_, int p4_, int p5_, int h1, int h2) : pos (dummy){
      p1 = p1_;
      p2 = p2_;
      p3 = p3_;
      p4 = p4_;
      p5 = p5_;
      height_1 = h1;
      height_2 = h2;
    }
		
    void update(  ) {
      if      ( pos < p1 ) force = 0;
      else if ( pos < p2 ) force =    0    + height_1 * ( pos - p1 ) / ( p2 - p1 );
      else if ( pos < p3 ) force =  height_1 - height_1 * ( pos - p2 ) / ( p3 - p2 );       
      else if ( pos < p4 ) force =    0    - height_2 * ( pos - p3 ) / ( p4 - p3 );       
      else if ( pos < p5 ) force = -height_2 + height_2 * ( pos - p4 ) / ( p5 - p4 );
      else                 force = 0;
    }     

    void update( float v ) {
      if      ( v < p1 ) force = 0;
      else if ( v < p2 ) force =    0    + height_1 * ( v - p1 ) / ( p2 - p1 );
      else if ( v < p3 ) force =  height_1 - height_1 * ( v - p2 ) / ( p3 - p2 );       
      else if ( v < p4 ) force =    0    - height_2 * ( v - p3 ) / ( p4 - p3 );       
      else if ( v < p5 ) force = -height_2 + height_2 * ( v - p4 ) / ( p5 - p4 );
      else                 force = 0;
    }   		
};

class DoorEffect_2D : public Effect_2D {
  public: 
    int p1,p2,p3,p4,p5;
    int height_1, height_2;
    int q1, q2, q3, q4;

    DoorEffect_2D(int p1_, int p2_, int p3_, int p4_, int p5_, int q1_, int q2_, int q3_, int q4_, int h1, int h2) {
      p1 = p1_;
      p2 = p2_;
      p3 = p3_;
      p4 = p4_;
      p5 = p5_;
      q1 = q1_;
      q2 = q2_;
      q3 = q3_;
      q4 = q4_;
      height_1 = h1;
      height_2 = h2;
    }

    void update( int p, int q ) {
      float alpha; 
      if      ( p < p1 ) force = 0;
      else if ( p < p2 ) force =    0      + height_1 * ( p - p1 ) / ( p2 - p1 );
      else if ( p < p3 ) force =  height_1 - height_1 * ( p - p2 ) / ( p3 - p2 );       
      else if ( p < p4 ) force =    0      - height_2 * ( p - p3 ) / ( p4 - p3 );       
      else if ( p < p5 ) force = -height_2 + height_2 * ( p - p4 ) / ( p5 - p4 );
      else               force = 0;
      
      if      ( q < q1 ) alpha = 1;
      else if ( q < q2 ) alpha = 1 - ( q - q1 ) / ( q2 - q1 );
      else if ( q < q3 ) alpha = 0;
      else if ( q < q4 ) alpha = 0 + ( q - q3 ) / ( q4 - q3 );
      else               alpha = 1;

      force *= alpha;
    }        
};