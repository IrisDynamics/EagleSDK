#pragma once 

class IIRFilter {
  public:
	
	float alpha = 1; 
	
	IIRFilter (float a = 1, float init = 0 ) : alpha(a) { history = init; }		
	virtual ~IIRFilter() {}
  	
    virtual float update (float new_sample) {			
			history = history * (1 - alpha) + new_sample * (alpha);
			return history; 		
    }
		
		float get () {return history;}

  protected:
		float history = 0;
};


class Normalized_IIRFilter : public IIRFilter {
  public:
	
		uint32_t micro_normalizer = 333;
		
		Normalized_IIRFilter (float a, uint32_t micro_norm = 333) : IIRFilter(a), micro_normalizer(micro_norm) {}		
  	
    float update (float new_sample) {
			uint32_t t_now = micros(); 			
			float normalized_alpha = alpha * (t_now - t_last) / (float)micro_normalizer;
			t_last = t_now;
			if (normalized_alpha > 1) normalized_alpha = 1;
			history = history * (1 - normalized_alpha) + new_sample * (normalized_alpha);
			return history; 		
    }
		
		float get () {return history;}

  protected:
		uint32_t t_last = 0; 
};


