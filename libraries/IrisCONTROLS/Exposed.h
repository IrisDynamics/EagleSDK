#pragma once
#include <IrisControls.h>


// inthese variables can be exposed to the GUI either for inspection, for manipulation, or both. 

class ExposedVariable {
  public: 
    const char * name;
    ExposedVariable() {
      id = id_generator++;
    }    
  protected:
    int id;
    static int id_generator;
};

class ExposedIO : public ExposedVariable {
  public: 
    int v; 
    int min = -0x7FFFFFFF;
    int max =  0x7FFFFFFF;    
    int state = -1;
  
	uint32_t last_send = 0;
	
    ExposedIO( const char * n, int lower, int upper, int value, int check = -1 ) {
      name = n;
      v = value;
      min = lower; 
      max = upper;
      state = check; 
    }
    
    int    update() { 
      v = (IrisControlsAPI::sliders[ id ]);
      return v;       
    }    
    void set(int new_v) { 
		v = new_v;
		IrisControlsAPI::sliders[ id ] = v;
		if ( millis() - last_send > MIN_VARIABLE_UPDATE_PERIOD ) {			
			if ( IrisControlsAPI::spamControl( id , v=new_v) ) last_send = millis(); 
		}	
	}
    void show()          { IrisControlsAPI::addControl  (name , id , min , max  , v , state); }
    void hide()          { IrisControlsAPI::removeControl( id ); }       
    void enable( int en )        { IrisControlsAPI::enableControl( id, en ); }      
		void newLowerBound(int new_bound) {
			IrisControlsAPI::reboundControl ( id, min = new_bound, max);
		}
		void  newUpperBound(int new_bound) {
			IrisControlsAPI::reboundControl ( id, min, max = new_bound);
		}		
    int get() {return v;}
};

class ExposedOutputBar : public ExposedVariable {
  public:     
    int v; 
    float dumb;
    int min = -0x7FFFFFFF;
    int max =  0x7FFFFFFF;   
    float & variable; 
	
	uint32_t last_send = 0;
	
    ExposedOutputBar( const char * n, int lower, int upper, int value) : variable(dumb) {
      name = n;
      v = value;
      min = lower; 
      max = upper;
      id = id_generator++;
    }
    ExposedOutputBar( const char * n, int lower, int upper, int value, float & var ) : variable(var) {
      name = n;
      v = value;
      min = lower; 
      max = upper;
      id = id_generator++;
    }
		
		void newLowerBound(int new_bound) {
			IrisControlsAPI::reboundFeedback ( id, min = new_bound, max);
		}
		void  newUpperBound(int new_bound) {
			IrisControlsAPI::reboundFeedback  ( id, min, max = new_bound);
		}		
    void update()        { 
		if ( millis() - last_send > MIN_VARIABLE_UPDATE_PERIOD ) {			
			if ( IrisControlsAPI::spamFeedback( id , variable ) ) last_send = millis(); 
		}			
	}
    void update(int new_v) { 	
		if ( millis() - last_send > MIN_VARIABLE_UPDATE_PERIOD ) {			
			if ( IrisControlsAPI::spamFeedback( id , v = new_v ) ) last_send = millis(); 
		}		 
	}
    void show()          { IrisControlsAPI::addBar ( name, id, min, max, v ); }
    void hide()          { IrisControlsAPI::removeFeedback( id ); }
    void enable( int en )        { IrisControlsAPI::enableFeedback( id, en ); }       
    
};


class ExposedOutputValue : public ExposedVariable {
  public:     
    float v;     
    float &variable;
	
	uint32_t last_send = 0;
	
    ExposedOutputValue( const char * n, float value) : variable(v) {
      name = n;
      v = value;
      id = id_generator++;
    }
 
    ExposedOutputValue( const char * n, float value, float & var) : variable(var) {
      name = n;
      v = value;
      id = id_generator++;
    }    

    void update() {
		if ( millis() - last_send > MIN_VARIABLE_UPDATE_PERIOD ) {			
			if ( IrisControlsAPI::spamFeedback( id , variable) ) last_send = millis(); 
		}	
    }
    void update(float new_v) { 
		variable = new_v;
		if ( millis() - last_send > MIN_VARIABLE_UPDATE_PERIOD ) {			
			if ( IrisControlsAPI::spamFeedback( id , v = new_v ) ) last_send = millis(); 
		}	
	}
    void show()          { IrisControlsAPI::addValue ( name, id, v ); }
    void hide()          { IrisControlsAPI::removeFeedback( id ); }    
};

class ExposedLabel : public ExposedVariable {
  public:     
    ExposedLabel( const char * n ) {
      name = n;
    }
    void update(const char * new_name) { IrisControlsAPI::updateLabel( name = new_name , id ); }
    void show()          { IrisControlsAPI::addLabel ( name, id ); }
    void hide()          { IrisControlsAPI::removeControl( id ); }    
};

class ExposedStatus : public ExposedVariable {
  public:        
    ExposedStatus( const char * n ) {
      name = n;
    }
    void update(const char * new_name) { IrisControlsAPI::updateStatus( name = new_name , id ); }
    void show()          { IrisControlsAPI::addStatus ( name, id ); }
    void hide()          { IrisControlsAPI::removeFeedback( id ); }  
};

enum {
  unchecked = 0,
  checked   = 2
};
class ExposedButton : public ExposedVariable {
  public: 
    ExposedButton(const char * n = "", int check_state = -1) {
      name = n;
      state = check_state;
			IrisControlsAPI::toggles [ id ] = check_state;
    }
    void show() { IrisControlsAPI::addButton( name, id, state );  }
    void hide() { IrisControlsAPI::removeButton ( id );   }   
    void enable( int en )        { IrisControlsAPI::enableButton( id, en ); }       
    int  pressed() { return IrisControlsAPI::getPresses( id );}                               
    void pressed( int is ) { IrisControlsAPI::setPresses( id, is );}                               
    int  toggled_on() { return state = IrisControlsAPI::toggles [ id ]; }
    void update(int check_state) { 
			IrisControlsAPI::checkButton( id , check_state); 
		}    
  private: 
    int state;
};

