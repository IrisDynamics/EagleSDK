#pragma once

#include "USB.h"

#define CHECK \
	if (!connected) return;
	
#define CHECK_ \
	if (!connected) return 0;
	
  #define ADD_ELEMENT_DELAY   				 25    
  
  #define MIN_VARIABLE_UPDATE_PERIOD         70
	
/**
  Adding Some Functionality to the App: 
	
	1) Add a command to this interface  (e.g. minimize())
	2) Implement it in the .cpp file by sending a serial command 
	3) Test Compile 
	4) In USB.cpp, add a receiving parser which emits a signal 
	5) Add the signal to USB.h
	6) In IrisControls.cpp, connect the signal to an appropriate slot. You might have to write that slot 



**/

class IrisControlsAPI
{
  public:
	
		static uint32_t timeout;
		
        
		virtual ~IrisControlsAPI() {}
		static USBComms USB;
		
		static const char * name;
		
		// The optimizer should reduce the size of these to be only as large as your use in your code 
    static int sliders[1000];
    static int presses[1000];
    static int toggles[1000];
    static int checks [1000];  
		
		
		IrisControlsAPI();
		
    // Reimplement these functions to build the interaction between the app and the device 
		
		static int check() {			  
			int ret;   
			ret = USB.check();
			if (ret) last_checkin = millis();
			if (millis() - last_checkin > timeout){
				connected = 0;	was_connected = 0;			
			}
			return ret; 
		}
		
    virtual void setup() {} ;
    virtual void run() {}		;
	virtual void shutdown() {};
	virtual void setFeedbacks() {};
	
	static int wasConnected() {	return was_connected;}
	
	static int isConnected() {	return was_connected = connected;}
	
	static void setTimeout(uint32_t to) {	timeout = to;}
	
	static int getControl(int index) {	return sliders[index];}
	
	static int getPresses(int index) {	int ret = presses [index];	presses[index] = 0;	return ret;	}
	static void setPresses(int index, int is ) {	presses [index] = is;	}
		
		
		// App - K20
		// This function is called whenever the USB Parser (this->parse()) receives updated control information from the App
		// ie: User moves a slider, App sends new slider position through USB, parse recieves command and calls this function 
		// This function should be overwritten by the application code to correctly use the new control information 
    static void receiveControl(int index, int data) {			
      sliders[index] = data;      
    }    
    static void receiveButtonPress(int index) {
		presses[index]++;
    }
		// Similar to recieve control but for button presses
    // These are called by the IrisControls library when appropriate data comes in 
    static void receiveButtonToggle(int index, int checked) { 
      toggles[index] = checked;
    }    
    static void receiveCheckbox(int index, int checkstate) {
      checks[index] = checkstate;
    }  		

    // Call this as part of the main serial parser whenever incoming data is available 
    static int parse(char* command, char * args);
		
		// Will cause the app to release the USB resource so another app (AVRdude for example) can use the connection
    static void disconnect();
		static void hold();
		static void resume();
		static void reset();
		static void resize();
		static void maximize();
		static void minimize();

		// K20 - App
    static void setFeedback(int index, float value);  // Use to update a bar or value 
    static int spamFeedback(int index, float value);  // Use to update a bar or value 
    static void addBar(const char * name, int index, int min, int max, int value, int checkState = - 1);
    static void addValue(const char * name, int index, int value);       
	static void addStatus(const char * name, int index);       
	static void updateStatus(const char * name, int index);       
    static void removeFeedback(unsigned int index);
	static void addLabel(const char * name, int index);       
	static void updateLabel(const char * name, int index);       
    static void addControl(const char * name, int index, int min, int max, int value, int checkState = - 1);
	static void setControl(int index, int value);
	static int spamControl(int index, int value);
    static void removeControl(unsigned int id);
    static void addButton(const char * name, unsigned int index, int state = - 1);
    static void removeButton(unsigned int inded);
    static void checkButton(unsigned int index, int state = -1);
    static void finishSetup() { Serial.send_now(); delay(100); Printl("}kay_done"); };
		
		
    static void enableFeedback (int index, int en);     
    static void enableControl (int index, int en);     
    static void enableButton (int index, int en);     

	static void reboundControl (int index, int upper, int lower);     
	static void reboundFeedback (int index, int upper, int lower);     
		
		
	static void addLogger ( int id, const char * name );
	static void writeToLog ( int id, const char * string );
	
	static void background ( int r, int g, int b, int a ); 
	static void text ( int r, int g, int b, int a ); 
			
		
		int is_active = 0;
		
	protected: 	
				
		int id = -1; 
		int is_setup = 0;
		
		static int connected;		
		int ID = 0;  // used for individual instances
		static int id_generator;
		static uint32_t last_checkin;
		static int was_connected;

};



class AppChain {
  public:

		~AppChain () {
			//cli();
			while (head) {
				AppLink * p = head; 
				head = head->next;
				delete p;
			}
			//sei();
		}
	
    void add (IrisControlsAPI * new_app) {
			if (contains(new_app)) return;
      head = new AppLink(new_app,head);    
			head->m_app->setup();
    }
    int contains (IrisControlsAPI * search_for) {
			AppLink * p = head; 
			while (p) {
				if (p->m_app == search_for) return 1;				
				p = p->next;
			}
			return 0;
		}
		int remove (IrisControlsAPI * to_remove) {
			if (!to_remove) 	return 0;
			if (!head) 				return 0;
			AppLink * p = head; 
			if (head->m_app == to_remove) {
				head = head->next;
				p->m_app->shutdown();
				delete p;
				return 1;
			} 
			while (p) {
				if (p->next && p->next->m_app == to_remove) {
					AppLink * q = p->next; 
					p->next=p->next->next; 
					q->m_app->shutdown();
					delete q;
					return 1;
				}
				p = p->next;
			}
			return 0;
		}
		
		void check() {  
      IrisControlsAPI::check();
			// if (!IrisControlsAPI::wasConnected() && IrisControlsAPI::isConnected()) { 
				// for (AppLink * p = head; p; p = p->next) {Serial.send_now(); p->m_app->setup(); }
				// IrisControlsAPI::finishSetup();			
			// }			
    }
    void setup(uint32_t timeout = 3000) {
	//	  uint32_t t = millis();
	 // 	while (millis() - t < timeout ) if (IrisControlsAPI::check()) break;
			
      for (AppLink * p = head; p; p = p->next) {
        p->m_app->setup();      
      }
	  
		delay(20); 
	  
		IrisControlsAPI::resize();
    }
    void run() {
      for (AppLink * p = head; p; p = p->next) {
        p->m_app->run();      
      }
    }
    static int serialParse(char* command, char * args);
  protected:
    class AppLink {			
      public:
		~AppLink () { 
			m_app->shutdown();
			//delete m_app;
		}
        AppLink(IrisControlsAPI * new_app, AppLink * next_link) {
          m_app = new_app;
          next = next_link;  
        }
        IrisControlsAPI * m_app;
        AppLink * next = 0;          
      };      
    
    AppLink * head = 0; 
};



