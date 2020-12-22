#include "IrisControls.h"
#include "Exposed.h"

	
	
	
  
  
int IrisControlsAPI::id_generator = 0;
int ExposedVariable::id_generator = 000;


int IrisControlsAPI::connected = 0;
uint32_t IrisControlsAPI::last_checkin = 0;
uint32_t IrisControlsAPI::timeout = 1500;
int IrisControlsAPI::was_connected = 0;
int IrisControlsAPI::sliders[1000];
int IrisControlsAPI::presses[1000];
int IrisControlsAPI::toggles[1000];
int IrisControlsAPI::checks [1000];  	
USBComms IrisControlsAPI::USB;

IrisControlsAPI::IrisControlsAPI(){
	id = id_generator++;
}
		
int IrisControlsAPI::parse(char* command, char * args) {	
  START_PARSING
  COMMAND_IS "{c" THEN_DO
    int index = USB.parse_int(args);
    int data = USB.parse_int(args);		
    receiveControl(index,data);      
  COMMAND_IS "{bp" THEN_DO 
    int index = USB.parse_int(args);
    receiveButtonPress(index);       
  COMMAND_IS "{bt" THEN_DO 
    int index = USB.parse_int(args);
    int checked = USB.parse_int(args);
    receiveButtonToggle(index,checked);       
  COMMAND_IS "{cb" THEN_DO 
    int index = USB.parse_int(args);
    int checked = USB.parse_int(args);
    receiveCheckbox(index,checked);                    
  COMMAND_IS "{WhaddupHomie?" THEN_DO
	Serial.send_now(); delay(5);
	Pline("}chillin.chillin.");
	Serial.send_now();
	was_connected = 0;
	connected = 0;	
	//Pline("");
  //Serial.send_now();
	//delay(200);
	//connected = 1;
  COMMAND_IS "{herro?" THEN_DO
  Pline("}herro.");
  //connected = 1;
  COMMAND_IS "{yo?" THEN_DO
  Pline("}yo:)");
  //connected = 1;
	//connected = 1;
  COMMAND_IS "{name_please?" THEN_DO
	Serial.send_now(); delay(5);
  Print(name);
	Serial.send_now(); delay(5);
  COMMAND_IS "{okgo" THEN_DO
	connected = 1;
  COMMAND_IS "{okstop" THEN_DO
  connected = 0;
  FINISH_PARSING	
}

void IrisControlsAPI::disconnect() {
	CHECK
  Serial.send_now();
  Pline("}later_yo");
  Serial.send_now();
	connected = 0;	was_connected = 0;	
}
void IrisControlsAPI::reset() {
	CHECK
  Serial.send_now();
  Pline("}wipe_it");
  Serial.send_now();
}
void IrisControlsAPI::resize() {
	CHECK
	delay(250);  // allow app time to finish sizing 
  Serial.send_now();
  Pline("}focus");
  Serial.send_now();
}
void IrisControlsAPI::maximize() {
	CHECK
  Serial.send_now();
  Pline("}get_big");
  Serial.send_now();
}
void IrisControlsAPI::minimize() {
	CHECK
  Serial.send_now();
  Pline("}get_small");                                                           
  Serial.send_now();
}

void IrisControlsAPI::hold() {
	CHECK
  Serial.send_now();
  Pline("}hold_up");
  Serial.send_now();
}
void IrisControlsAPI::resume() {
	CHECK
  Serial.send_now();
  Pline("}kay_go");
  Serial.send_now();
}

void IrisControlsAPI::enableFeedback (int index, int en) {
  CHECK 
  Serial.send_now();
  Pspace("}f_en"); Pspace(index); Pline(en);
  Serial.send_now(); 	
	IrisControlsAPI::check();
}
void IrisControlsAPI::enableControl (int index, int en) {
  CHECK 
  Serial.send_now();  	
  Pspace("}c_en"); Pspace(index); Pline(en);
  Serial.send_now(); 	
	IrisControlsAPI::check();
}     
void IrisControlsAPI::enableButton (int index, int en) {
  CHECK 
  Serial.send_now();  	
  Pspace("}b_en"); Pspace(index); Pline(en);
  Serial.send_now(); 	
	IrisControlsAPI::check();
}

void IrisControlsAPI::addBar(const char * name, int index, int min, int max, int value, int checkState) {     
	CHECK
  Serial.send_now();  	
  Pspace("}bar_add"); Pspace(name);Pspace(index); Pspace(min);Pspace(max);Pspace(value);Pline(checkState);         
  ////Serial.send_now();      
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::addValue(const char * name, int index, int value) {    
	CHECK 
  ////Serial.send_now();    
  Pspace("}val_add"); Pspace(name);Pspace(index); Pline(value);
  ////Serial.send_now();      
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}


void IrisControlsAPI::setFeedback(int index, float value )               {
  CHECK 
  Pspace("}f"); Pspace(index); Pline(value);
}   

void IrisControlsAPI::setControl(int index, int value)               {
  CHECK 
  Pspace("}c"); Pspace(index); Pline(value);
}   

// The spam functions will not block 
int IrisControlsAPI::spamFeedback(int index, float value )               {
  CHECK_
  if (Serial.availableForWrite() < 15) {
	  return 0;	  
  }
  Pspace("}f"); Pspace(index); Pline(value);
  return 1;
}   

int IrisControlsAPI::spamControl(int index, int value)               {
  CHECK_ 
  if (Serial.availableForWrite() < 15) {
	  return 0;	  
  }
  Pspace("}c"); Pspace(index); Pline(value);
  return 1;
}   

void IrisControlsAPI::removeFeedback(unsigned int index) {
	CHECK
  ////Serial.send_now();    
  Pspace("}f_rm"); Pline(index); 
  //Serial.send_now();        
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}    

void IrisControlsAPI::addControl(const char * name, int index, int min, int max, int value, int checkState) {     
	CHECK
	sliders[index] = value;
//	Printl("");
  //Serial.send_now();   
  Pspace("}c_add"); Pspace(name);Pspace(index); Pspace(min);Pspace(max);Pspace(value);Pline(checkState);         
  Serial.send_now();  
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}

void IrisControlsAPI::reboundControl( int index, int min, int max) {     
	CHECK
  // //Serial.send_now();   
  Pspace("}c_rb"); Pspace(index); Pspace(min);Pline(max);
  // //Serial.send_now();     
	IrisControlsAPI::check();
}

void IrisControlsAPI::reboundFeedback( int index, int min, int max) {     
	CHECK
  // //Serial.send_now();   
  Pspace("}c_rb"); Pspace(index); Pspace(min);Pline(max);
  // //Serial.send_now();     
	IrisControlsAPI::check();
}

void IrisControlsAPI::addLabel(const char * name, int index) {
	CHECK	
  //Serial.send_now();   
  Pspace("}lab_add"); Pspace(name);Pline(index);
  ////Serial.send_now();     
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::addStatus(const char * name, int index) {
	CHECK
  ////Serial.send_now();   
  Pspace("}stat_add"); Pspace(name);Pline(index);
  ////Serial.send_now();     
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::updateLabel(const char * name, int index) {
	CHECK
  // ////Serial.send_now();   
  Pspace("}lab"); Pspace(name);Pline(index);
  // ////Serial.send_now();     
  //delay(2);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::updateStatus(const char * name, int index) {
	CHECK
  // ////Serial.send_now();   
  Pspace("}stat"); Pspace(name);Pline(index);
  // ////Serial.send_now();     
  //delay(2);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::removeControl(unsigned int index) {
	CHECK
  //////Serial.send_now();    
  Pspace("}c_rm"); Pline(index); 
  //////Serial.send_now();      
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}    

void IrisControlsAPI::addButton(const char * name, unsigned int index, int state) {
	CHECK	
  ////Serial.send_now();    
  Pspace("}b_add"); Pspace(name);Pspace(index);Pline(state);
  ////Serial.send_now();     
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::removeButton(unsigned int index) {
	CHECK	
  ////Serial.send_now();    
  Pspace("}b_rm"); Pline(index); 
  ////Serial.send_now();     
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}
void IrisControlsAPI::checkButton(unsigned int index, int state) {
  CHECK 
  //////Serial.send_now();    
  Pspace("}b_ck"); Pspace(index); Pline(state); 
  receiveButtonToggle(index,state);       	
  //////Serial.send_now();      
}

void IrisControlsAPI::addLogger ( int id, const char * name ) {
	CHECK
	//Print("Adding log file: "); Printl(name);
	Serial.send_now(); 
	Pspace("}l_add"); Pspace(id); Pline(name); 
	Serial.send_now(); 
  delay(ADD_ELEMENT_DELAY);    
	IrisControlsAPI::check();
}

void IrisControlsAPI::writeToLog ( int id, const char * string ) {
	CHECK	
	Pspace ("}l_s"); Pspace (id); Pline(string);
}



void IrisControlsAPI::background ( int r, int g, int b, int a ) {
	CHECK
	//Serial.send_now(); 
	Pspace("}bgnd"); Pspace(r); Pspace(g); Pspace(b); Pline(a);  
	//Serial.send_now(); 
}
void IrisControlsAPI::text  ( int r, int g, int b, int a ) {
	CHECK
	//Serial.send_now(); 
	Pspace("}text"); Pspace(r); Pspace(g); Pspace(b); Pline(a);  
	//Serial.send_now(); 
}

