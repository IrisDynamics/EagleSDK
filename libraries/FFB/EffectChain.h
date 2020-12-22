#pragma once 
#include <Effect.h>


class EffectLink {
	
  public:
    static int administrator;
    int id; 

    EffectLink(Effect * p) {
      id = administrator++;
      effect = p;
    }
    ~EffectLink() {
      effect->enabled = 0;
    }
    Effect * effect = 0;
    EffectLink * next = 0; 
    
};


class EffectChain {
  public: 
    void add (Effect * p) {
//      if (this->contains(p)) return;   TODO 
      if (!head) {
        head = new EffectLink(p);
      }
      else {
        EffectLink * i = head; 
        while (i->next) i = i->next;
        i->next = new EffectLink(p);
      }
    }
    int remove(Effect * to_remove) {
      if (!to_remove)   return 0;
      if (!head)        return 0;
      EffectLink * p = head; 
      if (head->effect == to_remove) {
        head = head->next;   
        //delete p;
        return 1;
      } 
      while (p) {
        if (p->next && p->next->effect == to_remove) {
          //EffectLink * q = p->next; 
          p->next=p->next->next; 
          //delete q;
          return 1;
        }
        p = p->next;
      }
      return 0;      
    }
    int calculate () {
      EffectLink * i = head;
      int force = 0;
      while(i) {
        i->effect->update();
        force += i->effect->getForce();
        i = i->next;
      }
      return force;
    }
  protected:
    EffectLink * head = 0;    
};
