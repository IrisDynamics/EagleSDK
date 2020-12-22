#pragma once 
#include <EEPROM.h>


class SavedSettings {
  public:
    static void save(int id, uint32_t value) {
      id *= 4;      
      write(id + 0 ,value >> 24);
      write(id + 1 ,value >> 16);
      write(id + 2 ,value >> 8);
      write(id + 3 ,value >> 0);
    }

    static uint32_t get(int id) {
      id *= 4;
      return (EEPROM.read(id + 0) << 24) + 
             (EEPROM.read(id + 1) << 16) + 
             (EEPROM.read(id + 2) << 8) + 
             (EEPROM.read(id + 3) << 0);
    }

  protected: 
    static void write (int id, uint8_t value) {
      if (EEPROM.read(id) == value) return;
      EEPROM.write(id, value);
    }
    
};

