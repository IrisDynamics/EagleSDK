/**
 * @file normalize.h
 * @author Kyle
 * @brief converts force units used within by this firmware to API commands send to the actuators. 


 The actuators take the force commands received and simply convert them to scaled PWM values.  
 So the forces produced by a given API command depend on:
  1. the voltage 
  2. the temperature
  3. the force constant
  
 The following relationship describes how an API command C translates to a force F: 

  F = C/512 * Vdd * K@25 * (1 + (T-25)*-0.0017) 

  and to calculate an API command C for a force F  
  
  C = 512 * F / Vdd / K@25 / (1 + (T-25)*-0.0017)  
 

  K for DCA motors at 25 C and 24 V was found to be roughly 9.6 N / V or 2458/256. 
  Vdd is read from the orca library using actuator  
  T is read from the orca in degrees C    
  K@25 in N per V (and mN per mV) can be approximated by  1803 / 256. 

  To avoid floating point operations, integers and fixed point fractions are used 

  Note that Orcas take 9 bit commands for force: -255 to 255. 
  This library returns 10 bit commands (-511 to 511) because two Orcas are used per actuator, and the force is shared between them. 

 */
#ifndef _SRC_NORMALIZE_H_
#define _SRC_NORMALIZE_H_


class Normalize { 
  public: 

    static const int k_den = (1<<8);   // the denominator used for the motor constant (force per volt at 25 C) is statically defined as 2^17 or 131072 
    static const int dt_num = -14;
    static const int dt_den = (1<<13);
    static const int dead_codes = 11;

    /** 
     *  @brief translate a desired force in mN to an actuator duty cycle command, given the actuator voltage, temperature and force constant.
     *    
     */
    static int32_t  mN_to_API ( int32_t mN, int32_t mV, int32_t degC, int32_t k_num ) {  
      int64_t num = (int64_t)512 * dt_den * k_den * mN; 
      int64_t den = (int64_t)k_num * mV * ( dt_den + ( degC - 25 )*dt_num ); 
      int32_t raw_duty = num/den;
      if (raw_duty > 0) 
        return raw_duty + dead_codes;
      else 
        return raw_duty - dead_codes;
    }
  
};

#endif
