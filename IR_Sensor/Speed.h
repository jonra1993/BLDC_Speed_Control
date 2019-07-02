/*************************************************************
 *
 * $Workfile: Speed.h $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description:  Library for counting pulses $
 *************************************************************/

#include <Arduino.h>

#ifndef Speed_H_
#define Speed_H_

class Speed {
  private:  
    int _pin;
    volatile uint16_t pulses=0;    //Main revolution counter
    volatile uint16_t rpm=0;   //Revolution per minute
    volatile uint16_t rps=0;   //Revolution per second

    volatile uint16_t count2=0;    //Main revolution counter
    volatile uint16_t countms=0;    //Main revolution counter


  public:
    Speed(int pin);
    void init();
    int read();
    void interruptCount();
};

#endif