/*************************************************************
 *
 * $Workfile: FC_51.h $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: Library for controlling FC_51 IR sensor $
 *************************************************************/

#ifndef FC_51_h
#define FC_51_h

#include "Arduino.h"


class FC_51 {
  private:  
   int _pin;

  public:
   FC_51(int pin);
   boolean read();
};

#endif
