/*************************************************************
 *
 * $Workfile: FC_51.cpp $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: Library for controlling FC_51 IR sensor $
 *************************************************************/

#include "FC_51.h"

/***************************************************************************
 *
 * PID Constructor
 *
 ***************************************************************************/

//! Setups pin mode
FC_51::FC_51(int pin)
{
  pinMode(pin, INPUT);
  _pin = pin;
}


/*****************************************************************************
 *
 * functions
 *
 ****************************************************************************/

//! Detects obstacles
boolean FC_51::read()
{
	if(digitalRead(_pin)==HIGH)
	{
		return false;
	}
	else
	{
	  return true;
	}
}
