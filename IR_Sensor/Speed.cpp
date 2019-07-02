/*************************************************************
 *
 * $Workfile: Speed.cpp $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: Library for counting pulses $
 *************************************************************/

#include "Speed.h"

/***************************************************************************
 *
 * PID Constructor
 *
 ***************************************************************************/

//! Setups pin mode
Speed::Speed(int pin)
{
	pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), interruptCount, RISING); 
  _pin = pin;
	init();
}


/*****************************************************************************
 *
 * functions
 *
 ****************************************************************************/

//set timer1 interrupt
void Speed::init()
{
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A = 249;// = (16*10^6) / (1*32*1000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS12 for a 32 bits prescaler
  TCCR2B |= (1 << CS22)|(1 << CS20);  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), interruptCount, RISING); 
  sei();//allow interrupts
}

void Speed:: interruptCount()
{
  pulses++;
}

ISR(TIMER2_COMPA_vect)
{
  Speed::count2++;
  if(Speed::count2==500){
    //CPU Jumps here every 1 sec exactly!
    Speed::rps=Speed::pulses;
    Speed::rpm=rps*60;
    Speed::pulses=0;
    Speed::count2=0;
  }
}