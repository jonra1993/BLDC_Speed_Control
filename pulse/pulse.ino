/********************************************************************************
 *
 * $Workfile: pulse.ino $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: prints when a pulse is detected $
 ********************************************************************************/
#include <Arduino.h>

//! Pinout declaration
#define PULSE_PIN 2
unsigned long time_now = 0;
/***************************************************************************
 *
 * setup Function
 *
 ***************************************************************************/

void setup() {
  Serial.begin(115200);        
  //  External interrupt configuratión
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), interruptCount, RISING); 
}

/***************************************************************************
 *
 * Loop Function
 *
 ***************************************************************************/

void loop() {
  if(millis() > time_now + 1){
    time_now = millis();
    Serial.println(0);
  }
}

/***************************************************************************
 *
 * Functions
 *
 ***************************************************************************/


//! It calculates prints 1 when a pulse is detected
void interruptCount()
{  
  Serial.println(1);
}
