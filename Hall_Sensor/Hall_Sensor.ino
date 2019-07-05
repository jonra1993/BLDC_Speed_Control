/********************************************************************************
 *
 * $Workfile: Hall_Sensor.ino $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: PID Control speed of BLDC motor using hall sensosr. $
 ********************************************************************************/

//! Imports libraries
#include <Servo.h>
#include "PID.h"
#include "Serial_Protocol.h"

volatile int8_t ID = 1;  // Arduino identification, it is used in python script
//! Set BLDC motor constants
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
//! Pinout declaration
#define HALL_PIN 2
#define LED_PIN 13
#define MOTOR_PIN 9
//! Objects declaration
Serial_Protocol sp;
Servo motor;
PID pid( 0.1f, 0.0001f , 0.0f, 100, 1000, 1200); // Sets dicrete PID constants
//! Global variables for PID controller
volatile float control_action = 0;
volatile float setpoint = 1800.00;     // target speed in rpm
volatile bool mem = false;             // Memory for attaching BLDC motor
//! Global variables for calculating rpm
volatile long count2=10000;    // Counter each 0.1ms
volatile float period=0;
volatile float avg_rpm =0;   

/***************************************************************************
 *
 * setup Function
 *
 ***************************************************************************/

void setup() {
  pinMode(LED_PIN, OUTPUT);
  sp.begin();         
  timer2_init();
  //  External interrupt configuratión
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), interruptCount, RISING); 
  sei();          //Enable interrupts

}

/***************************************************************************
 *
 * Loop Function
 *
 ***************************************************************************/

void loop() {
  if (mem == true) 
    {
      control_action = pid.PID_ProcessIteration(avg_rpm,setpoint);
      motor.writeMicroseconds(int(control_action));
    }
 }

/***************************************************************************
 *
 * Functions
 *
 ***************************************************************************/

//! This function sets timer2 interrupt each 01ms>>
void timer2_init(){
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;   //initialize counter value to 0
  OCR2A = 99;   // = (16*10^6) / (1*8*10000) - 1 (must be <65536) 99 0.1ms
  TCCR2A |= (1 << WGM21);   // turn on CTC mode
  TCCR2B |= (1 << CS21);    // Set CS12 for a 32 bits prescaler
}
//! It calculates rpm after a revolution has finished
void interruptCount()
{  
  if(count2!=0){
    //delayMicroseconds(25);
    cli();
    period=(float)count2*0.0001f;
    avg_rpm=(120.0f/period);
    count2=0;
    TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
    sei();
  }
}
//! Increments a counter each 0.1 ms
ISR(TIMER2_COMPA_vect)
{
  count2++;
}
//! This function checks if any serial command was receive
void serialEvent(){
  if(Serial.available() > 0)
  {
    // The first byte received is the command
    Serial_Protocol::Order order_received = sp.read_order();    
    switch(order_received)
    {
      case sp.HELLO:
      {
        sp.write_i8(ID);
        break;
      }
      case sp.START:
      {
        motor.attach(MOTOR_PIN);
        motor.writeMicroseconds(1000);
        delay(3000);
        motor.writeMicroseconds(1000);
        delay(3000);
        mem = true; 
        break;
      }
      case sp.CALIBRATION:
      {
        motor.attach(MOTOR_PIN);
        motor.writeMicroseconds(MAX_PULSE_LENGTH);
        delay(5000); 
        digitalWrite(LED_PIN, HIGH);
        delay(4000);
        motor.writeMicroseconds(MIN_PULSE_LENGTH);
        delay(4000);
        digitalWrite(LED_PIN, LOW);
        mem = true; 
        break;
      }
      case sp.DATA:
      {
        sp.write_i16(int16_t(avg_rpm));
        sp.write_i16(int16_t(setpoint));
        sp.write_i16(int16_t(control_action));
        break;
      }
      case sp.STOP:
      {
        motor.writeMicroseconds(MIN_PULSE_LENGTH);
        mem=false;
        break;
      }
      // Unknown order
      default:
        return;
    }  
  }
}
