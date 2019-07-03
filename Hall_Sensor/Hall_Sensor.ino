#include <Servo.h>
#include "PID.h"
#include "Serial_Protocol.h"

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define IR_PIN 2
#define LED_PIN 13
#define MOTOR_PIN 9

// Serial protocol object
Serial_Protocol sp;

Servo motor;
const int buttonPin = 5;     // the number of the pushbutton pin


unsigned long previousMillis = 0;        // will store last time LED was updated

volatile long count2=10000;    //Main revolution counter
volatile float period=0;
volatile float avg_rpm =0;   
volatile uint16_t pulses=0;    //Main revolution counter

float control_action = 0;
float setpoint = 1800.00;
char data;
bool M = false;

PID pid( 0.50f, 0.001f , 0.0f, 50, 1000, 1200);





void setup() {
  
  sp.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  
//set timer1 interrupt at 1Hz>>
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A = 99;// = (16*10^6) / (1*8*10000) - 1 (must be <65536)
  //99 0.1ms
  //9 0.01ms
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS12 for a 32 bits prescaler
  TCCR2B |= (1 << CS21);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), interruptCount, RISING); 
  sei();//allow interrupts
  digitalWrite(LED_PIN, HIGH);

}

void loop() {
  if (M == true) 
    {
      control_action = pid.PID_ProcessIteration(avg_rpm,setpoint);
      motor.writeMicroseconds(int(control_action));
    }
 }



void interruptCount()
{  
  if(count2!=0){
    delayMicroseconds(25);
    cli();
    period=(float)count2*0.0001f;
    avg_rpm=(120.0f/period);
    count2=0;
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);
    sei();
  }
  
  
}

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
      case sp.START:
      {
        motor.attach(MOTOR_PIN);
        motor.writeMicroseconds(1050);
        delay(3000);
        motor.writeMicroseconds(1000);
        M = true; 
        break;
      }
      case sp.CALIBRATION:
      {
        motor.attach(MOTOR_PIN);
        motor.writeMicroseconds(MAX_PULSE_LENGTH);
        delay(5000); 
        digitalWrite(LED_PIN, HIGH);
        delay(3000);
        motor.writeMicroseconds(MIN_PULSE_LENGTH);
        delay(4000);
        digitalWrite(LED_PIN, LOW);
        M = true; 
        break;
      }
      case sp.DATA:
      {
        sp.write_i16(int16_t(avg_rpm));
        break;
      }
      // Unknown order
      default:
        
        return;
    }  
  }
}
