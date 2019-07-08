/********************************************************************************
 *
 * $Workfile: Hall_Sensor_v3.ino $
 *
 * $Creator: Jonathan Vargas $
 *
 * $Description: PID Control speed of BLDC motor using hall sensosr. $
 * It works in arduino serial monitor $
 ********************************************************************************/

//! Imports libraries
#include <Servo.h>
#include "PID.h"


//! Set BLDC motor constants
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
//! Pinout declaration
#define HALL_PIN 2
#define LED_PIN 13
#define HOME_PIN 5
#define MOTOR_PIN 9
//! Objects declaration
Servo motor;
PID pid( 0.000001f, 0.000125f , 0.0f, 50, 1000, 1200); // Sets dicrete PID constants
//! Global variables for PID controller
volatile float control_action = 0;
volatile float setpoint = 1800.00;     // target speed in rpm
volatile bool mem = false;             // Memory for attaching BLDC motor
//! Global variables for calculating rpm
volatile long count1=0;    // Counter each 0.5s
volatile long count2=10000;    // Counter each 0.1ms
volatile float period=0;
volatile float avg_rpm =0;
unsigned long time_now = 0;

/***************************************************************************
 *
 * setup Function
 *
 ***************************************************************************/

void setup() {
  // pinout definition
  pinMode(LED_PIN, OUTPUT);
  pinMode(HOME_PIN, OUTPUT);
  Serial.begin(115200);
  timer2_init();
  //  External interrupt configuratión
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), interruptCount, RISING); 
  sei();          //Enable interrupts

  Serial.println("===============================================================");
  Serial.println("	SPEED CONTROLLER USING HALL SENSOR v3");
  Serial.println("Caution: If you have already calibrated ESC and want another calibration, first desconnect ESC  from power source");
  Serial.println("0: Normal Operation, if you have calibrated ESC before");
  Serial.println("1: ESC Calibration");
  Serial.println("2: Stop motor");
  Serial.println("===============================================================");
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
    if(millis() > time_now + 1){
      time_now = millis();
      Serial.println(0);
    }
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
    digitalWrite(HOME_PIN, HIGH); 
    cli();
    Serial.println(1);
    period=(float)count2*0.0001f;
    avg_rpm=(120.0f/period);
    count1=0;
    count2=0;
    TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
    sei();
    digitalWrite(HOME_PIN, LOW); 
  }
}
//! Increments a counter each 0.1 ms
ISR(TIMER2_COMPA_vect)
{
  count1++;
  if(count1>=5000){
    count1=0;
    avg_rpm=0;
    count2=0;
  }
  count2++;
}
//! This function checks if any serial command was receive
void serialEvent(){
  if(Serial.available() > 0)
  {
    // The first byte received is the command
    int order_received = Serial.read();
    switch(order_received)
    {
      case '0':
      {
        motor.attach(MOTOR_PIN);
        Serial.println("Motor will start to rotate in some seconds");
        motor.writeMicroseconds(1000);
        delay(3000);
        motor.writeMicroseconds(1000);
        delay(3000);
        mem = true; 
        break;
      }
      case '1':
      {
        motor.attach(MOTOR_PIN);
        motor.writeMicroseconds(MAX_PULSE_LENGTH);
        Serial.println("Now connect the ESC to power and wait some seconds");
        Serial.println("Setting High Level");
        delay(5000); 
        digitalWrite(LED_PIN, HIGH);
        delay(4000);
        Serial.println("Setting Low Level");
        motor.writeMicroseconds(MIN_PULSE_LENGTH);
        delay(4000);
        Serial.println("Calibration finished!!");
        digitalWrite(LED_PIN, LOW);
        mem = true; 
        break;
      }
      case '2':
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
