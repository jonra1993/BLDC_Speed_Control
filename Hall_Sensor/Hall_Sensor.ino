/*ESC calibration sketch; author: ELECTRONOOBS */
#include <Arduino.h>
#include <Servo.h>
#include "PID.h"

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9
int DELAY = 1000;

volatile uint16_t pulses=0;    //Main revolution counter
volatile uint16_t rpm=0;   //Revolution per minute
volatile uint16_t rps=0;   //Revolution per second
volatile uint16_t count2=0;    //Main revolution counter
unsigned long previousMillis1 = 0;      
unsigned long previousMillis2 = 0;        
float control_action = 0;
float setpoint = 1800.00;

Servo motor;
PID pid( 0.010f, 0.0001f , 0.0f, 50, 1000, 1200);





void setup() {
  Serial.begin(115200);
  
  //set timer1 interrupt at 1Hz>>
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

  
  Serial.println("Don't forget to subscribe!");
  Serial.println("ELECTRONOOBS ESC calibration...");
  Serial.println(" ");
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  motor.attach(MOTOR_PIN);

  Serial.print("Now writing maximum output: (");Serial.print(MAX_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor.writeMicroseconds(MAX_SIGNAL);
  delay(5000);
  motor.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(4000);
}

void loop() {


  control_action = pid.PID_ProcessIteration(rpm,setpoint);
  motor.writeMicroseconds(int(control_action));

  Serial.println(rpm);
       

 }



void interruptCount()
{
  pulses++;
}



ISR(TIMER2_COMPA_vect)
{
  count2++;
  if(count2==500){
    //CPU Jumps here every 1 sec exactly!
    rps=pulses;
    rpm=rps*60;
    pulses=0;
    count2=0;
    //Serial.println(rpm);
  }
}
