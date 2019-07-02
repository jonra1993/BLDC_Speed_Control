/*ESC calibration sketch; author: ELECTRONOOBS */
#include <Arduino.h>
#include <Servo.h>
#include "PID.h"

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9
int DELAY = 1000;

volatile uint16_t count = 0;   //Main revolution counter
volatile uint16_t rpm = 0;     //Revolution per minute
volatile uint16_t rps = 0;     //Revolution per 
unsigned long previousMillis1 = 0;      
unsigned long previousMillis2 = 0;        
float control_action = 0;
Servo motor;
PID pid( 5.0f, 0.001f , 10.0f, 50, 1000, 1100);





void setup() {
  Serial.begin(9600);
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

  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");Serial.print(MIN_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
  motor.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(4000);
}

void loop() {

  unsigned long currentMillis = millis();

  control_action = pid.PID_ProcessIteration(rpm,1500.0);
  motor.writeMicroseconds(int(control_action));

  if (currentMillis - previousMillis1 >= 1000)
  {
    //CPU Jumps here every 1 sec exactly!
    previousMillis1 =  currentMillis;
    rps=count;
    rpm=rps*60;
    count=0;

  }
       
  if (currentMillis - previousMillis2 >= 100)
  {
    previousMillis2 =  currentMillis;
    Serial.println(rpm);
    Serial.println(control_action);

    
  }


 }



void interruptCount()
{
  count++;
}
