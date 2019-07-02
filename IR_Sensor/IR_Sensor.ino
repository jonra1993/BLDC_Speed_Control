#include <Servo.h>
#include "FC_51.h"
#include "PID.h"

#define IR_PIN 5
#define LED_PIN 13
#define MOTOR_PIN 9
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9

FC_51 ir_sensor(IR_PIN);

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN,ir_sensor.read()); //white hight and black low
  Serial.println(ir_sensor.read());

}
 
