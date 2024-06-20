/* Project: DTM, Gas Silo
Version: beta 0
Dev: HH
MCU: Atmega328P/PB: Arduino UNO, NANO and Arduino Mega2560
Library: Accelstepper.h documentation homepage: 
- https://www.airspayce.com/mikem/arduino/AccelStepper/index.html

This code works with the ULN2003 Stepper Motor Driver Module and the 28BYJ-48 stepper motor
*/

#include <Arduino.h>
#include <AccelStepper.h>

const int LED_standby = 2; // NANO digital pin 2, moving NO
const int LED_active = 3; // NANO digital pin 3, moving YES

const int optic_sensor = 6; // NANO digital pin 6, Position = 0
const int pushButton = 7; // NANO digital pin 7, activate (HIGH) move stepMotor

const int motorPin1 = 9;  // NANO digital pin 9, stepMotor  - IN1
const int motorPin2 = 10; // NANO digital pin 10, stepMotor - IN2
const int motorPin3 = 11; // NANO digital pin 11, stepMotor - IN3
const int motorPin4 = 12; // NANO digital pin 12, stepMotor - IN4

// accelstepper MotorInterfaceType 8 == DRIVER, to be used with ULN2003 driver module
#define MotorInterfaceType 8 

// ULN2003 and 28BYJ-48 stepper motor connections, IN1, IN3,  IN2, IN4
// 
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);



// Function declarations and definitions **********************************************
// Move stepM to position = 0 (see optic_sensor)
void moveTo_Sensor(int, int){

  while (digitalRead(optic_sensor) == HIGH) {

    digitalWrite(LED_standby, LOW);
    digitalWrite(LED_active, HIGH);
    stepper.run();
    delay(100); // 100ms
    
  } // END while

  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);

  stepper.stop();
  stepper.setCurrentPosition(0);

  return;

} // END function moveTo_Sensor
void setup() {
  Serial.begin(9600);

  pinMode(LED_standby, OUTPUT);
  pinMode(LED_active, OUTPUT);
  pinMode(optic_sensor, INPUT);
  pinMode(pushButton, INPUT);

  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);
  // stepper.moveTo(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}