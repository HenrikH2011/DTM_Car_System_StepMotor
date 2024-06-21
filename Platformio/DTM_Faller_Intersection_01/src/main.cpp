/* Project: DTM, Gas Silo
Version: beta 0.1
Dev: HH
MCU: Atmega328P/PB: Arduino UNO, NANO and Arduino Mega2560
IDE: VS-Code + PlatformIO
Library: Accelstepper.h documentation homepage: 
- https://www.airspayce.com/mikem/arduino/AccelStepper/index.html

This code works with the ULN2003 Stepper Motor Driver Module and the 28BYJ-48 stepper motor

Move stepM clockwise => step speed = 100
Move stepM counter-clockwise => step speed = -100
*/

#include <Arduino.h>
#include <AccelStepper.h>

const int LED_standby = 2; // NANO digital pin 2, moving NO
const int LED_active = 3;  // NANO digital pin 3, moving YES

const int optic_sensor = 6; // NANO digital pin 6, Position = 0
const int pushButton = 7;   // NANO digital pin 7, activate (HIGH) move stepMotor

const int motorPin1 = 9;  // NANO digital pin 9, stepMotor  - IN1
const int motorPin2 = 10; // NANO digital pin 10, stepMotor - IN2
const int motorPin3 = 11; // NANO digital pin 11, stepMotor - IN3
const int motorPin4 = 12; // NANO digital pin 12, stepMotor - IN4

int speed_CounterClockW = -100; // speed to move stepM counter-clockwise
int speed_ClockW = 100;  // speed to move stepM clockwise

int pos_Right = 500; // position to move stepM counter-clockwise or clockwise..?
int pos_Middle = 1000; // position to move stepM counter-clockwise or clockwise..?
int pos_Left = 1500; // position to move stepM counter-clockwise or clockwise..?
int pos_Sensor = -2000; // position to move stepM counter-clockwise or clockwise..?

char pos_Status = 'X'; // S = Sensor, R = Right, M = Middle, L = Left

// Functions and definitions ******************************************************

// Accellstepper library functions
// accelstepper MotorInterfaceType 4 == FULL4WIRE, full-step to be used with ULN2003 driver module
// accelstepper MotorInterfaceType 8 == HALF4WIRE, half-step to be used with 28BYJ-48 stepper motor
#define MotorInterfaceType 4
// ULN2003 and 28BYJ-48 stepper motor connections, IN1, IN3,  IN2, IN4

AccelStepper intersection_3_01 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// Function moveTo_Right
void moveTo_Right() {

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Right ");
  
  intersection_3_01.setSpeed(speed_CounterClockW);
  intersection_3_01.moveTo(pos_Right);
  intersection_3_01.runToPosition(); // move stepM counter-clockwise
  delay(100); // 100ms

  intersection_3_01.stop();   

  pos_Status = 'R';

  return;

} // END function moveTo_Right

// Function moveTo_Sensor
// Move stepM right to position = 0 (see optic_sensor)
void moveTo_Sensor(){
  // intersection_3_01.setSpeed(speed_ClockW);
  // intersection_3_01.move(40000);
  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Sensor ");

  while (digitalRead(optic_sensor) == LOW) {
        
    intersection_3_01.setSpeed(speed_ClockW);
    intersection_3_01.moveTo(pos_Sensor);
    intersection_3_01.runToPosition(); // move stepM counter-clockwise
    delay(100); // 100ms
    
  } // END while  

  intersection_3_01.stop();
  intersection_3_01.setCurrentPosition(0);

  moveTo_Right();

  return;

} // END function moveTo_Sensor

// Function move_Middle
void moveTo_Middle() {

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Middle ");

  intersection_3_01.setSpeed(speed_CounterClockW);
  intersection_3_01.runToNewPosition(200); // move stepM counter-clockwise
  delay(100); // 100ms

  intersection_3_01.stop();

  pos_Status = 'M';

  return;

} // END function move_Middle

// Function move_Left
void moveTo_Left() {

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Left ");

  intersection_3_01.setSpeed(speed_CounterClockW);
  intersection_3_01.runToNewPosition(300); // move stepM clockwise
  delay(100); // 100ms
  
  intersection_3_01.stop();

  pos_Status = 'L';

  return;
} // END function move_Left

// END Functions and definitions ************************************************

void setup() {
  Serial.begin(9600);

  pinMode(LED_standby, OUTPUT);
  pinMode(LED_active, OUTPUT);
  pinMode(optic_sensor, INPUT);
  pinMode(pushButton, INPUT);

  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);

  intersection_3_01.setMaxSpeed(500);
  intersection_3_01.setAcceleration(100);
  // intersection_3_01.moveTo(0);

  Serial.print("void setup - Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  
  if (digitalRead(optic_sensor) == LOW) { // StepM not at optic sensor position
    moveTo_Sensor();

  }
  
  if (digitalRead(optic_sensor) == HIGH) { // StepM at optic sensor position
    moveTo_Right();
  } // END if

} // END setup ---------------------------------------------------------------

void loop() {
  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);  

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));


  if (digitalRead(pushButton) == HIGH) {
    digitalWrite(LED_standby, LOW);
    digitalWrite(LED_active, HIGH);

    switch (pos_Status) {

      case 'S':

        moveTo_Right();

        break;

      case 'R':

        moveTo_Middle();

        break;

      case 'M':

        moveTo_Left();

        break;  
      
      case 'L':
        moveTo_Sensor();        
        
        break;  

      default:

        // if nothing else matches, do the default
        // default is optional - just break if no other case matches

        break;      

    } // END switch

  } // END if
  
} // END loop ----------------------------------------------------------------

