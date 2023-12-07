/*
  Project: Arduino - Steppermotor Control System
  DEV: Henrik Hansen, Electronic And Code (EAC), Denmark
  MCU: Arduino NANO or UNO (atmega328P)

  This code is in beta state and under construction.

  Arduino and 28byj-48 stepper motor with ULN2003 driver board
  AccelStepper library
  More info: 
  https://www.makerguides.com
  https://dronebotworkshop.com/stepper-motors-with-arduino/

  AccelStepper original library
  https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
*/

// Include the AccelStepper library:
#include <Arduino.h>
#include <AccelStepper.h>

// Optical Sensor pin definition:
#define optic_Sensor_1 2 //optical Sensor on Pin 2 - start position
#define car_Sensor 3 //Sensor on Pin 3 for Car count - SKAL FLYTTES TIL INTERRUPT PIN
#define stop_start_car 4 //Faller-Car start/stop signal Pin

// LED for visual indication of movement:
#define LED_stepper_move 5  // LED indicator for moving stepper - RED LED
#define LED_stepper_stop 6 // LED indicator for stop stepper - GREEN LED

// Stepper Motor pin definitions:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

// initialize variables
int car_count = 0; // Count variable for optic_Sensor_2
int stepper_pos = 0; // status of stepper motor position

int stepM_speed_right = 200; // speed for moving clockwise (right)

int stepM_speed_left = 200; // Speed for moving counter-clockwise (left)
int stepM_steps_left = -4096; // counts of steps counter-clockwise (left)

// STEPPER MOTOR CONTROL
// Define the AccelStepper interface type; 4 wire motor in half step mode:
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the 
// AccelStepper library with 28BYJ-48 stepper motor:
#define MotorInterfaceType 8 // definetion from accelStepper library
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// FUNCTION to move right
void move_right(int stepM_speed_right){
  digitalWrite(LED_stepper_move, HIGH); // set LED ON
  digitalWrite(LED_stepper_stop, LOW); // set LED OFF
  
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor right at x steps/second until optic sensor == LOW
  while (digitalRead(optic_Sensor_1) == HIGH){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(stepM_speed_right); // stepper motor speed
    stepper.runSpeed(); // start stepper motor
  }

  stepper.stop(); // stop stepper motor when optical sensor == LOW
  stepper.setCurrentPosition(0); // set position to 0 (start positition)

  digitalWrite(LED_stepper_move, LOW); // set LED OFF
  digitalWrite(LED_stepper_stop, HIGH); // set LED ON

} // FUNCTION RIGHT END --------------------------------------------

// FUNCTION to move left
void move_left(int stepM_speed_left, int stepM_steps_left){
  digitalWrite(LED_stepper_move, HIGH); // set LED ON
  digitalWrite(LED_stepper_stop, LOW); // set LED OFF
  
  // Reset the position to 0:
  stepper.setCurrentPosition(0); // set stepper start position 
  // Run the motor counter clockwise at x steps/second until the motor reaches -y steps (z revolution) DOWN/CLOSE : 
  while (stepper.currentPosition() != stepM_steps_left){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(stepM_speed_left);
    stepper.runSpeed();
  }

  stepper.stop(); // stop stepper motor

  digitalWrite(LED_stepper_move, LOW); // set LED OFF
  digitalWrite(LED_stepper_stop, HIGH); // set LED ON

} //function END -----------------------------------------------


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // setup and start serial communication default Tx0 / Rx0

  pinMode(optic_Sensor_1, INPUT);
  pinMode(car_Sensor, INPUT);

  pinMode(stop_start_car, OUTPUT);
  pinMode(LED_stepper_move, OUTPUT);
  pinMode(LED_stepper_stop, OUTPUT);
  
  stepper.setMaxSpeed(500); // max speed for stepper motor

  digitalWrite(LED_stepper_move, LOW); // set LED IN OFF
  digitalWrite(LED_stepper_stop, HIGH); // set LED IN ON

  car_count = 0;   // set variable to 0
  stepper_pos = 0; // set variable to 0

  
  // MOVE STEPPER RIGHT UNTIL OPTICAL SENSOR == HIGH
  move_right(stepM_speed_right); // call move right function

}

void loop() {
  
car_count = digitalRead(car_Sensor);

}
