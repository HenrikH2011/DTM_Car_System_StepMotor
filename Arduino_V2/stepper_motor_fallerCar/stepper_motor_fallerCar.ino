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
const int optic_Sensor_1 2 //optical Sensor on Pin 2 - start position
const int optic_Sensor_2 3 //optical Sensor on Pin 3 - Car count

// Start stepper motor PushButtom Pin defination:
const int pushB_start_Pin 4    // Pushbutton to start stepper motor (FOR TEST ONLY)

// LED for visual indication of movement:
const int LED_right 5  // LED indicator for moving stepper right
const int LED_center 6 // LED indicator for moving stepper to center
const int LED_left 7   // LED indicator for moving stepper left

// Stepper Motor pin definitions:
const int motorPin1  8      // IN1 on the ULN2003 driver
const int motorPin2  9      // IN2 on the ULN2003 driver
const int motorPin3  10     // IN3 on the ULN2003 driver
const int motorPin4  11     // IN4 on the ULN2003 driver

// initialize variables
int optic_Sensor_status = 0; // status of optical sensor
int step_pos_status = 0; // status of stepper motor position
int pushb_F1 = HIGH; // status variable for stepper Motor pushbutton

int stepM_speed_right = 500; // speed for moving clockwise (right)
int stepM_steps_right = 4096; // counts of steps clockwise (right)

int stepM_speed_left = -500; // Speed for moving counter-clockwise (left)
int stepM_steps_left = -4096; // counts of steps counter-clockwise (left)

// STEPPER MOTOR CONTROL
// Define the AccelStepper interface type; 4 wire motor in half step mode:
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the 
// AccelStepper library with 28BYJ-48 stepper motor:
#define MotorInterfaceType 8 // definetion from accelStepper library
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// FUNCTION to move right
void move_right(int stepM_speed_right){
  digitalWrite(LED_right, HIGH); // set LED ON

  optic_Sensor_status = optic_Sensor; // read optical sensor into sensor status variable
  
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor right at x steps/second until optic sensor == LOW
  while (digitalRead(optic_Sensor) == HIGH){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(stepM_speed_right); // stepper motor speed
    stepper.runSpeed(); // start stepper motor
  }

  stepper.stop(); // stop stepper motor when optical sensor == LOW
  stepper.setCurrentPosition(0); // set position to 0 (start positition)

  digitalWrite(LED_right, LOW); // set LED right OFF

} // FUNCTION RIGHT END --------------------------------------------

// FUNCTION to move center
void move_left(int stepM_speed_left, int stepM_steps_left){
  digitalWrite(LED_IN, HIGH); // set LED IN ON

  sensor_IN = optic_Sens_IN; // read sensor pin to variable sensor_IN

  // Reset the position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor backwards at x steps/second until the motor reaches -y steps (z revolution) DOWN/CLOSE : 
  while (digitalRead(optic_Sens_IN) == HIGH){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(speed_IN);
    stepper.runSpeed();
  }

  stepper.stop(); // stop stepper motor

  digitalWrite(LED_IN, LOW); // set LED IN OFF

} //function END -----------------------------------------------


// FUNCTION to move left
void move_left(int stepM_speed_left, int stepM_steps_left){
  digitalWrite(LED_IN, HIGH); // set LED IN ON

  sensor_IN = optic_Sens_IN; // read sensor pin to variable sensor_IN

  // Reset the position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor backwards at x steps/second until the motor reaches -y steps (z revolution) DOWN/CLOSE : 
  while (digitalRead(optic_Sens_IN) == HIGH){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(speed_IN);
    stepper.runSpeed();
  }

  stepper.stop(); // stop stepper motor

  digitalWrite(LED_IN, LOW); // set LED IN OFF

} //function END -----------------------------------------------

void setup() {
  // put your setup code here, to run once:

  LED_right = LOW;  // LED OFF
  LED_center = LOW; // LED OFF
  LED_left = LOW;   // LED ON

  // MOVE STEPPER RIGHT UNTIL OPTICAL SENSOR == HIGH



}

void loop() {
  // put your main code here, to run repeatedly:

}
