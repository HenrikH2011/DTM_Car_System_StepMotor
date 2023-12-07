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
#define optic_Sensor_2 3 //optical Sensor on Pin 3 - Car count
#define stop_start_car 4 //Faccer-Car start/stop signal Pin

// LED for visual indication of movement:
#define LED_right 5  // LED indicator for moving stepper right
#define LED_center 6 // LED indicator for moving stepper to center
#define LED_left 7   // LED indicator for moving stepper left

// Stepper Motor pin definitions:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

// initialize variables
int Car_count = 0; // Count variable for optic_Sensor_2
int stepper_pos = 0; // status of stepper motor position
int pushb_F1 = HIGH; // status variable for stepper Motor pushbutton

int stepM_speed_right = 500; // speed for moving clockwise (right)
// int stepM_steps_right = 4096; // counts of steps clockwise (right) NOT IN USE

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
  
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor right at x steps/second until optic sensor == LOW
  while (digitalRead(optic_Sensor_1) == HIGH){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(stepM_speed_right); // stepper motor speed
    stepper.runSpeed(); // start stepper motor
  }

  stepper.stop(); // stop stepper motor when optical sensor == LOW
  stepper.setCurrentPosition(0); // set position to 0 (start positition)

  digitalWrite(LED_right, LOW); // set LED right OFF

} // FUNCTION RIGHT END --------------------------------------------

// FUNCTION to move left
void move_left(int stepM_speed_left, int stepM_steps_left){
  digitalWrite(LED_center, HIGH); // set LED IN ON
  
  // Reset the position to 0:
  stepper.setCurrentPosition(0); // set stepper start position 
  // Run the motor counter clockwise at x steps/second until the motor reaches -y steps (z revolution) DOWN/CLOSE : 
  while (stepper.currentPosition() != -4096  ){ // run stepper motor until optical sensor goes LOW
    stepper.setSpeed(stepM_speed_left);
    stepper.runSpeed();
  }

  stepper.stop(); // stop stepper motor

  digitalWrite(LED_center, LOW); // set LED IN OFF

} //function END -----------------------------------------------


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // setup and start serial communication default Tx0 / Rx0

  pinMode(optic_Sensor_1, INPUT);
  pinMode(optic_Sensor_2, INPUT);
  pinMode(stop_start_car, OUTPUT);
  pinMode(LED_right, OUTPUT);
  pinMode(LED_center, OUTPUT);
  pinMode(LED_left, OUTPUT);

  stepper.setMaxSpeed(500); // max speed for stepper motor

  digitalWrite(LED_right, LOW); // set LED IN OFF
  digitalWrite(LED_center, LOW); // set LED IN OFF
  digitalWrite(LED_left, LOW); // set LED IN OFF
  
  // MOVE STEPPER RIGHT UNTIL OPTICAL SENSOR == HIGH
  move_right(stepM_speed_right); // call move right function

}

void loop() {
  


}
