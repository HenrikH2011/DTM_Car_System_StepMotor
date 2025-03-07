/* Project: DTM_Car_System_01 
- intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1)
- Stop_Start_01 (_01: nr. 1) Stop and Start with Intersection
- Stop_Start_02 (_02: nr. 2) Stop and Start for sigle lane road

Version: beta .0.0.1
Dev: HH
Test setup: FFS, HH
MCU: Atmega328P/PB: Arduino UNO, NANO and Arduino Mega2560
IDE: VS-Code (Microsoft) + PlatformIO
AI: Codeium extension for Visual Studio Code
Library: Accelstepper.h documentation homepage: 
- https://www.airspayce.com/mikem/arduino/AccelStepper/index.html

Description: 
- This code is for Stop and Start magnets and Intersection control for 2 lane road, 
  and Stop and Start magnet for single lane road.

This code works with the 3x ULN2003 Stepper Motor Driver Module and 3x 28BYJ-48 stepper motor.
When StepM not in use the accelstepper disableOutputs() function is used to set all output pin to LOW,
to save power, avoid the stepM is getting hot and then last longer.

NOTE: 
SerialMonitor: moserial Terminal, Databit 8, Stop Bit 1, Parity None, Handshake Software, Local Echo ON
baudrate: 9600 (Set as Serial.begin(9600);)
Serial print only used for testing feedback

Arduino NANO extension shield: Find and check datasheet - Power IN jack connector. 
Do not work properly with 7.5 VDC, work with 9VDC external power supply ??  

Optical sensor module signal (D0) is inverted on module. LOW signal when IR sensor is on.

IMPORTANT WHEN CONNETING DTM/Blue ULN2003 Driver Module to NANO Pins
Check Blue ULN2003 Driver module pins: IN1, IN2, IN3, IN4 if inverted on module.
pin_09 = IN4, pin_10 = IN3, pin_11 = IN2, pin_12 = IN1
*/

#include <Arduino.h>
#include <AccelStepper.h>

// const int LED_standby = 2; // NANO digital pin 2, moving NO, Red LED
// const int LED_active = 3;  // NANO digital pin 3, moving YES, Green LED

const int optic_sens_01 = 13; // NANO digital pin, active = HIGH (Object detected)
const int optic_sens_02 = 12; // NANO digital pin
const int optic_sens_03 = 11; // NANO digital pin

const int pushButton_1 = A0; // NANO analog pin, active = HIGH, move stepM, Pulldown 10K
const int pushButton_2 = A1; // NANO analog pin, 
const int pushButton_3 = A2; // NANO analog pin, 

// StepM for Intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1)
const int motorPin_7 = 7;   // NANO digital pin, stepMotor - IN1
const int motorPin_8 = 8;   // NANO digital pin, stepMotor - IN2
const int motorPin_9 = 9;   // NANO digital pin, stepMotor - IN3
const int motorPin_10 = 10; // NANO digital pin, stepMotor - IN4

// StepM for Stop_Start_01 (_01: nr. 1) Stop and Start with Intersection
const int motorPin_3 = 3; // NANO digital pin, stepMotor - IN1
const int motorPin_4 = 4; // NANO digital pin, stepMotor - IN2
const int motorPin_5 = 5; // NANO digital pin, stepMotor - IN3
const int motorPin_6 = 6; // NANO digital pin, stepMotor - IN4

// StepM for Stop_Start_02 (_02: nr. 2) Stop and Start for sigle lane road
const int motorPin_2 = 2;   // NANO digital pin, stepMotor - IN1
const int motorPin_A3 = A3; // NANO digital pin, stepMotor - IN2
const int motorPin_A6 = A6; // NANO digital pin, stepMotor - IN3
const int motorPin_A7 = A7; // NANO digital pin, stepMotor - IN4


int speed_CounterClockW = -100; // speed to move stepM // counter-clockwise move around
int speed_ClockW = 100;  // speed to move stepM // clockwise move around

int pos_Middle = -165; // position to move stepM counter-clockwise
int pos_Left = -360; // position to move stepM counter-clockwise

int serial_Print_Count = 0; // for serial print control

char pos_Status = 'X'; // S = Sensor, M = Middle, L = Left - position status

// Accellstepper library - MotorInterfaceType object
// accelstepper MotorInterfaceType 4 == FULL4WIRE: full-step or half-step to be used with ULN2003 driver
// accelstepper MotorInterfaceType 8 == HALF4WIRE: module and 28BYJ-48 stepper motor
#define MotorInterfaceType 4
// ULN2003 and 28BYJ-48 stepper motor connections, IN1, IN3,  IN2, IN4

// StepM for Intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1) 
AccelStepper intersection_2_01 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
Accelstepper Stop_Start_01 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
Accelstepper Stop_Start_02 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);  