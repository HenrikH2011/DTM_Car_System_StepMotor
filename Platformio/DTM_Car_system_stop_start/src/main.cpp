/* Project: DTM, Faller Car, Stop / Start magnet
Version: alpha - Date: 2023-01-28
Dev: HH
Test setup: FFS, HH
MCU: Atmega328P/PB: Arduino UNO, NANO and Arduino Mega2560
IDE: VS-Code (Microsoft) + PlatformIO
AI: Codeium extension for Visual Studio Code
Library: Accelstepper.h documentation homepage: 
- https://www.airspayce.com/mikem/arduino/AccelStepper/index.html

This code works with the ULN2003 Stepper Motor Driver Module and the 28BYJ-48 stepper motor.
When StepM not in use the accelstepper disableOutputs() function is used to set all output pin to LOW,
to save power, avoid the stepM is getting hot and then last longer.

NOTE: 
SerialMonitor: moserial Terminal, Databit 8, Stop Bit 1, Parity None, Handshake Software, Local Echo ON
baudrate: 9600 (Set as Serial.begin(9600);)
Serial print only used for testing feedback

Arduino NANO extension shield: Find and check datasheet - Power IN jack connector. 
Do not work properly with 7.5 VDC, work with 9VDC external power supply ??  

Optical sensor module signal (D0) is inverted on module. LOW signal when IR sensor is on / HIGH when IR sensor is off

IMPORTANT WHEN CONNETING DTM/Blue ULN2003 Driver Module to NANO Pins
Blue ULN2003 Driver module pins: IN1, IN2, IN3, IN4 is inverted on module.
pin_09 = IN4, pin_10 = IN3, pin_11 = IN2, pin_12 = IN1
*/

#include <Arduino.h>
#include <AccelStepper.h>

const int LED_standby = 2; // NANO digital pin 2, moving NO, Red LED
const int LED_active = 3;  // NANO digital pin 3, moving YES, Green LED

const int optic_sensor = 6; // NANO digital pin 6, Position = 0 (see optic_sensor)
const int pushButton = 7;   // NANO digital pin 7, activate (LOW : Input_pullup) move stepMotor


// Blue ULN2003 Driver module pins: IN1, IN3, IN2, IN4 is inverted on module.
const int motorPin1 = 9;  // NANO digital pin 9, stepMotor  - IN1
const int motorPin2 = 10; // NANO digital pin 10, stepMotor - IN2
const int motorPin3 = 11; // NANO digital pin 11, stepMotor - IN3
const int motorPin4 = 12; // NANO digital pin 12, stepMotor - IN4

/*
 // Green ULN2003 Driver module pins: IN1, IN3, IN2, IN4
const int motorPin1 = 12;  // NANO digital pin 9, stepMotor  - IN1
const int motorPin2 = 11; // NANO digital pin 10, stepMotor - IN2
const int motorPin3 = 10; // NANO digital pin 11, stepMotor - IN3
const int motorPin4 = 9; // NANO digital pin 12, stepMotor - IN4
*/

int speed_CounterClockW = 100; // speed to move stepM // counter-clockwise move around
int speed_ClockW = -100;  // speed to move stepM // clockwise move around

int pos_Stop = 235; // position to move stepM counter-clockwise
// int pos_Left = -50; // position to move stepM counter-clockwise

int serial_Print_Count = 0; // for serial print control

char pos_Status = 'X'; // S = Sensor, M = Middle, L = Left - position status

// Accellstepper library - MotorInterfaceType object
// accelstepper MotorInterfaceType 4 == FULL4WIRE: full-step or half-step to be used with ULN2003 driver
// accelstepper MotorInterfaceType 8 == HALF4WIRE: module and 28BYJ-48 stepper motor
#define MotorInterfaceType 4
// ULN2003 and 28BYJ-48 stepper motor connections, IN1, IN3,  IN2, IN4

AccelStepper Stop_Start_2_01 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// Functions and definitions ******************************************************
// Function moveTo_Sensor CHANGE THIS TO RIGHT LANE
// Move stepM right to position = 0 (see optic_sensor)
void moveTo_Sensor(){
  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Sensor ");

  Stop_Start_2_01.setSpeed(speed_ClockW);
  Serial.println("moving stepM clockwise");
  Serial.println("");

  Stop_Start_2_01.enableOutputs(); // enable output pins to move stepM

  while (digitalRead(optic_sensor) == LOW) {
    // move stepM clockwise until Sensor position
    Stop_Start_2_01.runSpeed();

  } // END while  

  Stop_Start_2_01.stop();
  Stop_Start_2_01.setCurrentPosition(0);
  Serial.println("setCurrentPosition(0)");
  Serial.println("stop stepM");
  Serial.println("");

  // moveTo_Middle(); // This not to be used here
  pos_Status = 'S'; // pos R == Sensor position

  Stop_Start_2_01.disableOutputs(); // set all output pin to LOW to stop stepM

  return;

} // END function moveTo_Sensor

// Function move_Middle
void moveTo_Stop() {

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Middle ");

  Stop_Start_2_01.setSpeed(speed_CounterClockW);
  Serial.println("moving stepM counter-clockwise"); 
  Serial.println("");

  Stop_Start_2_01.enableOutputs(); // enable output pins to move stepM

  while (Stop_Start_2_01.currentPosition() != pos_Stop) {  
    // move stepM counter-clockwise until pos_Middle  
    Stop_Start_2_01.runToNewPosition(pos_Stop); 
  
  } // END while

  Stop_Start_2_01.stop();
  Serial.println("stop stepM");
  Serial.println("");

  pos_Status = 'L'; // pos M == Middle

  Stop_Start_2_01.disableOutputs(); // set all output pin to LOW to stop stepM

  return;

} // END function move_Middle

/*
// Function move_Left
void moveTo_Left() {

  Serial.print("Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.print("moveTo_Left ");

  Stop_Start_2_01.setSpeed(speed_CounterClockW);
  Serial.println("moving stepM counter-clockwise");
  Serial.println("");

  Stop_Start_2_01.enableOutputs(); // enable output pins to move stepM
  
  while (Stop_Start_2_01.currentPosition() != pos_Left) {  
    // move stepM counter-clockwise until pos_Left  
    Stop_Start_2_01.runToNewPosition(pos_Left); 
  
  } // END while
  
  Stop_Start_2_01.stop();
  Serial.println("stop stepM");
  Serial.println("");

  pos_Status = 'L'; // pos L == Left

  Stop_Start_2_01.disableOutputs(); // set all output pin to LOW to stop stepM

  return;
} // END function move_Left

// END Functions and definitions ************************************************
*/

void setup() {
  Serial.begin(9600);

  pinMode(LED_standby, OUTPUT);
  pinMode(LED_active, OUTPUT);
  pinMode(optic_sensor, INPUT);
  pinMode(pushButton, INPUT_PULLUP);

  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);

  Stop_Start_2_01.setMaxSpeed(500);
  Stop_Start_2_01.setAcceleration(100);
  // Stop_Start_2_01.moveTo(0);

  Serial.print("void setup - Optic Sensor: ");
  Serial.println(digitalRead(optic_sensor));
  Serial.println("");
  
  if (digitalRead(optic_sensor) == LOW) { 
    // StepM not at optic sensor position
    // Sensor signal is inverted on sensor module
    moveTo_Sensor(); // move to sensor position
  } // END if
  
  if (digitalRead(optic_sensor) == HIGH) { 
    // StepM at optic sensor position
    // Sensor signal is inverted on sensor module
    pos_Status = 'S'; // pos R == Sensor position
  } // END if

} // END setup ---------------------------------------------------------------

void loop() {
  digitalWrite(LED_standby, HIGH);
  digitalWrite(LED_active, LOW);  

  if (serial_Print_Count == 0) {
    Serial.print("Optic Sensor: ");
    Serial.println(digitalRead(optic_sensor));
    serial_Print_Count = 1;
    Serial.print("pushButton: ");
    Serial.println(digitalRead(pushButton));
    Serial.println("");
  }
  
  if (digitalRead(pushButton) == LOW) {
    Serial.print("pushButton: ");
    Serial.println(digitalRead(pushButton));

    serial_Print_Count = 0;    
    Serial.print("Optic Sensor: ");
    Serial.println(digitalRead(optic_sensor));
    Serial.println("");    

    digitalWrite(LED_standby, LOW);
    digitalWrite(LED_active, HIGH);

    switch (pos_Status) {
      case 'S': // This not to be used here
        moveTo_Stop();
        break;
      
      case 'L':
        // moveTo_Right(); // This not to be used here. Changed to moveTO_sensor
        moveTo_Sensor(); // Right Lane        
        break;  

      default:
        // if nothing else matches, do the default
        // default is optional - just break if no other case matches
        break;      
    } // END switch

  } // END if
 
} // END void loop ----------------------------------------------------------------
// END OF CODE