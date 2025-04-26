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


ToDo:
1. edit Pin connections 
  - Digital
  - Analog pin's : Can't use analog pins AccelStepper.h
  - NEW pin edit:
    - stepM1 new pin settings, adjust in code : OK
    - stepM2 new pin settings, adjust in code : OK
    - stepM3 new pin settings, adjust in code : OK
    - Pushbutton new pin settings, adjust in code : OK
    - IR sensor new pin settings, adjust in code : OK
    - switch_Halt new pin settings, adjust in code : OK


2. Connect stepM, IR-sensor, switch_Halt and pushbutton to NANO extension shield, with new pin settings..!!!!

3. Test and debug code, check if all pins are connected properly.
4. Check if all stepM is working properly.
5. Test all functions and adjust code.

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

/*__________________________________ CODE Start ________________________________________*/

#include <Arduino.h>
#include <AccelStepper.h>

/*  ----------------------- Stepper motor control interface -------------------------- */  
int delay_1 = 1000; // milliseconds
int delay_2 = 2000; // milliseconds

// The 28BYJ-48 steppermotor have 4096 steps per rotation
int speed_ClockW = 500;   // speed to move stepM clockwise 
int speed_CClockW = -500; // speed to move stepM counter-clockwise

// NOTE: this stepM move CCW to IR sensor position
long stepM_1_StopPos = 72;    // move stepM CW to Stop position
long stepM_1_IRsensPos = -80; // move stepM CCW to IR sensor position

// NOTE: this stepM move CCW to IR sensor position
long stepM_2_StopPos = 150;    // move stepM CW to Stop position
long stepM_2_IRsensPos = -180; // move stepM CCW to IR sensor position

// NOTE: this stepM move CW to IR sensor position
//long stepM_3_StopPos = -200;  // move stepM CCW to Stop position
//long stepM_3_IRsensPos = 200; // move stepM CW to IR sensor position


/*  ---------------------------------------------------------------------------------- */

/* ************************* Define and initalize GLOBAL variables ******************* */
bool serialPrint_loop = true; // Serial Print run only if true
int serial_Print_Count = 0; // for serial print control

bool switch_Halt_loop = true; // serial print run only if true

char pos_Status = 'X'; // S = Sensor, M = Middle, L = Left - position status

/* ************************* Define golbale constants ******************************* */
// const int LED_standby = 2; // NANO digital pin 2, moving NO, Red LED
// const int LED_active = 3;  // NANO digital pin 3, moving YES, Green LED

// Define Pin constants
// active = HIGH (Object detected) Signal inverted in IC on PCB
const int IR_Sens_1 = 14; // NANO A0 analog pin, as digital input
const int IR_Sens_2 = 15; // NANO A1 analog pin, as digital input
const int IR_Sens_3 = 16; // NANO A2 analog pin, as digital input

//active = LOW - default = true (HIGH) pause code until false (LOW)
const int switch_Halt = 20;  // NANO analog A6 pin - ONLY Analog IN/OUT (A7 ONLY analog IN/OUT)

const int pushButton_1 = 17; // NANO Analog A3 Pin called as digital pin, active = HIGH, move stepM
const int pushButton_2 = 18; // NANO Analog A4 Pin called as digital pin, 
const int pushButton_3 = 19; // NANO Analog A5 Pin called as digital pin, NOT IN USE

// StepM 1 - StepM for Stop_Start_01 (_01: nr. 1) Stop and Start with Intersection
const int StepM1_IN1 = 2; // NANO digital pin, stepMotor - IN1
const int stepM1_IN2 = 3; // NANO digital pin, stepMotor - IN2
const int stepM1_IN3 = 4; // NANO digital pin, stepMotor - IN3
const int stepM1_IN4 = 5; // NANO digital pin, stepMotor - IN4

// StepM 2 - StepM for Stop_Start_02 (_01: nr. 2) Stop and Start for sigle lane road
const int stepM2_IN1 = 6;  // NANO digital pin, stepMotor - IN1
const int stepM2_IN2 = 7;  // NANO digital pin, stepMotor - IN2
const int stepM2_IN3 = 8; // NANO digital pin, stepMotor - IN3
const int stepM2_IN4 = 9; // NANO digital pin, stepMotor - IN4


// StepM 3 - StepM for Intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1)
const int stepM3_IN1 = 10 ; // NANO digital pin, stepMotor - IN1
const int stepM3_IN2 = 11 ; // NANO digital pin, stepMotor - IN2
const int stepM3_IN3 = 12 ; // NANO digital pin, stepMotor - IN3
const int stepM3_IN4 = 13 ; // NANO digital pin, stepMotor - IN4


// Accellstepper library - MotorInterfaceType object
// accelstepper MotorInterfaceType 4 == FULL4WIRE: full-step or half-step to be used with ULN2003 driver
// accelstepper MotorInterfaceType 8 == HALF4WIRE: module and 28BYJ-48 stepper motor

// ULN2003 and 28BYJ-48 stepper motor connections, IN1, IN3,  IN2, IN4
// MotorInterfaceType for Intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1)
#define MotorInterfaceType_1 4
// MotorInterfaceType for Stop_Start_01 (_01: nr. 1) Stop and Start with Intersection
#define MotorInterfaceType_2 4
// MotorInterfaceType for Stop_Start_02 (_02: nr. 2) Stop and Start for sigle lane road
//#define MotorInterfaceType_3 4

// StepM for Intersection_2_01 (_2: 2 Lane road. _01: Intersection nr. 1) 
AccelStepper stepM_1 = AccelStepper(MotorInterfaceType_1, StepM1_IN1, stepM1_IN3, stepM1_IN2, stepM1_IN4);

// StepM for Stop_Start_01 (_01: nr. 1) Stop and Start with Intersection
AccelStepper stepM_2 = AccelStepper(MotorInterfaceType_2, stepM2_IN1, stepM2_IN3, stepM2_IN2, stepM2_IN4);

// StepM for Stop_Start_02 (_02: nr. 2) Stop and Start for sigle lane road
//AccelStepper stepM_3 = AccelStepper(MotorInterfaceType_3, stepM3_IN1, stepM3_IN3, stepM3_IN2, stepM3_IN4);

// defination and initalization of functions: **************************************************************
void moveStopPos(AccelStepper& stepM, int steps, int8_t nr){//move steppermotors to OUT position --
  Serial.print ("move StopPos start: ");
  Serial.println( nr );
  stepM.enableOutputs();
  stepM.setCurrentPosition(0);
  stepM.moveTo(steps);
  // Only run stepM if IR sensor is LOW and switch_Halt is OFF (HIGH signal) and stop stepM when at posOut
  while (stepM.distanceToGo() != 0 && digitalRead(switch_Halt) == HIGH) {
    stepM.run();      
  }// END while
  stepM.stop();
  Serial.println("StepM stopped");
  Serial.print("Current position: ");
  Serial.println(stepM.currentPosition());
  Serial.print("Distance to go: ");    
  Serial.println(stepM.distanceToGo());
  stepM.setCurrentPosition(0);
  Serial.println("Current position set to 0");
  Serial.println("");
  stepM.disableOutputs();
  delay(delay_2);  
} // END moveOUT

void moveToIRsens(AccelStepper& stepM,int steps, int8_t IR_sens, int8_t nr){ // move all steppermotors to IN position ---------
  Serial.println("move IRsens start "); 
  stepM.enableOutputs();
  stepM.setCurrentPosition(0);
  stepM.moveTo(steps);
  // Only run stepM if switch_Halt is OFF (HIGH signal) and stop stepM when at outPos
  while (digitalRead(IR_sens) == LOW && digitalRead(switch_Halt) == HIGH) { // while IR sensor object not detected
    stepM.run();
  } // END while
  stepM.stop();
  Serial.println("StepM stopped");
  Serial.print("Current position: ");
  Serial.println(stepM.currentPosition());
  Serial.print("Distance to go: ");    
  Serial.println(stepM.distanceToGo());
  stepM.setCurrentPosition(0);
  Serial.println("Current position set to 0");
  Serial.println("");
  stepM.disableOutputs();
  delay(delay_2);
} // END moveIN

void setup() { /**********************************************************************************/
  Serial.begin(9600); // setup serial comminication

  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {} // Test serial connection
  Serial.println("Serial port ready");
  Serial.println("void setup start");
  Serial.println("");

  //disable all step motor output pin signals for sleep mode
  stepM_1.disableOutputs();
  stepM_2.disableOutputs();
  //stepM_3.disableOutputs();  
  Serial.println("all stepmotor output pins disabled");

  // Set pin modes 
  pinMode(switch_Halt, INPUT_PULLUP);  // switch ON = LOW / switch OFF = HIGH 
  pinMode(pushButton_1, INPUT_PULLUP); // Pushbutton 10K pulldown / pushb pressed = HIGH / not pressed = LOW
  pinMode(pushButton_2, INPUT_PULLUP); 
  //pinMode(pushButton_3, INPUT_PULLUP); 

  pinMode(IR_Sens_1, INPUT_PULLUP);  // IR sensor internal pullUp
  pinMode(IR_Sens_2, INPUT_PULLUP);  
  pinMode(IR_Sens_3, INPUT_PULLUP);  

  /* --------------------- AccelStepper functions setup: ---------------------------------------------- */
  // Set the acceleration, maximum steps and speed per second ^2 (steps per second squared) ^2 means x²
  // the acceleration is increasing at a rate of x steps per second, every second.
  
  stepM_1.setMaxSpeed(500);  // Set the maximum acceleration in steps per second^2:
  stepM_1.setAcceleration(50);  // Set the maximum steps per second:
  stepM_1.setSpeed(300); // Set the speed in steps per second

  stepM_2.setMaxSpeed(500);
  stepM_2.setAcceleration(100);
  stepM_2.setSpeed(300);
  
  //stepM_3.setMaxSpeed(500);
  //stepM_3.setAcceleration(50);
  //stepM_3.setSpeed(300);  
  
  /*--------------------------------------------------------------------------------------------------*/
  
  // Check IR sensors state:
  Serial.println("Check IR sensors state: LOW(object NOT detected) / HIGH (object detected) ");
  Serial.print("IR sens 1: ");
  Serial.println(digitalRead(IR_Sens_1));
  //Serial.print("IR sens 2: ");
  //Serial.println(digitalRead(IR_Sens_2));
  //Serial.print("IR sens 3: ");
  //Serial.println(digitalRead(IR_Sens_3));
  Serial.println("");
  
  // FOR TEST and emergency stop ONLY - 
  // wait for switch_Halt to be set OFF (OFF = HIGH signal) TTTTTTTTTTTTTTTTTTTTTTT */
  
  Serial.println("Check SWITCH_HALT state: ON/OFF");
  Serial.println("");
  while (digitalRead(switch_Halt) == LOW) {// if switch_Halt is ON (LOW signal)
    if (switch_Halt_loop == true) {//only one time serial print until switch_Halt is OFF
      Serial.println("switch_Halt is ON(LOW) - wait for switch_Halt to be set OFF(HIGH)");
      Serial.println("");
    } // END if 
    switch_Halt_loop = false;   
  } // END while
  Serial.println("void setup continue");
  Serial.println("");
  
  // delay(delay_1); // delay for system to be ready - TEST ONLY
  /* END TEST and emergency stop ONLY TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT*/
  

  // move all steppermotors to OUT position, if not there already
  Serial.println("move all steppermotors to IR_Sens position if not there already");
  Serial.println("");
  
  moveToIRsens(stepM_1, stepM_1_IRsensPos, IR_Sens_1, 1);
  delay(delay_1);
  
  //moveToIRsens(stepM_2, stepM_2_IRsensPos, IR_Sens_2, 2);
  //delay(delay_1);
  
  //moveToIRsens(stepM_3, stepM_1_IRsensPos, IR_Sens_3, 3);
  //delay(delay_1);
  

  Serial.println("all steppermotors at OUT position");
  Serial.println("");

  Serial.println("void setup end");
  Serial.println("");
} // END void setup /************************************************************************/

void loop() { /******************************************************************************/
  // SerialPrint only one time for testing
  if (serialPrint_loop == true) { 
    Serial.println("");
    Serial.println("void loop start");
    Serial.println("");
    Serial.println("Press pushbutton: 1, 2, or 3 ");
    Serial.println("");   
  } // END if - SerialPrint  

  // TEST MOVE TO STOPPOS
  
  if (digitalRead(pushButton_1) == LOW) {
    Serial.println("pushbutton 1 pressed");
    Serial.println("");
    moveStopPos(stepM_1, stepM_1_StopPos, 1);
    delay(delay_1);    
    
    moveToIRsens(stepM_1, stepM_1_IRsensPos, IR_Sens_1, 1);
    delay(delay_1);    
  } // END if
  
  /*
  if (digitalRead(pushButton_2) == LOW) {
    Serial.println("pushbutton 2 pressed");
    Serial.println("");
    moveStopPos(stepM_2, stepM_2_StopPos, 2);
    delay(delay_1);

    moveToIRsens(stepM_2, stepM_2_IRsensPos, IR_Sens_2, 2);
    delay(delay_1);    
  } // END if
  */
  
  /*
  if (digitalRead(pushButton_3) == LOW) {
    Serial.println("pushbutton 3 pressed");
    Serial.println("");
    moveStopPos(stepM_3, stepM_3_StopPos, 3);
    delay(delay_2);

    moveToIRsens(stepM_3, stepM_3_IRsensPos, IR_Sens_3, 3);
    delay(delay_2);
  } // END if
  */

  
  // SerialPrint only one time for testing
  if (serialPrint_loop == true) { 
    Serial.println("void loop end");
    Serial.println(""); // one time serial print only for first void loop run
  } // END if - SerialPrint

  serialPrint_loop = false; // set serialPrint_loop to false for only one time serialPrint

} // END void loop /*************************************************************************/