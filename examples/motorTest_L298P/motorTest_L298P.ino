/***************************************************
  Motor Test - L298P Motor Drive

  MA DIR-D4 PWM-D5;
  MB DIR-D7 PWM-D7;
  
  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library

  YFROBOT ZL
  08/13/2020
 ****************************************************/
#include <MotorDriver.h>

#define MOTORTYPE YF_L298P   // YF_PMR3
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offseta = 1;
const int offsetb = 1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDriver.motorConfig(offseta, offsetb);
}

void loop() {
  motorDriver.setMotor(255, 255);  // 电机AB 全速正转
  delay(500);
  motorDriver.setMotor(0, 0);  // 电机AB停止
  delay(500);
  motorDriver.setMotor(-255, -255);  // 电机AB 全速反转
  delay(500);
  motorDriver.setMotor(0, 0);  // 电机AB停止
  delay(1000);
}
