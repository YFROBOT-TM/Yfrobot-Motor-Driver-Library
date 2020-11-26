/*
  Valon motor Control - Valon 控制电机
  
  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library
  Valon : https://item.taobao.com/item.htm?id=623336047525
  
  ----------------------------------------------------------------
  This example code is in the public domain.
  www.yfrobot.com
*/

#include <MotorDriver.h>

#define MOTORTYPE YF_VALON
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetL = 1;
const int offsetR = 1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Valon Motor Drive test!");
  motorDriver.motorConfig(offsetL,offsetR);
}

void loop() {
  // put your main code here, to run repeatedly:
  motorDriver.setMotor(100, 100);     //前进
  delay(1000);
  motorDriver.setMotor(-100, -100);   //后退
  delay(2000);
  motorDriver.setMotor(100, -100);    //右旋转
  delay(1000);
  motorDriver.setMotor(-100, 100);    //左旋转
  delay(2000);
}
