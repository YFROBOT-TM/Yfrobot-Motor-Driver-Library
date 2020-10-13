/*
  Valon motor Control - Valon 控制电机
  ----------------------------------------------------------------
  This example code is in the public domain.
  www.yfrobot.com
*/

#include <MotorDriver.h>

#define MOTORTYPE YF_VALON   //
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
