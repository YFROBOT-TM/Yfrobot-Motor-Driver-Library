/***************************************************
  Motor Test - PM-R3 Motor Drive

  MA DIR-D4 PWM-D5;
  MB DIR-D7 PWM-D7;

  YFROBOT ZL
  08/13/2020
 ****************************************************/

#include <MotorDriver_PMR3.h>

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offseta = 1;
const int offsetb = 1;

// Initializing motors. 
MotorDriver_PMR3 motorDirver = MotorDriver_PMR3();
// MotorDriver_PMR3 motorDirver = MotorDriver_PMR3(offseta, offsetb);


void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
}

void loop() {
  motorDirver.setMotor(255, 255);  // 电机AB 全速正转
  delay(500);
  motorDirver.setMotor(0, 0);  // 电机AB停止
  delay(500);
  motorDirver.setMotor(-255, -255);  // 电机AB 全速反转
  delay(500);
  motorDirver.setMotor(0, 0);  // 电机AB停止
  delay(1000);
}
