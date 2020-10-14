/***************************************************
  Motor Test - 4WD mecanum wheel
  DRV8838 x 4

  YFROBOT ZL
  08/13/2020
 ****************************************************/

#include <MotorDriver.h>

#define MOTORTYPE YF_4WDMW

uint8_t SerialDebug = 1;

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetM1 = 1;
const int offsetM2 = 1;
const int offsetM3 = 1;
const int offsetM4 = 1;

// Initializing motors. 
MotorDriver motorDriver = MotorDriver(YF_4WDMW);

void setup() {
  Serial.begin(9600);
  Serial.println("4wd Motor Drive test!");
  motorDriver.motorConfig(offsetM1,offsetM2,offsetM3,offsetM4);
}

void loop() {
  motorDriver.setMotor(0, 4096, 2048, 1024); // 电机M1停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转
  delay(500);
  motorDriver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止
  delay(500);
  motorDriver.setMotor(0, -1024, -2048, -4096);  // 电机M1停止,电机M2 25%反转,电机M3 50%反转,电机M4 全速反转,
  delay(500);
  motorDriver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止
  delay(1000);

}
