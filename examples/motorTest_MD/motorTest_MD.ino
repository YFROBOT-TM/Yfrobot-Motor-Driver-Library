/***************************************************
  Motor Test - MD Motor Drive

  MD_01 , MD_02 , MD_03 , MD_04 , MD_GB36

  YFROBOT ZL
  08/13/2020
 ****************************************************/

#include <MotorDriver_MD.h>

#define M1DIR 4
#define M1PWM 5
#define M2DIR 7
#define M2PWM 6
#define MSLP 8  // or #define MSLP -1 slp connect to the high
#define M1CS A0
#define M2CS A1

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offset1 = 1;
const int offset2 = 1;

// Initializing motors. 
// MotorDriver_MD motor1 = MotorDriver_MD(M1DIR, M1PWM, 1, MSLP);
// MotorDriver_MD motor2 = MotorDriver_MD(M2DIR, M2PWM, 1, MSLP);
MotorDriver_MD motor1 = MotorDriver_MD(M1DIR, M1PWM, offset1, M1CS, MSLP);
MotorDriver_MD motor2 = MotorDriver_MD(M2DIR, M2PWM, offset2, M2CS, MSLP);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
}

void loop() {
  motor1.setMotor(255);  // 电机M1全速正转
  motor2.setMotor(255);  // 电机M2全速正转
  delay(500);
  motor1.setMotor(0);  // 电机M1停止
  motor2.setMotor(0);  // 电机M2停止
  delay(500);
  motor1.setMotor(-127);  // 电机M1 50%速度反转
  motor2.setMotor(-127);  // 电机M1 50%速度反转
  delay(500);
  motor1.setMotor(0);  // 电机M1停止
  motor2.setMotor(0);  // 电机M2停止
  delay(500);

  motor1.setMotor(255);  // 电机M1全速正转
  motor2.setMotor(255);  // 电机M2全速正转
  delay(500);
  motor1.getMotorCurrent();  // 获取电流检测口的模拟值 - 输出电压与电流电流成正比（50mV/A)，电流为零时电压为0.05V
}
