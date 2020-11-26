/***************************************************
  Motor Test - MD Motor Drive

  MD_01 , MD_02 , MD_03 , MD_04 , MD_GB36
  
  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library

  YFROBOT ZL
  10/14/2020
 ****************************************************/

#include <MotorDriver.h>

#define MOTORTYPE YF_MD

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offseta = 1;
const int offsetb = 1;

// Initializing motors. 
// MotorDriver motora = MotorDriver(MOTORTYPE, YF_MD04_ADIR_PIN, YF_MD04_APWM_PIN, offseta, YF_MD04_ASLP_PIN);
// MotorDriver motorb = MotorDriver(MOTORTYPE, YF_MD04_BDIR_PIN, YF_MD04_BPWM_PIN, offseta, YF_MD04_BSLP_PIN);
MotorDriver motora = MotorDriver(MOTORTYPE, YF_MD04_ADIR_PIN, YF_MD04_APWM_PIN, offseta, YF_MD04_ACS_PIN, YF_MD04_ASLP_PIN);
MotorDriver motorb = MotorDriver(MOTORTYPE, YF_MD04_BDIR_PIN, YF_MD04_BPWM_PIN, offsetb, YF_MD04_BCS_PIN, YF_MD04_BSLP_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
}

void loop() {
  motora.setMotor(255);  // 电机M1全速正转
  motorb.setMotor(255);  // 电机M2全速正转
  delay(500);
  motora.setMotor(0);  // 电机M1停止
  motorb.setMotor(0);  // 电机M2停止
  delay(500);
  motora.setMotor(-127);  // 电机M1 50%速度反转
  motorb.setMotor(-127);  // 电机M1 50%速度反转
  delay(500);
  motora.setMotor(0);  // 电机M1停止
  motorb.setMotor(0);  // 电机M2停止
  delay(500);

  motora.setMotor(255);  // 电机M1全速正转
  motorb.setMotor(255);  // 电机M2全速正转
  delay(500);
  motora.getMotorCurrent();  // 获取电流检测口的模拟值 - 输出电压与电流电流成正比（50mV/A)，电流为零时电压为0.05V
}