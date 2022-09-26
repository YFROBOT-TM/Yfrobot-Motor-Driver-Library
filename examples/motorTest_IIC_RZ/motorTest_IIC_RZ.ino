/***************************************************
  Motor Test - IIC Motor Drive (RZ7889 x 4)
  Servo Control

  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library
  motor driver iic Introduction: http://www.yfrobot.com.cn/wiki/index.php?title=MotorDriver_IIC
  motor driver iic：https://item.taobao.com/item.htm?id=626324653253

  YFROBOT ZL
  08/13/2020
 ****************************************************/
#include <MotorDriver.h>

#define MOTORTYPE YF_IIC_RZ   // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetm1 = 1;
const int offsetm2 = -1;
const int offsetm3 = 1;
const int offsetm4 = -1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  delay(1000);   // wait 2s
  Serial.println("Start...");
}

void loop() {

  motorDriver.servoWrite(S1, 0);
  motorDriver.servoWrite(S2, 0);
  motorDriver.servoWrite(S3, 0);
  motorDriver.servoWrite(S4, 0);
  motorDriver.servoWrite(S5, 0);
  delay(1500);
  motorDriver.servoWrite(S1, 180);
  motorDriver.servoWrite(S2, 180);
  motorDriver.servoWrite(S3, 180);
  motorDriver.servoWrite(S4, 180);
  motorDriver.servoWrite(S5, 180);
  delay(1500);

  motorDriver.setSingleMotor(M1, 4096);  // 电机M1全速正转
  delay(500);
  motorDriver.setSingleMotor(M1, 0);  // 电机M1自由停止
  delay(500);
  motorDriver.setSingleMotor(M2, -2048);  // 电机M2 50%速度反转
  delay(500);
  motorDriver.setSingleMotor(M2, 0);  // 电机M2自由停止
  delay(500);
  motorDriver.setSingleMotor(M3, 4096);  // 电机M3全速正转
  delay(500);
  motorDriver.setSingleMotor(M3, 0);  // 电机M3自由停止
  delay(500);
  motorDriver.setSingleMotor(M4, -2048);  // 电机M4 50%速度反转
  delay(500);
  motorDriver.setSingleMotor(M4, 0);  // 电机M4自由停止
  delay(1000);

  motorDriver.setMotor(0, 4096, 2048, 1024); // 电机M1自由停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转
  delay(500);
  motorDriver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4自由停止
  delay(500);
  motorDriver.setMotor(0, -1024, -2048, -4096);  // 电机M1自由停止,电机M2 25%反转,电机M3 50%反转,电机M4 全速反转,
  delay(500);
  motorDriver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4自由停止
  delay(1000);

  motorDriver.setAllMotor(4096);   // 电机M1/M2/M3/M4 全速正转
  delay(500);
  motorDriver.setAllMotor(0);  // 电机M1/M2/M3/M4 自由停止
  delay(500);
  motorDriver.setAllMotor(-4096);  // 电机M1/M2/M3/M4 全速反转
  delay(500);
  motorDriver.setAllMotor(0);  // 电机M1/M2/M3/M4 自由停止
  delay(1000);

  motorDriver.setMotor(4096, 4096, 4096, 4096);       // 电机M1/M2/M3/M4 全速正转
  delay(500);
  motorDriver.stopMotor(M1);    // 电机M1 刹车
  delay(500);
  motorDriver.setMotor(-4096, -4096, -4096, -4096);   // 电机M1/M2/M3/M4 全速反转
  delay(500);
  motorDriver.stopMotor(MAll);  // 电机M1/M2/M3/M4 刹车
  delay(1500);
}
