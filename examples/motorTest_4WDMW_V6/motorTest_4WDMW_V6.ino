/***************************************************
  Motor Test - IIC Motor Drive (RZ7889 x 4)
  Servo Control

  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library
  motor driver iic Introduction: https://pjfcckenlt.feishu.cn/wiki/LOE7wIExZiUhQkkxK1BcQwfZntg
  motor driver iic：https://yfrobot.taobao.com

  YFROBOT ZL
  01/23/2026
 ****************************************************/
#include <MotorDriver.h>

#define MOTORTYPE YF_4WDMW_V6   // 4路电机
#define SerialDebug 1  // 串口打印调试 0-否 1-是

// 小车行驶方向设置
// 1: 正常前进方向（默认）
// -1: 反方向作为前进方向
int directionConfig = 1;

// Initializing motors.
MotorDriver MD_4WDMW = MotorDriver(MOTORTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  MD_4WDMW.begin();
  // 配置小车方向，默认1 正方向，-1为反方向
  MD_4WDMW.motorConfig(directionConfig);
  MD_4WDMW.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  delay(1000);   // wait 1s
  Serial.println("Start...");
}

void loop() {

  MD_4WDMW.servoWrite(S1, 0);
  MD_4WDMW.servoWrite(S2, 0);
  MD_4WDMW.servoWrite(S3, 0);
  MD_4WDMW.servoWrite(S4, 0);
  delay(1500);
  MD_4WDMW.servoWrite(S1, 180);
  MD_4WDMW.servoWrite(S2, 180);
  MD_4WDMW.servoWrite(S3, 180);
  MD_4WDMW.servoWrite(S4, 180);
  delay(1500);

  MD_4WDMW.setSingleMotor(M1, 4096);  // 电机M1全速正转
  delay(500);
  MD_4WDMW.setSingleMotor(M1, 0);  // 电机M1自由停止
  delay(500);
  MD_4WDMW.setSingleMotor(M2, -2048);  // 电机M2 50%速度反转
  delay(500);
  MD_4WDMW.setSingleMotor(M2, 0);  // 电机M2自由停止
  delay(500);
  MD_4WDMW.setSingleMotor(M3, 4096);  // 电机M3全速正转
  delay(500);
  MD_4WDMW.setSingleMotor(M3, 0);  // 电机M3自由停止
  delay(500);
  MD_4WDMW.setSingleMotor(M4, -2048);  // 电机M4 50%速度反转
  delay(500);
  MD_4WDMW.setSingleMotor(M4, 0);  // 电机M4自由停止
  delay(1000);

  MD_4WDMW.setMotor(0, 4096, 2048, 1024); // 电机M1自由停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转
  delay(500);
  MD_4WDMW.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4自由停止
  delay(500);
  MD_4WDMW.setMotor(0, -1024, -2048, -4096);  // 电机M1自由停止,电机M2 25%反转,电机M3 50%反转,电机M4 全速反转,
  delay(500);
  MD_4WDMW.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4自由停止
  delay(1000);

  MD_4WDMW.setAllMotor(4096);   // 电机M1/M2/M3/M4 全速正转
  delay(500);
  MD_4WDMW.setAllMotor(0);  // 电机M1/M2/M3/M4 自由停止
  delay(500);
  MD_4WDMW.setAllMotor(-4096);  // 电机M1/M2/M3/M4 全速反转
  delay(500);
  MD_4WDMW.setAllMotor(0);  // 电机M1/M2/M3/M4 自由停止
  delay(1000);

  MD_4WDMW.setMotor(4096, 4096, 4096, 4096);       // 电机M1/M2/M3/M4 全速正转
  delay(500);
  MD_4WDMW.stopMotor(M1);    // 电机M1 刹车
  delay(500);
  MD_4WDMW.setMotor(-4096, -4096, -4096, -4096);   // 电机M1/M2/M3/M4 全速反转
  delay(500);
  MD_4WDMW.stopMotor(MAll);  // 电机M1/M2/M3/M4 刹车
  delay(1500);
}
