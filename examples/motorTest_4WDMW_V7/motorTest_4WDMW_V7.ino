/*
  程序名称：MotorDriver 库 V7 综合测试
  适用硬件：YFROBOT 4WD 麦轮小车 V7
  程序说明：
  1. 本程序演示 YF_4WDMW_V7 型号的电机与舵机基础控制方法。
  2. 程序会依次测试 S1~S4 舵机、单个电机、四电机组合与全部电机同步运行。
  3. 第一次测试时请先把小车架空，防止轮子突然启动。
  作者：yfrobot
*/
#define SerialDebug 1          // 串口打印调试：0 关闭，1 打开
#include <MotorDriver.h>

#define MOTORTYPE YF_4WDMW_V7  // 正式 V7 小车型号

// 小车行驶方向设置：
// 1  表示当前方向就是“前进”
// -1 表示整体反向，常用于整车装配方向不同的情况
int directionConfig = 1;

// 创建一个电机驱动对象，后面所有动作都通过它来完成。
MotorDriver MD_4WDMW = MotorDriver(MOTORTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("4WD Mecanum Robot V7 library demo start.");
  MD_4WDMW.begin();
  MD_4WDMW.motorConfig(directionConfig);  // 配置整车前进方向
  MD_4WDMW.setPWMFreq(50);                // 使用舵机时，需要把 PCA9685 调到约 50Hz
  delay(1000);                            // 给硬件一点启动时间
  Serial.println("Start test...");
}

void loop() {
  // 先让 4 个舵机一起从 0 度转到 180 度，确认 S1~S4 通道正常。
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

  // 单电机测试：依次确认 M1、M2、M3、M4 四个电机是否都能独立转动。
  MD_4WDMW.setSingleMotor(M1, 4096);  // 电机 M1 全速正转
  delay(500);
  MD_4WDMW.setSingleMotor(M1, 0);  // 电机 M1 停止
  delay(500);
  MD_4WDMW.setSingleMotor(M2, -2048);  // 电机 M2 约 50% 速度反转
  delay(500);
  MD_4WDMW.setSingleMotor(M2, 0);  // 电机 M2 停止
  delay(500);
  MD_4WDMW.setSingleMotor(M3, 4096);  // 电机 M3 全速正转
  delay(500);
  MD_4WDMW.setSingleMotor(M3, 0);  // 电机 M3 停止
  delay(500);
  MD_4WDMW.setSingleMotor(M4, -2048);  // 电机 M4 约 50% 速度反转
  delay(500);
  MD_4WDMW.setSingleMotor(M4, 0);  // 电机 M4 停止
  delay(1000);

  // 四电机独立组合测试：四个轮子分别设置成不同速度。
  MD_4WDMW.setMotor(0, 4096, 2048, 1024);
  delay(500);
  MD_4WDMW.setMotor(0, 0, 0, 0);
  delay(500);
  MD_4WDMW.setMotor(0, -1024, -2048, -4096);
  delay(500);
  MD_4WDMW.setMotor(0, 0, 0, 0);
  delay(1000);

  // 全部电机同时前进、停止、后退。
  MD_4WDMW.setAllMotor(4096);
  delay(500);
  MD_4WDMW.setAllMotor(0);
  delay(500);
  MD_4WDMW.setAllMotor(-4096);
  delay(500);
  MD_4WDMW.setAllMotor(0);
  delay(1000);

  // 刹车测试：先让四轮一起转动，再单独刹住 M1，最后刹住全部电机。
  MD_4WDMW.setMotor(4096, 4096, 4096, 4096);
  delay(500);
  MD_4WDMW.stopMotor(M1);
  delay(500);
  MD_4WDMW.setMotor(-4096, -4096, -4096, -4096);
  delay(500);
  MD_4WDMW.stopMotor(MAll);
  delay(1500);
}
