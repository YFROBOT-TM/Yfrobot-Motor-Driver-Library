/***************************************************
  Motor Test - IIC Motor Drive

  YFROBOT ZL
  08/13/2020
 ****************************************************/

#include <MotorDriver_PCA9685_8838.h>

// called this way, it uses the default address 0x40
MotorDriver_PCA9685_8838 motorDirver = MotorDriver_PCA9685_8838();

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDirver.begin();
  // motorDirver.setPWMFreq(1600);       // This is the maximum PWM frequency
  // motorDirver.setMotorDirReverse(0);  // 设置电机反向, 0-默认,1-反向.
  Wire.begin();  // join the TWI as the master
  delay(2000);   // wait 2s
  Serial.println("Start...");
}

void loop() {
  motorDirver.setSingleMotor(M1, 4096);  // 电机M1全速正转
  delay(500);
  motorDirver.setSingleMotor(M1, 0);  // 电机M1停止
  delay(500);
  motorDirver.setSingleMotor(M2, -2048);  // 电机M2 50%速度反转
  delay(500);
  motorDirver.setSingleMotor(M2, 0);  // 电机M2停止
  delay(500);
  motorDirver.setSingleMotor(M3, 4096);  // 电机M3全速正转
  delay(500);
  motorDirver.setSingleMotor(M3, 0);  // 电机M3停止
  delay(500);
  motorDirver.setSingleMotor(M4, -2048);  // 电机M4 50%速度反转
  delay(500);
  motorDirver.setSingleMotor(M4, 0);  // 电机M4停止
  delay(1000);

  motorDirver.setMotor(0, 4096, 2048,1024);  // 电机M1停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转
  delay(500);
  motorDirver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止
  delay(500);
  motorDirver.setMotor(0, -1024, -2048, -4096);  // 电机M1停止,电机M2 25%反转,电机M3 50%反转,电机M4 全速反转,
  delay(500);
  motorDirver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止
  delay(1000);

  motorDirver.setMotor(4096, 4096, 4096, 4096);  // 电机M1/M2/M3/M4 全速正转
  delay(500);
  motorDirver.stopMotor(M1);  // 电机M1 刹车
  delay(500);
  motorDirver.setMotor(-4096, -4096, -4096, -4096);  // 电机M1/M2/M3/M4 全速反转
  delay(500);
  motorDirver.stopMotor(MAll);  // 电机M1/M2/M3/M4 刹车
  delay(1500);
}
