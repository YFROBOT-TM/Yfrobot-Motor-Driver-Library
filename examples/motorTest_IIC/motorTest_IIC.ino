/***************************************************
  Motor Test - IIC Motor Drive

  YFROBOT ZL
  08/13/2020
 ****************************************************/

#include <MotorDriver_PCA9685.h>

// called this way, it uses the default address 0x40
MotorDriver_PCA9685 motorDirver = MotorDriver_PCA9685();

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDirver.begin();
  //motorDirver.setPWMFreq(1600);       // This is the maximum PWM frequency
  //motorDirver.setMotorDirReverse(0);  // 设置电机反向, 0-默认,1-反向.
}

void loop() {

  motorDirver.setSingleMotor(1, 4096); // 电机M1全速正转
  delay(1000);
  motorDirver.setSingleMotor(1, 0); // 电机M1停止
  delay(1000);
  motorDirver.setSingleMotor(1, -2048); // 电机M1 50%速度反转
  delay(1000);
  motorDirver.setSingleMotor(1, 0); // 电机M1停止
  delay(2000);

  motorDirver.setMotor(4096, 0, 0, 0); // 电机M1全速正转
  delay(1000);
  motorDirver.setMotor(0, 0, 0, 0); // 电机M1停止
  delay(500);
  motorDirver.setMotor(-4096, 0, 0, 0); // 电机M1全速反转
  delay(1000);
  motorDirver.setMotor(0, 0, 0, 0); // 电机M1停止
  delay(2000);

}
