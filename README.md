# Yfrobot-Motor-Driver-Library
YFROBOT Motor Driver Library for Arduino

MotorDriver_IIC 使用说明：

调用库：

`#include <MotorDriver_PCA9685.h>   // 包含头文件`

创建对象：

`MotorDriver_PCA9685 motorDirver = MotorDriver_PCA9685();`

Methods：

初始化：

`motorDirver.begin();`

设置单个电机反向，参数：0-默认，1-反向

`motorDirver.setMotorDirReverse(0, 1, 0, 1);// M1/M3 默认方向 M2/M4反向`

设置所有电机反向， 0-默认，1-反向

`motorDirver.setMotorDirReverse(0); // 所有电机默认方向`

驱动单个电机，参数：电机序号 M1,M2,M3,M4；电机速度 -4096 ~ 4096

`motorDirver.setSingleMotor(M1, 4096);   // M1电机全速正转`

`motorDirver.setSingleMotor(M1, 0);   // M1电机停止`

驱动4路电机，参数：电机速度 -4096 ~ 4096

`motorDirver.setMotor(0, 4096, 2048,1024);  // 电机M1停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转`

`motorDirver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止`

电机刹车/急停

`motorDirver.stopMotor(M1);  // 电机M1 刹车`

`motorDirver.stopMotor(MAll);  // 电机M1/M2/M3/M4 刹车`

