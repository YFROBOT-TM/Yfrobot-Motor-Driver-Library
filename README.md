# Yfrobot-Motor-Driver-Library
YFROBOT Motor Driver Library for Arduino
可用模块：
L298P / PM-R3 / MD01 / MD02 / MD03 / MD04 / MD_GB36 / IIC_MOTORDRIVER 
小车套件：
VALON / 4WD Mecanum Wheel 

调用库：

`#include <MotorDriver.h>   // 包含头文件`

#### MotorDriver_IIC 使用说明：

适用对象：[Motor Driver iic](https://item.taobao.com/item.htm?id=626324653253)、[Motor Driver iic模块简介](http://www.yfrobot.com.cn/wiki/index.php?title=MotorDriver_IIC)

创建对象：

`MotorDriver motorDirver = MotorDriver(YF_IIC_TB);`

Methods：

初始化

`motorDirver.begin();`

设置电机方向，参数：1-默认，-1-反向 (可选，不使用该函数，电机方向默认)

`motorDirver.motorConfig(1, -1, 1, -1);// M1/M3 默认方向 M2/M4反向`

设置所有电机反向， 参数：1-默认，-1-反向

`motorDirver.motorConfig(1); // 所有电机默认方向`

驱动单个电机，参数：电机序号 M1,M2,M3,M4；电机速度 -4096 ~ 4096

`motorDirver.setSingleMotor(M1, 4096);   // M1电机全速正转`

`motorDirver.setSingleMotor(M1, 0);   // M1电机停止`

驱动4路电机，参数：电机速度 -4096 ~ 4096

`motorDirver.setMotor(0, 4096, 2048, 1024);  // 电机M1停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转`

`motorDirver.setMotor(0, 0, 0, 0);  // 电机M1/M2/M3/M4停止`

`motorDirver.setAllMotor(4096); // 电机M1/M2/M3/M4 全速正转`

`motorDirver.setAllMotor(0);  // 电机M1/M2/M3/M4停止`

电机刹车/急停

`motorDirver.stopMotor(M1);  // 电机M1 刹车`

`motorDirver.stopMotor(MAll);  // 电机M1/M2/M3/M4 刹车`

#### MotorDriver_MD 使用说明：

适用对象：[YF_MD01](https://item.taobao.com/item.htm?id=537440531147) / [YF_MD02](https://item.taobao.com/item.htm?id=540604466130) / [YF_MD03](https://item.taobao.com/item.htm?id=541284819405) / [YF_MD04](https://item.taobao.com/item.htm?id=573547230434) / [YF_MDGP36](https://item.taobao.com/item.htm?id=626907019504)

创建对象：

`MotorDriver motora = MotorDriver(YF_MD, _dirPin, _pwmPin); // 设置电机引脚`
`//MotorDriver motora = MotorDriver(YF_MD, _dirPin, _pwmPin, _offset); // 设置电机引脚、方向`
`//MotorDriver motora = MotorDriver(YF_MD, _dirPin, _pwmPin, _offset, _slpPin); // 设置电机引脚、方向、使能引脚`
`//MotorDriver motora = MotorDriver(YF_MD, _dirPin, _pwmPin, _offset, _slpPin, _csPin); // 设置电机引脚、方向、使能引脚、电流检测引脚`

Methods：

驱动电机，参数：电机速度 -255 ~ 255

`motora.setMotor(255);  // A电机全速正转`

`motora.setMotor(0);  // A电机停止`

睡眠/唤醒，参数：无（需要配置使能引脚 _slpPin）

`motora.sleep();  // 模块睡眠，低功耗`

`motora.wakeup();  // 模块唤醒使能`

电流检测，

`motora.getMotorCurrent();  // 获取电流检测口的模拟值 - 输出电压与电流电流成正比（50mV/A)，电流为零时电压为0.05V`

#### [MotorDriver_L298P](https://item.taobao.com/item.htm?id=20695931042) 使用说明：

适用对象：[L298P](https://item.taobao.com/item.htm?id=20695931042)

创建对象：

`MotorDriver motorDirver = MotorDriver(YF_L298P);`

Methods：

设置电机方向，参数：1-默认，-1-反向 (可选，不使用该函数，电机方向默认)

`motorDirver.motorConfig(1, 1);// A B电机默认方向`

`motorDirver.motorConfig(1); // A B电机默认方向`

驱动电机，参数：电机速度 -255 ~ 255

`motorDirver.setMotor(255, 128);  // A电机全速正转,B电机50%速度正转`

`motorDirver.setMotor(0, 0);  // AB电机停止`

#### MotorDriver_PMR3 使用说明：

适用对象：[PM-R3](https://item.taobao.com/item.htm?id=38073351291)

创建对象：

`MotorDriver motorDirver = MotorDriver(YF_PMR3);`

Methods：

设置电机方向，参数：1-默认，-1-反向 (可选，不使用该函数，电机方向默认)

`motorDirver.motorConfig(1, 1);// A B电机默认方向`

`motorDirver.motorConfig(1); // A B电机默认方向`

驱动电机，参数：电机速度 -255 ~ 255

`motorDirver.setMotor(255, 128);  // A电机全速正转,B电机50%速度正转`

`motorDirver.setMotor(0, 0);  // AB电机停止`

#### Valon-I 使用说明：

适用对象：[VALON-I](https://item.taobao.com/item.htm?id=623336047525)

创建对象：

`MotorDriver motorDirver = MotorDriver(YF_VALON);`

Methods：

设置电机方向，参数：1-默认，-1-反向 (可选，不使用该函数，电机方向默认)

`motorDirver.motorConfig(1, 1);// 左右电机默认方向`

`motorDirver.motorConfig(1); // 所有电机默认方向`

驱动电机，参数：电机速度 -255 ~ 255

`motorDirver.setMotor(255, 128);  // 左电机全速正转,右电机 50%速度正转`

`motorDirver.setMotor(0, 0);  // 左右电机停止`

#### 4WD 麦轮小车（mecanum wheel） 使用说明：

适用对象：[4WD麦轮小车](https://yfanmcu.taobao.com)

创建对象：

`MotorDriver motorDirver = MotorDriver(YF_4WDMW);`

Methods：

设置电机方向，参数：1-默认，-1-反向 (可选，不使用该函数，电机方向默认)

`motorDirver.motorConfig(1, 1, 1, 1); // 4电机默认方向`

`motorDirver.motorConfig(1); // 所有电机默认方向`

驱动电机，参数：电机速度 -255 ~ 255

`motorDriver.setMotor(0, 255, 128, 64); // 电机M1停止,电机M2 全速正转,电机M3 50%正转,电机M4 25%正转`

`motorDirver.setMotor(0, 0, 0, 0); // 电机M1/M2/M3/M4停止`

驱动单电机，参数：电机速度 -255 ~ 255

`motorDriver.setSingleMotor(M1, 255); // 电机M1全速正转`

`motorDriver.setSingleMotor(M2, 255); // 电机M2全速正转`

`motorDriver.setSingleMotor(M3, 255); // 电机M3全速正转`

`motorDriver.setSingleMotor(M4, 255); // 电机M4全速正转`

驱动所有电机，参数：电机速度 -255 ~ 255

`motorDriver.setAllMotor(128); // 电机M1/M2/M3/M4 50%反转`