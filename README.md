# Yfrobot-Motor-Driver-Library
YFROBOT Motor Driver Library for Arduino

功能函数说明：

// 驱动单个电机，参数:电机序号；电机速度
void setSingleMotor(int8_t motorNum, int16_t speedM2);  

// 驱动4路电机，参数：电机速度（范围-4096 ~ 4096）
void setMotor(int16_t speedM1,int16_t speedM2,int16_t speedM3,int16_t speedM4);

// 设置单个电机反向，参数：0-默认，1-反向
void setMotorDirReverse(bool m1Dir, bool m2Dir, bool m3Dir, bool m4Dir);

// 设置所有电机反向， 0-默认，1-反向
void setMotorDirReverse(bool MAllDir);
