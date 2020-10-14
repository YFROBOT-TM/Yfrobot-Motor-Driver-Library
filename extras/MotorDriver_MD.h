/*
 *  @file MotorDriver_MD.h
 *
 *  This is a library for Motor driver.
 *
 *  Designed specifically to work with the Yfrobot Motor driver.
 *    MD01 MD02 MD03 MD04 MD_GB36
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef _MotorDriver_MD_H
#define _MotorDriver_MD_H

#include <Arduino.h>

/*!
 *  @brief  Class that stores state and functions for interacting with MD PWM chip
 */
class MotorDriver_MD {
 public:
  MotorDriver_MD(uint8_t _dir, uint8_t _pwm, int8_t _offset, int8_t _slp);
  MotorDriver_MD(uint8_t _dir, uint8_t _pwm, int8_t _offset, uint8_t _cs, int8_t _slp);

  void sleep();
  void wakeup();

  void setMotor(int16_t speed);   // 驱动电机
  // void stopMotor(int8_t motorNum);  // 刹车
  unsigned int getMotorCurrent();

 private:
  uint8_t _MDIR;        // 电机M1 方向
  uint8_t _MPWM;        // 电机M1 PWM
  uint8_t _MCS;         // 电机M1 电流检测引脚 CS
  uint8_t _MSLP;        // 电机 睡眠引脚

  int8_t _OFFSET;       // 电机方向设置

};

#endif