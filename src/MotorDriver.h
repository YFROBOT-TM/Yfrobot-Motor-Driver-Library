/*
 *  @file MotorDriver.h
 *
 *  This is a library for Motor driver.
 *
 *  Designed specifically to work with the Yfrobot Motor driver.
 *    L298P PM-R3(tb6612) 
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef _MotorDriver_H
#define _MotorDriver_H

#include <Arduino.h>

/* Define types of motor driver module. */
#define YF_L298P  1   // yfrobot L298P module
#define YF_PMR3   2   // yfrobot PM-R3 module

/*!
 *  @brief  Class that stores state and functions for interacting with motor chip
 */
class MotorDriver {
 public:
  MotorDriver(uint8_t type);
  void motorConfig(int8_t offsetA, int8_t offsetB); // Motor direction configuration - 电机方向配置
  void setMotor(int16_t speedA, int16_t speedB);    // Motor drive - 驱动电机

 private:
  uint8_t _MADIR;        // 电机MA 方向
  uint8_t _MAPWM;        // 电机MA PWM
  uint8_t _MBDIR;        // 电机MB 方向
  uint8_t _MBPWM;        // 电机MB PWM

  int8_t _OFFSETA;
  int8_t _OFFSETB;
  
  uint8_t _TYPE_MODULE;  // 模块类型
};

#endif
