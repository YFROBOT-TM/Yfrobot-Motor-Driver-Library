/*
 *  @file MotorDriver.h
 *
 *  This is a library for Motor driver.
 *
 *  Designed specifically to work with the Yfrobot Motor driver.
 *    L298P
 *
 *  BSD license, all text above must be included in any redistribution
 */

// #ifndef _MotorDriver_H
// #define _MotorDriver_H

#include <Arduino.h>

/*!
 *  @brief  Class that stores state and functions for interacting with motor chip
 */
class MotorDriver {
 public:
//  #ifdef YF_ARDUINO_SHIELD_L298P || YF_ARDUINO_SHIELD_PMR3
  MotorDriver();
  MotorDriver(int8_t offsetA, int8_t offsetB);
  void setMotor(int16_t speedA, int16_t speedB);   // 驱动电机
// #endif

 private:
  uint8_t _MADIR;        // 电机MA 方向
  uint8_t _MAPWM;        // 电机MA PWM
  uint8_t _MBDIR;        // 电机MB 方向
  uint8_t _MBPWM;        // 电机MB PWM

  int8_t _OFFSETA;
  int8_t _OFFSETB;
};

// #endif