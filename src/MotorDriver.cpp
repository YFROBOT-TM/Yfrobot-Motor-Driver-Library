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

#include "MotorDriver.h"
#include <Arduino.h>

/*!
 *  @brief  Constructor. Mainly sets up pins.
 */
MotorDriver::MotorDriver(uint8_t type) {
  _MADIR = 4;
  _MAPWM = 5;
  _MBDIR = 7;
  _MBPWM = 6;
  _OFFSETA = 1;
  _OFFSETB = 1;
  _TYPE_MODULE = type;
  pinMode(_MADIR, OUTPUT);
  pinMode(_MAPWM, OUTPUT);
  pinMode(_MBDIR, OUTPUT);
  pinMode(_MBPWM, OUTPUT);
}

/*!
 *  @brief  Motor Config. Forward or Reverse.
 *  @param offsetA: Value can be 1 or -1;
 *  @param offsetB: Value can be 1 or -1;
 */
void MotorDriver::motorConfig(int8_t offsetA = 1, int8_t offsetB = 1) {
  _OFFSETA = offsetA;
  _OFFSETB = offsetB;
}

/*!
 *  @brief Drive motor ,
 *  @param speedA: M1 motor speed, range -255 ~ 255;
 *  @param speedB: M2 motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t speedA, int16_t speedB) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    speedA = max(speedA, 255);
    speedA = min(-255, speedA);
    speedA = speedA * _OFFSETA;

    speedB = max(speedB, 255);
    speedB = min(-255, speedB);
    speedB = speedB * _OFFSETB;

    if (speedA > 0) {
      digitalWrite(_MADIR, HIGH);
      analogWrite(_MAPWM, speedA);
    } else {
      digitalWrite(_MADIR, LOW);
      analogWrite(_MAPWM, -speedA);
    }

    if (speedB > 0) {
      digitalWrite(_MBDIR, HIGH);
      analogWrite(_MBPWM, speedB);
    } else {
      digitalWrite(_MBDIR, LOW);
      analogWrite(_MBPWM, -speedB);
    }
  }
}
