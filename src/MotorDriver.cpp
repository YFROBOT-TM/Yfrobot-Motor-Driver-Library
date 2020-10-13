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
#include "MotorDriverPin.h"

/*!
 *  @brief  Constructor. Mainly sets up pins.
 */
MotorDriver::MotorDriver(uint8_t type) {
  _TYPE_MODULE = type;
  _OFFSETA = 1;
  _OFFSETB = 1;
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _MADIRPIN = YF_LP_ADIR_PIN;
    _MAPWMPIN = YF_LP_APWM_PIN;
    _MBDIRPIN = YF_LP_BDIR_PIN;
    _MBPWMPIN = YF_LP_BPWM_PIN;
    pinMode(_MADIRPIN, OUTPUT);
    pinMode(_MAPWMPIN, OUTPUT);
    pinMode(_MBDIRPIN, OUTPUT);
    pinMode(_MBPWMPIN, OUTPUT);
  }
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
 *  @brief Drive motor , 2 Motor
 *  @param speedA: M1 motor speed, range -255 ~ 255;
 *  @param speedB: M2 motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t speedA, int16_t speedB) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    if(SerialDebug == 1){
      Serial.println("Driver Motor - L298P/PMR3");
    }
    speedA = max(speedA, 255);
    speedA = min(-255, speedA);
    speedA = speedA * _OFFSETA;

    speedB = max(speedB, 255);
    speedB = min(-255, speedB);
    speedB = speedB * _OFFSETB;

    if (speedA > 0) {
      digitalWrite(_MADIRPIN, HIGH);
      analogWrite(_MAPWMPIN, speedA);
    } else {
      digitalWrite(_MADIRPIN, LOW);
      analogWrite(_MAPWMPIN, -speedA);
    }

    if (speedB > 0) {
      digitalWrite(_MBDIRPIN, HIGH);
      analogWrite(_MBPWMPIN, speedB);
    } else {
      digitalWrite(_MBDIRPIN, LOW);
      analogWrite(_MBPWMPIN, -speedB);
    }
  }
}