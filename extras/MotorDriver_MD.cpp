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

#include "MotorDriver_MD.h"
#include <Arduino.h>


//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param _slp: sleep pin;
 *  @param _dir: dir pin;
 *  @param _pwm: pwm pin;
 */
MotorDriver_MD::MotorDriver_MD(uint8_t _dir, uint8_t _pwm, int8_t _offset, int8_t _slp) {
  _MSLP = _slp;
  _MDIR = _dir;
  _MPWM = _pwm;
  _OFFSET = _offset;
  pinMode(_MDIR, OUTPUT);
  pinMode(_MPWM, OUTPUT);
  if(_MSLP >= 0) {
    pinMode(_MSLP, OUTPUT);
    digitalWrite(_MSLP, HIGH);
  }
}

/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param _slp: sleep pin;
 *  @param _dir: dir pin;
 *  @param _pwm: pwm pin;
 *  @param _cs: cs pin; 电流检测引脚
 */
MotorDriver_MD::MotorDriver_MD(uint8_t _dir, uint8_t _pwm, int8_t _offset, uint8_t _cs, int8_t _slp) {
  MotorDriver_MD( _dir, _pwm, _offset, _slp);
  _MCS = _cs;
  pinMode(_MCS,INPUT);
}

// /*!
//  *  @brief  Constructor. Mainly sets up pins.
//  *  @param _slp: sleep pin;
//  *  @param _dir1: M1 dir pin;
//  *  @param _pwm1: M1  pwm pin;
//  *  @param _dir2: M2 dir pin;
//  *  @param _pwm2: M2  pwm pin;
//  */
// MotorDriver_MD::MotorDriver_MD(uint8_t _slp, uint8_t _dir1, uint8_t _pwm1, uint8_t _dir2, uint8_t _pwm2){
//   _MSLP = _slp;
//   _M1DIR = _dir1;
//   _M1PWM = _pwm1;
//   _M2DIR = _dir2;
//   _M2PWM = _pwm2;
//   pinMode(_MSLP, OUTPUT);
//   pinMode(_M1DIR, OUTPUT);
//   pinMode(_M1PWM, OUTPUT);
//   pinMode(_M2DIR, OUTPUT);
//   pinMode(_M2PWM, OUTPUT);
//   digitalWrite(_MSLP, HIGH);
// }

// /*!
//  *  @brief  Constructor. Mainly sets up pins.
//  *  @param _slp: sleep pin;
//  *  @param _dir1: M1 dir pin;
//  *  @param _pwm1: M1  pwm pin;
//  *  @param _cs1: M1 cs pin; M1 电流检测引脚
//  *  @param _dir2: M2 dir pin;
//  *  @param _pwm2: M2  pwm pin;
//  *  @param _cs2: M2 cs pin; M2 电流检测引脚
//  */
// MotorDriver_MD::MotorDriver_MD(uint8_t _slp, uint8_t _dir1, uint8_t _pwm1, uint8_t _cs1, uint8_t _dir2, uint8_t _pwm2, uint8_t _cs2){
//   MotorDriver_MD(_slp, _dir1, _pwm1, _dir2, _pwm2);
//   _M1CS = _cs1;
//   _M2CS = _cs2;
//   pinMode(_M1CS,INPUT);
//   pinMode(_M2CS,INPUT);
// }

/*!
 *  @brief  Puts board into sleep mode
 */
void MotorDriver_MD::sleep() {
  if(_MSLP >= 0) {
    digitalWrite(_MSLP, LOW);
  }else{
    /* slp 引脚连接至高电平，没有睡眠模式 */
  }
}

/*!
 *  @brief  Wakes board from sleep
 */
void MotorDriver_MD::wakeup() {
  if(_MSLP >= 0) {
    digitalWrite(_MSLP, HIGH);
  }else{
    /* slp 引脚连接至高电平，没有睡眠模式 */
  }
}

/*!
 *  @brief Drive motor ,
 *  @param speed: M1 motor speed, range -255 ~ 255;
 */
void MotorDriver_MD::setMotor(int16_t speed) {
  speed = max(speed, 255);
  speed = min(-255, speed);
  speed = speed * _OFFSET;
  if(speed < 0){
    digitalWrite(_MDIR, LOW);
    analogWrite(_MPWM, -speed);
  }else{
    digitalWrite(_MDIR, HIGH);
    analogWrite(_MPWM, speed);
  }
}

/*!
 *  @brief stop motor , Emergency stop / brake 紧急停止/刹车
 */
// void MotorDriver_MD::stopMotor() {
// }

unsigned int MotorDriver_MD::getMotorCurrent() {  //获取电流检测口的模拟值
  return analogRead(_MCS);
}

/***************** private ******************/

