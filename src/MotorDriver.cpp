/*
 *  @file MotorDriver.h
 *
 *  This is a library for Motor driver.
 *
 *  Designed specifically to work with the Yfrobot Motor driver.
 *    L298P PM-R3(tb6612) MD(MD_01 , MD_02 , MD_03 , MD_04 , MD_GB36)
 *    IIC_TB(PCA9685 TB6612) valon(DRV8838X2) 4WDMW(DRV8838X4)
 *
 *  BSD license, all text above must be included in any redistribution
 *  
 *  V0.0.5 IIC driver initialize pin
 */

#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "MotorDriverPin.h"

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Constructor. Mainly sets up pins.
 */
MotorDriver::MotorDriver(uint8_t type) {
  _TYPE_MODULE = type;
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = DIRP;
    _OFFSETB = DIRP;
    _MADIRPIN = YF_LP_ADIR_PIN;
    _MAPWMPIN = YF_LP_APWM_PIN;
    _MBDIRPIN = YF_LP_BDIR_PIN;
    _MBPWMPIN = YF_LP_BPWM_PIN;
    pinMode(_MADIRPIN, OUTPUT);
    pinMode(_MAPWMPIN, OUTPUT);
    pinMode(_MBDIRPIN, OUTPUT);
    pinMode(_MBPWMPIN, OUTPUT);
  } else if (_TYPE_MODULE == YF_IIC_TB) {
    setAddress(PCA9685_I2C_ADDRESS);
    _OFFSETM1 = DIRP;
    _OFFSETM2 = DIRP;
    _OFFSETM3 = DIRP;
    _OFFSETM4 = DIRP;
    _M1IN1 = YF_PCA9685_CH0;   // 电机M1 输入1
    _M1IN2 = YF_PCA9685_CH1;   // 电机M1 输入2
    _M1PWM = YF_PCA9685_CH2;   // 电机M1 PWM
    _M2IN1 = YF_PCA9685_CH3;   // 电机M2 输入1
    _M2IN2 = YF_PCA9685_CH4;   // 电机M2 输入2
    _M2PWM = YF_PCA9685_CH5;   // 电机M2 PWM
    _M3IN1 = YF_PCA9685_CH8;   // 电机M3 输入1
    _M3IN2 = YF_PCA9685_CH9;   // 电机M3 输入2
    _M3PWM = YF_PCA9685_CH10;  // 电机M3 PWM
    _M4IN1 = YF_PCA9685_CH11;  // 电机M4 输入1
    _M4IN2 = YF_PCA9685_CH12;  // 电机M4 输入2
    _M4PWM = YF_PCA9685_CH13;  // 电机M4 PWM
  } else if (_TYPE_MODULE == YF_VALON) {
    _OFFSETA = DIRN;
    _OFFSETB = DIRN;
    _MADIRPIN = YF_VALON_LDIR_PIN;
    _MAPWMPIN = YF_VALON_LPWM_PIN;
    _MBDIRPIN = YF_VALON_RDIR_PIN;
    _MBPWMPIN = YF_VALON_RPWM_PIN;
    pinMode(_MADIRPIN, OUTPUT);
    pinMode(_MAPWMPIN, OUTPUT);
    pinMode(_MBDIRPIN, OUTPUT);
    pinMode(_MBPWMPIN, OUTPUT);
  } else if (_TYPE_MODULE == YF_4WDMW) {
    _OFFSETM1 = DIRN;
    _OFFSETM2 = DIRP;
    _OFFSETM3 = DIRN;
    _OFFSETM4 = DIRP;
    pinMode(YF_4WDMW_M1DIR_PIN, OUTPUT);
    pinMode(YF_4WDMW_M1PWM_PIN, OUTPUT);
    pinMode(YF_4WDMW_M2DIR_PIN, OUTPUT);
    pinMode(YF_4WDMW_M2PWM_PIN, OUTPUT);
    pinMode(YF_4WDMW_M3DIR_PIN, OUTPUT);
    pinMode(YF_4WDMW_M3PWM_PIN, OUTPUT);
    pinMode(YF_4WDMW_M4DIR_PIN, OUTPUT);
    pinMode(YF_4WDMW_M4PWM_PIN, OUTPUT);
  }
}

/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param type: moudle type
 *  @param _dirPin: dir pin;
 *  @param _pwmPin: pwm pin;
 */
MotorDriver::MotorDriver(uint8_t type, uint8_t _dirPin, uint8_t _pwmPin) {
  _TYPE_MODULE = type;
  if (_TYPE_MODULE == YF_MD) {
    _OFFSET = DIRP;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
  }
}
/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param type: moudle type
 *  @param _dirPin: dir pin;
 *  @param _pwmPin: pwm pin;
 */
MotorDriver::MotorDriver(uint8_t type, uint8_t _dirPin, uint8_t _pwmPin,
                         int8_t _offset) {
  _TYPE_MODULE = type;
  _offset = _offset >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD) {
    _OFFSET = _offset;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    _MSLP_PIN = -1;
    _MCS_PIN = -1;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
  }
}
/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param type: moudle type
 *  @param _dirPin: dir pin;
 *  @param _pwmPin: pwm pin;
 *  @param _slpPin: sleep pin;
 */
MotorDriver::MotorDriver(uint8_t type, uint8_t _dirPin, uint8_t _pwmPin,
                         int8_t _offset, uint8_t _slpPin) {
  _TYPE_MODULE = type;
  _offset = _offset >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD) {
    _OFFSET = _offset;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    _MSLP_PIN = _slpPin;
    _MCS_PIN = -1;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
    if(_MSLP_PIN != -1){
      pinMode(_MSLP_PIN, OUTPUT);
      digitalWrite(_MSLP_PIN, HIGH);  // 默认使能
    }
  }
}
/*!
 *  @brief  Constructor. Mainly sets up pins.
 *  @param type: moudle type
 *  @param _dirPin: dir pin;
 *  @param _pwmPin: pwm pin;
 *  @param _csPin: cs pin 电流检测;
 *  @param _slpPin: sleep pin;
 */
MotorDriver::MotorDriver(uint8_t type, uint8_t _dirPin, uint8_t _pwmPin,
                         int8_t _offset, uint8_t _slpPin, uint8_t _csPin) {
  _TYPE_MODULE = type;
  _offset = _offset >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD) {
    _OFFSET = _offset;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    _MSLP_PIN = _slpPin;
    _MCS_PIN = _csPin;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
    if(_MSLP_PIN != -1){
      pinMode(_MSLP_PIN, OUTPUT);
      digitalWrite(_MSLP_PIN, HIGH);  // 默认使能
    }
    if(_MCS_PIN != -1)
      pinMode(_MCS_PIN, INPUT);
  }
}

/*!
 *  @brief set ALL motor direction,
 *  @param offsetAll: motor all direction, eg: 1 - all motor default direction,-1 - all motor reverse direction;
 */
void MotorDriver::motorConfig(int8_t offsetAll) {
  offsetAll = offsetAll >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_IIC_TB) {
    _OFFSETM1 = offsetAll;
    _OFFSETM2 = offsetAll;
    _OFFSETM3 = offsetAll;
    _OFFSETM4 = offsetAll;
  } else if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3 ) {
    _OFFSETA = offsetAll;
    _OFFSETB = offsetAll;
  } else if (_TYPE_MODULE == YF_VALON) {
    _OFFSETA = offsetAll*DIRN;
    _OFFSETB = offsetAll*DIRN;
  } else if (_TYPE_MODULE == YF_4WDMW) {
    _OFFSETM1 = offsetAll*DIRN;
    _OFFSETM2 = offsetAll;
    _OFFSETM3 = offsetAll*DIRN;
    _OFFSETM4 = offsetAll;
  }
}

/*!
 *  @brief  Motor Config. Forward or Reverse.
 *  @param offsetA: Value can be 1 or -1;
 *  @param offsetB: Value can be 1 or -1;
 */
void MotorDriver::motorConfig(int8_t offsetA = 1, int8_t offsetB = 1) {
  offsetA = offsetA >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  offsetB = offsetB >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = offsetA;
    _OFFSETB = offsetB;
  } else if (_TYPE_MODULE == YF_VALON) {
    _OFFSETA = offsetA*DIRN;
    _OFFSETB = offsetB*DIRN;
  }
}

/*!
 *  @brief set motor direction,
 *  @param offsetM1: motor M1 direction, eg: 1 - M1 default direction ,1 - M1 reverse direction;
 *  @param offsetM2: motor M2 direction;
 *  @param offsetM3: motor M3 direction;
 *  @param offsetM4: motor M4 direction;
 */
void MotorDriver::motorConfig(int8_t offsetM1, int8_t offsetM2, int8_t offsetM3,
                              int8_t offsetM4) {
  offsetM1 = offsetM1 >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  offsetM2 = offsetM2 >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  offsetM3 = offsetM3 >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  offsetM4 = offsetM4 >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_IIC_TB) {
    _OFFSETM1 = offsetM1;
    _OFFSETM2 = offsetM2;
    _OFFSETM3 = offsetM3;
    _OFFSETM4 = offsetM4;
    setAllMotor(0);
  } else if (_TYPE_MODULE == YF_4WDMW) {
    _OFFSETM1 = offsetM1*DIRN;
    _OFFSETM2 = offsetM2;
    _OFFSETM3 = offsetM3*DIRN;
    _OFFSETM4 = offsetM4;
  }
}

/*!
 *  @brief Drive motor , 1 Motor 固定引脚
 *  @param _dirPin: motor dir pin;
 *  @param _pwmPin: motor pwm pin;
 *  @param _mspeed: motor speed, range -255 ~ 255;
 *  @param _moffset: motor direction, eg: 1 - default direction ,-1 - reverse direction;
 */
void MotorDriver::driverOneMotor(uint8_t _dirPin, uint8_t _pwmPin,
                                 int16_t _mspeed, int8_t _moffset = 1) {
  _moffset = _moffset >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  _mspeed = min(_mspeed, 255);
  _mspeed = max(-255, _mspeed);
  _mspeed = _mspeed * _moffset;

  if (_mspeed > 0)
    digitalWrite(_dirPin, HIGH);
  else
    digitalWrite(_dirPin, LOW);
  analogWrite(_pwmPin, abs(_mspeed));
}

/*!
 *  @brief Drive motor in the speed. MD 1234
 *  @param _mspeed: motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t _mspeed) {
  if (_TYPE_MODULE == YF_MD) {
    _mspeed = min(_mspeed, 255);
    _mspeed = max(-255, _mspeed);
    _mspeed = _mspeed * _OFFSET;
    if (_mspeed < 0)
      digitalWrite(_MDIR_PIN, LOW);
    else
      digitalWrite(_MDIR_PIN, HIGH);
    analogWrite(_MPWM_PIN, abs(_mspeed));
  }
}

/*!
 *  @brief Drive motor , 2 Motor
 *  @param speedA: M1 motor speed, range -255 ~ 255;
 *  @param speedB: M2 motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t speedA, int16_t speedB) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3 || _TYPE_MODULE == YF_VALON) {
    if (SerialDebug == 1) Serial.println("Driver Motor - L298P/PMR3/valon");

    speedA = min(speedA, 255);
    speedA = max(-255, speedA);
    speedA = speedA * _OFFSETA;

    speedB = min(speedB, 255);
    speedB = max(-255, speedB);
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

/*!
 *  @brief Drive single motor ,
 *  @param _in1Pin: moudle pin;
 *  @param _in2Pin: moudle pin;
 *  @param _pwmPin: moudle pwm pin;
 *  @param _mspeed: speed: motor speed, range -4096 ~ 4096;
 */
void MotorDriver::driverOneMotor_IIC(uint8_t _in1Pin, uint8_t _in2Pin,
                                 uint8_t _pwmPin, int16_t _mspeed,
                                 int8_t _moffset = 1) {
  _moffset = _moffset >= 0 ? DIRP : DIRN;  // 限制正反方向值 1、-1
  _mspeed = _mspeed * _moffset;
  if (_mspeed > 0) {
    setPin(_in1Pin, 4096, 0);
    setPin(_in2Pin, 0, 0);
    setPin(_pwmPin, _mspeed, 0);
  } else if (_mspeed < 0) {
    setPin(_in1Pin, 0, 0);
    setPin(_in2Pin, 4096, 0);
    setPin(_pwmPin, 0 - _mspeed, 0);
  } else {
    setPin(_in1Pin, 0, 0);
    setPin(_in2Pin, 0, 0);
  }
}

/*!
 *  @brief Drive single motor ,
 *  @param _mNum: motor number, eg:1 - M1 motor ... 5 - all motor;
 *  @param _mspeed: motor speed, range -4096~4096 / -255~255;
 */
void MotorDriver::setSingleMotor(uint8_t _mNum, int16_t _mspeed) {
  if (_TYPE_MODULE == YF_IIC_TB) {
    if (_mNum == M1) { // MOTOR 1
      driverOneMotor_IIC(_M1IN1, _M1IN2, _M1PWM, _mspeed, _OFFSETM1);
    } else if (_mNum == M2) { // MOTOR 2
      driverOneMotor_IIC(_M2IN1, _M2IN2, _M2PWM, _mspeed, _OFFSETM2);
    } else if (_mNum == M3) { // MOTOR 3
      driverOneMotor_IIC(_M3IN1, _M3IN2, _M3PWM, _mspeed, _OFFSETM3);
    } else if (_mNum == M4) { // MOTOR 4
      driverOneMotor_IIC(_M4IN1, _M4IN2, _M4PWM, _mspeed, _OFFSETM4);
    } else {
#ifdef ENABLE_DEBUG_OUTPUT
      Serial.print("Error: Motor number error.");
#endif
    }
  } else if (_TYPE_MODULE == YF_4WDMW) {
    if (_mNum == M1) { // MOTOR 1
      driverOneMotor(YF_4WDMW_M1DIR_PIN, YF_4WDMW_M1PWM_PIN, _mspeed, _OFFSETM1);
    } else if (_mNum == M2) { // MOTOR 2
      driverOneMotor(YF_4WDMW_M2DIR_PIN, YF_4WDMW_M2PWM_PIN, _mspeed, _OFFSETM2);
    } else if (_mNum == M3) { // MOTOR 3
      driverOneMotor(YF_4WDMW_M3DIR_PIN, YF_4WDMW_M3PWM_PIN, _mspeed, _OFFSETM3);
    } else if (_mNum == M4) { // MOTOR 4
      driverOneMotor(YF_4WDMW_M4DIR_PIN, YF_4WDMW_M4PWM_PIN, _mspeed, _OFFSETM4);
    }
  }
}

/*!
 *  @brief Drive motor , 4 Motor
 *  @param speedM1: M1 motor speed, range -4096 ~ 4096;
 *  @param speedM2: M2 motor speed, range -4096 ~ 4096;
 *  @param speedM3: M3 motor speed, range -4096 ~ 4096;
 *  @param speedM4: M4 motor speed, range -4096 ~ 4096;
 */
void MotorDriver::setMotor(int16_t speedM1, int16_t speedM2, int16_t speedM3,
                           int16_t speedM4) {
  if (_TYPE_MODULE == YF_IIC_TB) {
    if (SerialDebug == 1) Serial.println("Driver Motor - YF_IIC_TB");
    // MOTOR 1
    setSingleMotor(M1, speedM1);
    // MOTOR 2
    setSingleMotor(M2, speedM2);
    // MOTOR 3
    setSingleMotor(M3, speedM3);
    // MOTOR 4
    setSingleMotor(M4, speedM4);
  } else if (_TYPE_MODULE == YF_4WDMW) {
    driverOneMotor(YF_4WDMW_M1DIR_PIN, YF_4WDMW_M1PWM_PIN, speedM1, _OFFSETM1);
    driverOneMotor(YF_4WDMW_M2DIR_PIN, YF_4WDMW_M2PWM_PIN, speedM2, _OFFSETM2);
    driverOneMotor(YF_4WDMW_M3DIR_PIN, YF_4WDMW_M3PWM_PIN, speedM3, _OFFSETM3);
    driverOneMotor(YF_4WDMW_M4DIR_PIN, YF_4WDMW_M4PWM_PIN, speedM4, _OFFSETM4);
  }
}

/*!
 *  @brief Drive ALL motor in the same speed.
 *  @param speedall: M1 M2 M3 M4 motor speed, range -4096 ~ 4096;
 *  MD 1234
 *  @param speedall: motor speed, range -255 ~ 255;
 */
void MotorDriver::setAllMotor(int16_t speedall) {
  if (_TYPE_MODULE == YF_IIC_TB) {
    // MOTOR 1
    setSingleMotor(M1, speedall);
    // MOTOR 2
    setSingleMotor(M2, speedall);
    // MOTOR 3
    setSingleMotor(M3, speedall);
    // MOTOR 4
    setSingleMotor(M4, speedall);
  } else if (_TYPE_MODULE == YF_4WDMW) {
    driverOneMotor(YF_4WDMW_M1DIR_PIN, YF_4WDMW_M1PWM_PIN, speedall, _OFFSETM1);
    driverOneMotor(YF_4WDMW_M2DIR_PIN, YF_4WDMW_M2PWM_PIN, speedall, _OFFSETM2);
    driverOneMotor(YF_4WDMW_M3DIR_PIN, YF_4WDMW_M3PWM_PIN, speedall, _OFFSETM3);
    driverOneMotor(YF_4WDMW_M4DIR_PIN, YF_4WDMW_M4PWM_PIN, speedall, _OFFSETM4);
  } else if (_TYPE_MODULE == YF_MD) {
    speedall = min(speedall, 255);
    speedall = max(-255, speedall);
    speedall = speedall * _OFFSET;
    if (speedall < 0)
      digitalWrite(_MDIR_PIN, LOW);
    else
      digitalWrite(_MDIR_PIN, HIGH);
    analogWrite(_MPWM_PIN, abs(speedall));
  }
}

/*!
 *  @brief stop motor , Emergency stop / brake 紧急停止/刹车
 *  @param _mNum: motor number, eg:1 - M1 motor ... 5 - all motor;
 */
void MotorDriver::stopMotor(uint8_t _mNum) {
  if (_mNum == M1) {  // MOTOR 1
    setPin(_M1PWM, 0, 0);
  } else if (_mNum == M2) {  // MOTOR 2
    setPin(_M2PWM, 0, 0);
  } else if (_mNum == M3) {  // MOTOR 3
    setPin(_M3PWM, 0, 0);
  } else if (_mNum == M4) {  // MOTOR 4
    setPin(_M4PWM, 0, 0);
  } else if (_mNum == MAll) {  // MOTOR 1 2 3 4
    setPin(_M1PWM, 0, 0);
    setPin(_M2PWM, 0, 0);
    setPin(_M3PWM, 0, 0);
    setPin(_M4PWM, 0, 0);
  } else {
    /* code */
  }
}

/*!
 *  @brief  Puts board into sleep mode
 */
void MotorDriver::sleep() {
  if (_TYPE_MODULE == YF_IIC_TB) {
    uint8_t awake = read8(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;  // set sleep bit high
    write8(PCA9685_MODE1, sleep);
    delay(5);  // wait until cycle ends for sleep to be active
  } else if (_TYPE_MODULE == YF_MD) {
    if(_MSLP_PIN != -1)
      digitalWrite(_MSLP_PIN, LOW);
  }
}

/*!
 *  @brief  Wakes board from sleep
 */
void MotorDriver::wakeup() {
  if (_TYPE_MODULE == YF_IIC_TB) {
    uint8_t sleep = read8(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP;  // set sleep bit low
    write8(PCA9685_MODE1, wakeup);
  } else if (_TYPE_MODULE == YF_MD) {
    if(_MSLP_PIN != -1)
      digitalWrite(_MSLP_PIN, HIGH);
  }
}

/*!
 *  @brief get Motor Current，电流检测功能
 *  获取电流检测引脚模拟电压值
 *  电流检测：50mV/A （仅在H桥工作时有效）
 */
unsigned int MotorDriver::getMotorCurrent() { 
  if(_MCS_PIN != -1)
    return analogRead(_MCS_PIN); 
  else 
    return 0;
}
