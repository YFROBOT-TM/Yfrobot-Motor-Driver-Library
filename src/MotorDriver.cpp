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
#include <Wire.h>

#include "MotorDriverPin.h"

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Constructor. Mainly sets up pins.
 */
MotorDriver::MotorDriver(uint8_t type) {
  _TYPE_MODULE = type;
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = 1;
    _OFFSETB = 1;
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
    _OFFSETA = -1;
    _OFFSETB = -1;
    _MADIRPIN = YF_VALON_LDIR_PIN;
    _MAPWMPIN = YF_VALON_LPWM_PIN;
    _MBDIRPIN = YF_VALON_RDIR_PIN;
    _MBPWMPIN = YF_VALON_RPWM_PIN;
    pinMode(_MADIRPIN, OUTPUT);
    pinMode(_MAPWMPIN, OUTPUT);
    pinMode(_MBDIRPIN, OUTPUT);
    pinMode(_MBPWMPIN, OUTPUT);
  } else if (_TYPE_MODULE == YF_4WDMW) {
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
  if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
      _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
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
  _offset = _offset >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
      _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    _OFFSET = _offset;
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
 *  @param _slpPin: sleep pin;
 */
MotorDriver::MotorDriver(uint8_t type, uint8_t _dirPin, uint8_t _pwmPin,
                         int8_t _offset, uint8_t _slpPin) {
  _TYPE_MODULE = type;
  _offset = _offset >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
      _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    _OFFSET = _offset;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    _MSLP_PIN = _slpPin;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
    pinMode(_MSLP_PIN, OUTPUT);
    digitalWrite(_MSLP_PIN, HIGH);  // 默认使能
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
  _offset = _offset >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
      _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    _OFFSET = _offset;
    _MDIR_PIN = _dirPin;
    _MPWM_PIN = _pwmPin;
    _MSLP_PIN = _slpPin;
    _MCS_PIN = _csPin;
    pinMode(_MDIR_PIN, OUTPUT);
    pinMode(_MPWM_PIN, OUTPUT);
    pinMode(_MSLP_PIN, OUTPUT);
    digitalWrite(_MSLP_PIN, HIGH);  // 默认使能
    pinMode(_MCS_PIN, INPUT);
  }
}

/*!
 *  @brief set ALL motor direction,
 *  @param MAllDir: motor all direction, eg: 1 - all motor default direction ,-1
 * - all motor reverse direction;
 */
void MotorDriver::motorConfig(int8_t MAllDir) {
  MAllDir = MAllDir >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_IIC_TB) {
    _MOTORM1REVERSE = MAllDir;
    _MOTORM2REVERSE = MAllDir;
    _MOTORM3REVERSE = MAllDir;
    _MOTORM4REVERSE = MAllDir;
  } else if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = MAllDir;
    _OFFSETB = MAllDir;
  }
}

/*!
 *  @brief  Motor Config. Forward or Reverse.
 *  @param offsetA: Value can be 1 or -1;
 *  @param offsetB: Value can be 1 or -1;
 */
void MotorDriver::motorConfig(int8_t offsetA = 1, int8_t offsetB = 1) {
  offsetA = offsetA >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  offsetB = offsetB >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = offsetA;
    _OFFSETB = offsetB;
  } else if (_TYPE_MODULE == YF_VALON) {
    _OFFSETA = offsetA;
    _OFFSETB = offsetB;
  }
}

/*!
 *  @brief set motor direction,
 *  @param m1Dir: motor M1 direction, eg: 1 - M1 motor default direction ,1 - M1
 * motor reverse direction;
 *  @param m2Dir: motor M2 direction;
 *  @param m3Dir: motor M3 direction;
 *  @param m4Dir: motor M4 direction;
 */
void MotorDriver::motorConfig(int8_t m1Dir, int8_t m2Dir, int8_t m3Dir,
                              int8_t m4Dir) {
  m1Dir = m1Dir >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  m2Dir = m2Dir >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  m3Dir = m3Dir >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  m4Dir = m4Dir >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  if (_TYPE_MODULE == YF_IIC_TB) {
    _MOTORM1REVERSE = m1Dir;
    _MOTORM2REVERSE = m2Dir;
    _MOTORM3REVERSE = m3Dir;
    _MOTORM4REVERSE = m4Dir;
  }
}

/*!
 *  @brief Drive motor , 1 Motor 固定引脚
 *  @param pin_d: motor dir pin;
 *  @param pin_p: motor pwm pin;
 *  @param m_speed: motor speed, range -255 ~ 255;
 */
void MotorDriver::driverOneMotor(int8_t pin_d, int8_t pin_p, int16_t m_speed,
                                 int8_t m_offset = 1) {
  m_offset = m_offset >= 0 ? 1 : -1;  // 限制正反方向值 1、-1
  m_speed = max(m_speed, 255);
  m_speed = min(-255, m_speed);
  m_speed = m_speed * m_offset;

  if (m_speed > 0)
    digitalWrite(pin_d, HIGH);
  else
    digitalWrite(pin_d, LOW);
  analogWrite(pin_p, abs(m_speed));
}

/*!
 *  @brief Drive motor in the speed. MD 1234
 *  @param m_speed: motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t m_speed) {
  if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
      _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    m_speed = max(m_speed, 255);
    m_speed = min(-255, m_speed);
    m_speed = m_speed * _OFFSET;
    if (m_speed < 0)
      digitalWrite(_MDIR_PIN, LOW);
    else
      digitalWrite(_MDIR_PIN, HIGH);
    analogWrite(_MPWM_PIN, abs(m_speed));
  }
}

/*!
 *  @brief Drive motor , 2 Motor
 *  @param speedA: M1 motor speed, range -255 ~ 255;
 *  @param speedB: M2 motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t speedA, int16_t speedB) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3 ||
      _TYPE_MODULE == YF_VALON) {
    if (SerialDebug == 1) Serial.println("Driver Motor - L298P/PMR3");

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

/*!
 *  @brief Drive single motor ,
 *  @param _in1: moudle pin;
 *  @param _in2: moudle pin;
 *  @param _pwm: moudle pwm pin;
 *  @param speed: speed: motor speed, range -4096 ~ 4096;
 */
void MotorDriver::setSingleMotor(uint8_t _in1, uint8_t _in2, uint8_t _pwm,
                                 int16_t speed) {
  if (speed > 0) {
    setPin(_in1, 4096, 0);
    setPin(_in2, 0, 0);
    setPin(_pwm, speed, 0);
  } else if (speed < 0) {
    setPin(_in1, 0, 0);
    setPin(_in2, 4096, 0);
    setPin(_pwm, 0 - speed, 0);
  } else {
    setPin(_in1, 0, 0);
    setPin(_in2, 0, 0);
  }
}

/*!
 *  @brief Drive single motor ,
 *  @param motorNum: motor number, eg:1 - M1 motor ;
 *  @param speed: speed: motor speed, range -4096 ~ 4096;
 */
void MotorDriver::setSingleMotor(int8_t motorNum, int16_t speed) {
  if (motorNum == M1) {
    speed = speed * _MOTORM1REVERSE;
    // MOTOR 1
    setSingleMotor(_M1IN1, _M1IN2, _M1PWM, speed);
  } else if (motorNum == M2) {
    speed = speed * _MOTORM2REVERSE;
    // MOTOR 2
    setSingleMotor(_M2IN1, _M2IN2, _M2PWM, speed);
  } else if (motorNum == M3) {
    speed = speed * _MOTORM3REVERSE;
    // MOTOR 3
    setSingleMotor(_M3IN1, _M3IN2, _M3PWM, speed);
  } else if (motorNum == M4) {
    speed = speed * _MOTORM4REVERSE;
    // MOTOR 4
    setSingleMotor(_M4IN1, _M4IN2, _M4PWM, speed);
  } else {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Error: Motor number error.");
#endif
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
    driverOneMotor(YF_4WDMW_M1DIR_PIN, YF_4WDMW_M1PWM_PIN, speedM1, 1);
    driverOneMotor(YF_4WDMW_M2DIR_PIN, YF_4WDMW_M2PWM_PIN, speedM2, 1);
    driverOneMotor(YF_4WDMW_M3DIR_PIN, YF_4WDMW_M3PWM_PIN, speedM3, 1);
    driverOneMotor(YF_4WDMW_M4DIR_PIN, YF_4WDMW_M4PWM_PIN, speedM4, 1);
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
    driverOneMotor(YF_4WDMW_M1DIR_PIN, YF_4WDMW_M1PWM_PIN, speedall, 1);
    driverOneMotor(YF_4WDMW_M2DIR_PIN, YF_4WDMW_M2PWM_PIN, speedall, 1);
    driverOneMotor(YF_4WDMW_M3DIR_PIN, YF_4WDMW_M3PWM_PIN, speedall, 1);
    driverOneMotor(YF_4WDMW_M4DIR_PIN, YF_4WDMW_M4PWM_PIN, speedall, 1);
  } else if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
             _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    speedall = max(speedall, 255);
    speedall = min(-255, speedall);
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
 *  @param motorNum: motor number, eg:1 - M1 motor ;
 */
void MotorDriver::stopMotor(int8_t motorNum) {
  if (motorNum == M1) {  // MOTOR 1
    setPin(_M1PWM, 0, 0);
  } else if (motorNum == M2) {  // MOTOR 2
    setPin(_M2PWM, 0, 0);
  } else if (motorNum == M3) {  // MOTOR 3
    setPin(_M3PWM, 0, 0);
  } else if (motorNum == M4) {  // MOTOR 4
    setPin(_M4PWM, 0, 0);
  } else if (motorNum == MAll) {  // MOTOR 1 2 3 4
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
  } else if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
             _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
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
  } else if (_TYPE_MODULE == YF_MD01 || _TYPE_MODULE == YF_MD02 ||
             _TYPE_MODULE == YF_MD03 || _TYPE_MODULE == YF_MD04) {
    digitalWrite(_MSLP_PIN, HIGH);
  }
}

/*!
 *  @brief get Motor Current，电流检测功能
 *  获取电流检测引脚模拟电压值
 *  电流检测：50mV/A （仅在H桥工作时有效）
 */
unsigned int MotorDriver::getMotorCurrent() {  
  return analogRead(_MCS_PIN);
}
