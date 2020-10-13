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
  }
}

/*!
 *  @brief  Motor Config. Forward or Reverse.
 *  @param offsetA: Value can be 1 or -1;
 *  @param offsetB: Value can be 1 or -1;
 */
void MotorDriver::motorConfig(int8_t offsetA = 1, int8_t offsetB = 1) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = offsetA;
    _OFFSETB = offsetB;
  }
}

/*!
 *  @brief Drive motor , 2 Motor
 *  @param speedA: M1 motor speed, range -255 ~ 255;
 *  @param speedB: M2 motor speed, range -255 ~ 255;
 */
void MotorDriver::setMotor(int16_t speedA, int16_t speedB) {
  if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
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

/*******************************************************************************
yfrobot PCA9685 IIC 4路电机驱动模块
*******************************************************************************/

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
// void MotorDriver::setAddress(const uint8_t addr) : _i2caddr(addr),
// _i2c(&Wire) {}
void MotorDriver::setAddress(const uint8_t addr) {
  _i2caddr = addr;
  _i2c = &Wire;
}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
void MotorDriver::setAddress(const uint8_t addr, TwoWire &i2c) {
  _i2caddr = addr;
  _i2c = &i2c;
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale Sets External Clock (Optional)
 */
void MotorDriver::begin(uint8_t prescale) {
  _i2c->begin();
  reset();

  if (prescale) {
    setExtClk(prescale);
  } else {
    // set a default frequency
    setPWMFreq(1000);
  }
  // set the default internal frequency
  setOscillatorFrequency(FREQUENCY_OSCILLATOR);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void MotorDriver::reset() {
  write8(PCA9685_MODE1, MODE1_RESTART);
  delay(10);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void MotorDriver::sleep() {
  uint8_t awake = read8(PCA9685_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP;  // set sleep bit high
  write8(PCA9685_MODE1, sleep);
  delay(5);  // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void MotorDriver::wakeup() {
  uint8_t sleep = read8(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP;  // set sleep bit low
  write8(PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void MotorDriver::setExtClk(uint8_t prescale) {
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
  write8(PCA9685_MODE1, newmode);  // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  write8(PCA9685_PRESCALE, prescale);  // set the prescaler

  delay(5);
  // clear the SLEEP bit to start
  write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void MotorDriver::setPWMFreq(float freq) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif
  // Range output modulation frequency is dependant on oscillator
  if (freq < 1) freq = 1;
  if (freq > 3500) freq = 3500;  // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: ");
  Serial.println(prescale);
#endif

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
  write8(PCA9685_MODE1, newmode);                              // go to sleep
  write8(PCA9685_PRESCALE, prescale);  // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void MotorDriver::setOutputMode(bool totempole) {
  uint8_t oldmode = read8(PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  write8(PCA9685_MODE2, newmode);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t MotorDriver::readPrescale(void) { return read8(PCA9685_PRESCALE); }

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t MotorDriver::getPWM(uint8_t num) {
  _i2c->requestFrom((int)_i2caddr, PCA9685_LED0_ON_L + 4 * num, (int)4);
  return _i2c->read();
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void MotorDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif

  _i2c->beginTransmission(_i2caddr);
  _i2c->write(PCA9685_LED0_ON_L + 4 * num);
  _i2c->write(on);
  _i2c->write(on >> 8);
  _i2c->write(off);
  _i2c->write(off >> 8);
  _i2c->endTransmission();
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void MotorDriver::setPin(uint8_t num, uint16_t val, bool invert) {
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, val);
    }
  }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void MotorDriver::writeMicroseconds(uint8_t num, uint16_t Microseconds) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM Via Microseconds on output");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(Microseconds);
  Serial.println("->");
#endif

  double pulse = Microseconds;
  double pulselength;
  pulselength = 1000000;  // 1,000,000 us per second

  // Read prescale
  uint16_t prescale = readPrescale();

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(prescale);
  Serial.println(" PCA9685 chip prescale");
#endif

  // Calculate the pulse for PWM based on Equation 1 from the datasheet section
  // 7.3.5
  prescale += 1;
  pulselength *= prescale;
  pulselength /= _oscillator_freq;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulselength);
  Serial.println(" us per bit");
#endif

  pulse /= pulselength;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulse);
  Serial.println(" pulse for PWM");
#endif

  setPWM(num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t MotorDriver::getOscillatorFrequency(void) { return _oscillator_freq; }

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void MotorDriver::setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}

/*!
 *  @brief set motor direction,
 *  @param m1Dir: motor M1 direction, eg: 0 - M1 motor default direction ,1 - M1
 * motor reverse direction;
 *  @param m2Dir: motor M2 direction;
 *  @param m3Dir: motor M3 direction;
 *  @param m4Dir: motor M4 direction;
 */
void MotorDriver::motorConfig(bool m1Dir, bool m2Dir, bool m3Dir, bool m4Dir) {
  if (_TYPE_MODULE == YF_IIC_TB) {
    _MOTORM1REVERSE = m1Dir;
    _MOTORM2REVERSE = m2Dir;
    _MOTORM3REVERSE = m3Dir;
    _MOTORM4REVERSE = m4Dir;
  }
}

/*!
 *  @brief set ALL motor direction,
 *  @param MAllDir: motor all direction, eg: 0 - all motor default direction ,1
 * - all motor reverse direction;
 */
void MotorDriver::motorConfig(bool MAllDir) {
  if (_TYPE_MODULE == YF_IIC_TB) {
    _MOTORMALLREVERSE = MAllDir;
  }else if (_TYPE_MODULE == YF_L298P || _TYPE_MODULE == YF_PMR3) {
    _OFFSETA = MAllDir;
    _OFFSETB = MAllDir;
  }
}

/*!
 *  @brief Drive single motor ,
 *  @param motorNum: motor number, eg:1 - M1 motor ;
 *  @param speed: speed: motor speed, range -4096 ~ 4096;
 */
void MotorDriver::setSingleMotor(int8_t motorNum, int16_t speed) {
  if (motorNum == M1) {
    if (_MOTORM1REVERSE || _MOTORMALLREVERSE) speed = 0 - speed;
    // MOTOR 1
    if (speed > 0) {
      setPin(_M1IN1, 4096, 0);
      setPin(_M1IN2, 0, 0);
      setPin(_M1PWM, speed, 0);
    } else if (speed < 0) {
      setPin(_M1IN1, 0, 0);
      setPin(_M1IN2, 4096, 0);
      setPin(_M1PWM, 0 - speed, 0);
    } else {
      setPin(_M1IN1, 0, 0);
      setPin(_M1IN2, 0, 0);
    }
  } else if (motorNum == M2) {
    if (_MOTORM2REVERSE || _MOTORMALLREVERSE) speed = 0 - speed;
    // MOTOR 2
    if (speed > 0) {
      setPin(_M2IN1, 4096, 0);
      setPin(_M2IN2, 0, 0);
      setPin(_M2PWM, speed, 0);
    } else if (speed < 0) {
      setPin(_M2IN1, 0, 0);
      setPin(_M2IN2, 4096, 0);
      setPin(_M2PWM, 0 - speed, 0);
    } else {
      setPin(_M2IN1, 0, 0);
      setPin(_M2IN2, 0, 0);
    }
  } else if (motorNum == M3) {
    if (_MOTORM3REVERSE || _MOTORMALLREVERSE) speed = 0 - speed;
    // MOTOR 3
    if (speed > 0) {
      setPin(_M3IN1, 4096, 0);
      setPin(_M3IN2, 0, 0);
      setPin(_M3PWM, speed, 0);
    } else if (speed < 0) {
      setPin(_M3IN1, 0, 0);
      setPin(_M3IN2, 4096, 0);
      setPin(_M3PWM, 0 - speed, 0);
    } else {
      setPin(_M3IN1, 0, 0);
      setPin(_M3IN2, 0, 0);
    }
  } else if (motorNum == M4) {
    if (_MOTORM4REVERSE || _MOTORMALLREVERSE) speed = 0 - speed;
    // MOTOR 4
    if (speed > 0) {
      setPin(_M4IN1, 4096, 0);
      setPin(_M4IN2, 0, 0);
      setPin(_M4PWM, speed, 0);
    } else if (speed < 0) {
      setPin(_M4IN1, 0, 0);
      setPin(_M4IN2, 4096, 0);
      setPin(_M4PWM, 0 - speed, 0);
    } else {
      setPin(_M4IN1, 0, 0);
      setPin(_M4IN2, 0, 0);
    }
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
    if (SerialDebug == 1) {
      Serial.println("Driver Motor - YF_IIC_TB");
    }
    // MOTOR 1
    setSingleMotor(M1, speedM1);
    // MOTOR 2
    setSingleMotor(M2, speedM2);
    // MOTOR 3
    setSingleMotor(M3, speedM3);
    // MOTOR 4
    setSingleMotor(M4, speedM4);
  }
}

/*!
 *  @brief Drive ALL motor in the same speed.
 *  @param speedall: M1 M2 M3 M4 motor speed, range -4096 ~ 4096;
 */
void MotorDriver::setMotor(int16_t speedall) {
  // MOTOR 1
  setSingleMotor(M1, speedall);
  // MOTOR 2
  setSingleMotor(M2, speedall);
  // MOTOR 3
  setSingleMotor(M3, speedall);
  // MOTOR 4
  setSingleMotor(M4, speedall);
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

/******************* Low level I2C interface */
uint8_t MotorDriver::read8(uint8_t addr) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return _i2c->read();
}

void MotorDriver::write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->write(d);
  _i2c->endTransmission();
}