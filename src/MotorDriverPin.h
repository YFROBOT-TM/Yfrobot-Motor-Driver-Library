#ifndef _MotorDriverPin_H
#define _MotorDriverPin_H

#include <Arduino.h>

/* Define pins of motor driver module. */
// L298P & PMR3 motor pins
#define YF_LP_ADIR_PIN  4   // motor a dir pin
#define YF_LP_APWM_PIN  5   // motor a pwm pin
#define YF_LP_BDIR_PIN  7   // motor b dir pin
#define YF_LP_BPWM_PIN  6   // motor b pwm pin

// 4路电机驱动 IIC PCA9685
#define YF_PCA9685_CH0  0   // Channel 0
#define YF_PCA9685_CH1  1   // Channel 1
#define YF_PCA9685_CH2  2   // Channel 2
#define YF_PCA9685_CH3  3   // Channel 3
#define YF_PCA9685_CH4  4   // Channel 4
#define YF_PCA9685_CH5  5   // Channel 5
#define YF_PCA9685_CH8  8   // Channel 8
#define YF_PCA9685_CH9  9   // Channel 9
#define YF_PCA9685_CH10  10   // Channel 10
#define YF_PCA9685_CH11  11   // Channel 11
#define YF_PCA9685_CH12  12   // Channel 12
#define YF_PCA9685_CH13  13   // Channel 13

// valon DRV8838
#define YF_VALON_LDIR_PIN  4   // motor left dir pin
#define YF_VALON_LPWM_PIN  5   // motor left pwm pin
#define YF_VALON_RDIR_PIN  9   // motor right dir pin
#define YF_VALON_RPWM_PIN  6   // motor right pwm pin

// 4wd mecanum wheel
#define YF_4WDMW_M1DIR_PIN 4
#define YF_4WDMW_M1PWM_PIN 3
#define YF_4WDMW_M2DIR_PIN 7
#define YF_4WDMW_M2PWM_PIN 5
#define YF_4WDMW_M3DIR_PIN 8
#define YF_4WDMW_M3PWM_PIN 6
#define YF_4WDMW_M4DIR_PIN 12
#define YF_4WDMW_M4PWM_PIN 11

// MD 01 02 03

// MD04
#define YF_MD04_ADIR_PIN  4   // motor a dir pin
#define YF_MD04_APWM_PIN  5   // motor a pwm pin
#define YF_MD04_ASLP_PIN  2   // motor a slp pin
#define YF_MD04_BDIR_PIN  7   // motor b dir pin
#define YF_MD04_BPWM_PIN  6   // motor b pwm pin
#define YF_MD04_BSLP_PIN  8   // motor b slp pin
#define YF_MD04_ACS_PIN A0
#define YF_MD04_BCS_PIN A1

#endif
