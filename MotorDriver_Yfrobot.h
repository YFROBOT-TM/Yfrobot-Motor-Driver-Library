/*
 *  @file MotorDriver_Yfrobot.h
 *
 *  This is a library for Motor driver.
 *
 *  Designed specifically to work with the Yfrobot Motor driver.
 *   L298P MD01 MD02 MD03 MD04 PM-R3 DRV8833 TB6612
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef _YFROBOT_MotorDriver_H
#define _YFROBOT_MotorDriver_H


#ifdef PCA9685_MotorDriver

#include "MotorDriver_PCA9685.h"

#elif L298P_MotorDriver

#elif MD_MotorDriver

#elif PMR3_MotorDriver

#elif DRV8833_MotorDriver

#elif TB6612_MotorDriver

#endif

#endif