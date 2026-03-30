

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
//#include <stdbool.h>
#include "usart2.h"
#include "delay.h"
#include "timer.h"
#include "i2c.h"
#include "spi3.h"
#include "adc.h"

#include "foc_utils.h"
#include "MagneticSensor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "LowsideCurrentSense.h"
#include "CurrentSense.h"

#define M0_Disable   TIM_Cmd(TIM1, DISABLE);        //关闭M0输出

//SPI接口用GPIO1做为CS0引脚，SPI接口用GPIO2做为CS1引脚
//I2C接口用M0的A/B为SCL0/SDA0，M1的A/B为SCL1/SDA1
//设置使用的编码器为1，不使用的为0
#define M0_AS5600    1   //编码器类型，只能选一
#define M0_AS5047P   0
#define M0_TLE5012B  0
#define M0_MA730     0
#define M0_MT6701    0
#define M0_MT6835    0
#define M0_ABZ       0      //ABZ需要设置下面的INDEX和CPR
#define M0_INDEX     0      //Z信号使用=1，不使用=0
#define M0ABZ_CPR    4000   //需要准确设置ABZ的CPR。AS5047P=4000，TLE5012B=16384


#endif

/*
ABZ信号分为：不用Z信号和使用Z信号，
1、不使用Z信号，只有AB信号，输出的是相对角度，电机每次上电机械角度为0，但是电角度不确定，
   所以每次都需要校准，而且每次校准的零点偏移值都不一样。
2、使用Z信号，零点校准的偏移值固定，但每次上电都需要找机械零点，也就是Z的过零点。

所以ABZ信号的
优点，接口简单统一，方便使用各种不同的电机，
缺点，每次上电都需要校准的过程，
*/

