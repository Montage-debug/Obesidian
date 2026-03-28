

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_it.h" 
#include "usart2.h"
#include "delay.h"
#include "timer.h"
#include "i2c.h"
#include "spi3.h"

#include "foc_utils.h"
#include "MagneticSensor.h"
#include "Encoder.h"
#include "Sensor.h"

//SPI接口用GPIO1做为CS0引脚，GPIO2做为CS1引脚
//I2C接口用M0的A/B为SCL0/SDA0，M1的A/B为SCL1/SDA1
//设置使用的编码器为1，不使用的为0
#define M0_AS5600    1   //编码器类型，只能选一
#define M0_AS5047P   0
#define M0_TLE5012B  0
#define M0_MA730     0
#define M0_MT6701    0
#define M0_ABZ       0     //ABZ需要设置下面的CPR
#define M0ABZ_CPR    4000  //需要准确设置ABZ的CPR。AS5047P=4000，TLE5012B=16384

#define M1_AS5600    1   //编码器类型，只能选一
#define M1_AS5047P   0
#define M1_TLE5012B  0
#define M1_MA730     0
#define M1_MT6701    0
#define M1_ABZ       0     //ABZ需要设置下面的CPR
#define M1ABZ_CPR    16384 //需要准确设置ABZ的CPR。AS5047P=4000，TLE5012B=16384

#endif

