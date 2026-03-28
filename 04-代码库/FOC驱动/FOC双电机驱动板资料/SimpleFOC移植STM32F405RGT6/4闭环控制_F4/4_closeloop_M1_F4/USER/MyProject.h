

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include "usart2.h"
#include "delay.h"
#include "timer.h"
#include "i2c.h"
#include "spi3.h"

#include "foc_utils.h"
#include "MagneticSensor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "lowpass_filter.h"
#include "pid.h"

#define M1_Disable   TIM_Cmd(TIM8, DISABLE);        //关闭M1输出

//设置使用的编码器为1，不使用的为0
#define M1_AS5600    1   //编码器类型，只能选一
#define M1_AS5047P   0
#define M1_TLE5012B  0
#define M1_MA730     0
#define M1_MT6701    0
#define M1_ABZ       0      //ABZ需要设置下面的CPR。当前代码只支持AB信号不支持Z信号。第5节的代码增加了ABZ的支持。
#define M1ABZ_CPR    16384  //需要准确设置ABZ的CPR。AS5047P=4000，TLE5012B=16384

#endif

