
#ifndef SENSOR_LIB_H
#define SENSOR_LIB_H

#include "stm32f4xx.h"

typedef struct 
{
	uint8_t id;                  //=0为E0,=1为E1
	long  cpr;
	long  velocity_calc_timestamp;  //速度计时，用于计算速度
	long  angle_data_prev;          //获取角度用
	float angle_prev;               //获取速度用
	float full_rotation_offset;     //角度累加
}ENCODER_S;

extern ENCODER_S  E0,E1;
/******************************************************************************/
#define  AS5600_CPR       4096       //12bit
#define  AS5047P_CPR      16384      //14bit
#define  TLE5012B_CPR     32768      //15bit
#define  MA730_CPR        65536      //14bit,左对齐,低两位补0,所以是65536
#define  MT6701_CPR       65536      //14bit,左对齐,低两位补0,所以是65536
/******************************************************************************/
void MagneticSensor_Init(void);
float getAngle(ENCODER_S *E);
float getVelocity(ENCODER_S *E);
/******************************************************************************/

#endif


