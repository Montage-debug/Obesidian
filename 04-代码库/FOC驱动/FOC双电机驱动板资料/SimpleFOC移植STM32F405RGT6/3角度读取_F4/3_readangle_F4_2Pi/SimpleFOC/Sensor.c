

#include "MyProject.h"


/******************************************************************************/
ENCODER_S  E0,E1;
/******************************************************************************/
uint16_t getRawCount(uint8_t x);
/******************************************************************************/
//初始化三种SPI接口的编码器的参数, 初始化I2C接口或者SPI接口
void MagneticSensor_Init(void)
{
#if M0_AS5600
	I2C_Init_(0);                  //AS5600
	E0.cpr = AS5600_CPR;
	printf("M0_AS5600\r\n");
#elif M0_AS5047P
	SPI3_Init_(0,SPI_CPOL_Low);    //AS5047P
	E0.cpr = AS5047P_CPR;
	printf("M0_AS5047P\r\n");
#elif M0_TLE5012B
	SPI3_Init_(0,SPI_CPOL_Low);    //TLE5012B
	E0.cpr = TLE5012B_CPR;
	printf("M0_TLE5012B\r\n");
#elif M0_MA730
  SPI3_Init_(0,SPI_CPOL_High);   //MA730
	E0.cpr = MA730_CPR;
	printf("M0_MA730\r\n");
#elif M0_MT6701
  SPI3_Init_(0,SPI_CPOL_Low);    //MT6701
	E0.cpr = MT6701_CPR;
	printf("M0_MT6701\r\n");
#elif M0_ABZ
  TIM3_Encoder_Init();           //ABZ
	//EXTI_Encoder_Init(0);        //Z信号中断
	E0.cpr = M0ABZ_CPR;
	printf("M0_ABZ\r\n");
#endif

#if M1_AS5600
	I2C_Init_(1);                  //AS5600
	E1.cpr = AS5600_CPR;
	printf("M1_AS5600\r\n");
#elif M1_AS5047P
	SPI3_Init_(1,SPI_CPOL_Low);    //AS5047P
	E1.cpr = AS5047P_CPR;
	printf("M1_AS5047P\r\n");
#elif M1_TLE5012B
	SPI3_Init_(1,SPI_CPOL_Low);    //TLE5012B
	E1.cpr = TLE5012B_CPR;
	printf("M1_TLE5012B\r\n");
#elif M1_MA730
  SPI3_Init_(1,SPI_CPOL_High);   //MA730
	E1.cpr = MA730_CPR;
	printf("M1_MA730\r\n");
#elif M1_MT6701
  SPI3_Init_(1,SPI_CPOL_Low);    //MT6701
	E1.cpr = MT6701_CPR;
	printf("M1_MT6701\r\n");
#elif M1_ABZ
  TIM4_Encoder_Init();           //ABZ
	//EXTI_Encoder_Init(1);        //Z信号中断
	E1.cpr = M1ABZ_CPR;
	printf("M1_ABZ\r\n");
#endif

	
	delay_ms(10);
	
	E0.angle_data_prev = getRawCount(0);
	E0.full_rotation_offset = 0;
	E0.velocity_calc_timestamp=0;//SysTick->VAL;
	delay_ms(5);
	E0.angle_prev = getAngle(&E0);
	
	E1.angle_data_prev = getRawCount(1);
	E1.full_rotation_offset = 0;
	E1.velocity_calc_timestamp=0;//SysTick->VAL;
	delay_ms(5);
	E1.angle_prev = getAngle(&E1);
}
/******************************************************************************/
uint16_t getRawCount(uint8_t x)  //获取编码器的原始值
{
	uint16_t val;
	
	if(x==0)
	{
#if M0_AS5600
	val = ReadAS5600(0)&0x0FFF;
#elif M0_AS5047P 
	val = ReadAS5047P(0)&0x3FFF;
#elif M0_TLE5012B
	val = ReadTLE5012B(0)&0x7FFF;
#elif M0_MA730
	val = ReadMA730(0);    //左对齐，低两位补0
#elif M0_MT6701
	val = ReadMT6701(0);   //左对齐，低两位补0
#elif M0_ABZ
	val = ReadABZ(0);
#endif
	}
	else
	{
#if M1_AS5600
	val = ReadAS5600(1)&0x0FFF;
#elif M1_AS5047P 
	val = ReadAS5047P(1)&0x3FFF;
#elif M1_TLE5012B
	val = ReadTLE5012B(1)&0x7FFF;
#elif M1_MA730
	val = ReadMA730(1);    //左对齐，低两位补0
#elif M1_MT6701
	val = ReadMT6701(1);   //左对齐，低两位补0
#elif M1_ABZ
	val = ReadABZ(1);
#endif
	}
	
	return val;
}
/******************************************************************************/
float getAngle(ENCODER_S *E)
{
	long angle_data,d_angle;
	
	angle_data = getRawCount(E->id);
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - E->angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8f*E->cpr) ) E->full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	E->angle_data_prev = angle_data;
	
	
//	if(E->full_rotation_offset >= ( _2PI*1000)) //转动圈数过多后浮点数精度下降，并导致堵转，每隔一千圈归零一次
//	{                                        //这个问题针对电机长时间连续转动；如果不是长时间一个方向转动也可以屏蔽掉这几句
//		E->full_rotation_offset=0;
//		E->angle_prev = E->angle_prev - _2PI*1000;
//	}
//	if(E->full_rotation_offset <= (-_2PI*1000))
//	{
//		E->full_rotation_offset=0;
//		E->angle_prev = E->angle_prev + _2PI*1000;
//	}
	
	
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return (E->full_rotation_offset + ( (float)angle_data / E->cpr) * _2PI);
}
/******************************************************************************/
// Shaft velocity calculation
float getVelocity(ENCODER_S *E)
{
	long now_us;
	float Ts, angle_now, vel;

	// calculate sample time
	now_us = SysTick->VAL;
	if(now_us < E->velocity_calc_timestamp)Ts = (float)(E->velocity_calc_timestamp - now_us)/21*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + E->velocity_calc_timestamp)/21*1e-6f;
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	// current angle
	angle_now = getAngle(E);
	// velocity calculation
	vel = (angle_now - E->angle_prev)/Ts;

	// save variables for future pass
	E->angle_prev = angle_now;
	E->velocity_calc_timestamp = now_us;
	return vel;
}
/******************************************************************************/



