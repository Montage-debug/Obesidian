
#include <string.h>
#include "MyProject.h"
#include "user.h"

/************************************************
电机驱动板405
双电机控制 演示
=================================================
使用教程：https://blog.csdn.net/loop222/article/details/128339581
        《SimpleFOC移植STM32（七）—— 移植STM32F405RGT6》
创建日期：20230403
作    者：loop222 @郑州
************************************************/
//1、串口通信用USART2(排针的GPIO 3/4引脚)，3为TXD2，4为RXD2
//2、串口发送升级为DMA方式，配合sprintf函数，在主循环中打印不再影响电机运行
//3、初始化过程中的打印信息，为了防止数据覆盖，仍用printf()
/******************************************************************************/
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
/******************************************************************************/
extern void delay_s(uint32_t i);
void commander_run(void);
/******************************************************************************/
MOTORController M0,M1;

uint32_t timecntr_pre=0;
uint32_t time_cntr=0;
/******************************************************************************/
//us计时，每71.5分钟溢出循环一次
uint32_t timecount(void)
{
	uint32_t  diff,now_us;
	
	now_us = _micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;
	
	return diff;
}
/******************************************************************************/
void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOD,GPIOC,GPIOB,GPIOA;
	
	//指示灯为PD2，低电平灯亮，高电平灯灭，
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //IO输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);       //亮
}
/*****************************************************************************/
//void adc1_deal(void)
//{
//	float m0_temp,m1_temp,aux_temp,vbus_vol;
//	
//	m0_temp = (float)adc1_value[0]*3.3f/4096;
//	m1_temp = (float)adc1_value[1]*3.3f/4096;
//	aux_temp= (float)adc1_value[2]*3.3f/4096;
//	vbus_vol= (float)adc1_value[3]*3.3f*19/4096;
//}
/******************************************************************************/
/******************************************************************************/
int main(void)
{
	GPIO_Configure();
	USART2_Init(115200);         //GPIO 3/4引脚，3为TXD2，4为RXD2
	TIM1_PWM_Init();             //M0接口，完成配置，但没有使能
	TIM8_PWM_Init();             //M1接口，完成配置，但没有使能
	systick_CountInit();         //systick时钟开启1ms中断模式
	M0.id=0;                     //写结构体的id
	M1.id=1;
	strcpy(M0.str, "M0");
	strcpy(M1.str, "M1");
	
	delay_ms(200);
	MagneticSensor_Init();       //初始化编码器
	LowsideCurrentSense(&M0,0.001f,20, NOT_SET,ADC_Channel_10,ADC_Channel_11);    //采样电阻阻值，运放倍数，A相，B相，C相
	LowsideCurrentSense(&M1,0.001f,20, NOT_SET,ADC_Channel_13,ADC_Channel_12);    //采样电阻阻值，运放倍数，A相，B相，C相
	ADC_Init_synch();               //配置ADC，使能TIM1和TIM8，开始采样
	LowsideCurrentSense_Init(&M0);  //测量偏置电压
	LowsideCurrentSense_Init(&M1);
	LPF_init(&M0);               //LPF参数初始化
	LPF_init(&M1);
	PID_init(&M0);               //PID参数初始化
	PID_init(&M1);
	
	voltage_power_supply = get_vbus_voltage();    //V 电源电压，受电阻精度影响会有0.2V左右的误差
	printf("vbus=%.2f\r\n",voltage_power_supply);
	M0.voltage_limit = voltage_power_supply/2;    //V，主要为限制电机最大电流
	M0.pole_pairs=7;               //电机极对数，按照实际设置，虽然可以上电检测但有失败的概率
	M0.voltage_sensor_align=2.5;    //V 航模电机设置的值小一点比如0.5-1，云台电机设置的大一点比如2-3，电压增加这个参数也要增加
	M0.velocity_limit=6.928;           //rad/s 角度模式时限制最大转速，力矩模式和速度模式不起作用
	M0.current_limit=50;            //A，foc_current和dc_current模式限制电流，不能为0。速度模式和位置模式起作用
	M0.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage; 
	M0.controller=Type_angle;    //Type_torque; //Type_velocity;  //Type_angle;
//	M0.PID_d.P=0.8;            //0.6 电流环PI参数，可以进入 PID_init() 函数中修改其它参数
//	M0.PID_d.I=0.2;            //电流环I参数不太好调试，设置为0只用P参数也可以
//	M0.PID_q.P=0.8;
//	M0.PID_q.I=0.2;
	M0.PID_vel.P=0.05;          //0.2, 速度环PI参数，只用P参数方便快速调试
	M0.PID_vel.I=5;            //1
	M0.P_ang.P=20;             //位置环参数，只需P参数，一般不需要改动
	M0.PID_vel.output_ramp=0;  //速度爬升斜率，如果不需要可以设置为0。可能会影响位置模式的反应速度，不能设置太小。
	M0.LPF_vel.Tf=0.01;
	M0.target=0;
	
	M1.voltage_limit = voltage_power_supply/2;      //V，主要为限制电机最大电流
	M1.pole_pairs=7;                //电机极对数，按照实际设置，虽然可以上电检测但有失败的概率
	M1.voltage_sensor_align=2.5;    //V 航模电机设置的值小一点比如0.5-1，云台电机设置的大一点比如2-3，电压增加这个参数也要增加
	M1.velocity_limit=6.928;           //rad/s 角度模式时限制最大转速，力矩模式和速度模式不起作用
	M1.current_limit=50;            //A，foc_current和dc_current模式限制电流，不能为0。速度模式和位置模式起作用
	M1.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage; 
	M1.controller=Type_angle;    //Type_torque; //Type_velocity;  //Type_angle;
//	M1.PID_d.P=1;              //0.6 电流环PI参数，可以进入 PID_init() 函数中修改其它参数
//	M1.PID_d.I=0.5;            //电流环I参数不太好调试，设置为0只用P参数也可以
//	M1.PID_q.P=1;
//	M1.PID_q.I=0.5;
	M1.PID_vel.P=0.05;          //0.2, 速度环PI参数，只用P参数方便快速调试
	M1.PID_vel.I=5;            //1
	M1.P_ang.P=20;             //位置环参数，只需P参数，一般不需要改动
	M1.PID_vel.output_ramp=0;  //速度爬升斜率，如果不需要可以设置为0
	M1.LPF_vel.Tf=0.01;
	M1.target=0;
	
	M0.Index_needsSearch=M0_INDEX;    //如果使用ABZ编码器的Z信号=1
	M1.Index_needsSearch=M1_INDEX;    //如果使用ABZ编码器的Z信号=1
	if(M0.Index_needsSearch==1)EXTI_Encoder_Init(0);   //如果使用Z信号，开中断
	if(M1.Index_needsSearch==1)EXTI_Encoder_Init(1);   //如果使用Z信号，开中断
	Motor_init(&M0);
	Motor_init(&M1);
	Motor_initFOC(&M0, 0,UNKNOWN);  //(0,UNKNOWN);  //(3.25,CW); 第一次先获得偏移角和方向，填入代码编译后再下载，以后可以跳过零点校准。只用AB信号需要每次上电都零点校准。
	Motor_initFOC(&M1, 0,UNKNOWN);
	printf("Motor ready.\r\n");
	
	while(1)
	{
		time_cntr +=timecount();
		if(time_cntr>=1000000)  //us
		{
			time_cntr=0;
			LED_blink;
			//USART2_SendDMA(sprintf(snd2_buff,"Vel=%.4f\r\n",shaft_velocity));    //DMA方式发送,不影响电机运行；sprintf函数把要打印的字符串格式化
			//len=sprintf(snd2_buff,"m0B=%.2f,m0C=%.2f\r\n", m0_phB,m0_phC);
			//USART2_SendDMA(len);
		}
		move(&M0, M0.target);
		loopFOC(&M0);
		move(&M1, M1.target);
		loopFOC(&M1);
		commander_run();
	}
}
/******************************************************************************/
void commander_run(void)
{
	if(rcv2_flag==1)
	{
		rcv2_flag=0;
		switch(rcv2_buff[0])
		{
			case 'H':
				USART2_SendDMA(sprintf(snd2_buff,"Hello World!\r\n"));
				break;
			case 'A':   //A6.28   设置M0的目标值
				M0.target=atof((const char *)(rcv2_buff+1));
				USART2_SendDMA(sprintf(snd2_buff,"A=%.2f\r\n",M0.target));
				break;
			case 'B':   //B6.28   设置M1的目标值
				M1.target=atof((const char *)(rcv2_buff+1));
				USART2_SendDMA(sprintf(snd2_buff,"B=%.2f\r\n",M1.target));
				break;
			case 'T':   //T6.28   设置M0、M1的目标值
				M0.target=atof((const char *)(rcv2_buff+1));
				M1.target=M0.target;
				USART2_SendDMA(sprintf(snd2_buff,"T=%.2f\r\n",M0.target));
				break;
			
			case 'M':   //设置M0
				switch(rcv2_buff[1])
				{
					case 'P':   //MP0.5  设置速度环的P参数  
						M0.PID_vel.P=atof((const char *)(rcv2_buff+2));
						USART2_SendDMA(sprintf(snd2_buff,"P0=%.2f\r\n", M0.PID_vel.P));
						break;
					case 'I':   //MI0.2  设置速度环的I参数  
						M0.PID_vel.I=atof((const char *)(rcv2_buff+2));
						USART2_SendDMA(sprintf(snd2_buff,"I0=%.2f\r\n", M0.PID_vel.I));
						break;
					case 'V':   //MV  读实时速度
						USART2_SendDMA(sprintf(snd2_buff,"Vel0=%.2f\r\n", M0.shaft_velocity));
						break;
					case 'A':   //MA  读绝对角度
						USART2_SendDMA(sprintf(snd2_buff,"Ang0=%.2f\r\n", M0.shaft_angle));
						break;
				}
				break;
			
			case 'N':   //设置M1
				switch(rcv2_buff[1])
				{
					case 'P':   //NP0.5  设置速度环的P参数  
						M1.PID_vel.P=atof((const char *)(rcv2_buff+2));
						USART2_SendDMA(sprintf(snd2_buff,"P1=%.2f\r\n", M1.PID_vel.P));
						break;
					case 'I':   //NI0.2  设置速度环的I参数  
						M1.PID_vel.I=atof((const char *)(rcv2_buff+2));
						USART2_SendDMA(sprintf(snd2_buff,"I1=%.2f\r\n", M1.PID_vel.I));
						break;
					case 'V':   //NV  读实时速度
						USART2_SendDMA(sprintf(snd2_buff,"Vel1=%.2f\r\n", M1.shaft_velocity));
						break;
					case 'A':   //NA  读绝对角度
						USART2_SendDMA(sprintf(snd2_buff,"Ang1=%.2f\r\n", M1.shaft_angle));
						break;
				}
				break;
		}
		memset(rcv2_buff,0,16);  //USART2_BUFFER_SIZE //清空接收数组,长度覆盖接收的字节数即可
	}
}
/******************************************************************************/


