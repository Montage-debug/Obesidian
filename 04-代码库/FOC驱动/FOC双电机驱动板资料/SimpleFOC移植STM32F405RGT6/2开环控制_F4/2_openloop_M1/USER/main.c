
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include "MyProject.h"

/****************************************************************************/
//串口打印用USART2(排针的GPIO 3/4引脚)，3为TXD2，4为RXD2
/************************************************
电机驱动板405
开环速度控制和开环位置控制  演示
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/128339581
        《SimpleFOC移植STM32（七）—— 移植STM32F405RGT6》
创建日期：20230405
作    者：loop222 @郑州
************************************************/
/****************************************************************************/
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
/****************************************************************************/
uint32_t timecntr_pre=0;
uint32_t time_cntr=0;
float target;
/****************************************************************************/
void commander_run(void);
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
/****************************************************************************/
void GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOD,GPIOC,GPIOB,GPIOA;
	
	//指示灯为PD2，低电平灯亮，高电平灯灭，
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //IO输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);       //亮
}
/*****************************************************************************/
/*****************************************************************************/
//开环控制最重要的参数就是voltage_limit
//1、电机抖动转不起来把voltage_limit设置的大一点，
//2、电机发热严重的把voltage_limit设置的小一点，
//3、串口发送指令“T10”，后面要有回车换行符
//4、电机能转就表示一切正常，可以学习下一章了，开环不是电机控制的常态，不要纠结太久。
int main(void)
{
	GPIO_Configure();
	USART2_Init(115200);         //GPIO 3/4引脚，3为TXD2，4为RXD2
	TIM8_PWM_Init();
	systick_CountInit();         //systick时钟开启1ms中断模式
	
	delay_ms(500);
	
	voltage_power_supply=12;   //V
	voltage_limit=2.5f;        //V，最大值需小于12/1.732=6.9。大功率航模电机设置的小一点0.5-1；小功率云台电机设置的大一点1-3
	velocity_limit=20;         //rad/s angleOpenloop() use it
	controller=Type_velocity_openloop;  //Type_angle_openloop;  //Type_velocity_openloop
	pole_pairs=7;              //极对数
	
	printf("Motor ready.\r\n");
	
	target = 6.28;   //上电后以6.28rad/s的速度转动（1圈/秒）
	
	while(1)
	{
		time_cntr +=timecount();
		if(time_cntr>=500000)  //us
		{