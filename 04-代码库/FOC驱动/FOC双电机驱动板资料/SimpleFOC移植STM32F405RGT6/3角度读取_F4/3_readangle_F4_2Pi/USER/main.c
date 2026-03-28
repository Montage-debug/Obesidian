
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include "MyProject.h"


/******************************************************************************/
//串口打印用USART2(排针的GPIO 3/4引脚)，3为TXD2，4为RXD2
/************************************************
mDrive开发板
角度读取  演示
在“MyProject.h”文件中选择编码器类型
I2C采用模拟IO实现，分别使用了M0和M1的A/B引脚，A接SCL，B接SDA
SPI使用外设SPI3(即排针引出的SPI引脚)，CS分别用排针的1/2引脚
=================================================
本程序仅供学习，引用代码请标明出处
当前代码仍然可以参考下面的使用教程
使用教程：https://blog.csdn.net/loop222/article/details/128339581
        《SimpleFOC移植STM32（七）—— 移植STM32F405RGT6》
创建日期：20220925
作    者：loop222 @郑州
************************************************/
/****************************************************************************/
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
/****************************************************************************/
uint32_t time_cntr1,time_cntr2;
/****************************************************************************/
extern void delay_s(uint32_t i);
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
int main(void)
{
	float ang0,ang1;
	//float vel0,vel1;

	GPIO_Configure();
	USART2_Init(115200);       //GPIO 3/4引脚，3为TXD2，4为RXD2
	E0.id=0;
	E1.id=1;
	delay_ms(500);
	MagneticSensor_Init();
	TIM7_1ms_Init();
	
	delay_ms(200);             //最大延时798ms
	printf("Motor ready.\r\n");
	systick_CountMode();       //不能再调用delay_us()和delay_ms()函数
	
	while(1)
	{
		if(time_cntr1>=200)  //0.2s
		{
			time_cntr1=0;
			LED_blink;
		}
		
		ang0 = getAngle(&E0);
		ang1 = getAngle(&E1);
		//vel0 = getVelocity(&E0);
		//vel1 = getVelocity(&E1);
		
		if(time_cntr2>=500)   //每500ms打印一次
		{
			time_cntr2=0;
			printf("angle0=%.2f,angle1=%.2f\r\n", ang0,ang1);   //单位弧度，角度累加
			//printf("vel0=%.2f,vel1=%.2f\r\n", vel0,vel1);
		}
		
		delay_s(5000);      //软件延时
	}
}
/*****************************************************************************/


