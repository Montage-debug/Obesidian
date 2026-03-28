
#include "stm32f4xx.h"
#include "delay.h"

/****************************************************************************/
#define LED_Shan   GPIOD->ODR^=(1<<2)  //PD2
/****************************************************************************/
void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//使能GPIOD
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOD,GPIOC,GPIOB,GPIOA;AFIO;
	
	//指示灯为PD2，低电平灯亮，高电平灯灭，
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //IO输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}
/*****************************************************************************/
int main(void)
{
	GPIO_Configure();
	systick_CountInit();
	
	delay_ms(500);
	
	while(1)
	{
		LED_Shan;
		delay_ms(500);
	}
}
/*****************************************************************************/


