
#include "MyProject.h"


/*****************************************************************************/
void TIM3_Encoder_Init(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	
  TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 4;  //滤波器值
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
  TIM_SetCounter(TIM3,0); //TIM3->CNT=0
  TIM_Cmd(TIM3, ENABLE); 
}
/*****************************************************************************/
void TIM4_Encoder_Init(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	
  TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟分割
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 4;  //滤波器值
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_SetCounter(TIM4,0); //TIM3->CNT=0
  TIM_Cmd(TIM4, ENABLE); 
}
/******************************************************************************/
uint16_t  pulse0_counter, pulse1_counter;
uint16_t  pulse0_pre=0,   pulse1_pre=0;
long      pulse0_total=0, pulse1_total=0;

uint16_t ReadABZ(uint8_t x)
{
	short delta_pluse;
	
	if(x==0)
	{
		pulse0_counter = TIM_GetCounter(TIM3);
		delta_pluse = (short)pulse0_counter - (short)pulse0_pre;
		pulse0_pre = pulse0_counter;
		pulse0_total = (pulse0_total + delta_pluse) % E0.cpr;
		if(pulse0_total<0)pulse0_total += E0.cpr;
		return  pulse0_total;
	}
	else
	{
		pulse1_counter = TIM_GetCounter(TIM4);
		delta_pluse = (short)pulse1_counter - (short)pulse1_pre;
		pulse1_pre = pulse1_counter;
		pulse1_total = (pulse1_total + delta_pluse) % E1.cpr;
		if(pulse1_total<0)pulse1_total += E1.cpr;
		return  pulse1_total;
	}
}
/******************************************************************************/
void EXTI_Encoder_Init(uint8_t x)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	if(x==0)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   //PC9 M0的Z信号
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);  //PC9
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line9;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  //PC15 M1的Z信号
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource15); //PC15
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line15;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}
/******************************************************************************/



