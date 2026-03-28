
#include "usart2.h"

/********************************************************************/
char snd2_buff[USART2_BUFFER_SIZE];
char rcv2_buff[USART2_BUFFER_SIZE];
unsigned long rcv2_cntr;
unsigned long rcv2_flag;
/******************************************************************************/
//加入以下代码，支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
_sys_exit(int x) 
{
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART2->SR&0x40)==0);
	USART2->DR = (u8) ch;
	return ch;
}
#endif 
/********************************************************************/
void USART2_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_Struct;
	DMA_InitTypeDef DMA_InitStructure;
	
	//RCC_Config
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//GPIO_AF_Map
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	//GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //推挽复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//NVIC
	NVIC_Struct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Struct);
	
	//USART2
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&	USART_InitStructure);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	
	// DMA Rec
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rcv2_buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = USART2_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5,ENABLE);
}
/********************************************************************
 * USART2 接收空闲中断服务函数 
********************************************************************/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)
	{
		rcv2_cntr=USART2->SR;
		rcv2_cntr=USART2->DR;     //清除中断标志USART_ClearITPendingBit(USART2,USART_IT_RXNE)
		rcv2_cntr=USART2_BUFFER_SIZE-DMA_GetCurrDataCounter(DMA1_Stream5);  //读取接收字节个数
		DMA_Cmd(DMA1_Stream5,DISABLE);                           //关闭DMA传输
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}       //确保DMA可以被设置
		DMA_SetCurrDataCounter(DMA1_Stream5,USART2_BUFFER_SIZE); //数据传输量
		DMA_Cmd(DMA1_Stream5,ENABLE);                            //开启DMA传输
		rcv2_flag=1;
	}
}
/********************************************************************
void USART2_one(u8 da)
{
	USART_SendData(USART2,da);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);	//判断是否发送完成
}
***************************************
void USART2_send(u8 *p,u32 num)
{
	while(num--)
	{
		USART_SendData(USART2,*p++);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);	//判断是否发送完成
	}
}
********************************************************************/

