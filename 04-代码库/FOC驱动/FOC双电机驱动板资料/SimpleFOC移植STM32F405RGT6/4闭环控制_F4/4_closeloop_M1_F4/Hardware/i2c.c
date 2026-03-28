
#include "i2c.h"

/***************************************************************************/
#define SDA0_IN()  {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=0<<(5*2);}
#define SDA0_OUT() {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=1<<(5*2);}
#define READ_SDA0  (GPIOB->IDR&(1<<5))

#define IIC0_SCL_1  GPIO_SetBits(GPIOB,GPIO_Pin_4)
#define IIC0_SCL_0  GPIO_ResetBits(GPIOB,GPIO_Pin_4)
#define IIC0_SDA_1  GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define IIC0_SDA_0  GPIO_ResetBits(GPIOB,GPIO_Pin_5)
/***************************************************************************/
#define SDA1_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<(7*2);}
#define SDA1_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<(7*2);}
#define READ_SDA1  (GPIOB->IDR&(1<<7))

#define IIC1_SCL_1  GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define IIC1_SCL_0  GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define IIC1_SDA_1  GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define IIC1_SDA_0  GPIO_ResetBits(GPIOB,GPIO_Pin_7)
/*****************************************************************************/
extern  void delay_s(uint32_t i);
/*****************************************************************************/
void I2C_Init_(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7);  //SCL和SDA输出高电平
}
/***************************************************************************/
void IIC1_Start(void)
{
	IIC1_SDA_1;
	IIC1_SCL_1;
	delay_s(16);    //延时4时读不出角度，延时8可以读出
	IIC1_SDA_0;
	delay_s(16);
	IIC1_SCL_0;
}
/***************************************************************************/
void IIC1_Stop(void)
{
	IIC1_SCL_0;
	IIC1_SDA_0;
	delay_s(16);
	IIC1_SCL_1;
	IIC1_SDA_1;
	delay_s(16);
}
/***************************************************************************/
//1-fail,0-success
unsigned char IIC1_Wait_Ack(void)
{
	unsigned char ucErrTime=0;
	
	SDA1_IN();
	IIC1_SDA_1;
	IIC1_SCL_1;
	delay_s(8);
	while(READ_SDA1!=0)
	{
		if(++ucErrTime>250)
			{
				SDA1_OUT();
				IIC1_Stop();
				return 1;
			}
	}
	SDA1_OUT();
	IIC1_SCL_0;
	return 0; 
}
/***************************************************************************/
void IIC1_Ack(void)
{
	IIC1_SCL_0;
	IIC1_SDA_0;
	delay_s(16);
	IIC1_SCL_1;
	delay_s(16);
	IIC1_SCL_0;
}
/***************************************************************************/
void IIC1_NAck(void)
{
	IIC1_SCL_0;
	IIC1_SDA_1;
	delay_s(16);
	IIC1_SCL_1;
	delay_s(16);
	IIC1_SCL_0;
}
/***************************************************************************/
void IIC1_Send_Byte(unsigned char txd)
{
	unsigned long i;
	
	IIC1_SCL_0;
	for(i=0;i<8;i++)
	{
		if((txd&0x80)!=0)IIC1_SDA_1;
		else
			IIC1_SDA_0;
		txd<<=1;
		delay_s(16);
		IIC1_SCL_1;
		delay_s(16);
		IIC1_SCL_0;
		delay_s(16);
	}
}
/***************************************************************************/
unsigned char IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,rcv=0;
	
	SDA1_IN();
	for(i=0;i<8;i++)
	{
		IIC1_SCL_0; 
		delay_s(16);
		IIC1_SCL_1;
		rcv<<=1;
		if(READ_SDA1!=0)rcv++;
		delay_s(8);
	}
	SDA1_OUT();
	if(!ack)IIC1_NAck();
	else
		IIC1_Ack();
	return rcv;
}
/***************************************************************************/

