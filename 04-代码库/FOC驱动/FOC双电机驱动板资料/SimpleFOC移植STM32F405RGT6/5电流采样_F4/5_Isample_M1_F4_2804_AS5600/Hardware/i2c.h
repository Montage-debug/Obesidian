
#ifndef STM32_I2C_H
#define STM32_I2C_H

/******************************************************************************/
#include "stm32f4xx.h"

/******************************************************************************/
void I2C_Init_(void);

void IIC1_Start(void);
void IIC1_Stop(void);
unsigned char IIC1_Wait_Ack(void);
void IIC1_Ack(void);
void IIC1_NAck(void);
void IIC1_Send_Byte(unsigned char txd);
unsigned char IIC1_Read_Byte(unsigned char ack);
/******************************************************************************/


#endif
