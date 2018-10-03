#ifndef __RS485_H
#define __RS485_H			 
#include "hub_motor.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stdint.h"

//模式控制
#define RS485_TX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET)	//485模式控制 PA0写
#define RS485_RX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET)	//485模式控制 PA0读

#define u32 uint32_t
#define u8 uint8_t

void USART2_Init(u32 bound);
void RS485_Send_Data(USART_TypeDef* UARTx,u8 buf);	
#endif	   

