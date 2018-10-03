#ifndef _DMA_H_
#define _DMA_H_

#include "sys.h"

extern u16 USART1_RX_DMA_MEM_LEN;//保存DMA每次数据接收的长度 
extern u16 USART1_TX_DMA_MEM_LEN;//保存DMA每次发送数据的长度
extern u16 USART3_RX_DMA_MEM_LEN;
//void DMA_CHANNEL_INIT(void);
void Enable_DMA_TRANSMIT(DMA_Channel_TypeDef*DMA_CHx);
void USART_TX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);



#endif

