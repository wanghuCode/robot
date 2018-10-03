#include "dma.h"
#include "protocol.h"
#include "usart.h"
/******************************DMA代码*******************************************/

DMA_InitTypeDef DMA_InitStructure; //定义DMA结构体


u16 USART1_RX_DMA_MEM_LEN;//保存DMA每次接收数据的长度 
u16 USART1_TX_DMA_MEM_LEN;//保存DMA每次发送数据的长度 

u16 USART2_TX_DMA_MEM_LEN;//保存DMA每次发送数据的长度 
u16 USART3_RX_DMA_MEM_LEN;//保存DMA每次发送数据的长度 

   /* 
     配置串口1DMA发送参数	 
     这里的传输形式是固定的,这点要根据不同的情况来修改,从存储器->外设模式/8位数据宽度/存储器增量模式
     DMA_CHx:DMA通道CHx
     cpar:外设地址
     cmar:存储器地址
     cndtr:数据传输量 
			*/		
void USART_TX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输 时钟
	DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	if(DMA_CHx==DMA1_Channel4)
	{
	 USART1_TX_DMA_MEM_LEN=cndtr;//保存DMA每次发送数据的长度 
	}
	else if(DMA_CHx==DMA1_Channel7)
	{
	USART2_TX_DMA_MEM_LEN=cndtr; //一次DMA数据传输量
	}
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
} 


 /* 
     配置串口1DMA接收参数	 
     这里的传输形式是固定的,这点要根据不同的情况来修改,从存储器->外设模式/8位数据宽度/存储器增量模式
     DMA_CHx:DMA通道CHx
     cpar:外设地址
     cmar:存储器地址
     cndtr:数据传输量 
			*/		
 void USART1_RX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输时钟
	USART1_RX_DMA_MEM_LEN=cndtr;
	DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	DMA_ClearFlag(DMA1_FLAG_TC5); //清除DMA接收完成标志
		
} 

void USART3_RX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输时钟
	USART3_RX_DMA_MEM_LEN=cndtr;
	DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	DMA_ClearFlag(DMA1_FLAG_TC3); //清除DMA接收完成标志
		
} 


//开启一次DMA传输  (通过USART1_TX 向电脑传输数据)
void Enable_DMA_TRANSMIT(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA 所指示的通道  
  if(DMA_CHx==DMA1_Channel4)
	{	
 	  //DMA_SetCurrDataCounter(DMA_CHx,USART1_TX_DMA_MEM_LEN);//DMA通道的DMA缓存的大小
	DMA_SetCurrDataCounter(DMA_CHx,TransportProtocol_Manager.FrameTotalLength);
	}
	else if(DMA_CHx==DMA1_Channel7)
	{
	 	DMA_SetCurrDataCounter(DMA_CHx,USART2_TX_DMA_MEM_LEN);//DMA通道的DMA缓存的大小
	}
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART2 TX DMA1 所指示的通道 
}	  


/**********************************************************************************************
	*函数名：void COM_DMA_NVIC_Config(void)
	* 参 数
	* 返回值：
	* 功能：配置串口DMA的中断优先级
	**********************************************************************************************/
void COM_DMA_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}






