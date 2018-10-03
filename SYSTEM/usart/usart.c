#include "sys.h"
#include "usart.h"	  
#include "dma.h"
#include "protocol.h"
#include "string.h"
#include "DriversMain.h"
#include "CommunicationMain.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   

u8 USART1_TX_BUF[USART_SEN_LEN]={0};     //发送缓冲,最大USART_REC_LEN个字节.

//接收状态
u16 USART1_RX_STA=0;       //串口1接收状态标记	  
u16 USART1_RX_CNT=0;	      //串口1接收的字节数


void uart_init(u32 bound)
{

	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x06 ;//抢占优先级6
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接收中断

	USART_ClearFlag(USART1,USART_FLAG_TC);	
	USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

//////////////////////////////////////////////////////////////////////////////////

u8 	SREF=0;												//SREF串口接收服务器数据标志位;
u8 	CRCF=0;												//CRC校验值									//
u16 LEN;													//串口接收的数据长度值
u16 UART1_RX_ph;									//串口队列头
u16 UART1_RX_pe;									//串口队列尾
u8  UART1_RX_BUF[UART1_RBUF_Size];//串口数据接收缓冲字节
u8  TEMPBUF[256]; 								//临时缓冲区数组
u8 g_USART1SendCompleteflag=0; //陀螺仪数据读数标志位

void USART1_IRQHandler(void)
{
   UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus=portSET_INTERRUPT_MASK_FROM_ISR(); 
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 	//接收到数据
	{	 	 			 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 				//清除中断标志
		ReceiveSpeedTimeOut=0;
		UART1_task();
	} 	
  portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);	
		
}

#endif

#if EN_USART2_RX		  //如果使能了接收  

u8 U2TxBuffer[256];
u8 U2TxCounter=0;
u8 U2count=0; 

void uart2_init(u32 bound)
{	
							   
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	 
	
    USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	
    USART_Cmd(USART2, ENABLE);                    //使能串口 
	
}

#endif  
	

u8 USART3_RX_BUF[50];     //接收缓冲,最大USART_REC_LEN个字节.
void uart3_init(u32 bound)
{	
							   
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	

    USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 
		
   //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x07 ;//抢占优先级7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_ClearFlag(USART3,USART_FLAG_TC);	
	USART_Cmd(USART3, ENABLE);
	

}
unsigned char Re_buf[11],counter;
unsigned char sign;
unsigned char Temp[11];

void USART3_IRQHandler(void)		  
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus=portSET_INTERRUPT_MASK_FROM_ISR();
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
		CopeSerial3Data((unsigned char)USART3->DR);//处理数据
		
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
 portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
}


