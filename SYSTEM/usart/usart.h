#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
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
//#define USART_REC_LEN  			256  	//定义最大接收字节数 512
#define USART_SEN_LEN  			256 	//定义最大接收字节数 512
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX	  	1 

#define  UART1_RBUF_Size 	 256				//环形队列串口接收  4K字节		
////////////////////////////////////////////////////////////////////////////////// 
extern u8 	SREF;												//SREF串口接收服务器数据标志位;
extern u8 	CRCF;												//CRC校验值									//
extern u16 LEN;													//串口接收的数据长度值
extern u16 UART1_RX_ph;									//串口队列头
extern u16 UART1_RX_pe;									//串口队列尾
extern u8  UART1_RX_BUF[UART1_RBUF_Size];//串口数据接收缓冲字节
extern u8  TEMPBUF[256]; 								//临时缓冲区数组

//extern u8 USART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
extern u8 USART1_TX_BUF[USART_SEN_LEN];     //接收缓冲,最大USART_REC_LEN个字节.


extern u16 USART1_RX_STA;//接收状态标记	  
extern u16 USART1_RX_CNT;	      //接收的字节数


//extern u16 USART_RX_STA;         		//接收状态标记	
extern u8 g_USART1SendCompleteflag; //陀螺仪数据读数标志位

void uart_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
#endif


