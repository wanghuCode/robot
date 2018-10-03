#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮������������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
//#define USART_REC_LEN  			256  	//�����������ֽ��� 512
#define USART_SEN_LEN  			256 	//�����������ֽ��� 512
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX	  	1 

#define  UART1_RBUF_Size 	 256				//���ζ��д��ڽ���  4K�ֽ�		
////////////////////////////////////////////////////////////////////////////////// 
extern u8 	SREF;												//SREF���ڽ��շ��������ݱ�־λ;
extern u8 	CRCF;												//CRCУ��ֵ									//
extern u16 LEN;													//���ڽ��յ����ݳ���ֵ
extern u16 UART1_RX_ph;									//���ڶ���ͷ
extern u16 UART1_RX_pe;									//���ڶ���β
extern u8  UART1_RX_BUF[UART1_RBUF_Size];//�������ݽ��ջ����ֽ�
extern u8  TEMPBUF[256]; 								//��ʱ����������

//extern u8 USART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
extern u8 USART1_TX_BUF[USART_SEN_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.


extern u16 USART1_RX_STA;//����״̬���	  
extern u16 USART1_RX_CNT;	      //���յ��ֽ���


//extern u16 USART_RX_STA;         		//����״̬���	
extern u8 g_USART1SendCompleteflag; //���������ݶ�����־λ

void uart_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
#endif

