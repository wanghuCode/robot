#include "sys.h"
#include "usart.h"	  
#include "dma.h"
#include "protocol.h"
#include "string.h"
#include "DriversMain.h"
#include "CommunicationMain.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   

u8 USART1_TX_BUF[USART_SEN_LEN]={0};     //���ͻ���,���USART_REC_LEN���ֽ�.

//����״̬
u16 USART1_RX_STA=0;       //����1����״̬���	  
u16 USART1_RX_CNT=0;	      //����1���յ��ֽ���


void uart_init(u32 bound)
{

	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x06 ;//��ռ���ȼ�6
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�

	USART_ClearFlag(USART1,USART_FLAG_TC);	
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

//////////////////////////////////////////////////////////////////////////////////

u8 	SREF=0;												//SREF���ڽ��շ��������ݱ�־λ;
u8 	CRCF=0;												//CRCУ��ֵ									//
u16 LEN;													//���ڽ��յ����ݳ���ֵ
u16 UART1_RX_ph;									//���ڶ���ͷ
u16 UART1_RX_pe;									//���ڶ���β
u8  UART1_RX_BUF[UART1_RBUF_Size];//�������ݽ��ջ����ֽ�
u8  TEMPBUF[256]; 								//��ʱ����������
u8 g_USART1SendCompleteflag=0; //���������ݶ�����־λ

void USART1_IRQHandler(void)
{
   UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus=portSET_INTERRUPT_MASK_FROM_ISR(); 
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 	//���յ�����
	{	 	 			 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 				//����жϱ�־
		ReceiveSpeedTimeOut=0;
		UART1_task();
	} 	
  portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);	
		
}

#endif

#if EN_USART2_RX		  //���ʹ���˽���  

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
	
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
	
}

#endif  
	

u8 USART3_RX_BUF[50];     //���ջ���,���USART_REC_LEN���ֽ�.
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
		
   //Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x07 ;//��ռ���ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

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
		CopeSerial3Data((unsigned char)USART3->DR);//��������
		
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
 portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
}


