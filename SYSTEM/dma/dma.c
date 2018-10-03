#include "dma.h"
#include "protocol.h"
#include "usart.h"
/******************************DMA����*******************************************/

DMA_InitTypeDef DMA_InitStructure; //����DMA�ṹ��


u16 USART1_RX_DMA_MEM_LEN;//����DMAÿ�ν������ݵĳ��� 
u16 USART1_TX_DMA_MEM_LEN;//����DMAÿ�η������ݵĳ��� 

u16 USART2_TX_DMA_MEM_LEN;//����DMAÿ�η������ݵĳ��� 
u16 USART3_RX_DMA_MEM_LEN;//����DMAÿ�η������ݵĳ��� 

   /* 
     ���ô���1DMA���Ͳ���	 
     ����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�,�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
     DMA_CHx:DMAͨ��CHx
     cpar:�����ַ
     cmar:�洢����ַ
     cndtr:���ݴ����� 
			*/		
void USART_TX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA���� ʱ��
	DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	if(DMA_CHx==DMA1_Channel4)
	{
	 USART1_TX_DMA_MEM_LEN=cndtr;//����DMAÿ�η������ݵĳ��� 
	}
	else if(DMA_CHx==DMA1_Channel7)
	{
	USART2_TX_DMA_MEM_LEN=cndtr; //һ��DMA���ݴ�����
	}
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	  	
} 


 /* 
     ���ô���1DMA���ղ���	 
     ����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�,�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
     DMA_CHx:DMAͨ��CHx
     cpar:�����ַ
     cmar:�洢����ַ
     cndtr:���ݴ����� 
			*/		
 void USART1_RX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����ʱ��
	USART1_RX_DMA_MEM_LEN=cndtr;
	DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	DMA_ClearFlag(DMA1_FLAG_TC5); //���DMA������ɱ�־
		
} 

void USART3_RX_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����ʱ��
	USART3_RX_DMA_MEM_LEN=cndtr;
	DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	DMA_ClearFlag(DMA1_FLAG_TC3); //���DMA������ɱ�־
		
} 


//����һ��DMA����  (ͨ��USART1_TX ����Դ�������)
void Enable_DMA_TRANSMIT(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA ��ָʾ��ͨ��  
  if(DMA_CHx==DMA1_Channel4)
	{	
 	  //DMA_SetCurrDataCounter(DMA_CHx,USART1_TX_DMA_MEM_LEN);//DMAͨ����DMA����Ĵ�С
	DMA_SetCurrDataCounter(DMA_CHx,TransportProtocol_Manager.FrameTotalLength);
	}
	else if(DMA_CHx==DMA1_Channel7)
	{
	 	DMA_SetCurrDataCounter(DMA_CHx,USART2_TX_DMA_MEM_LEN);//DMAͨ����DMA����Ĵ�С
	}
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART2 TX DMA1 ��ָʾ��ͨ�� 
}	  


/**********************************************************************************************
	*��������void COM_DMA_NVIC_Config(void)
	* �� ���U
	* ����ֵ��
	* ���ܣ����ô���DMA���ж����ȼ�
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






