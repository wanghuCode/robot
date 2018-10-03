#include "rs485.h"
  	  
// ���� ���ջ����� 	
u8 RS485_RX_BUF[64];  	//���ջ���,���64���ֽ�.
// ����     ���յ������ݳ���
u8 RS485_RX_CNT=0;   

//USART2�жϺ��� ����
void USART2_IRQHandler(void){

	u8 res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
		{
		res = USART_ReceiveData(USART2);	//��ȡ���յ�������
	
     if(RS485_RX_CNT<64){
		 
		 RS485_RX_BUF[RS485_RX_CNT]=res;
		
		 RS485_RX_CNT++;
		 
		 }			 
    }
		
}
//rs485ͨ�ų�ʼ��
 void USART2_Init(u32 bound){
	
	USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��UART2   GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//UART2_TX   GPIOA2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA2
   
  //USART2_RX	  GPIOA3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA3  
	
  //���շ���ʹ��PA0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA0

   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���
	
   //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//��ռ���ȼ�5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

}

//RS485����1���ֽ�����
//buf:����ֵ
void RS485_Send_Data(USART_TypeDef* UARTx,u8 buf)
{
  
	while(USART_GetFlagStatus(UARTx,USART_FLAG_TC)!=SET);//�ȴ����ͽ���		  
	USART_SendData(UARTx,buf); //��������
	while(USART_GetFlagStatus(UARTx,USART_FLAG_TC)!=SET);
}


