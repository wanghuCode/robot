#include "DriversMain.h"
#include "CommunicationMain.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "dma.h"
#include "protocol.h"
#include "check.h"
#include "string.h"
#include "timers.h"
#include "delay.h"

u16 CCR3_Val = 17888;//��ʱ���ļ������塣 ����19999+1��*��71+1����/(72*1000000)=20000us=20ms    
u8 TmpBuf[250];  //������ջ�����
u8 SendBuf[Sendbuf_LEN];  //���巢�ͻ�����

u8 TEXT_TO_SEND[250];
u8 *Date_pose; //ָ�������ݵ�ָ��
u8 Length_TEXT_TO_SEND=0; //���͵���Ч���ݳ���

//����һ֡Э��
TransportProtocol_Typedef TransportProtocol;

void SendDateToUSART1(u8* Date,u16 Length)
{
   int i;
	USART_GetFlagStatus(USART1,USART_FLAG_TC);
	delay_ms(1);
	 for(i=0;i<Length;i++)
	{
	 USART_SendData(USART1,Date[i]);//�򴮿�1��������
 	 while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
  }
}


void APP_CommunicationService_Task(void *pvParameters)
{
 
  extern FlagStatus DriverService_InitComplete_Flage;
  extern FlagStatus RobotRunService_InitComplete_Flage;
  extern FlagStatus Communication_InitComplete_Flage;

  memset(USART1_TX_BUF, 0, 256); //���㷢����������
  memset(TEXT_TO_SEND, 0, 250); //���㷢����������
  TransportProtocol_Init(&TransportProtocol,USART1_TX_BUF,Checksum_XOR); //��ʼ������Э��
	
  Date_pose=TEXT_TO_SEND; //ָ�������������׵�ַ
  Communication_InitComplete_Flage = SET;
	
  while((DriverService_InitComplete_Flage == RESET) 
          || (RobotRunService_InitComplete_Flage == RESET) 
          || (Communication_InitComplete_Flage == RESET))
         
    {
         vTaskDelay(1);	//����1ms
    }
		
  while(1)
    {
		Update_TextToSend(Date_pose,&Length_TEXT_TO_SEND);	//���·�������
		TransportProtocol_Manager.Packed();   //�������	
			
		vTaskSuspendAll();//������
		SendDateToUSART1(TransportProtocol_Manager.Buf,TransportProtocol_Manager.FrameTotalLength);	 //���㷨�巢������
		xTaskResumeAll ();
			
	  vTaskDelay(3); //��ʱ3MS
    }
}
