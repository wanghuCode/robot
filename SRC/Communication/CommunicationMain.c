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

u16 CCR3_Val = 17888;//定时器的计数脉冲。 （（19999+1）*（71+1））/(72*1000000)=20000us=20ms    
u8 TmpBuf[250];  //定义接收缓冲区
u8 SendBuf[Sendbuf_LEN];  //定义发送缓冲区

u8 TEXT_TO_SEND[250];
u8 *Date_pose; //指向发送数据的指针
u8 Length_TEXT_TO_SEND=0; //发送的有效数据长度

//定义一帧协议
TransportProtocol_Typedef TransportProtocol;

void SendDateToUSART1(u8* Date,u16 Length)
{
   int i;
	USART_GetFlagStatus(USART1,USART_FLAG_TC);
	delay_ms(1);
	 for(i=0;i<Length;i++)
	{
	 USART_SendData(USART1,Date[i]);//向串口1发送数据
 	 while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
  }
}


void APP_CommunicationService_Task(void *pvParameters)
{
 
  extern FlagStatus DriverService_InitComplete_Flage;
  extern FlagStatus RobotRunService_InitComplete_Flage;
  extern FlagStatus Communication_InitComplete_Flage;

  memset(USART1_TX_BUF, 0, 256); //清零发送数据数组
  memset(TEXT_TO_SEND, 0, 250); //清零发送数据数组
  TransportProtocol_Init(&TransportProtocol,USART1_TX_BUF,Checksum_XOR); //初始化传输协议
	
  Date_pose=TEXT_TO_SEND; //指向发送数据数组首地址
  Communication_InitComplete_Flage = SET;
	
  while((DriverService_InitComplete_Flage == RESET) 
          || (RobotRunService_InitComplete_Flage == RESET) 
          || (Communication_InitComplete_Flage == RESET))
         
    {
         vTaskDelay(1);	//休眠1ms
    }
		
  while(1)
    {
		Update_TextToSend(Date_pose,&Length_TEXT_TO_SEND);	//更新发送数据
		TransportProtocol_Manager.Packed();   //打包数据	
			
		vTaskSuspendAll();//调度锁
		SendDateToUSART1(TransportProtocol_Manager.Buf,TransportProtocol_Manager.FrameTotalLength);	 //向算法板发送数据
		xTaskResumeAll ();
			
	  vTaskDelay(3); //延时3MS
    }
}
