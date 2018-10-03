#include "encoder.h"
#include "delay.h"
#include "protocol.h"
#include "MotorControl.h"
#include "DriversMain.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "RobotRunControlMain.h"
#include "GPIO_Config.h"

volatile unsigned long long  sys_time=0;     //����Ϊ����ϵͳ��ϵͳʱ��
u16 CCR1_Val = 99;//��ʱ���ļ������塣��ʱ0.1ms ����99+1��*��71+1����/(72*1000000)=100us=0.1ms      
unsigned short gsLeft_Pulse;  
unsigned short gsRight_Pulse;

u32   right_lasttime ;   //��¼������һ��ʱ��
u32   left_lasttime ;    //��¼������һ��ʱ��

#define READ_RIGHT_WHEEL_PULSE (GPIO_ReadInputDataBit(MR_PG))   //��ȡ��������PG
#define READ_LEFT_WHEEL_PULSE (GPIO_ReadInputDataBit(ML_PG))   //��ȡ��������PG

/*************************************************
��������void init_time(void)
���ܣ���ʼ����ʱ��2��ʹ����Ϊϵͳʱ�ӡ�
���룺��
*************************************************/
void init_time(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = CCR1_Val;          
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  	
  TIM_Cmd(TIM2, ENABLE);
  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update , ENABLE);
	
  //��ʼ��NVIC�ж����ȼ�����
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;					 //TIM2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;				 //��ռ���ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  					 //�����ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 //IRQͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure);										 //��ʼ��NVIC
}

/******************************************************************
���ܣ�������������
���ߣ�PA6����������
	  PC6���ҳ�������
******************************************************************/ 
	
FlagStatus TimeOutfalg=RESET;
void TIM2_IRQHandler(void)
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus=portSET_INTERRUPT_MASK_FROM_ISR();
	static u8 r_flag = 0;
    static u8 l_flag = 0; 
	 
	sys_time++;
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		
	 if((ReceiveSpeedTimeOut++)< 30000)  //��ʱ3S
	 {	  
		  TimeOutfalg=RESET;
	 }
	 else
	 {
	  TimeOutfalg=SET;
	 }
   //�����������
    switch(l_flag) 
	 {
		 case 0:	
		  if(READ_LEFT_WHEEL_PULSE == Bit_RESET)			//Bit_RESET = 0	        //PA6   (�����ٶ�)
		  {
				MOTORDIR leftdir;
				GetLeftDirection(&leftdir);
				if(leftdir==DIRFRONT)
				gsLeft_Pulse++;
				else if(leftdir==DIRBACK)
				gsLeft_Pulse--; 
				 l_flag = 1;
           }
			break;
		case 1:
			 if(READ_LEFT_WHEEL_PULSE!= Bit_RESET)
			 {
			  l_flag = 0;
			 }
			 break;
		 default:
			  l_flag = 0;
			 break;
	  }
	 //�����������
	 switch(r_flag)
	 {
		 case 0:		
			 if(READ_RIGHT_WHEEL_PULSE == Bit_RESET)	  //Bit_RESET = 0		 //PC0   (�����ٶ�)
			 {
				MOTORDIR rightdir;
				GetRightDirection(&rightdir);
				if(rightdir==DIRFRONT)
				gsRight_Pulse++; 
				else if(rightdir==DIRBACK)
				gsRight_Pulse--;
				r_flag = 1;
			 }
		   break;
		 case 1:
			 if(READ_RIGHT_WHEEL_PULSE != Bit_RESET)
			 {
			  r_flag = 0;
			 }
			 break;
		 default:
			  r_flag = 0;
			 break;	 
     }	 		 
 }

	 TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); 
	 portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
}


//�������̼���
void GetLeftInterruptCount(u16 *left_count)
{

  	 *left_count = gsLeft_Pulse;
}

//�������̼���
void GetRightInterruptCount(u16 *right_count)
{
  
   	*right_count = gsRight_Pulse;

}


 
