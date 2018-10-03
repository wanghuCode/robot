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

volatile unsigned long long  sys_time=0;     //定义为整个系统的系统时钟
u16 CCR1_Val = 99;//定时器的计数脉冲。定时0.1ms （（99+1）*（71+1））/(72*1000000)=100us=0.1ms      
unsigned short gsLeft_Pulse;  
unsigned short gsRight_Pulse;

u32   right_lasttime ;   //记录右轮上一次时间
u32   left_lasttime ;    //记录左轮上一次时间

#define READ_RIGHT_WHEEL_PULSE (GPIO_ReadInputDataBit(MR_PG))   //读取右轮脉冲PG
#define READ_LEFT_WHEEL_PULSE (GPIO_ReadInputDataBit(ML_PG))   //读取左轮脉冲PG

/*************************************************
函数名：void init_time(void)
功能：初始化定时器2，使其作为系统时钟。
输入：无
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
	
  //初始化NVIC中断优先级分组
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;					 //TIM2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;				 //先占优先级0级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  					 //从优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 //IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);										 //初始化NVIC
}

/******************************************************************
功能：码盘驱动代码
接线：PA6是左车轮码盘
	  PC6是右车轮码盘
******************************************************************/ 
	
FlagStatus TimeOutfalg=RESET;
void TIM2_IRQHandler(void)
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus=portSET_INTERRUPT_MASK_FROM_ISR();
	static u8 r_flag = 0;
    static u8 l_flag = 0; 
	 
	sys_time++;
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{
		
	 if((ReceiveSpeedTimeOut++)< 30000)  //定时3S
	 {	  
		  TimeOutfalg=RESET;
	 }
	 else
	 {
	  TimeOutfalg=SET;
	 }
   //左轮脉冲计数
    switch(l_flag) 
	 {
		 case 0:	
		  if(READ_LEFT_WHEEL_PULSE == Bit_RESET)			//Bit_RESET = 0	        //PA6   (左轮速度)
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
	 //右轮脉冲计数
	 switch(r_flag)
	 {
		 case 0:		
			 if(READ_RIGHT_WHEEL_PULSE == Bit_RESET)	  //Bit_RESET = 0		 //PC0   (右轮速度)
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


//左轮码盘计数
void GetLeftInterruptCount(u16 *left_count)
{

  	 *left_count = gsLeft_Pulse;
}

//右轮码盘计数
void GetRightInterruptCount(u16 *right_count)
{
  
   	*right_count = gsRight_Pulse;

}


 
