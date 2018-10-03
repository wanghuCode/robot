#include "MotorControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GPIO_Config.h"
#include "MotorControl.h"
/**************************************************************
�������ƣ�void  Init_PWM(void)
��������ʼ��pwm�������򣬶�ʱ��4 ����Ϊpwm1ģʽ��ʹ��
      ���ض���ģʽ���ϼ���
���룺��
�������
**************************************************************/
int g_leftdirection, g_rightdirection;
int g_leftspeed, g_rightspeed;
MOTORDIR LefWheeltDIR; //��ȡ���ַ���
MOTORDIR RightWheelDIR; //��ȡ���ַ���

FlagStatus Left_TurningFlag=RESET;
FlagStatus Right_TurningFlag=RESET;
MOTORDIR Last_LefWheeltDIR;
MOTORDIR Last_RightWheelDIR;
 
////��ʼ��PWM��ͨ��PWM��ռ�ձȶԵ�����е���
//void  Init_PWM(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE);	
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //TIM1��ȫ��ӳ��

//  ///////////////////////���������ĸ������ֵĿ����ź�//////////////
//	  /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 999;   //PWM��װֵ ������ARR����    PWMƵ��=72000/(999+1)=72Khz / (35+1)=2Khz Ƶ�ʵͿ��ƾ��ȸ�    
//    TIM_TimeBaseStructure.TIM_Prescaler = 10;   //ʱ�ӷ�Ƶϵ�� ,����PSC����   
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;   //ʱ�ӷ�Ƶ���ӣ���CR1�Ĵ�����
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //������ʽ����CR1�Ĵ�����  
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//		
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM_OCMode_PWM1 PWM1/PWM2
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ͨ���Ĺ���ģʽ
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//������Ը�
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//����ʱ�ĵ�ƽΪ��Ч��ƽ

//	TIM_OCInitStructure.TIM_Pulse = 0;//������Ч���
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);//��ʼ��TIM1ͨ��2
//	
//	TIM_OCInitStructure.TIM_Pulse = 0;//������Ч���
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);//��ʼ��TIM1ͨ��2
//	
//	TIM_Cmd(TIM1, ENABLE);
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	
//    Last_LefWheeltDIR=LefWheeltDIR;
//	Last_RightWheelDIR=RightWheelDIR;
//	
//   
//}

//��﷽��������,��AGVС�������й� ˵�����ߵ�ƽ��ת
void SetMotorDir(MOTORDIR leftwheel,MOTORDIR rightwheel)
{
	
		if(leftwheel==DIRFRONT) //����ǰת
		{
			GPIO_SetBits(ML_FR); 
			//GPIO_ResetBits(ML_FR);
			LefWheeltDIR=DIRFRONT;
		}
		else   //���ֺ�ת
		{
			GPIO_ResetBits(ML_FR);
			//GPIO_SetBits(ML_FR); 
			LefWheeltDIR=DIRBACK;
		}
		if(rightwheel==DIRFRONT) //����ǰת
		{
			//GPIO_SetBits(MR_FR);
			GPIO_ResetBits(MR_FR);
			RightWheelDIR=DIRFRONT;
		}
		else  //���ֺ�ת
		{
			//GPIO_ResetBits(MR_FR);
			GPIO_SetBits(MR_FR);
			RightWheelDIR=DIRBACK;
		}
		
		if(Last_LefWheeltDIR!=LefWheeltDIR)
		{
			Last_LefWheeltDIR=LefWheeltDIR;
			Left_TurningFlag=SET;
		}
		
		if(Last_RightWheelDIR!=RightWheelDIR)
		{
			Last_RightWheelDIR=RightWheelDIR;
			Right_TurningFlag=SET;
		}
			
}

////�������ʹ�����
//void SetMotorEn(MOTOREN leftwheel,MOTOREN rightwheel)
//{
//	 if(leftwheel==MOTOR_ENABLE)
//	{
//		GPIO_ResetBits(ML_EN);
//	}
//	else
//	{
//		GPIO_SetBits(ML_EN);
//	}
//	if(rightwheel==MOTOR_ENABLE)
//	{
//		GPIO_ResetBits(MR_EN);
//	}
//	else
//	{
//  	GPIO_SetBits(MR_EN);
//	}
//}


////�������ɲ�����
//void SetMotorBk(MOTORBK leftwheel,MOTORBK rightwheel)
//{
//	 if(leftwheel==BK_ENABLE)
//	{
//		GPIO_ResetBits(ML_BK);
//	}
//	else
//	{
//		GPIO_SetBits(ML_BK);
//	}
//	if(rightwheel==BK_ENABLE)
//	{
//		GPIO_ResetBits(MR_BK);
//	}
//	else
//	{
//  	GPIO_SetBits(MR_BK);
//	}
//}

/**********************������ֵķ���*********************/
void GetLeftDirection(MOTORDIR *leftdirection)
{

	*leftdirection = LefWheeltDIR;

}

/**********************������ֵķ���*********************/
void GetRightDirection(MOTORDIR *rightdirection)
{

	*rightdirection = RightWheelDIR;

}

/**********************��ó��ֵ��ٶ�*********************/
void GetMotorSpeed(int *leftspeed, int *rightspeed)
{

	*leftspeed = g_leftspeed;
	*rightspeed = g_rightspeed;

}






