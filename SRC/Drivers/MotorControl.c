#include "MotorControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GPIO_Config.h"
#include "MotorControl.h"
/**************************************************************
程序名称：void  Init_PWM(void)
描述：初始化pwm驱动程序，定时器4 配置为pwm1模式，使用
      边沿对齐模式向上计数
输入：无
输出：无
**************************************************************/
int g_leftdirection, g_rightdirection;
int g_leftspeed, g_rightspeed;
MOTORDIR LefWheeltDIR; //获取左轮方向
MOTORDIR RightWheelDIR; //获取右轮方向

FlagStatus Left_TurningFlag=RESET;
FlagStatus Right_TurningFlag=RESET;
MOTORDIR Last_LefWheeltDIR;
MOTORDIR Last_RightWheelDIR;
 
////初始化PWM，通过PWM的占空比对电机进行调速
//void  Init_PWM(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE);	
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //TIM1完全重映射

//  ///////////////////////下面驱动四个左右轮的控制信号//////////////
//	  /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 999;   //PWM重装值 ，就是ARR参数    PWM频率=72000/(999+1)=72Khz / (35+1)=2Khz 频率低控制精度高    
//    TIM_TimeBaseStructure.TIM_Prescaler = 10;   //时钟分频系数 ,就是PSC参数   
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;   //时钟分频因子，在CR1寄存器中
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数方式，在CR1寄存器中  
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//		
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM_OCMode_PWM1 PWM1/PWM2
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//通道的工作模式
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//输出极性高
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//空闲时的电平为有效电平

//	TIM_OCInitStructure.TIM_Pulse = 0;//脉冲有效宽度
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);//初始化TIM1通道2
//	
//	TIM_OCInitStructure.TIM_Pulse = 0;//脉冲有效宽度
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);//初始化TIM1通道2
//	
//	TIM_Cmd(TIM1, ENABLE);
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	
//    Last_LefWheeltDIR=LefWheeltDIR;
//	Last_RightWheelDIR=RightWheelDIR;
//	
//   
//}

//马达方向控制输出,与AGV小车方向有关 说明：高电平正转
void SetMotorDir(MOTORDIR leftwheel,MOTORDIR rightwheel)
{
	
		if(leftwheel==DIRFRONT) //左轮前转
		{
			GPIO_SetBits(ML_FR); 
			//GPIO_ResetBits(ML_FR);
			LefWheeltDIR=DIRFRONT;
		}
		else   //左轮后转
		{
			GPIO_ResetBits(ML_FR);
			//GPIO_SetBits(ML_FR); 
			LefWheeltDIR=DIRBACK;
		}
		if(rightwheel==DIRFRONT) //右轮前转
		{
			//GPIO_SetBits(MR_FR);
			GPIO_ResetBits(MR_FR);
			RightWheelDIR=DIRFRONT;
		}
		else  //右轮后转
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

////设置马达使能输出
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


////设置马达刹车输出
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

/**********************获得左轮的方向*********************/
void GetLeftDirection(MOTORDIR *leftdirection)
{

	*leftdirection = LefWheeltDIR;

}

/**********************获得右轮的方向*********************/
void GetRightDirection(MOTORDIR *rightdirection)
{

	*rightdirection = RightWheelDIR;

}

/**********************获得车轮的速度*********************/
void GetMotorSpeed(int *leftspeed, int *rightspeed)
{

	*leftspeed = g_leftspeed;
	*rightspeed = g_rightspeed;

}






