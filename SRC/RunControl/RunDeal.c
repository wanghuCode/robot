#include "RunDeal.h"
#include "MotorControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GPIO_Config.h"
#include "MotorControl.h"
#include "math.h"
#include "protocol.h"
#include "DriversMain.h"
#include "encoder.h"
#include "hub_motor.h"
#include "math.h"
#include "stdlib.h"

float FastSpeed=0;  //当旋转半径的绝对值大于1mm时，下发的速度为fastspeed,通过算法求得的为slowspeed.
float SlowSpeed=0; //当旋转半径的绝对值大于1mm时，下发的速度为fastspeed,通过算法求得的为slowspeed.

extern unsigned short gsLETF_MotorSpeed; //左轮速度
extern unsigned short gsRIGHT_MotorSpeed; //右轮速度

//获取小车运动方向 和速度
void GetRobotRunDIRSpeed(short runspeed,short radiusval) //获取小车运动方向 
{
    
	  short speed,radius;	
	  speed=runspeed;
	  radius=radiusval;	
	  if(radius==0)
		{
			if(speed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); //左右轮正转
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 else if(speed<0)
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); //左右轮反转
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
			 gsLETF_MotorSpeed = (unsigned short)abs(speed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(speed);
		}
		else if(radius==1)
		{
			if(speed>0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); //原地左转
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
			 else if(speed<0)
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); //原地右转
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
			 
			 gsLETF_MotorSpeed = (unsigned short)abs(speed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(speed);
		}
		else if(radius>1) //半径大于1mm时，机器人向左转
		{
			FastSpeed=speed; //向左前时SPEED大于1，向左后时SPEED小于1.
			SlowSpeed =((speed*(radius-b/2))/(radius+b/2));
			
		
			 gsLETF_MotorSpeed = (unsigned short)abs(SlowSpeed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(FastSpeed);
			
		if(SlowSpeed>0 && FastSpeed>0 )
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); //  左前向行进，当半径>b/2时，左轮向前，右轮向前
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 
		else if(SlowSpeed<0 && FastSpeed>0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); //  左前向行进，当半径<b/2时，左轮向后，右轮向前
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
	  else  if(SlowSpeed<0 && FastSpeed<0 )
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); /// 左后向行进，当半径>b/2时，左轮向后，右轮向后
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
			
		else if(SlowSpeed>0 && FastSpeed<0 )
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); //  左后向行进，当半径<b/2时，左轮向前，右轮向后
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
		}
		else if(radius<-1)  //半径小于-1mm时，机器人向右转
		{
		  SlowSpeed=speed;
			FastSpeed = ((speed*(radius+b/2))/(radius-b/2));
			
			 gsLETF_MotorSpeed = (unsigned short)abs(FastSpeed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(SlowSpeed);
			
		if(FastSpeed<0 && SlowSpeed<0) 
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); // 右后向行进，当半径>b/2时，左轮向后，右轮向后
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
		else if(FastSpeed>0 && SlowSpeed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); // 右前向行进，当半径>b/2时，左轮向前，右轮向前
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 
		else if(FastSpeed>0 && SlowSpeed<0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); // 右后向行进，当半径<b/2时，左轮向后，右轮向前
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
	  else if(FastSpeed<0 && SlowSpeed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); // 右前向行进，当半径<b/2时，左轮向前，右轮向后
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
		}				
}

/*******************************************************************************
* Function Name  : 获取当前的电机的PWM值
* Description    : 
* Input          : 
* Output         : None
* Return         : None
*******************************************************************************/
short gs_Left_PWM=0,gs_Right_PWM=0;//左右轮的PWM值
void GetCurrentPWM(short *left_pwm,short *right_pwm,unsigned short left_motorspeed,unsigned short right_motorspeed)
{
   
	//左轮PWM
	if(LefWheeltDIR==DIRFRONT)
	 {
	   *left_pwm=left_motorspeed;
	 }
	 else if(LefWheeltDIR==DIRBACK)
	 {
	  *left_pwm=-left_motorspeed;
	 }
	 
	 //右轮PWM
	 if(RightWheelDIR==DIRFRONT)
	 {
	   *right_pwm=right_motorspeed;
	 }
	 else if(RightWheelDIR==DIRBACK)
	 {
	  *right_pwm=-right_motorspeed;
	 }
	 
}

///*******************************************************************************
//* Function Name  : 小车速度输出控制
//* Description    : 
//* Input          : 
//* Output         : None
//* Return         : None
//*******************************************************************************/
//FlagStatus gsReceiveSpeedTimeoutFlag=RESET; //串口1接收速度信息超时标志
//unsigned short L_speed,R_speed; //左右轮输出速度
//u32 oldtime; //记录上一系统时间
//void MotorSpeedOutput(unsigned short left_motorspeed,unsigned short right_motorspeed)
//{
//	/********
//	left_motorspeed,right_motorspeed为占空比分子，分母为1000
//	left_motorspeed,right_motorspeed为占空比分子，分母为1000
//	**********/
//	if(left_motorspeed > 1000)
//		left_motorspeed = 1000;		
//	if(right_motorspeed>1000)
//		right_motorspeed = 1000;

//	if(TimeOutfalg==SET)  //如果3S没有接收到数据，电机刹车
//	{
//	    gsReceiveSpeedTimeoutFlag=SET;
//		gsRunSpeed=0;
//		gsRadiusVal=0;
//		SetMotorBk(BK_ENABLE,BK_ENABLE);
//	}	
//	else
//	{
//	  gsReceiveSpeedTimeoutFlag=RESET;
//	}
//	
//	if(gsReceiveSpeedTimeoutFlag!=SET)
//	{
//	
//	 if ((left_motorspeed != 0) && (right_motorspeed!=0))
//		{
//		  //run
//		  //SetMotorBk(BK_DISABLE,BK_DISABLE);
//		}
//	 else if((left_motorspeed == 0) && (right_motorspeed==0))
//		{
//		 //stop
//		 //SetMotorBk(BK_ENABLE,BK_ENABLE);
//		}		 		
//	}

//	 TIM_SetCompare1(TIM1,left_motorspeed);  //设置左轮速度
//	 TIM_SetCompare2(TIM1,right_motorspeed);  //设置右轮速度
//	
//	 GetCurrentPWM(&gs_Left_PWM,&gs_Right_PWM,left_motorspeed,right_motorspeed); //获取左右轮当前PWM值
//	
//}

///******************************************************
//  计算小车左右轮速度
//*******************************************************/
//void CalcWheelSpeed(unsigned short *leftmotorspeed,unsigned short *rightmotorspeed,short *runspeed,short *radiusval)
//{
//      
//	 float tmpval;
//	 float speed,radius;	
//	
//	 speed=*runspeed;   //小车线速度 MM/S
//	 radius=*radiusval;  //小车旋转半径 单位MM
// //限制小车轮子速度
//	if(speed>(perimeter*1000))
//   {					 
//		 speed=(perimeter*1000);
//   }
//	 	if(speed<(-(perimeter*1000)))
//   {					 
//		 speed=-(perimeter*1000);
//   }
//   
//   
//   if(radius==0 || radius==1)  //小车直走 （radius==0）  与  原点旋转时（radius==1）的轮子速度
//	 {
//	    if(speed>=0)
//			{
//				tmpval = speed/perimeter;
//				*rightmotorspeed = ((unsigned short)tmpval) * retarder/percent;	//右轮占空比		    									 
//				*leftmotorspeed=*rightmotorspeed;	        //左轮占空比
//			}
//			else 
//			{
//				speed = -speed;
//				tmpval = speed/perimeter;	
//				*rightmotorspeed = ((unsigned short)tmpval) * retarder/percent; //右轮占空比
//				*leftmotorspeed=*rightmotorspeed;	   //左轮占空比
//			}
//	}
//	else if(radius>1)//小车逆时针转动
//	{
//		FastSpeed=speed;
//		SlowSpeed = ((speed*(radius-b/2))/(radius+b/2));

//		tmpval = FastSpeed/perimeter;
//		*rightmotorspeed = (unsigned short)fabs(tmpval * retarder/percent); //右轮占空比

//		tmpval = SlowSpeed/perimeter;
//		*leftmotorspeed  = (unsigned short)fabs(tmpval * retarder/percent);   //左轮占空比
//							
//	}
//	else if(radius<-1) //小车顺时针转动
//	{
//		SlowSpeed=speed;
//		FastSpeed = ((speed*(radius+b/2))/(radius-b/2));  

//		tmpval = FastSpeed/perimeter;
//		*rightmotorspeed = (unsigned short)fabs(tmpval * retarder/percent); //右轮占空比

//		tmpval = SlowSpeed/perimeter;
//		*leftmotorspeed  = (unsigned short)fabs(tmpval * retarder/percent);		 //左轮占空比 	
//	}

//}	



