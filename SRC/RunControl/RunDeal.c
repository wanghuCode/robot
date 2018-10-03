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

float FastSpeed=0;  //����ת�뾶�ľ���ֵ����1mmʱ���·����ٶ�Ϊfastspeed,ͨ���㷨��õ�Ϊslowspeed.
float SlowSpeed=0; //����ת�뾶�ľ���ֵ����1mmʱ���·����ٶ�Ϊfastspeed,ͨ���㷨��õ�Ϊslowspeed.

extern unsigned short gsLETF_MotorSpeed; //�����ٶ�
extern unsigned short gsRIGHT_MotorSpeed; //�����ٶ�

//��ȡС���˶����� ���ٶ�
void GetRobotRunDIRSpeed(short runspeed,short radiusval) //��ȡС���˶����� 
{
    
	  short speed,radius;	
	  speed=runspeed;
	  radius=radiusval;	
	  if(radius==0)
		{
			if(speed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); //��������ת
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 else if(speed<0)
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); //�����ַ�ת
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
			 gsLETF_MotorSpeed = (unsigned short)abs(speed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(speed);
		}
		else if(radius==1)
		{
			if(speed>0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); //ԭ����ת
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
			 else if(speed<0)
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); //ԭ����ת
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
			 
			 gsLETF_MotorSpeed = (unsigned short)abs(speed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(speed);
		}
		else if(radius>1) //�뾶����1mmʱ������������ת
		{
			FastSpeed=speed; //����ǰʱSPEED����1�������ʱSPEEDС��1.
			SlowSpeed =((speed*(radius-b/2))/(radius+b/2));
			
		
			 gsLETF_MotorSpeed = (unsigned short)abs(SlowSpeed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(FastSpeed);
			
		if(SlowSpeed>0 && FastSpeed>0 )
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); //  ��ǰ���н������뾶>b/2ʱ��������ǰ��������ǰ
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 
		else if(SlowSpeed<0 && FastSpeed>0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); //  ��ǰ���н������뾶<b/2ʱ���������������ǰ
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
	  else  if(SlowSpeed<0 && FastSpeed<0 )
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); /// ������н������뾶>b/2ʱ����������������
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
			
		else if(SlowSpeed>0 && FastSpeed<0 )
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); //  ������н������뾶<b/2ʱ��������ǰ���������
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
		}
		else if(radius<-1)  //�뾶С��-1mmʱ������������ת
		{
		  SlowSpeed=speed;
			FastSpeed = ((speed*(radius+b/2))/(radius-b/2));
			
			 gsLETF_MotorSpeed = (unsigned short)abs(FastSpeed);
			 gsRIGHT_MotorSpeed = (unsigned short)abs(SlowSpeed);
			
		if(FastSpeed<0 && SlowSpeed<0) 
			 {
			   //SetMotorDir(DIRBACK,DIRBACK); // �Һ����н������뾶>b/2ʱ����������������
				 set_hub_motor_dir(BackwardR,BackwardL );
			 }
		else if(FastSpeed>0 && SlowSpeed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRFRONT); // ��ǰ���н������뾶>b/2ʱ��������ǰ��������ǰ
				 set_hub_motor_dir(ForwardR,ForwardL );
			 }
			 
		else if(FastSpeed>0 && SlowSpeed<0)
			 {
			   //SetMotorDir(DIRBACK,DIRFRONT); // �Һ����н������뾶<b/2ʱ���������������ǰ
				 set_hub_motor_dir(ForwardR,BackwardL );
			 }
	  else if(FastSpeed<0 && SlowSpeed>0)
			 {
			   //SetMotorDir(DIRFRONT,DIRBACK); // ��ǰ���н������뾶<b/2ʱ��������ǰ���������
				 set_hub_motor_dir(BackwardR,ForwardL );
			 }
		}				
}

/*******************************************************************************
* Function Name  : ��ȡ��ǰ�ĵ����PWMֵ
* Description    : 
* Input          : 
* Output         : None
* Return         : None
*******************************************************************************/
short gs_Left_PWM=0,gs_Right_PWM=0;//�����ֵ�PWMֵ
void GetCurrentPWM(short *left_pwm,short *right_pwm,unsigned short left_motorspeed,unsigned short right_motorspeed)
{
   
	//����PWM
	if(LefWheeltDIR==DIRFRONT)
	 {
	   *left_pwm=left_motorspeed;
	 }
	 else if(LefWheeltDIR==DIRBACK)
	 {
	  *left_pwm=-left_motorspeed;
	 }
	 
	 //����PWM
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
//* Function Name  : С���ٶ��������
//* Description    : 
//* Input          : 
//* Output         : None
//* Return         : None
//*******************************************************************************/
//FlagStatus gsReceiveSpeedTimeoutFlag=RESET; //����1�����ٶ���Ϣ��ʱ��־
//unsigned short L_speed,R_speed; //����������ٶ�
//u32 oldtime; //��¼��һϵͳʱ��
//void MotorSpeedOutput(unsigned short left_motorspeed,unsigned short right_motorspeed)
//{
//	/********
//	left_motorspeed,right_motorspeedΪռ�ձȷ��ӣ���ĸΪ1000
//	left_motorspeed,right_motorspeedΪռ�ձȷ��ӣ���ĸΪ1000
//	**********/
//	if(left_motorspeed > 1000)
//		left_motorspeed = 1000;		
//	if(right_motorspeed>1000)
//		right_motorspeed = 1000;

//	if(TimeOutfalg==SET)  //���3Sû�н��յ����ݣ����ɲ��
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

//	 TIM_SetCompare1(TIM1,left_motorspeed);  //���������ٶ�
//	 TIM_SetCompare2(TIM1,right_motorspeed);  //���������ٶ�
//	
//	 GetCurrentPWM(&gs_Left_PWM,&gs_Right_PWM,left_motorspeed,right_motorspeed); //��ȡ�����ֵ�ǰPWMֵ
//	
//}

///******************************************************
//  ����С���������ٶ�
//*******************************************************/
//void CalcWheelSpeed(unsigned short *leftmotorspeed,unsigned short *rightmotorspeed,short *runspeed,short *radiusval)
//{
//      
//	 float tmpval;
//	 float speed,radius;	
//	
//	 speed=*runspeed;   //С�����ٶ� MM/S
//	 radius=*radiusval;  //С����ת�뾶 ��λMM
// //����С�������ٶ�
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
//   if(radius==0 || radius==1)  //С��ֱ�� ��radius==0��  ��  ԭ����תʱ��radius==1���������ٶ�
//	 {
//	    if(speed>=0)
//			{
//				tmpval = speed/perimeter;
//				*rightmotorspeed = ((unsigned short)tmpval) * retarder/percent;	//����ռ�ձ�		    									 
//				*leftmotorspeed=*rightmotorspeed;	        //����ռ�ձ�
//			}
//			else 
//			{
//				speed = -speed;
//				tmpval = speed/perimeter;	
//				*rightmotorspeed = ((unsigned short)tmpval) * retarder/percent; //����ռ�ձ�
//				*leftmotorspeed=*rightmotorspeed;	   //����ռ�ձ�
//			}
//	}
//	else if(radius>1)//С����ʱ��ת��
//	{
//		FastSpeed=speed;
//		SlowSpeed = ((speed*(radius-b/2))/(radius+b/2));

//		tmpval = FastSpeed/perimeter;
//		*rightmotorspeed = (unsigned short)fabs(tmpval * retarder/percent); //����ռ�ձ�

//		tmpval = SlowSpeed/perimeter;
//		*leftmotorspeed  = (unsigned short)fabs(tmpval * retarder/percent);   //����ռ�ձ�
//							
//	}
//	else if(radius<-1) //С��˳ʱ��ת��
//	{
//		SlowSpeed=speed;
//		FastSpeed = ((speed*(radius+b/2))/(radius-b/2));  

//		tmpval = FastSpeed/perimeter;
//		*rightmotorspeed = (unsigned short)fabs(tmpval * retarder/percent); //����ռ�ձ�

//		tmpval = SlowSpeed/perimeter;
//		*leftmotorspeed  = (unsigned short)fabs(tmpval * retarder/percent);		 //����ռ�ձ� 	
//	}

//}	



