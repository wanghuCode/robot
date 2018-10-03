#ifndef _MOTORCONTROL_H
#define _MOTORCONTROL_H

#include "sys.h"

typedef enum
{
   DIRNULL=0, 
	 DIRFRONT=1, //电机正转
	 DIRBACK=2  //电机反转
}MOTORDIR;

typedef enum
{
	MOTOR_DISABLE=0,//电机禁能
	MOTOR_ENABLE= 1 //电机使能
}MOTOREN;


typedef enum
{
	BK_DISABLE=0,//电机启动
	BK_ENABLE= 1 //电机刹车
}MOTORBK;


void SetMotorDir(MOTORDIR leftwheel,MOTORDIR rightwheel);
void SetMotorEn(MOTOREN leftwheel,MOTOREN rightwheel);
void SetMotorBk(MOTORBK leftwheel,MOTORBK rightwheel);
void Init_PWM(void);
void GetLeftDirection(MOTORDIR *leftdirection);
void GetRightDirection(MOTORDIR *rightdirection);

extern int g_leftdirection, g_rightdirection;
extern int g_leftspeed, g_rightspeed;

extern MOTORDIR LefWheeltDIR; //获取左轮方向
extern MOTORDIR RightWheelDIR; //获取右轮方向;
extern FlagStatus Left_TurningFlag;
extern FlagStatus Right_TurningFlag;
#endif

