#ifndef _MOTORCONTROL_H
#define _MOTORCONTROL_H

#include "sys.h"

typedef enum
{
   DIRNULL=0, 
	 DIRFRONT=1, //�����ת
	 DIRBACK=2  //�����ת
}MOTORDIR;

typedef enum
{
	MOTOR_DISABLE=0,//�������
	MOTOR_ENABLE= 1 //���ʹ��
}MOTOREN;


typedef enum
{
	BK_DISABLE=0,//�������
	BK_ENABLE= 1 //���ɲ��
}MOTORBK;


void SetMotorDir(MOTORDIR leftwheel,MOTORDIR rightwheel);
void SetMotorEn(MOTOREN leftwheel,MOTOREN rightwheel);
void SetMotorBk(MOTORBK leftwheel,MOTORBK rightwheel);
void Init_PWM(void);
void GetLeftDirection(MOTORDIR *leftdirection);
void GetRightDirection(MOTORDIR *rightdirection);

extern int g_leftdirection, g_rightdirection;
extern int g_leftspeed, g_rightspeed;

extern MOTORDIR LefWheeltDIR; //��ȡ���ַ���
extern MOTORDIR RightWheelDIR; //��ȡ���ַ���;
extern FlagStatus Left_TurningFlag;
extern FlagStatus Right_TurningFlag;
#endif

