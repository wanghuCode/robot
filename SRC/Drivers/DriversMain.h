#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
#include "DriversMain.h"
#include "protocol.h"


struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};


extern u16 gsTimeStamp; //ʱ���ʴ�
extern  u8 g_Groyreaddataflag; //���������ݶ�����־λ
extern u32 oldsystime;

void Read_Encoder_Buffer(unsigned short *leftcoder,unsigned short *rightcoder,unsigned short *timestamp);
void GetALMState(void);
void GetGYROData(GYRO_Z_st *gyroangledata,GYRO_Original_Data_st *gyrooriginaldata);
void CopeSerial3Data(unsigned char ucData);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);	 			
u16 Timestampcalc(u32 *oldtime,u32 newtime);
u8 GetBumperState(void); //��ȡ��ײ��������״̬
#endif
