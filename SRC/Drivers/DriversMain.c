#include "DriversMain.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "protocol.h"
#include "MotorControl.h"
#include "mpu6050manage.h"
#include "sys.h"
#include "RobotRunControlMain.h"
#include "timers.h"
#include "GPIO_Config.h"
#include "RunDeal.h"
#include "dma.h"
#include "string.h"
#include "delay.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "CarParameterSet.h"
#include "hub_motor.h"

u32 oldsystime; //��¼��һϵͳʱ��
u16 gsTimeStamp; //ʱ���ʴ�
static u8 WWDG_CNT=0x7f;  //���Ź�����ֵ
/**************************************************************
�����ܣ�IO�ڷ�ת
���룺 ��
����� ��
˵���� 
**************************************************************/
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR ^= GPIO_Pin;
}
/**************************************************************
�����ܣ�ι��
���룺 ��
����� ��
˵���� 
**************************************************************/
void WWDG_Set_Counter(u8 cnt)
{
    WWDG_Enable(cnt);       
}

/**************************************************************
�����ܣ����Ź��ж����ȼ�
���룺 ��
����� ��
˵���� 
**************************************************************/
void WWDG_NVIC_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;              
    NVIC_Init(&NVIC_InitStructure);                            
}
/**************************************************************
�����ܣ����Ź���ʼ��
���룺 ��
����� ��
˵���� 
**************************************************************/
void WWDG_Init(u8 tr, u8 wr, u32 fprer)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);  

    WWDG_CNT=tr&WWDG_CNT;            

    WWDG_SetPrescaler(fprer);         

    WWDG_SetWindowValue(wr);         

    WWDG_Enable(WWDG_CNT);                        

    WWDG_ClearFlag();                 

    WWDG_NVIC_Init();                 

    WWDG_EnableIT();                
}
/**************************************************************
�����ܣ����Ź��ж�
���룺 ��
����� ��
˵���� 
**************************************************************/
u16 wwdgcont;
void WWDG_IRQHandler(void)
{
    
	wwdgcont--;
	if(wwdgcont>1)   
   {
	   WWDG_SetCounter(WWDG_CNT);    
   }
	  
    WWDG_ClearFlag();               
	WWDG_ClearFlag();              
}

struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	    stcAngle;
struct SMag 		stcMag;

/**************************************************************
�����ܣ�����3�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
���룺 ��
����� ��
˵���� 
**************************************************************/
void CopeSerial3Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{		
		switch(ucRxBuffer[1])
		{
      //memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݹ�ͬ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}
/**************************************************************
�����ܣ����GY901������
���룺 ��
����� ��
˵���� 
**************************************************************/
 float gAngle[3];
 short gAac[3], gGyro[3];
 void GetGYROFromUsart3( GYRO_Z_st *gyroangledata,GYRO_Original_Data_st* gyrooriginaldata)
{  
	unsigned char j;	
	gGyro[0] = stcGyro.w[0]/32768.0*2000;    
	gGyro[1] = stcGyro.w[1]/32768.0*2000;      
	gGyro[2] = stcGyro.w[2]/32768.0*2000;     

	gAngle[2] =  stcAngle.Angle[2]/32768.0*180;   

	for (j = 1; j < 3; j++)
	{  
	gyrooriginaldata->RGYRO_DATA[0][j-1] = gyrooriginaldata->RGYRO_DATA[0][j]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[1][j-1] = gyrooriginaldata->RGYRO_DATA[1][j]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[2][j-1] = gyrooriginaldata->RGYRO_DATA[2][j]; //ԭʼ����������
	}
	gyrooriginaldata->RGYRO_DATA[0][2] = gGyro[0]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[1][2] = gGyro[1]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[2][2] = gGyro[2]; //ԭʼ����������

	gyroangledata->Angle=(short)(100*gAngle[2]); //��ƫ���Ǹ�ֵ���Ƕ� 
	gyroangledata->Angle_rate=(u16)(gGyro[2]);  //���ٶ����  

}
	
/**************************************************************
�����ܣ����MPU6050������
���룺 ��
����� ��
˵���� 
**************************************************************/
float angle[3]={0};//�ںϺ�ŷ����
short gyro[3]={0}; //ԭʼ������ֵ
short accel[3]={0}; //ԭʼ���ٶ�ֵ
void GetGYROData(GYRO_Z_st *gyroangledata,GYRO_Original_Data_st* gyrooriginaldata)
{   	 

	short j;
	GetCollectData(angle,gyro,accel,3);	  //��������ǵ�����
	
	for (j = 1; j < 3; j++)
	{  
	gyrooriginaldata->RGYRO_DATA[0][j-1] = gyrooriginaldata->RGYRO_DATA[0][j]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[1][j-1] = gyrooriginaldata->RGYRO_DATA[1][j]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[2][j-1] = gyrooriginaldata->RGYRO_DATA[2][j]; //ԭʼ����������
	}
	gyrooriginaldata->RGYRO_DATA[0][2] = gyro[0]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[1][2] = gyro[1]; //ԭʼ����������
	gyrooriginaldata->RGYRO_DATA[2][2] = gyro[2]; //ԭʼ����������

	gyroangledata->Angle=(short)(100*angle[2]); //��ƫ���Ǹ�ֵ���Ƕ� 
	gyroangledata->Angle_rate=(u16)(gyro[2]);  //���ٶ����   
}

/**************************************************************
�����ܣ�����ʱ���ʴ�
���룺 ��
����� ��
˵���� 
**************************************************************/
u16 Timestampcalc(u32 *oldtime,u32 newtime) //����ʱ���ʴ�
{
	u16 time;
	u32 sys_newtime;
	sys_newtime=newtime;
	newtime=sys_newtime/10;
	if((*oldtime)<newtime)
	  time=(u16)(newtime-(*oldtime));
	else
	  time=(u16)(4294967295-(*oldtime)+newtime+1);
	*oldtime=(u32)newtime;
	return time;
}

/**************************************************************
�����ܣ���ȡ��������������
���룺 ��
����� ��
˵���� 
**************************************************************/
void GetBasicSensorData(BasicSensorData_st *basicSensor) //��ȡ��������������
{
  gsTimeStamp+= Timestampcalc(&oldsystime,sys_time);	 //ʱ���ʴ�
	basicSensor->TimeStamp=gsTimeStamp;  //ʱ���ʴ�
	basicSensor->Left_PWM=abs(gsLETF_MotorSpeed)/8;  //�����ٶȷ���
	basicSensor->Right_PWM=abs(gsRIGHT_MotorSpeed)/8; //�����ٶȷ���
	
	vTaskSuspendAll();//������
	basicSensor->Right_encoder += GetCount(RMotor);
	basicSensor->Left_encoder += GetCount(LMotor);//�������̼���
	xTaskResumeAll ();
}

/**************************************************************
�����ܣ�APP_DriverService_Task������
���룺 ��
����� ��
˵���� 
**************************************************************/
void APP_DriverService_Task(void *pvParameters)
{
    u16 ledtimecount=0; 
    extern FlagStatus DriverService_InitComplete_Flage;
    extern FlagStatus RobotRunService_InitComplete_Flage;
    extern FlagStatus Communication_InitComplete_Flage;

	DriverService_InitComplete_Flage = SET;
    while((DriverService_InitComplete_Flage == RESET) 
          || (RobotRunService_InitComplete_Flage == RESET) 
		
          || (Communication_InitComplete_Flage == RESET))
         
    {
         vTaskDelay(1);	//����1ms
    }
	
	WWDG_Init(0X7F,0X5F,WWDG_Prescaler_4);  //���Ź���ʼ��
  	while(1)
    {       		
		GetBasicSensorData(&BasicSensor);   //��ȡ��������������
		wwdgcont=20;	//���Ź�����
		#ifdef USE_GY901
			GetGYROFromUsart3(&GYRO_Angle,&GYRO_OriginalData);  //��ȡGY901����
	    #endif	
		
		#ifdef USE_MPU6050
		    GetGYROData(&GYRO_Angle,&GYRO_OriginalData); //��ȡMPU6050����
		#endif	
		if((ledtimecount++)==100) {ledtimecount=0;GPIO_ToggleBits(IO_LEDCPURUN);}   //��ʱ100MS 	LED�Ʒ�תһ��
		vTaskDelay(3);
    }
}   

