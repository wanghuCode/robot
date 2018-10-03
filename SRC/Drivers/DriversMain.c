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

u32 oldsystime; //记录上一系统时间
u16 gsTimeStamp; //时间邮戳
static u8 WWDG_CNT=0x7f;  //看门狗窗口值
/**************************************************************
程序功能：IO口反转
输入： 无
输出： 无
说明： 
**************************************************************/
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR ^= GPIO_Pin;
}
/**************************************************************
程序功能：喂狗
输入： 无
输出： 无
说明： 
**************************************************************/
void WWDG_Set_Counter(u8 cnt)
{
    WWDG_Enable(cnt);       
}

/**************************************************************
程序功能：看门狗中断优先级
输入： 无
输出： 无
说明： 
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
程序功能：看门狗初始化
输入： 无
输出： 无
说明： 
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
程序功能：看门狗中断
输入： 无
输出： 无
说明： 
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
程序功能：串口3中断调用函数，串口每收到一个数据，调用一次这个函数。
输入： 无
输出： 无
说明： 
**************************************************************/
void CopeSerial3Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{		
		switch(ucRxBuffer[1])
		{
      //memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}
/**************************************************************
程序功能：获得GY901的数据
输入： 无
输出： 无
说明： 
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
	gyrooriginaldata->RGYRO_DATA[0][j-1] = gyrooriginaldata->RGYRO_DATA[0][j]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[1][j-1] = gyrooriginaldata->RGYRO_DATA[1][j]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[2][j-1] = gyrooriginaldata->RGYRO_DATA[2][j]; //原始陀螺仪数据
	}
	gyrooriginaldata->RGYRO_DATA[0][2] = gGyro[0]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[1][2] = gGyro[1]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[2][2] = gGyro[2]; //原始陀螺仪数据

	gyroangledata->Angle=(short)(100*gAngle[2]); //将偏航角赋值给角度 
	gyroangledata->Angle_rate=(u16)(gGyro[2]);  //角速度输出  

}
	
/**************************************************************
程序功能：获得MPU6050的数据
输入： 无
输出： 无
说明： 
**************************************************************/
float angle[3]={0};//融合后欧拉角
short gyro[3]={0}; //原始陀螺仪值
short accel[3]={0}; //原始加速度值
void GetGYROData(GYRO_Z_st *gyroangledata,GYRO_Original_Data_st* gyrooriginaldata)
{   	 

	short j;
	GetCollectData(angle,gyro,accel,3);	  //获得陀螺仪的数据
	
	for (j = 1; j < 3; j++)
	{  
	gyrooriginaldata->RGYRO_DATA[0][j-1] = gyrooriginaldata->RGYRO_DATA[0][j]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[1][j-1] = gyrooriginaldata->RGYRO_DATA[1][j]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[2][j-1] = gyrooriginaldata->RGYRO_DATA[2][j]; //原始陀螺仪数据
	}
	gyrooriginaldata->RGYRO_DATA[0][2] = gyro[0]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[1][2] = gyro[1]; //原始陀螺仪数据
	gyrooriginaldata->RGYRO_DATA[2][2] = gyro[2]; //原始陀螺仪数据

	gyroangledata->Angle=(short)(100*angle[2]); //将偏航角赋值给角度 
	gyroangledata->Angle_rate=(u16)(gyro[2]);  //角速度输出   
}

/**************************************************************
程序功能：计算时间邮戳
输入： 无
输出： 无
说明： 
**************************************************************/
u16 Timestampcalc(u32 *oldtime,u32 newtime) //计算时间邮戳
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
程序功能：获取基本传感器数据
输入： 无
输出： 无
说明： 
**************************************************************/
void GetBasicSensorData(BasicSensorData_st *basicSensor) //获取基本传感器数据
{
  gsTimeStamp+= Timestampcalc(&oldsystime,sys_time);	 //时间邮戳
	basicSensor->TimeStamp=gsTimeStamp;  //时间邮戳
	basicSensor->Left_PWM=abs(gsLETF_MotorSpeed)/8;  //左轮速度反馈
	basicSensor->Right_PWM=abs(gsRIGHT_MotorSpeed)/8; //右轮速度反馈
	
	vTaskSuspendAll();//调度锁
	basicSensor->Right_encoder += GetCount(RMotor);
	basicSensor->Left_encoder += GetCount(LMotor);//左右码盘计数
	xTaskResumeAll ();
}

/**************************************************************
程序功能：APP_DriverService_Task任务函数
输入： 无
输出： 无
说明： 
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
         vTaskDelay(1);	//休眠1ms
    }
	
	WWDG_Init(0X7F,0X5F,WWDG_Prescaler_4);  //看门狗初始化
  	while(1)
    {       		
		GetBasicSensorData(&BasicSensor);   //获取基本传感器数据
		wwdgcont=20;	//看门狗计数
		#ifdef USE_GY901
			GetGYROFromUsart3(&GYRO_Angle,&GYRO_OriginalData);  //读取GY901数据
	    #endif	
		
		#ifdef USE_MPU6050
		    GetGYROData(&GYRO_Angle,&GYRO_OriginalData); //读取MPU6050数据
		#endif	
		if((ledtimecount++)==100) {ledtimecount=0;GPIO_ToggleBits(IO_LEDCPURUN);}   //定时100MS 	LED灯翻转一次
		vTaskDelay(3);
    }
}   

