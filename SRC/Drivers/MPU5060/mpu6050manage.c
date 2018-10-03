#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050manage.h"
#include "delay.h"
#include <stdio.h>


/**************************************************************
程序功能：MPU6050初始化
输入： 无
输出： 整形
说明： 
**************************************************************/
int InitMpu6050(void)
{
   int sum=0;
	
	while(MPU_Init()!=0)
	{
	   sum++;
	  if(sum>5)
		{
	    sum=0;
			break;
		}
		delay_ms(2);
	}

	while(mpu_dmp_init())
	{
	  sum++;
	  if(sum>5)
	    break;
		delay_ms(2);
	
	}
  MPU6050_InitGyro_Offset();
	
	if(sum>5)	return -1;	
	
	return 0;
}


/**************************************************************
程序功能：读取MPU6050的数据
输入： 无
输出： 整形
说明： 
**************************************************************/
float g_angle[3];
short g_aac[3], g_gyro[3];
void GetCollectData( float *angle, short *gyro, short *accel, int len)
{
	
    if(mpu_dmp_get_data(&angle[0],&angle[1],&angle[2])==0){ 			
	    MPU_Get_Accelerometer(&accel[0],&accel[1],&accel[2]);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyro[0],&gyro[1],&gyro[2]);	//得到陀螺仪数据			
	}

}

/**************************************************************
程序功能：角速度去漂移
输入： 无
输出： 无
说明： 
**************************************************************/
short Gx_offset=0,Gy_offset=0,Gz_offset=0;
void MPU6050_InitGyro_Offset(void)
{
int	tempgx=0,tempgy=0,tempgz=0;
short temp[3];
int i;
	
Gx_offset=0;
Gy_offset=0;
Gz_offset=0;
 for(i=0;i<100;i++){
	delay_us(200);
	 
	MPU_Get_Gyroscope(&temp[0],&temp[1],&temp[2]);
	tempgx+= temp[0];
	tempgy+= temp[1];
	tempgz+= temp[2];
	}
	
Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];	
	
	
	
	
}
