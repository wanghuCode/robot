#ifndef __MPU6050MANAGE_H
#define __MPU6050MANAGE_H
#include "sys.h" 

int InitMpu6050(void);

void GetCollectData( float *angle, short *gyro, short *accel, int len);
void MPU6050_InitGyro_Offset(void);
#endif
