/*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2006; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/


/*********************************************************************************************************
*
*                                       APPLICATION CONFIGURATION
*
*                                          Atmel AT91SAM3U4
*                                                on the
*                                 Atmel AT91SAM3U-EK Development Board.
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************/


#ifndef  APP_CFG_MODULE
#define  APP_CFG_MODULE


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//任务优先级
#define START_TASK_PRIO		1
#define DriverService_TASK_PRIO		3
#define CommunicationService_TASK_PRIO		3
#define RobotRunControlService_TASK_PRIO		4

//任务堆栈大小	
#define START_STK_SIZE 		500  
#define DriverService_STK_SIZE 		500  
#define CommunicationService_STK_SIZE 		500  
#define RobotRunControlService_STK_SIZE 		800  
//任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t DriverService_TASK_Handler;
TaskHandle_t CommunicationService_TASK_Handler;
TaskHandle_t RobotRunControlService_TASK_Handler;


#endif

