#include "DriversMain.h"
#include "delay.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "app_cfg.h"
#include "GPIO_Config.h"
#include "dma.h"
#include "mpu6050manage.h"
#include "MotorControl.h"
#include "timers.h"
#include "RobotRunControlMain.h"
#include "CommunicationMain.h"
#include "hub_motor.h"

/************************************************

************************************************/
//任务函数
void Start_Task(void *pvParameters);
void APP_DriverService_Task(void *pvParameters);
void APP_CommunicationService_Task(void *pvParameters);
void APP_RobotRunControlService_Task(void *pvParameters);

FlagStatus DriverService_InitComplete_Flage = RESET;
FlagStatus RobotRunService_InitComplete_Flage = RESET;
FlagStatus Communication_InitComplete_Flage = RESET;


//开始任务任务函数
 void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建DriverService任务
    xTaskCreate((TaskFunction_t )APP_DriverService_Task,     	
                (const char*    )"APP_DriverService_task",   	
                (uint16_t       )DriverService_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DriverService_TASK_PRIO,	
                (TaskHandle_t*  )&DriverService_TASK_Handler);   
     //创建RobotRunControlService任务   
    xTaskCreate((TaskFunction_t )APP_RobotRunControlService_Task,     
                (const char*    )"APP_RobotRunControlService_Task",   
                (uint16_t       )RobotRunControlService_STK_SIZE ,
                (void*          )NULL,
                (UBaseType_t    )RobotRunControlService_TASK_PRIO,
                (TaskHandle_t*  )&RobotRunControlService_TASK_Handler);   
    //创建CommunicationService任务
    xTaskCreate((TaskFunction_t )APP_CommunicationService_Task,     
                (const char*    )"APP_CommunicationService_Task",   
                (uint16_t       )CommunicationService_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )CommunicationService_TASK_PRIO,
                (TaskHandle_t*  )&CommunicationService_TASK_Handler);   

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}


void Init_TimeParameters(void)  //初始化时间参数
{		 
	right_lasttime = sys_time;
	left_lasttime = sys_time ;
	oldsystime=sys_time/10;
}

static void Init_Config(void)
{
	delay_init();	    	//延时函数初始化	
  GPIO_Configuration();   //GPIO口初始化
	uart_init(115200);		//串口1初始化，波特率为115200
	Hub_motor_Init();        //轮毂电机初始化 使用串口2 波特率115200
	//uart3_init(115200);    //串口3初始化，波特率为115200 用于GY901
	InitMpu6050(); //初始化MPU6050
	Init_TimeParameters(); //初始化时间参数	
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 	 
	 					
	Init_Config(); //初始化底层硬件
	 
	//创建开始任务
    xTaskCreate((TaskFunction_t )Start_Task,            //任务函数
                (const char*    )"Start_Task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}




