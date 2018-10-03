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
//������
void Start_Task(void *pvParameters);
void APP_DriverService_Task(void *pvParameters);
void APP_CommunicationService_Task(void *pvParameters);
void APP_RobotRunControlService_Task(void *pvParameters);

FlagStatus DriverService_InitComplete_Flage = RESET;
FlagStatus RobotRunService_InitComplete_Flage = RESET;
FlagStatus Communication_InitComplete_Flage = RESET;


//��ʼ����������
 void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //����DriverService����
    xTaskCreate((TaskFunction_t )APP_DriverService_Task,     	
                (const char*    )"APP_DriverService_task",   	
                (uint16_t       )DriverService_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DriverService_TASK_PRIO,	
                (TaskHandle_t*  )&DriverService_TASK_Handler);   
     //����RobotRunControlService����   
    xTaskCreate((TaskFunction_t )APP_RobotRunControlService_Task,     
                (const char*    )"APP_RobotRunControlService_Task",   
                (uint16_t       )RobotRunControlService_STK_SIZE ,
                (void*          )NULL,
                (UBaseType_t    )RobotRunControlService_TASK_PRIO,
                (TaskHandle_t*  )&RobotRunControlService_TASK_Handler);   
    //����CommunicationService����
    xTaskCreate((TaskFunction_t )APP_CommunicationService_Task,     
                (const char*    )"APP_CommunicationService_Task",   
                (uint16_t       )CommunicationService_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )CommunicationService_TASK_PRIO,
                (TaskHandle_t*  )&CommunicationService_TASK_Handler);   

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}


void Init_TimeParameters(void)  //��ʼ��ʱ�����
{		 
	right_lasttime = sys_time;
	left_lasttime = sys_time ;
	oldsystime=sys_time/10;
}

static void Init_Config(void)
{
	delay_init();	    	//��ʱ������ʼ��	
  GPIO_Configuration();   //GPIO�ڳ�ʼ��
	uart_init(115200);		//����1��ʼ����������Ϊ115200
	Hub_motor_Init();        //��챵����ʼ�� ʹ�ô���2 ������115200
	//uart3_init(115200);    //����3��ʼ����������Ϊ115200 ����GY901
	InitMpu6050(); //��ʼ��MPU6050
	Init_TimeParameters(); //��ʼ��ʱ�����	
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 	 
	 					
	Init_Config(); //��ʼ���ײ�Ӳ��
	 
	//������ʼ����
    xTaskCreate((TaskFunction_t )Start_Task,            //������
                (const char*    )"Start_Task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}




