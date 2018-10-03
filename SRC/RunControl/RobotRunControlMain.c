#include "GPIO_Config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "RobotRunControlMain.h"
#include "RunDeal.h"
#include "protocol.h"
#include "MotorControl.h"
#include "hub_motor.h"


unsigned short gsLETF_MotorSpeed=0; //�����ٶ�
unsigned short gsRIGHT_MotorSpeed=0; //�����ٶ�

void ServiceRobotRun_Init(void)
{
	 //SetMotorEn(MOTOR_DISABLE,MOTOR_DISABLE);  //��ʹ�ܵ��
	 //SetMotorBk(BK_ENABLE,BK_ENABLE);  //ɲ��
}

void APP_RobotRunControlService_Task(void *pvParameters)
{

	extern FlagStatus DriverService_InitComplete_Flage;
	extern FlagStatus RobotRunService_InitComplete_Flage;
	extern FlagStatus Communication_InitComplete_Flage;	

	RobotRunService_InitComplete_Flage = SET;     
	while((DriverService_InitComplete_Flage == RESET) 
	|| (RobotRunService_InitComplete_Flage == RESET) 
	|| (Communication_InitComplete_Flage == RESET))

	{
		vTaskDelay(1);	//����1ms
	}

	while(1)
	{
		GetRobotRunDIRSpeed(gsRunSpeed,gsRadiusVal); //ͨ���ٶȺͰ뾶ֵ��ȡС���ķ���
		set_hub_motor_speed(gsLETF_MotorSpeed, gsRIGHT_MotorSpeed);	
		Motor_heartbeat();
		
		vTaskDelay(2);	//��ʱ2MS
	}
}   
