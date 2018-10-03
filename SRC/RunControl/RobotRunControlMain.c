#include "GPIO_Config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "RobotRunControlMain.h"
#include "RunDeal.h"
#include "protocol.h"
#include "MotorControl.h"
#include "hub_motor.h"


unsigned short gsLETF_MotorSpeed=0; //左轮速度
unsigned short gsRIGHT_MotorSpeed=0; //右轮速度

void ServiceRobotRun_Init(void)
{
	 //SetMotorEn(MOTOR_DISABLE,MOTOR_DISABLE);  //不使能电机
	 //SetMotorBk(BK_ENABLE,BK_ENABLE);  //刹车
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
		vTaskDelay(1);	//休眠1ms
	}

	while(1)
	{
		GetRobotRunDIRSpeed(gsRunSpeed,gsRadiusVal); //通过速度和半径值获取小车的方向
		set_hub_motor_speed(gsLETF_MotorSpeed, gsRIGHT_MotorSpeed);	
		Motor_heartbeat();
		
		vTaskDelay(2);	//延时2MS
	}
}   
