#include "GPIO_Config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	//打开所有IO的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);		
	//打开串口1与的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
	//打开串口2，串口3，串口4与CAN1,CAN2的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1|RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_UART4|RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3,PB4,PA15作普通IO口,PA13&14用作SWD调试
	
//	//控制电机信号输出引脚配置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_10;				
//    //EN_L , FR_L, BK_L ,
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOE, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
//	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;				
//    //(EN_R),FR_R 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
//	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;			//PE8
//    //(BK_R)
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
//	
//   //PE9,PE11左右轮前进后退速度控制 （TIM1默认输出引脚）
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 ;	  
//	//SV_LEFTWHEEL,SV_RIGHTWHEEL
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*****************CPU运行指示灯*****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	  
	//CPURunLED
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	/*****************红外信号输入引脚配置*****************/
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_15 ;//PD5
	//Infrared_Left
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //设置成下拉输入
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_3 | GPIO_Pin_4;//PB3,4
	//Infrared_Central,Infrared_Right
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //设置成下拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	/*****************按键输入引脚配置*****************/
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;//PC10,11,12
	//BUTTON_ZERO,BUTTON_FIRST,BUTTON_SECOND
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;; //设置成上拉输入
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOB3,4,5
	
	
//	/********************码盘脉冲输入检测引脚配置********************/
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PA6
//	//PluseSpeedLeftWheel
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;; //设置成浮空输入
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA6
//	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PC6
//	//PluseSpeedRightWheel
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;; //设置成浮空输入
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC6
	
	
	/********************串口1引脚配置********************/
		//USART1_TX	  PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10


//    /********************串口2引脚配置********************/
//	//USART2_TX	  PA.2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2

//	//USART2_RX	  PA.3
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
 
 
 /********************串口3引脚配置********************/
	//USART3_TX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PA10

	//USART3_RX	  PA.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PA11
 
   //电压检测
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;                               //采集电压的引脚为PC4
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;                                  //模拟输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                //速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
//  //ALM警告信号
//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PA7
//	//L_ALM
//  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //设置成下拉输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA7 

//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PC7
//	//R_ALM
//  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //设置成下拉输入
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC7

}

