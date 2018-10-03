#include "GPIO_Config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	//������IO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);		
	//�򿪴���1���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
	//�򿪴���2������3������4��CAN1,CAN2��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1|RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_UART4|RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3,PB4,PA15����ͨIO��,PA13&14����SWD����
	
//	//���Ƶ���ź������������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_10;				
//    //EN_L , FR_L, BK_L ,
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOE, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
//	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;				
//    //(EN_R),FR_R 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
//	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;			//PE8
//    //(BK_R)
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
//	
//   //PE9,PE11������ǰ�������ٶȿ��� ��TIM1Ĭ��������ţ�
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 ;	  
//	//SV_LEFTWHEEL,SV_RIGHTWHEEL
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*****************CPU����ָʾ��*****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	  
	//CPURunLED
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	/*****************�����ź�������������*****************/
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_15 ;//PD5
	//Infrared_Left
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //���ó���������
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_3 | GPIO_Pin_4;//PB3,4
	//Infrared_Central,Infrared_Right
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //���ó���������
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	/*****************����������������*****************/
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;//PC10,11,12
	//BUTTON_ZERO,BUTTON_FIRST,BUTTON_SECOND
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;; //���ó���������
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOB3,4,5
	
	
//	/********************����������������������********************/
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PA6
//	//PluseSpeedLeftWheel
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;; //���óɸ�������
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA6
//	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PC6
//	//PluseSpeedRightWheel
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;; //���óɸ�������
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC6
	
	
	/********************����1��������********************/
		//USART1_TX	  PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10


//    /********************����2��������********************/
//	//USART2_TX	  PA.2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA2

//	//USART2_RX	  PA.3
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA3
 
 
 /********************����3��������********************/
	//USART3_TX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��PA10

	//USART3_RX	  PA.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PA11
 
   //��ѹ���
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;                               //�ɼ���ѹ������ΪPC4
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;                                  //ģ������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                //�ٶ�Ϊ50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
//  //ALM�����ź�
//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PA7
//	//L_ALM
//  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //���ó���������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA7 

//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PC7
//	//R_ALM
//  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;; //���ó���������
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC7

}

