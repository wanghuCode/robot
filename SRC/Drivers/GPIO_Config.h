#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

void GPIO_Configuration(void); //���ų�ʼ��

//��﷽�����Ŷ���
#define ML_FR  GPIOE,GPIO_Pin_8	 //ML_F/R
#define MR_FR  GPIOC,GPIO_Pin_9	   //MR_F/R

//���ʹ�����Ŷ���
#define ML_EN  GPIOE,GPIO_Pin_7	 //ML_EN
#define MR_EN  GPIOC,GPIO_Pin_8	 //MR_EN

//���ɲ�����Ŷ���
#define ML_BK  GPIOE,GPIO_Pin_10	//ML_BK
#define MR_BK  GPIOA,GPIO_Pin_8	 //MR_BK

#define ML_PG  GPIOA,GPIO_Pin_6	 //MR_BK
#define MR_PG  GPIOC,GPIO_Pin_6	 //MR_BK
                                      
#define IO_LEDCPURUN    GPIOE,GPIO_Pin_15      //CPU����ָʾ

//����վ���Ӧ
#define RIGHT_INFRARED    GPIOB,GPIO_Pin_4      
#define CENTRE_INFRARED    GPIOB,GPIO_Pin_3      
#define LEFT_INFRARED    GPIOD,GPIO_Pin_15      
//����ɨ�� 
#define BUTTON_0   GPIOC,GPIO_Pin_10 
#define BUTTON_1   GPIOC,GPIO_Pin_11 
#define BUTTON_2   GPIOC,GPIO_Pin_12 

//L_ALM,R_ALM
#define L_ALM  GPIOA,GPIO_Pin_7
#define R_ALM  GPIOC,GPIO_Pin_7

#endif

