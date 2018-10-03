#ifndef _HUB_MOTOR_H
#define _HUB_MOTOR_H
#include "rs485.h"
#include "stdint.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

//����ָ��
//0xff Ϊ�Զ���ָ��
#define PING 0X01
#define READ_DATA (0X02)
#define WRITE_DATA (0X03)
#define REG_WRITE 0X04
#define ACTION 0X05
#define RESET_CON 0X06
#define START_END 0X07
#define SYNC_WRITE 0X83
#define TRAJ_WRITE 0X0A
#define TRAJ_ACTION 0X0B

//Ӧ���ERROR״̬
#define STOP 0X40
#define ERROR_INS 0X20
#define ERROR_CHECKSUM 0X10
#define LACK_VOLTAGE 0X08
#define OVER_VOLTAGE 0X04
#define OVER_CURRENT 0X02
#define OVER_HOT 0X01

#define MOTOR_USART USART2

#define RMotor 0x0b
#define LMotor 0x0a

#define ANGLE_PROTECT 90

#define SPEEDRATE 2

#define ForwardR 1
#define BackwardR 0

#define ForwardL 0
#define BackwardL 1

//�����ṹ��
typedef struct _parmeter{
    uint8_t count;//�����ֽ���
    uint8_t data[8];//�����������ݣ���������8���ֽ�����
}par;

//ָ����ṹ��
typedef struct _instruction{
	//֡ͷ
	uint8_t head1;
	uint8_t head2;
	
	//ID��
	uint8_t ID;
	
	//���ݳ���
	uint8_t length;
	
	//ָ���ֽ�
	uint8_t Instruction;

    //У���
	uint8_t  checksum;
}instruction;

//Ӧ����ṹ��
typedef struct _response{
	//֡ͷ
	uint8_t head1;
	uint8_t head2;
	
	//ID��
	uint8_t ID;
	
	//���ݳ���
	uint8_t length;
	
	//��ǰ״̬
	uint8_t error;
	
	//����
	par parm;
	
	//У���
	uint8_t  checksum;

}response;

void  Hub_motor_Init(void);
void SetDelayTime(USART_TypeDef* UARTx,uint8_t id);
void SetBoud(USART_TypeDef* UARTx,uint8_t id);
static uint8_t checksum_deal(void);
static void Inspack_Set(uint8_t ins,uint8_t id);
static void Motor_send_insturction(USART_TypeDef* UARTx);
void Motor_response(uint8_t *data,uint8_t rx_cnt);
void motor_run(USART_TypeDef* UARTx,uint32_t speed,uint8_t direct,uint8_t id);
//void Reset_MOTOR(USART_TypeDef* UARTx);
void recive_res(void);
uint16_t Recive_angle_data(uint8_t cnt,uint8_t *buf);
uint16_t Read_angle(void);
void ReadAngleData(USART_TypeDef* UARTx,uint8_t id);
char MotorLockR(void);
char MotorLockL(void);
char MotorLock(void);
void set_hub_motor_speed(uint16_t lSpeed, uint16_t rSpeed);
void set_hub_motor_dir(uint8_t dir_r,uint8_t dir_l);
void Motor_heartbeat(void);
uint16_t GetCount(uint8_t id);
uint16_t CalculateAngleR(void);
uint16_t CalculateAngleL(void);
#endif

