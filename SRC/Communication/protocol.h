#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "sys.h"

typedef enum 
{
	No_Bumper=0,		
	Right_Bumper=0x01,
	Centre_Bumper=0x02,
	Left_Bumper=0x04		
}Bumper_st; //��������ʶ


typedef enum
{
	No_WheelDrop=0,	
	Right_WheelDrop=0x01,	
	Left_WheelDrop=0x02		
}WheelDrop_st;//�ֵ����ʶ

typedef enum
{
	NO_Clif,
	Right_Clif=0x01,
	Centre_Clif=0x02,
	Left_Clif=0x04		
}Clif_st; //�������ʶ

typedef enum 
{
	NO_Button,
	Zero_Button=0x01,
	First_Button=0x02,
	Second_Button=0x04
} Button_st; //��ť��ʶ

typedef enum 
{
	NO_Charge=0,
	Docking_Charged=2,
	Docking_Charging=6,
	Adapter_Charged=18,
	Adapter_Charging=22
} ChargeState_st;//���״̬��ʶ

typedef enum
{
	NO_OverCurrent,
	LetfWheel_OverCurrent=0x01,
	RightWheel_OverCurrent=0x02
}Over_current_st; //����������ʶ
	
	
typedef  struct
{
	 //ʱ���	
	u16 TimeStamp; 
	//������	
	u8 Bumper;  
	//�ֵ���
	WheelDrop_st WheelDrop; 
	//������
	Clif_st Clif; 
	//���ֱ�����
	u16 Left_encoder; 
	//���ֱ�����
	u16 Right_encoder; 
	//�������з����������ű�ʾ�������
	char Left_PWM;  
	//�������з����������ű�ʾ�������
	char Right_PWM; 
	//��ť
	Button_st Button; 
	 //���״̬
	ChargeState_st ChargeState;
	//��ص���
	u8 BatteryValue; 
	 //��������־
	Over_current_st Over_current_falg;
}BasicSensorData_st; //��������������
	
typedef enum 
{
	NO_INFRARED,
	Near_LeftRegion=0x01,
	Near_CentreRegion=0x02,
	Near_RightRegion=0x04,
	Far_CentreRegion=0x08,
	Far_LeftRegion=0x10,
	Far_RightRegion=0x20
	
}Infrared_DockingRegion_st;//����Խ������ʶ


typedef struct
{
	Infrared_DockingRegion_st RightInfrared_Signal;
	Infrared_DockingRegion_st CentreInfrared_Signal;
	Infrared_DockingRegion_st LeftInfrared_Signal;	
	
}Infrared_Docking_Station_st;//����Խ�վ�ṹ��

__packed typedef  struct
{
	u16 Angle;
	u16 Angle_rate;
	u8 Unused0;
	u8 Unused1;
	u8 Unused2;	
}GYRO_Z_st; //����������(ƫ����yaw)

__packed typedef  struct
{
	u16 RightCliffSensorData;
	u16 CentreCliffSensorData;
	u16 LeftCliffSensorData;
}CliffSensorData_st; //�����䴫��������(ƫ����yaw)

__packed typedef  struct
{
	u8 LeftMotor_CurrentValue;
	u8 RightMotor_CurrentValue;

}Wheel_CurrentData_st; //�����ֵ���ֵ


//__packed typedef struct
//{
//	short x;
//	short y;
//	short z;
//}Raw_GYRO_DATA; //������ԭʼ����

__packed typedef struct
{
	u8 Frame_ID;
	u8 GYRO_DATA_LENGTH;
	short RGYRO_DATA[3][3];

}GYRO_Original_Data_st;//�����ǽṹ��


__packed typedef struct
{
	u16 Digital_Input;
	u16 Analog_Input_ch0;
	u16 Analog_Input_ch1;
	u16 Analog_Input_ch2;
	u16 Analog_Input_ch3;
	u16 Notused_1;
	u16 Notused_2;
	u16 Notused_3;
	
}General_Input_st;//ͨ������


//����֡����
__packed typedef struct{
	//���֡����
	#define MAX_FRAME_LENGTH 30
	//��С֡����
	#define MIN_FRAME_LENGTH  2	
	//��Ч���ݳ���
	u8 Data_Length;
	//ͷ���ֽ�0  �̶�Ϊ0xaa
	u8 Header0;
	//ͷ���ֽ�1  �̶�Ϊ0x55
	u8 Header1;
	//֡����
	//u8 Sequence;
	//����
	u8 *Data;
	//У��ֵ
	u16 Checksum;

}TransportProtocol_Typedef;



typedef enum{
 //֡��ʽ����
	FRAME_FORMAT_ERR = 1,		
	//У��ֵ��ʽ����
	FRAME_HEAD_ERR = 2,
	//У��ֵ��λ
	CHECK_ERR = 3,
	//����ɹ�
	UPACKED_SUCCESS = 4,
	FRAME_HEAD_RIG=5
	

}TransportProtocol_Result;




//Э�������
__packed typedef  struct{
	
	//����֡
 TransportProtocol_Typedef * TransportProtocol;
	//���յ��ֽ���
	u32  RecieveByteCount;
		
	//����֡������
	u8* ReceBuf;
	
	//����֡����
	u8* Buf;
	//֡�ܳ���
	u16 FrameTotalLength;
	//�������
	TransportProtocol_Result (*Unpacked)(void);
	//�������
	void (*Packed)(void);
	//У�麯��
	u16 (*Check)(u8 *,u16 len);

}TransportProtocol_Manager_Typedef;

typedef enum
{
 HEAD0=0,
 HEAD1,
 LENGTH,
 PACKAGE		
}Receive_st;  //����״̬


//�ⲿ����Э�������
extern TransportProtocol_Manager_Typedef TransportProtocol_Manager;
extern BasicSensorData_st BasicSensor;

extern Infrared_Docking_Station_st Infrared_Docking;
extern GYRO_Z_st GYRO_Angle;
extern CliffSensorData_st CliffSensorData;
extern Wheel_CurrentData_st WheelCurrentData;
extern GYRO_Original_Data_st GYRO_OriginalData;
extern General_Input_st General_Input;
extern u16  ReceiveSpeedTimeOut;  //���ճ�ʱ

extern short gsRadiusVal; //ARM���·�����ת�뾶
extern short	gsRunSpeed;  //ARM���·����˶��ٶ�
extern FlagStatus ReceiveMusicOrder;


void UART1_task(void);
void TransportProtocol_Unpacked(void);
void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *sendbuf,u16 (*check)(u8 *,u16 len));
void Update_TextToSend(u8 *data_pose,u8 *sum_length);
#endif

