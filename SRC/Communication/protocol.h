#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "sys.h"

typedef enum 
{
	No_Bumper=0,		
	Right_Bumper=0x01,
	Centre_Bumper=0x02,
	Left_Bumper=0x04		
}Bumper_st; //减震器标识


typedef enum
{
	No_WheelDrop=0,	
	Right_WheelDrop=0x01,	
	Left_WheelDrop=0x02		
}WheelDrop_st;//轮跌落标识

typedef enum
{
	NO_Clif,
	Right_Clif=0x01,
	Centre_Clif=0x02,
	Left_Clif=0x04		
}Clif_st; //防跌落标识

typedef enum 
{
	NO_Button,
	Zero_Button=0x01,
	First_Button=0x02,
	Second_Button=0x04
} Button_st; //按钮标识

typedef enum 
{
	NO_Charge=0,
	Docking_Charged=2,
	Docking_Charging=6,
	Adapter_Charged=18,
	Adapter_Charging=22
} ChargeState_st;//充电状态标识

typedef enum
{
	NO_OverCurrent,
	LetfWheel_OverCurrent=0x01,
	RightWheel_OverCurrent=0x02
}Over_current_st; //电流过流标识
	
	
typedef  struct
{
	 //时间戳	
	u16 TimeStamp; 
	//减震器	
	u8 Bumper;  
	//轮跌落
	WheelDrop_st WheelDrop; 
	//防跌落
	Clif_st Clif; 
	//左轮编码器
	u16 Left_encoder; 
	//右轮编码器
	u16 Right_encoder; 
	//左轮马达，有符号数，负号表示方向向后
	char Left_PWM;  
	//右轮马达，有符号数，负号表示方向向后
	char Right_PWM; 
	//按钮
	Button_st Button; 
	 //充电状态
	ChargeState_st ChargeState;
	//电池电量
	u8 BatteryValue; 
	 //过电流标志
	Over_current_st Over_current_falg;
}BasicSensorData_st; //基本传感器数据
	
typedef enum 
{
	NO_INFRARED,
	Near_LeftRegion=0x01,
	Near_CentreRegion=0x02,
	Near_RightRegion=0x04,
	Far_CentreRegion=0x08,
	Far_LeftRegion=0x10,
	Far_RightRegion=0x20
	
}Infrared_DockingRegion_st;//红外对接区域标识


typedef struct
{
	Infrared_DockingRegion_st RightInfrared_Signal;
	Infrared_DockingRegion_st CentreInfrared_Signal;
	Infrared_DockingRegion_st LeftInfrared_Signal;	
	
}Infrared_Docking_Station_st;//红外对接站结构体

__packed typedef  struct
{
	u16 Angle;
	u16 Angle_rate;
	u8 Unused0;
	u8 Unused1;
	u8 Unused2;	
}GYRO_Z_st; //陀螺仪数据(偏航角yaw)

__packed typedef  struct
{
	u16 RightCliffSensorData;
	u16 CentreCliffSensorData;
	u16 LeftCliffSensorData;
}CliffSensorData_st; //防跌落传感器数据(偏航角yaw)

__packed typedef  struct
{
	u8 LeftMotor_CurrentValue;
	u8 RightMotor_CurrentValue;

}Wheel_CurrentData_st; //左右轮电流值


//__packed typedef struct
//{
//	short x;
//	short y;
//	short z;
//}Raw_GYRO_DATA; //陀螺仪原始数据

__packed typedef struct
{
	u8 Frame_ID;
	u8 GYRO_DATA_LENGTH;
	short RGYRO_DATA[3][3];

}GYRO_Original_Data_st;//陀螺仪结构体


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
	
}General_Input_st;//通用输入


//传输帧对象
__packed typedef struct{
	//最大帧长度
	#define MAX_FRAME_LENGTH 30
	//最小帧长度
	#define MIN_FRAME_LENGTH  2	
	//有效数据长度
	u8 Data_Length;
	//头部字节0  固定为0xaa
	u8 Header0;
	//头部字节1  固定为0x55
	u8 Header1;
	//帧序列
	//u8 Sequence;
	//数据
	u8 *Data;
	//校验值
	u16 Checksum;

}TransportProtocol_Typedef;



typedef enum{
 //帧格式错误
	FRAME_FORMAT_ERR = 1,		
	//校验值格式错误
	FRAME_HEAD_ERR = 2,
	//校验值错位
	CHECK_ERR = 3,
	//解包成功
	UPACKED_SUCCESS = 4,
	FRAME_HEAD_RIG=5
	

}TransportProtocol_Result;




//协议管理器
__packed typedef  struct{
	
	//传输帧
 TransportProtocol_Typedef * TransportProtocol;
	//接收的字节数
	u32  RecieveByteCount;
		
	//接收帧缓存区
	u8* ReceBuf;
	
	//发送帧缓存
	u8* Buf;
	//帧总长度
	u16 FrameTotalLength;
	//解包函数
	TransportProtocol_Result (*Unpacked)(void);
	//打包函数
	void (*Packed)(void);
	//校验函数
	u16 (*Check)(u8 *,u16 len);

}TransportProtocol_Manager_Typedef;

typedef enum
{
 HEAD0=0,
 HEAD1,
 LENGTH,
 PACKAGE		
}Receive_st;  //接收状态


//外部声明协议管理器
extern TransportProtocol_Manager_Typedef TransportProtocol_Manager;
extern BasicSensorData_st BasicSensor;

extern Infrared_Docking_Station_st Infrared_Docking;
extern GYRO_Z_st GYRO_Angle;
extern CliffSensorData_st CliffSensorData;
extern Wheel_CurrentData_st WheelCurrentData;
extern GYRO_Original_Data_st GYRO_OriginalData;
extern General_Input_st General_Input;
extern u16  ReceiveSpeedTimeOut;  //接收超时

extern short gsRadiusVal; //ARM端下发的旋转半径
extern short	gsRunSpeed;  //ARM端下发的运动速度
extern FlagStatus ReceiveMusicOrder;


void UART1_task(void);
void TransportProtocol_Unpacked(void);
void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *sendbuf,u16 (*check)(u8 *,u16 len));
void Update_TextToSend(u8 *data_pose,u8 *sum_length);
#endif

