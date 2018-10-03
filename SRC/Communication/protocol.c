#include "protocol.h"
#include "string.h"
#include "CommunicationMain.h"
#include "usart.h"

TransportProtocol_Manager_Typedef TransportProtocol_Manager;  //定义传输数据结构体
BasicSensorData_st BasicSensor={0};
Infrared_Docking_Station_st Infrared_Docking;
GYRO_Z_st GYRO_Angle={0};
CliffSensorData_st CliffSensorData={0};
Wheel_CurrentData_st WheelCurrentData={0};
GYRO_Original_Data_st GYRO_OriginalData={0};
General_Input_st General_Input={0};
short gsRadiusVal=0; //ARM端下发的旋转半径
short	gsRunSpeed=0;  //ARM端下发的运动速度
u16  ReceiveSpeedTimeOut=0;
FlagStatus ReceiveMusicOrder=SET;

//设置基本传感器
static u8 SetBasicSensorData(u8 *pose,u8 *len)
{

	unsigned short i;
    u8 TmpBuf[25]={0};
   //sub-payload帧头
	TmpBuf[0] = 0x01; //
	TmpBuf[1] = 0x0F;
	
	//timestamp
	TmpBuf[2] = (BasicSensor.TimeStamp & 0xff);
	TmpBuf[3] = (BasicSensor.TimeStamp >> 8)&0xff;
		
	//Bumper
    TmpBuf[4] =(u8)BasicSensor.Bumper;
	
	//WheelDrop
    TmpBuf[5] =(u8)BasicSensor.WheelDrop;
	
	//Clif
    TmpBuf[6] =(u8)BasicSensor.Clif;
	
	//left encoder     
	TmpBuf[7] =  (u8)(BasicSensor.Left_encoder&0xff);       //L 
	TmpBuf[8] =  (BasicSensor.Left_encoder >> 8)&0xff;  //H
	
	//right encoder     
	TmpBuf[9] =  (u8)(BasicSensor.Right_encoder&0xff);       //L 
	TmpBuf[10] = (u8)(BasicSensor.Right_encoder >> 8)&0xff;  //H
		
	//left pwm  
	TmpBuf[11] = (u8)(BasicSensor.Left_PWM & 0xffff);      
	//right pwm
	TmpBuf[12] = (u8)(BasicSensor.Right_PWM & 0xffff);
	//Button
	TmpBuf[13] = (u8)BasicSensor.Button;
	//ChargeState
	TmpBuf[14] = (u8)BasicSensor.ChargeState;
	//BatteryValue
	TmpBuf[15] = (u8)BasicSensor.BatteryValue &0xff;
	//Over_current_falg
	TmpBuf[16] = (u8)BasicSensor.Over_current_falg;
 
  for(i=0;i<17;i++)
	{
		pose[i+(*len)]=TmpBuf[i];
	}
	return 17;

}

static u8 SetInfraredDockingStationData(u8 *pose,u8 *len)
{
 
	 unsigned short i;
     u8 TmpBuf[20]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x03; 
	 TmpBuf[1] = 0x03;
	 //right_signal
	 TmpBuf[2] =(u8)Infrared_Docking.RightInfrared_Signal&0xff;
	 //centre_signal
	 TmpBuf[3] =(u8)Infrared_Docking.CentreInfrared_Signal&0xff;
	 //left_signal
	 TmpBuf[4] =(u8)Infrared_Docking.LeftInfrared_Signal&0xff;
	 
	 for(i=0;i<5;i++)
	{
		pose[i+(*len)]=TmpBuf[i];
	}
	return 5;
	
}

static u8 SetGYRO_YawAngleData(u8 *pose,u8 *len)
{

	 unsigned short i;
     u8 TmpBuf[20]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x04; 
	 TmpBuf[1] = 0x07;
	 //Angle
	 TmpBuf[2] =(GYRO_Angle.Angle & 0xff);   //L
	 TmpBuf[3] =(GYRO_Angle.Angle >> 8)& 0xff;  //H
	 	 //Angle_rate
	 TmpBuf[4] =(GYRO_Angle.Angle_rate & 0xff); //L
	 TmpBuf[5] =(GYRO_Angle.Angle_rate >> 8)& 0xff;// H
	 //Unused0
	 TmpBuf[6] =0x00;
	 //Unused1
	 TmpBuf[7] =0x00;
	 //Unused2
	 TmpBuf[8] =0x00;
	 
	 for(i=0;i<9;i++)
	{
		pose[i+(*len)]=TmpBuf[i];
	}

	return 9;
}


static u8 SetCliffSensorData(u8 *pose,u8 *len)
{

	 unsigned short i;
     u8 TmpBuf[20]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x05; 
	 TmpBuf[1] = 0x06;
	 //RightCliff
	 TmpBuf[2] =(CliffSensorData.RightCliffSensorData & 0xff);   //L
	 TmpBuf[3] =(CliffSensorData.RightCliffSensorData >> 8)& 0xff;  //H
	 	//CentreCliff
	 TmpBuf[4] =(CliffSensorData.CentreCliffSensorData & 0xff); //L
	 TmpBuf[5] =(CliffSensorData.CentreCliffSensorData >> 8)& 0xff;// H	 
	 	//LeftCliff
	 TmpBuf[6] =(CliffSensorData.LeftCliffSensorData & 0xff); //L
	 TmpBuf[7] =(CliffSensorData.LeftCliffSensorData >> 8)& 0xff;// H
	 	 
	 for(i=0;i<8;i++)
	 {
		pose[i+(*len)]=TmpBuf[i];
	 }
	return 8;
}
/**************************************************************
程序功能：设置轮子电流
输入： 无
输出： 无
说明： 
**************************************************************/
static u8 SetWheelCurrentData(u8 *pose,u8 *len)
{
	 unsigned short i;
   u8 TmpBuf[20]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x06; 
	 TmpBuf[1] = 0x02;
	 //LeftMotor
	 TmpBuf[2] =(WheelCurrentData.LeftMotor_CurrentValue & 0xff);  
	
	 	//RightMotor
	 TmpBuf[3] =(WheelCurrentData.RightMotor_CurrentValue & 0xff); 
	 	 
	 for(i=0;i<4;i++)
	 {
		pose[i+(*len)]=TmpBuf[i];
	 }
	return 4;
}

/**************************************************************
程序功能：设置陀螺仪原始数据
输入： 无
输出： 无
说明： 
**************************************************************/
static u8 SetGYRO_Original_Data(u8 *pose,u8 *len)
{
	 unsigned short i,j;
   u8 TmpBuf[30]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x0D; 
	 TmpBuf[1] = 0x14;
	 //Frame_ID
	 TmpBuf[2] =(GYRO_OriginalData.Frame_ID & 0xff);  
	 
	 GYRO_OriginalData.Frame_ID++;
	 
	 //GYRO_DATA_LENGTH
	 TmpBuf[3] =0x09; //3N
	 
	 //GYRO_DATA_LENGTH
	 for(i=0,j=4;i<3;i++)
	 {
		 //X
		 TmpBuf[j++] =(GYRO_OriginalData.RGYRO_DATA[i][0]& 0xff); //L
		
		 TmpBuf[j++] =(GYRO_OriginalData.RGYRO_DATA[i][0]>> 8)& 0xff;// H	  
	
		 //Y
		 TmpBuf[j] =(GYRO_OriginalData.RGYRO_DATA[i][1]& 0xff); //L
		 j++;
		 TmpBuf[j] =(GYRO_OriginalData.RGYRO_DATA[i][1]>> 8)& 0xff;// H	  
		 j++;
		 //Z
		 TmpBuf[j] =(GYRO_OriginalData.RGYRO_DATA[i][2] & 0xff); //L
		 j++;
		 TmpBuf[j]=(GYRO_OriginalData.RGYRO_DATA[i][2] >> 8) & 0xff;// H	 
     j++;		 
	 }
 
	 for(i=0;i<22;i++)
	 {
		pose[i+(*len)]=TmpBuf[i];
	 }
	return 22;
}
/**************************************************************
程序功能：设置普通输入数据
输入： 无
输出： 无
说明： 
**************************************************************/
static u8 SetGeneral_InputData(u8 *pose,u8 *len)
{
	 unsigned short i;
   u8 TmpBuf[20]={0};
	 //sub-payload帧头
	 TmpBuf[0] = 0x10; 
	 TmpBuf[1] = 0x10;
	 //Digital_Input
	 TmpBuf[2] =(General_Input.Digital_Input & 0xff);   //L
	 TmpBuf[3] =(General_Input.Digital_Input >> 8)& 0xff;  //H
	  //Analog_Input_ch0
	 TmpBuf[4] =(General_Input.Analog_Input_ch0 & 0xff);   //L
	 TmpBuf[5] =(General_Input.Analog_Input_ch0 >> 8)& 0xff;  //H
	  //Analog_Input_ch1
	 TmpBuf[6] =(General_Input.Analog_Input_ch1 & 0xff);   //L
	 TmpBuf[7] =(General_Input.Analog_Input_ch1 >> 8)& 0xff;  //H
	  //Analog_Input_ch2
	 TmpBuf[8] =(General_Input.Analog_Input_ch2 & 0xff);   //L
	 TmpBuf[9] =(General_Input.Analog_Input_ch2 >> 8)& 0xff;  //H
	  //Analog_Input_ch3
	 TmpBuf[10] =(General_Input.Analog_Input_ch3 & 0xff);   //L
	 TmpBuf[11] =(General_Input.Analog_Input_ch3 >> 8)& 0xff;  //H	 
	 //Notused_1
	 TmpBuf[12] =(General_Input.Notused_1 & 0xff);   //L
	 TmpBuf[13] =(General_Input.Notused_1 >> 8)& 0xff;  //H	 
	 //Notused_2
	 TmpBuf[14] =(General_Input.Notused_2 & 0xff);   //L
	 TmpBuf[15] =(General_Input.Notused_2 >> 8)& 0xff;  //H	 
	 //Notused_3
	 TmpBuf[16] =(General_Input.Notused_3 & 0xff);   //L
	 TmpBuf[17] =(General_Input.Notused_3 >> 8)& 0xff;  //H
	 for(i=0;i<18;i++)
	 {
		pose[i+(*len)]=TmpBuf[i];
	 }
	return 18;
}

/**************************************************************
程序功能：更新发送数组
输入： 无
输出： 无
说明： 

**************************************************************/
void Update_TextToSend(u8 *data_pose,u8 *sum_length)
{  
	memset(TEXT_TO_SEND, 0, 250); //清零发送数据数组
	*sum_length+=SetBasicSensorData(data_pose,sum_length);       
	*sum_length+=SetInfraredDockingStationData(data_pose,sum_length);
	*sum_length+=SetGYRO_YawAngleData(data_pose,sum_length);
	*sum_length+=SetCliffSensorData(data_pose,sum_length);
	*sum_length+=SetWheelCurrentData(data_pose,sum_length);
  *sum_length+=SetGYRO_Original_Data(data_pose,sum_length);
	*sum_length+=SetGeneral_InputData(data_pose,sum_length);
		
	TransportProtocol.Header0= 0xAA;     //帧头第一固定字节
	TransportProtocol.Header1= 0x55;				//帧头第二固定字节
	TransportProtocol.Data_Length = *sum_length; //有效数据大小
	TransportProtocol.Data = (u8*)TEXT_TO_SEND;				//要发送的数据
	*sum_length=0;
	
}


//****************************串口数据处理*********************//
void TransportProtocol_Unpacked(void)
{
	u16 check_sum=0;      //接收到的校验值
	u8 check_sum_cal=0;  //计算得到的校验值
	u8 *check_sum_pos=0; //校验值的偏移位置

	check_sum_pos = (u8 *)(3+TEMPBUF+LEN);
	check_sum = (*check_sum_pos)&0xff;
	//计算 头和数据的校验值
	check_sum_cal = TransportProtocol_Manager.Check(TEMPBUF+2,1+LEN);
	//校验错误
	if(check_sum==check_sum_cal)
	{
		switch(SREF)															//串口接收数据处理 
		{
			case 0x01:															//接收的标识
			{	
			  gsRadiusVal = (short)((TEMPBUF[8] << 8)|TEMPBUF[7]);	 //旋转半径（旋转中心离两轮中心的距离）
			  gsRunSpeed = (short)((TEMPBUF[6] << 8)|TEMPBUF[5]);	  //速度（带符号）					
			}
			break;
			case 0x04:																
			{
				if(TEMPBUF[4]==0x01)
				{

				}
			}
			break;
			case 0x08:																
			{

			}
			break;
			case 0x06:																
			{																					

			}break;
			default:break;
	   }	
	}
	else
	{
		
	}
	memset(TEMPBUF, 0, 256); //清零接收数据数组
}
/**************************************************************
程序功能：串口一接收数据
输入： 无
输出： 无
说明： 
**************************************************************/
#define Uart1_FrameStart0    (0xAA)    //帧头第一个字节
#define Uart1_FrameStart1    (0x55)     //帧头第二个字节
FlagStatus Pack_flag=RESET;
void UART1_task(void)
{
	static u16 pt = 0;
	static Receive_st REC_State=HEAD0;
	u8 	Res;	
	Res= USART_ReceiveData(USART1);

	switch(REC_State)
	{
		case HEAD0:
		if(Res==Uart1_FrameStart0)
		{
		  REC_State=HEAD1;
		}
		else
		{
		  memset(TEMPBUF, 0, 256); //清零接收数据数组
		  REC_State=HEAD0;
		}

		break;
		case HEAD1:
		{
			if(Res==Uart1_FrameStart1)
			{		  
				TEMPBUF[0] = Uart1_FrameStart0;
			  TEMPBUF[1] = Uart1_FrameStart1;
				pt = 2;				
				REC_State=LENGTH;
			}
			else
			{
				memset(TEMPBUF, 0, 256); //清零接收数据数组
				REC_State=HEAD0;
			}	
	 }		
		break;
		case LENGTH:
		{	
				TEMPBUF[pt]=Res;
				LEN=Res;
				REC_State=PACKAGE;
				Pack_flag=SET;
		}
		break;
		case PACKAGE:
		{
			pt ++;
			TEMPBUF[pt] = Res;	
			if((Pack_flag==SET)&&(pt>=LEN+2+1+1-1))
			{
				pt = 0;
				Pack_flag=RESET;
				SREF = TEMPBUF[3]; 										  //标识位送出，开始处理	
				TransportProtocol_Unpacked();		
				REC_State=HEAD0;
			}
	   }
		break;
		default:break;
	}
}


//复制数组
void Datecpy(u8* Date,u16 Length)
{
  u8 i;
	for(i=0;i<Length;i++)
	{
	 TransportProtocol_Manager.Buf[3+i]=Date[i];
	}	
}
	
//打包
static void TransportProtocol_Packed()
{
	u16 checksum=0;
	u8 *check_pos;  //校验值的偏移位置
    //帧头第一字节
	TransportProtocol_Manager.Buf[0] = TransportProtocol_Manager.TransportProtocol->Header0;
	//帧头第二字节
	TransportProtocol_Manager.Buf[1] = TransportProtocol_Manager.TransportProtocol->Header1;
	//帧有效数据大小
	TransportProtocol_Manager.Buf[2] = TransportProtocol_Manager.TransportProtocol->Data_Length;	
	//帧数据
	Datecpy(TransportProtocol_Manager.TransportProtocol->Data,TransportProtocol_Manager.TransportProtocol->Data_Length);
	//计算校验值
	checksum = TransportProtocol_Manager.Check(TransportProtocol_Manager.Buf+2,1+TransportProtocol_Manager.TransportProtocol->Data_Length);
	//校验值
	check_pos = TransportProtocol_Manager.Buf+3+TransportProtocol_Manager.TransportProtocol->Data_Length;	
	(*check_pos) = (u8)(checksum&0xff);
	//记录帧总长度
	TransportProtocol_Manager.FrameTotalLength = 3+TransportProtocol_Manager.TransportProtocol->Data_Length+1;
}


//初始化传输协议
//TransportProtocol：传输帧
//buf:收发缓冲区
//check:校验方式
//Checksum_Sum:和校验
//Checksum_XOR：异或校验
//Checksum_CRC8:CRC8校验
//Checksum_CRC16:CRC16校验
void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *sendbuf,u16 (*check)(u8 *,u16 len))
{	
	//协议包
	TransportProtocol_Manager.TransportProtocol = TransportProtocol;
	 //传输协议缓存 
	TransportProtocol_Manager.Buf = sendbuf;
	//选择校验方式 
	TransportProtocol_Manager.Check = check;
	//打包函数
	TransportProtocol_Manager.Packed = TransportProtocol_Packed;	
}



