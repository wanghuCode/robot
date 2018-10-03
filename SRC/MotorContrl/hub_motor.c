#include "hub_motor.h"
#include  <string.h>
#include "math.h"
#include "stdlib.h"
#include "stdio.h"

  extern uint8_t RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
  extern uint8_t RS485_RX_CNT;   			//���յ������ݳ���
  
  instruction inst;//ָ����ṹ��
	par parm_tx;//ָ��������ṹ��
  response res;
	
  char Flag_lock_R=0;//��ת��־
  char Flag_lock_L=0;//��ת��־
  
   uint16_t last_angle_r=0;//�������Ƕ�ֵ
   uint16_t now_angle_r=0;
   uint16_t last_angle_l=0;
   uint16_t now_angle_l=0;
  
   uint8_t DirectR=0;//����ת��־ 0����ת 1����ת
   uint8_t DirectL=0;//
  
   uint16_t uSpeedL=0;//���浼��ģ���·��ٶ�ֵ
   uint16_t uSpeedR=0;
   
   char frist_measure_r=1;
   char frist_measure_l=1;
	
  extern void delay_ms(uint32_t nms);
	extern void delay_us(uint32_t nus);

//������ʱʱ��
void SetDelayTime(USART_TypeDef* UARTx,uint8_t id)
{
  parm_tx.count = 2;
  parm_tx.data[0]=0x05;//�Ĵ�����ַ
  parm_tx.data[1]=1;//������ʱʱ��Ϊ500us

  Inspack_Set(WRITE_DATA,id);
  Motor_send_insturction(UARTx);

}
//���ò�����
void SetBoud(USART_TypeDef* UARTx,uint8_t id)
{
  parm_tx.count = 2;
  parm_tx.data[0]=0x04;//�Ĵ�����ַ
	parm_tx.data[1]=0x04;//����������Ϊ115200

  Inspack_Set(WRITE_DATA ,id);
  Motor_send_insturction(UARTx);

}
//��������:��ʼ�����
void  Hub_motor_Init(void) {

  memset(&inst, 0,sizeof(inst));
  memset(&res, 0,sizeof(res));
  memset(&parm_tx,0,sizeof(parm_tx));//��ս��սṹ��
	
  USART2_Init(115200);
	
//	SetDelayTime(MOTOR_USART,RMotor);
//	delay_ms(1);
//	SetDelayTime(MOTOR_USART,LMotor);	
	
//  set_hub_motor_speed(0,0);
//  Motor_heartbeat();
  
}

////�������ܣ�����ָ�����ֵ
//void Reset_MOTOR(USART_TypeDef* UARTx){
//
//  parm_tx.count=0;
//  Inspack_Set(RESET_CON,parm_tx,RMotor);
//  Motor_send_insturction(UARTx,inst);
//
//}

//�������ܣ�����У���
static uint8_t checksum_deal(void){
  char i;  
  uint16_t checksum=0;
  uint16_t sum=0;
  uint16_t temp=0;
 
  sum=inst.ID+inst.length+inst.Instruction;
	
  for( i=0;i<parm_tx.count;i++){
     temp+=parm_tx.data[i];
     }
  
  checksum=temp+sum;
  if(checksum>255)
          checksum=(uint8_t)(checksum&0xff);
  else 
          checksum=(uint8_t)checksum;
  return  checksum;
  
}

//�������ܣ�ָ�������
//���������ins:ָ�par:�����ṹ�壬
static void Inspack_Set(uint8_t ins,uint8_t id){


  inst.head1=0xd5;
  inst.head2=0x5d;
  
  inst.ID=id;
  
  inst.length = parm_tx.count+2;
	
  inst.Instruction=ins;
  
  inst.checksum=checksum_deal();
	
}

//�������ܣ�����ָ���
//���������ָ����ṹ��
//          Ҫʹ�õĴ���
//ʹ��֮ǰ��������ָ�����ʼ��������������
static void Motor_send_insturction(USART_TypeDef* UARTx){
	char i;
	//����ʹ��
  RS485_TX_EN;
	
  //��������
  RS485_Send_Data(UARTx,inst.head1);
  RS485_Send_Data(UARTx,inst.head2);
  
  RS485_Send_Data(UARTx,inst.ID);
  
  RS485_Send_Data(UARTx,inst.length);
  
  RS485_Send_Data(UARTx,inst.Instruction);

  for(i =0;i<parm_tx.count;i++)
	{
			RS485_Send_Data(UARTx,parm_tx.data[i]);
  }
	
  RS485_Send_Data(UARTx,inst.checksum);

  //while(USART_GetFlagStatus(UARTx,USART_FLAG_TC)==RESET);
		
	//ʹ�ܽ���
  RS485_RX_EN;
  RS485_RX_CNT=0;

}

//�������ܣ����յ������ݻ��浽��Ӧ���ṹ��
//���������RS485ͨ�Ž��յ�������,���յ����ֽ��� 
//          *data:RS485_R_RX_BUF
//                RS485_L_RX_BUF
//          rx_cnt��                 
void Motor_response(uint8_t *data,uint8_t rx_cnt){
  char i;  
  
  memset(&res,0,sizeof(res));//��ս��սṹ��
  
  res.head1=data[rx_cnt++];
  res.head2=data[rx_cnt++];
	
  res.ID=data[rx_cnt++];
  
  res.length=data[rx_cnt++];
  
  res.error=data[rx_cnt++];
          
  for(i=0;i<res.length-2;i++){
              res.parm.data[i]=data[rx_cnt++];
          }
  res.parm.count=res.length-2;    
  	
  res.checksum=data[rx_cnt];
		
  memset(data,0,rx_cnt);//��ս��սṹ��
}

//�������ܣ��������
//���������speed: 2*PI*R*data/360=speed ��λ��mm/s  R=0.070m         
//          direct:1,��ת  0,��ת 
//��speed=0ʱ���ֹͣת�����˴�ͨ��д0������ɲ��
void motor_run(USART_TypeDef* UARTx,uint32_t speed,uint8_t direct,uint8_t id){
	
  float angle_data=speed*360/(2*3.14*70);
  uint16_t temp=(uint16_t)angle_data;

	if(id==RMotor)
	{
		if(speed!=0)
			DirectR=direct;
	}
	else if(id==LMotor)
	{
		if(speed!=0)
		 DirectL=direct;
	}

  parm_tx.count = 3;
  parm_tx.data[0]=0x20;//�Ĵ�����ַ
  parm_tx.data[1]=(uint8_t)(temp&0xff);//�ٶ�ֵ��λ
  parm_tx.data[2]=(uint8_t)((temp&0xff00)>>8|(direct<<7));//�ٶ�ֵ��λ ���λΪ����λ

  Inspack_Set(WRITE_DATA,id);
  Motor_send_insturction(UARTx);

}
//��õ������
void set_hub_motor_dir(uint8_t dir_r,uint8_t dir_l)
{
	
	DirectR = dir_r;
	DirectL = dir_l;

}
//��ȡ����ٶ�
void set_hub_motor_speed(uint16_t lSpeed, uint16_t rSpeed)
{
    uSpeedL = lSpeed ;
    uSpeedR = rSpeed ;
}
//�ⲿ����
void Motor_heartbeat(void){
    
	uint8_t dir_r=0;
	uint8_t dir_l=0;
	uint32_t speedR=0;
	uint32_t speedL=0;
          
  speedR=(uint32_t)uSpeedR;
  speedL=(uint32_t)uSpeedL;
	
	dir_r = DirectR;
	dir_l = DirectL;
	
//	speedR=100;
//  speedL=100;
//	
//	dir_r = 0;
//	dir_l = 0;
	
	motor_run(MOTOR_USART,speedL, dir_l,LMotor);
  delay_ms(1);
	delay_us(500);
	motor_run(MOTOR_USART,speedR, dir_r,RMotor);
  delay_ms(1);
	delay_us(500);
        	
}
//�������ܣ�����ǰ�Ƕ�ֵ
//���Ҷ�Ҫ����ʱ������һ������֮����ʱһ��ʱ���ٽ��ж���һ��
void ReadAngleData(USART_TypeDef* UARTx,uint8_t id){
	
  parm_tx.count=2;
  parm_tx.data[0]=0x28;//�Ĵ�����ַ����ǰ�ǶȼĴ�����ַ
  parm_tx.data[1]=0x02;//�������ֽ�

  Inspack_Set(READ_DATA,id);
  Motor_send_insturction(UARTx);
  
}

//�������ܣ�������ϴε��ת���ǶȲ�ֵ
uint16_t Read_angle(void){
	
  uint16_t angle=0;

  angle=res.parm.data[0]|res.parm.data[1]<<8; 

  return angle;
  
}

//�������ܣ���ѯ�յ��ĽǶ�ֵ
//���������RS485ͨ�Ž��յ�������,���յ����ֽ��� 
//          *data:RS485_RX_BUF
// ���������16λ�޷���ֵ
uint16_t Recive_angle_data(uint8_t cnt,uint8_t *buf){
  char i=0;
  for(i=0;i<cnt;i++){
      if(buf[i]==0xd5&&buf[i+1]==0x5d){
        if(buf[i+2]==0x0A){
                
          switch(buf[i+3]){
        
                case STOP:Flag_lock_R=1;Motor_response(buf,i);break;
                default:Flag_lock_R=0;Motor_response(buf,i);break;
                        
          }
        }
        else if(buf[i+2]==0x0B){
        
                switch(buf[i+3]){
        
                case STOP:Flag_lock_L=1;Motor_response(buf,i);break;
                default:Flag_lock_L=0;Motor_response(buf,i);break;
                        
          }
        }
      }
   }
    return Read_angle();
}

//��õ�ǰ���ת���Ƕ�  360ΪһȦ
uint16_t GetCount(uint8_t id)
{
	
	uint16_t value = 0;
	
	ReadAngleData(MOTOR_USART,id);
	
	delay_ms(2);
	
	if(LMotor == id)
	{
	
	now_angle_l = Recive_angle_data(RS485_RX_CNT,RS485_RX_BUF);
	
	value = CalculateAngleL();
		
  last_angle_l = now_angle_l;
	
	return value;
	}
	if(RMotor == id)
	{
	
	now_angle_r = Recive_angle_data(RS485_RX_CNT,RS485_RX_BUF);
	
	value = CalculateAngleR();
		
  last_angle_r = now_angle_r;
	
	return value;
	}

	
	return 0;
}

uint16_t CalculateAngleR(void){

    uint16_t angle_increment_r=0;
  
    if(DirectR == ForwardR){
  
          if((now_angle_r > last_angle_r)&&((now_angle_r - last_angle_r)>1000))
          {  
            angle_increment_r = 65535 - now_angle_r + last_angle_r;
          }

          else if(((now_angle_r < last_angle_r)||(now_angle_r == last_angle_r))&&((last_angle_r - now_angle_r) < 1000 )){ 
                 angle_increment_r =last_angle_r-now_angle_r;
                 }
                else
                  angle_increment_r =  0;
      
        last_angle_r = now_angle_r;  
  
    } 
  
    else if(DirectR == BackwardR){
  
        if((now_angle_r < last_angle_r)&&((last_angle_r - now_angle_r) > 1000))
        {
              angle_increment_r = 65535 - last_angle_r + now_angle_r;
        }
      
        else if(((now_angle_r > last_angle_r)||(now_angle_r ==last_angle_r))&&((now_angle_r - last_angle_r) < 1000 ))  
               {
              angle_increment_r = now_angle_r - last_angle_r;
                }
              else
                angle_increment_r = 0;
        
        last_angle_r = now_angle_r;
  
    }
  
    return angle_increment_r;

}

uint16_t CalculateAngleL(void){

    uint16_t angle_increment_l=0;
    
    if(DirectL == BackwardL){  
  
      if((now_angle_l > last_angle_l)&&((now_angle_l - last_angle_l) > 1000 ))  
        angle_increment_l = 65536 - now_angle_l + last_angle_l;       
      
      else if(((now_angle_l < last_angle_l)||(now_angle_l == last_angle_l))&&((last_angle_l - now_angle_l) < 1000 ))
               angle_increment_l = last_angle_l - now_angle_l;
           else
               angle_increment_l =  0;
    
    last_angle_l = now_angle_l;
  
    } 
  
    else if(DirectL == ForwardL){
  
      if((now_angle_l < last_angle_l)&&((last_angle_l - now_angle_l) > 1000 ))
          angle_increment_l = 65536 - last_angle_l + now_angle_l;

      else if(((now_angle_l > last_angle_l)||(now_angle_l == last_angle_l))&&((now_angle_l - last_angle_l) < 1000 ))
               angle_increment_l = now_angle_l - last_angle_l;
           else
               angle_increment_l = 0;
    
    last_angle_l = now_angle_l;   

    }
    return angle_increment_l;
}

//�������ܣ���ѯ�ҵ���Ƿ��ת
//���������
//���������
char MotorLockR(void){
	
   return Flag_lock_R;

}
//�������ܣ���ѯ�����Ƿ��ת
//���������
//���������
char MotorLockL(void){
	
    return Flag_lock_L;

}
char MotorLock(void){
  
    return MotorLockR()|MotorLockL();

}

