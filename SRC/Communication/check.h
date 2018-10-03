#ifndef __CHECK_H
#define __CHECK_H

#include "stm32f10x.h"



u16 Checksum_Sum(u8* buf,u16 len);   //��У�� �����ֽ�֮��Ϊ0
u16 Checksum_XOR(u8* buf, u16 len);	//���У�飬�����ֽ����
u16 Checksum_CRC8(u8 *buf,u16 len);	//CRC8 У��
u16 Checksum_CRC16(u8 *buf,u16 len);//CRC16 У��



#endif

