#ifndef __CHECK_H
#define __CHECK_H

#include "stm32f10x.h"



u16 Checksum_Sum(u8* buf,u16 len);   //和校验 所有字节之和为0
u16 Checksum_XOR(u8* buf, u16 len);	//异或校验，所有字节异或
u16 Checksum_CRC8(u8 *buf,u16 len);	//CRC8 校验
u16 Checksum_CRC16(u8 *buf,u16 len);//CRC16 校验



#endif

