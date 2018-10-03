#ifndef _COMMUNICATIONMAIN_H
#define _COMMUNICATIONMAIN_H
#include "protocol.h"
#include "sys.h"
#define Sendbuf_LEN  		1024 //定义最大发送字节数 1K
extern u8 TEXT_TO_SEND[250];
extern TransportProtocol_Typedef TransportProtocol;
extern u8 Length_TEXT_TO_SEND;
void SendDateToUSART1(u8* Date,u16 Length);
void init_time3(void);
#endif




