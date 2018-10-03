#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "sys.h"

extern volatile unsigned long long  sys_time;
extern unsigned short gsLeft_Pulse;
extern  unsigned short gsRight_Pulse;
extern u32   right_lasttime ; 
extern u32   left_lasttime ;
extern u32 L_speed_Now,R_speed_Now;
extern FlagStatus TimeOutfalg;

void init_time(void);
void GetLeftInterruptCount(u16 *left_count);
void GetRightInterruptCount(u16 *right_count);


#endif

