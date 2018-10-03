#ifndef _RUNDEAL_H
#define _RUNDEAL_H
#include "sys.h"
void GetRobotRunDIRSpeed(short runspeed,short radiusval);
void MotorSpeedOutput(unsigned short left_motorspeed,unsigned short right_motorspeed);
//void CalcWheelSpeed(unsigned short *leftmotorspeed,unsigned short *rightmotorspeed,short *runspeed,short *radiusval);
extern FlagStatus gsInfraredStopFlag;
extern FlagStatus gsReceiveSpeedTimeoutFlag;
//void Speed_Smooth_Output(unsigned short* motorspeedset, unsigned short motorspeedset_target);

#define ForwardR 1
#define BackwardR 0

#define ForwardL 0
#define BackwardL 1

extern const float perimeter;
extern const float WHEEL_RADIUS;
extern const u16 PlusePerRound;
extern const float b;

extern unsigned short gsLETF_MotorSpeed,gsRIGHT_MotorSpeed; //×óÓÒÂÖµÄËÙ¶È
#endif

