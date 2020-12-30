#ifndef STM8S_IT_H
#define STM8S_IT_H
#include "stm8s.h"



extern u16 DelayCntForTap;
extern u16 ResetCnt;

extern u32 MotorRunTime;
extern u32 MotorRunTotal;
extern u32 MotorStopTime;
extern u32 RunTimeTemp;
extern u16 T500msCnt;
extern u8 TimeFlag;
extern u16 T2sCnt,T5secCnt;
extern u8 T1sFlag, T1secFlag, T5secFlag;
extern u16 T1sCnt;
extern u16 T30sCnt;
extern u8 BlinkFlag;
extern u32 T45MinuteCnt;
extern u8  T45MinuteFlag;
extern u32  T5MinuteCnt, T1MinuteCnt;
extern u16 RSTCnt;
extern u8 T100msCnt1;//电机1速度计算计时值
extern u8 T100msCnt2;//电机2速度计算计时值
extern u8 T10msFlag;
extern u16 T400msCnt;

extern u8 RetardStop;
extern u16 RetardStopCnt;

extern u16 BuzzerTestCnt;
extern u32 AgingTime;
extern u16 AgingTurnTime;
extern u32 AgingAllTime;
extern u8  AgingTurnFlag;
extern u16 T100msCnt3;
extern u16 M1ErrCnt;
extern u16 M2ErrCnt;
extern u32 T18minCnt;
extern u16 InitCnt;
extern u16 T2sCnt1;
extern u16 M1T3sCnt;
extern u16 M2T3sCnt;
extern u16 RunCnt;


extern u16 MenuTime;
extern u16 UpKeyDelay;
extern u16 DownKeyDelay;

#endif