#ifndef BALANCE_H
#define BALANCE_H
#include "stm8s.h"

#define MAX_HEIGHT_DIFF         10  //30CM

#define MAX_HALL_DIFF           6235//(MAX_HEIGHT_DIFF * 155.8742) 

typedef struct
{
  //u8 PowerInit_Flag;            //系统刚上电标志位
  //u8 Balance_EN;                //使能调整平衡
  //u8 RunMotorState;             //电机运行状态位，1为电机M1运行标志，2位电机M2运行标志
  
  u8 BalanceAdjuseState;        //记录平衡调整的各个状态  0：未进入调整状态  1：正在调整  2：调整减速阶段

  u8  RetardM1Stop;
  u16 RetardM1StopCnt;
  
  u8  RetardM2Stop;
  u16 RetardM2StopCnt;
  
  //s32 AccXl_yTemp;
  
  s16 TwoMotorOffsetHall;       //调平衡过程中，两电机间产生的HALL偏差值，偏差值 = M1HALL值 - M2HALL值
  
  
  u16 TableLenth;               //桌子在两个电机方向上的距离（单位cm）
  
  u16 DecideMotorRunTimerCnt;
  u16 DecideMotorRunState;
  
  u16 MotorSlideTimerCnt;       //电机由关闭电源后进行滑行阶段的时间计数器
  
  u16 AnglePidTimerCnt;
   
  s16 Acc_yTemp;
  
  //u16 BuzzerOnTimerCnt;
  u16 HallCheckDelayTimerCnt;
  u8  HallCheckState;          
  
  u8 TwoMotorRunFaultFlag;
  
  u8 TwoMotorRunFlag;           //当需要进行平衡调整时，若该值为0，，且Y值大于0，则电机运行方向为M1Up，M2Down，
                                                      //若该值为0，，且Y值小于0，则电机运行方向为M1Down，M2Up，
                                //当需要进行平衡调整时，若该值为1，，且Y值大于0，则电机运行方向为M1Down，M2Up，
                                                      //若该值为1，，且Y值小于0，则电机运行方向为M1Up，M2Down，
  u16 DelayTimerCnt;
  
  u16 MaxHallDif;
  
}BALANCE_STR;



#define BALANCE_DATE_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

extern void SetBalance();
extern void SetBalaceState(u8 state);
extern u8 GetBalaceState();
extern s16 GetTwoMotorOffsetHall();
extern u8 Ref_Para_MiniSpeed;
extern BALANCE_STR Balance_Data;
extern void Adjust_Balance_StepByStepo();
#endif