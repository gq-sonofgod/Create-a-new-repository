#ifndef PID_H
#define PID_H
#include "stm8s.h"

typedef struct
{
  u8 SetSpeed;   //目标速度
  u8 CurrSpeed;  //当前实际速度
  s8 SyncDev;    //电机同步偏差值（多电机交叉耦合控制）
  u16 PWMDuty;   //PWM占空比调节值即经过PID调节后需要设定的PWM值
  float Kp;      //比例系数
  float Ki;      //积分系数
  float Kd;      //微分系数
  s16 LastDev;    //上一次的偏差值 Dev[-1]
  s16 PrevDev;    //上上次的偏差值 Dev[-2]
}PID;

typedef struct
{
  s16 CurDev;  //当前高度偏差值
  s16 LastDev; //上次高度偏差值
  s16 PrevDev; //上上此的高度偏差值
  float SyncVal;  //同步偏差值
  float Sync_KP; //同步比例系数
  float Sync_KI; //同步积分系数
  float Sync_KD; //同步微分系数
}SyncPID;

typedef struct 
{
  /*
  u16 Ref_Para;
  u16 Fact_Para;
  */
  //s16 CurDiffValur;
  
  s16 Ref_Para;
  s16 Fact_Para;
  
  s16 LastDiffValur;
  s16 PrevDiffValur;
  float Kp;
  float Ki;
  float Kd;
  //u16 PWMDuty;          //PWM占空比调节值即经过PID调节后需要设定的PWM值
  
  u16 OutPut;          //AD的PI输出值作为参考速度
}AnglePid;



#define  KP1  0.15//0.35
#define  KI1  0.12//0.27
#define  KD1  0.04//0.11


#define  KP2  0.15
#define  KI2  0.12
#define  KD2  0.05


#define  SKP  0.37
#define  SKI  0.20
#define  SKD  0.18

/*
#define  KP1  0.35//0.35
#define  KI1  0.27//0.27
#define  KD1  0.11//0.11


#define  KP2  0.24
#define  KI2  0.21
#define  KD2  0.12*/



void M1PID_Set(u8 speedref, u16 Duty);
void M2PID_Set(u8 speedref, u16 Duty);

void PID_Set(u8 speedref, u16 Duty);
void PIDCal(PID* velo); 
void SyncPIDCal(SyncPID *syncpid);

void AnglePIDCal(AnglePid *anglepid);
extern PID M1PID;
extern PID M2PID;
extern PID M3PID;
extern SyncPID  M1SyncPID;
extern SyncPID  M2SyncPID;

extern AnglePid AnglePID;



#endif