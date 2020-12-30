#ifndef PID_H
#define PID_H
#include "stm8s.h"

typedef struct
{
  u8 SetSpeed;   //Ŀ���ٶ�
  u8 CurrSpeed;  //��ǰʵ���ٶ�
  s8 SyncDev;    //���ͬ��ƫ��ֵ������������Ͽ��ƣ�
  u16 PWMDuty;   //PWMռ�ձȵ���ֵ������PID���ں���Ҫ�趨��PWMֵ
  float Kp;      //����ϵ��
  float Ki;      //����ϵ��
  float Kd;      //΢��ϵ��
  s16 LastDev;    //��һ�ε�ƫ��ֵ Dev[-1]
  s16 PrevDev;    //���ϴε�ƫ��ֵ Dev[-2]
}PID;

typedef struct
{
  s16 CurDev;  //��ǰ�߶�ƫ��ֵ
  s16 LastDev; //�ϴθ߶�ƫ��ֵ
  s16 PrevDev; //���ϴ˵ĸ߶�ƫ��ֵ
  float SyncVal;  //ͬ��ƫ��ֵ
  float Sync_KP; //ͬ������ϵ��
  float Sync_KI; //ͬ������ϵ��
  float Sync_KD; //ͬ��΢��ϵ��
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
  //u16 PWMDuty;          //PWMռ�ձȵ���ֵ������PID���ں���Ҫ�趨��PWMֵ
  
  u16 OutPut;          //AD��PI���ֵ��Ϊ�ο��ٶ�
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