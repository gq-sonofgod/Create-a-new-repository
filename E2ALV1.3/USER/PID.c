#include "PID.h"
#include "Motor.h"
#include "Timer.h"
#include "stm8s_it.h"
#include "Balance.h"

#define BALANCE_MAXSPD  35
#define BALANCE_MINSPD  6


PID M1PID;
PID M2PID;

SyncPID  M1SyncPID;
SyncPID  M2SyncPID;

AnglePid AnglePID;

/***********************************************************************************************************
* ��������: M1PID_Init()
* �������: speedref, Ŀ���ٶ�
* ����ֵ  : ��
* ��    ��: ���1��PID������ʼ��
************************************************************************************************************/
void M1PID_Set(u8 speedref, u16 Duty)   //��speedref=MINSPEED=35��Duty=100��
{
  M1PID.SetSpeed = speedref;    //�ٶ�18 ��ӦPID Ϊ Kp=0.25, Ki=0.31, Kd = 0.14
  M1PID.PWMDuty = Duty;         //�ٶ�8 ��ӦPID Ϊ Kp=0.18, Ki=0.19, Kd = 0.17
  M1PID.CurrSpeed = 0;
  if(speedref == Speed)         // Speed = SPEED #define SPEED 60
  {
    M1PID.Kp = KP1;
    M1PID.Ki = KI1;
    M1PID.Kd = KD1;
  }
  else if(speedref == MINSPEED)
  {
    M1PID.Kp = KP2;
    M1PID.Ki = KI2;
    M1PID.Kd = KD2;
  }
  M1PID.LastDev = 0;    //M1ת����һ�ε�ƫ��ֵ
  M1PID.PrevDev = 0;    //M1ת�����ϴε�ƫ��ֵ
  M1PID.SyncDev = 0;    //���ͬ��ƫ��ֵ������������Ͽ��ƣ�
  
  M1SyncPID.CurDev = 0; //��ǰ�߶�ƫ��ֵ
  M1SyncPID.LastDev = 0;//�ϴθ߶�ƫ��ֵ        
  M1SyncPID.PrevDev = 0;//���ϴθ߶�ƫ��ֵ
  M1SyncPID.SyncVal = 0;//ͬ��ƫ��ֵ
  T100msCnt1 = 0;
}

/***********************************************************************************************************
* ��������: M2PID_Init()
* �������: speedref, Ŀ���ٶ�
* ����ֵ  : ��
* ��    ��: ���2��PID������ʼ��
************************************************************************************************************/
void M2PID_Set(u8 speedref, u16 Duty)
{
  M2PID.SetSpeed = speedref; //�ٶ�18 ��ӦPID Ϊ Kp=0.25, Ki=0.31, Kd = 0.14
  M2PID.PWMDuty = Duty;         //�ٶ�8 ��ӦPID Ϊ Kp=0.18, Ki=0.19, Kd = 0.17
  M2PID.CurrSpeed = 0;
  if(speedref == Speed)
  {
    M2PID.Kp = KP1;
    M2PID.Ki = KI1;
    M2PID.Kd = KD1;
  }
  else if(speedref == MINSPEED)
  {
    M2PID.Kp = KP2;
    M2PID.Ki = KI2;
    M2PID.Kd = KD2;
  }
  M2PID.LastDev = 0;
  M2PID.PrevDev = 0;
  M2PID.SyncDev = 0;
  M2SyncPID.CurDev = 0;
  M2SyncPID.LastDev = 0;
  M2SyncPID.PrevDev = 0;
  M2SyncPID.SyncVal = 0;
  T100msCnt2 = 0;
}





/***********************************************************************************************************
* ��������: PID_Init()
* �������: speedref
* ����ֵ  : ��
* ��    ��: PID��ʼ��
************************************************************************************************************/
void PID_Set(u8 speedref, u16 Duty)
{
  M1PID_Set(speedref, Duty);
  M2PID_Set(speedref, Duty);
  //M3PID_Set(speedref);
}

/***********************************************************************************************************
* ��������: PIDCal()
* �������:  *v , PID����ֵ
* ����ֵ  : ��
* ��    ��: ����ʽPID���ں���
************************************************************************************************************/
s16 yyp;
s8 yp;;
void PIDCal(PID* velo)
{
  s16 CurDev;
  
  float Incpid;
  
  //if (SysCmd == CmdSetBalance)
  if (Balance_Data.BalanceAdjuseState != 0)
  {
    CurDev = velo->SetSpeed - velo->CurrSpeed;
    Incpid = velo->Kp*CurDev - velo->Ki*velo->LastDev + velo->Kd*velo->PrevDev;    
  }
  else
  {
    yp = velo->SyncDev;           //���ͬ��ƫ��ֵ
    CurDev = velo->SetSpeed - velo->CurrSpeed - velo->SyncDev;                    //���������ٶ�ƫ������Ҫ����ͬ��ƫ����
    Incpid = velo->Kp*CurDev - velo->Ki*velo->LastDev + velo->Kd*velo->PrevDev;
  }
  yyp = velo->PrevDev;
  velo->PrevDev = velo->LastDev;
  velo->LastDev = CurDev;
  
  if(Incpid*1050/Speed + ((s16)velo->PWMDuty)<0)
  {
    velo->PWMDuty = 10;
  }
  else
  {
    velo->PWMDuty = (u16)(velo->PWMDuty + Incpid*1050/Speed);   //��ǰһ��ռ�ձȵĻ����Ͻ�������ʽ����
  }

  if (Balance_Data.BalanceAdjuseState != 0)
  {
    if(velo->PWMDuty > 700)
    velo->PWMDuty = 700;
    if(velo->PWMDuty < 100)
    velo->PWMDuty = 100;
  }
  else
  {
    //ȷ��ռ�ձȷ�Χ
    if(velo->PWMDuty > 1215)
      velo->PWMDuty = 1215;
    if(velo->PWMDuty < 10)
      velo->PWMDuty = 10;
  }
}

/***********************************************************************************************************
* ��������: SyncPIDCal()
* �������: syncpid, ͬ��PID����
* ����ֵ  : ��
* ��    ��: ˫����г�ͬ��PID����
************************************************************************************************************/
void SyncPIDCal(SyncPID *syncpid)
{
  float Incpid;
  
  Incpid = syncpid->Sync_KP * (syncpid->CurDev) - syncpid->Sync_KI * (syncpid->LastDev) + syncpid->Sync_KD * ( syncpid->PrevDev);
  syncpid->PrevDev = syncpid->LastDev;
  syncpid->LastDev = syncpid->CurDev;
  syncpid->SyncVal = Incpid*0.35;
}

/**********************************************************************************************************/
void AnglePIDCal(AnglePid *anglepid)
{
  float Incpid;
  s16 CurDiffValur;
  
  //CurDiffValur = anglepid->Fact_Para - anglepid->Ref_Para;       //
  
  if (Balance_Data.Acc_yTemp >= 0)
  {
    CurDiffValur = anglepid->Fact_Para - anglepid->Ref_Para;
  }
  else 
  {
    CurDiffValur = anglepid->Ref_Para  - anglepid->Fact_Para;
  }
  
  Incpid = anglepid->Kp * CurDiffValur + anglepid->Ki * anglepid->LastDiffValur + anglepid->Kd * anglepid->PrevDiffValur;
  anglepid->PrevDiffValur = anglepid->LastDiffValur;
  anglepid->LastDiffValur = CurDiffValur;
  
  //Incpid = (s32)(Incpid)>>1;
  
  Incpid = (s32)(Incpid)>>1;
  
  //anglepid->PWMDuty = (u16)(anglepid->PWMDuty + Incpid);
  
  anglepid->OutPut = (u16)(anglepid->OutPut + Incpid);
  
  
  if (anglepid->OutPut > BALANCE_MAXSPD)
  {
    anglepid->OutPut = BALANCE_MAXSPD;
  }
  else if (anglepid->OutPut < BALANCE_MINSPD)
  {
    anglepid->OutPut = BALANCE_MINSPD;
  }
  
    /*
    if(anglepid->PWMDuty > 1215)
      anglepid->PWMDuty = 1215;
    if(anglepid->PWMDuty < 10)
      anglepid->PWMDuty = 10;
    */
  
    /*
    if(anglepid->PWMDuty > 600)         //����ٶȼ���
      anglepid->PWMDuty = 600;
    if(anglepid->PWMDuty < 200)
      anglepid->PWMDuty = 200;
    */
  
    /*
    if(anglepid->PWMDuty > 200)         //����ٶȼ���
      anglepid->PWMDuty = 200;
    if(anglepid->PWMDuty < 200)
      anglepid->PWMDuty = 200;
    */
    
  /*
    if(anglepid->PWMDuty > 700)         //����ٶȼ���
      anglepid->PWMDuty = 700;
    if(anglepid->PWMDuty < 350)
      anglepid->PWMDuty = 350;
  
    if (((anglepid->Fact_Para < 130) && (anglepid->Fact_Para > 0)) ||
        ((anglepid->Fact_Para > -130) && (anglepid->Fact_Para < 0)))
    {
      anglepid->PWMDuty = 250;
    }
  */
  
}

