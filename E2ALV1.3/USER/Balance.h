#ifndef BALANCE_H
#define BALANCE_H
#include "stm8s.h"

#define MAX_HEIGHT_DIFF         10  //30CM

#define MAX_HALL_DIFF           6235//(MAX_HEIGHT_DIFF * 155.8742) 

typedef struct
{
  //u8 PowerInit_Flag;            //ϵͳ���ϵ��־λ
  //u8 Balance_EN;                //ʹ�ܵ���ƽ��
  //u8 RunMotorState;             //�������״̬λ��1Ϊ���M1���б�־��2λ���M2���б�־
  
  u8 BalanceAdjuseState;        //��¼ƽ������ĸ���״̬  0��δ�������״̬  1�����ڵ���  2���������ٽ׶�

  u8  RetardM1Stop;
  u16 RetardM1StopCnt;
  
  u8  RetardM2Stop;
  u16 RetardM2StopCnt;
  
  //s32 AccXl_yTemp;
  
  s16 TwoMotorOffsetHall;       //��ƽ������У�������������HALLƫ��ֵ��ƫ��ֵ = M1HALLֵ - M2HALLֵ
  
  
  u16 TableLenth;               //������������������ϵľ��루��λcm��
  
  u16 DecideMotorRunTimerCnt;
  u16 DecideMotorRunState;
  
  u16 MotorSlideTimerCnt;       //����ɹرյ�Դ����л��н׶ε�ʱ�������
  
  u16 AnglePidTimerCnt;
   
  s16 Acc_yTemp;
  
  //u16 BuzzerOnTimerCnt;
  u16 HallCheckDelayTimerCnt;
  u8  HallCheckState;          
  
  u8 TwoMotorRunFaultFlag;
  
  u8 TwoMotorRunFlag;           //����Ҫ����ƽ�����ʱ������ֵΪ0������Yֵ����0���������з���ΪM1Up��M2Down��
                                                      //����ֵΪ0������YֵС��0���������з���ΪM1Down��M2Up��
                                //����Ҫ����ƽ�����ʱ������ֵΪ1������Yֵ����0���������з���ΪM1Down��M2Up��
                                                      //����ֵΪ1������YֵС��0���������з���ΪM1Up��M2Down��
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