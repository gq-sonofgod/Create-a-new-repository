#include "HealthMode.h"
#include "Main.h"
#include "stm8s_it.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "EEPROM.h"
#include "string.h"

u32 TIMEcnt; 
u8 TimeNow = TIMEVAL;   //TIMEVAL = 45
u8 MinValue = 45;       //���ѵ���ʱ��ʱ��ֵ
u8 SecAlarmFlag = 0;
u8 AlarmFlag = 0;
u8 BuzzerFlag = 0;
u8 SetFlag = 0;
u8 FlashFlag = 0;
u16 KeyPressDownCnt = 0;
/***********************************************************************************************************
* ��������: HealthModeTimeProc()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����ģʽʱ�䴦�� ����ʱ���ж�ִ�У�ÿ1msִ��һ�Σ�
************************************************************************************************************/
void HealthModeTimeProc(void)
{
  if(HealthMode == 1)
  {
    if(T45MinuteCnt>0)
      T45MinuteCnt--;
    if(++T1sCnt > 1000)
      T1sFlag = 1;
    if((T45MinuteCnt%60000)==0)//1����
    {
      if(MinValue > 0)
        MinValue--;
    }
    if(SecAlarmFlag ==1)
    {
      T5MinuteCnt++;
      if(T5MinuteCnt >= 300000)
      {
        T5MinuteCnt = 0;
        AlarmFlag = 1;
        SecAlarmFlag = 2;
      }
    }
    if(AlarmFlag == 1)
    {
      if((T30sCnt%350)==0)
      {
        BuzzerFlag ^= 1;
      }
      T30sCnt++;
      if(T30sCnt >= 10000)
      {
        T30sCnt = 0;
        HealthModeReset();
        /*if(SecAlarmFlag != 2)
          SecAlarmFlag = 1;
        else if(SecAlarmFlag == 2)
        {
          SecAlarmFlag = 0;
          T5MinuteCnt = 0;
          AlarmFlag = 0;
          T30sCnt = 0;
          BuzzerState=OFF;
          HealthMode = 0;
          DisplayMode = HeightMode;
          T45MinuteFlag = 0;
          MinValue = 0;
        }
        AlarmFlag = 0;
        BuzzerState=OFF;*/
      }
    }
    if((T45MinuteCnt == 0)&&(SecAlarmFlag!=1))
    {
      T45MinuteCnt = TIMEcnt;
      T45MinuteFlag = 1;
      AlarmFlag = 1;
    }   
  }
  if(((HealthMode == 1)||(HealthMode == 2)||(HealthMode == 3)||(HealthMode == 6))&&(DisplayRemind==ON)&&( sensKeyFlag!=2))
  {
    if((M1Cmd == CmdNull)&&(M2Cmd == CmdNull))
    {
      if(T2sCnt < 1500)
        T2sCnt++;
      if(T2sCnt >= 1500)
      {
        if(HealthMode == 2)
        {
          DisplayMode = RemindMode;
          HealthMode = 1;
        }
        else if(HealthMode == 3)
        {
          HealthMode = 0;
          DisplayMode = HeightMode;
        }
        else if((HealthMode == 1)&&(ErrCode==0))
        {
          DisplayMode = RemindMode;
        }
        else if(HealthMode == 6)
        {
          HealthMode = 5;
          if(SetFlag == 0)
          {
            Dis_Char[0] = Char_Pt;
            Dis_Char[1] = Data_Char[TimeNow/10];
            Dis_Char[2] = Data_Char[TimeNow%10];
            
            SetFlag = 1;
            T1MinuteCnt = 0;
            T1secFlag = 0;
            FlashFlag = 1;
            T5secCnt = 0;
          }
        }
      }
    }
  }
  if(HealthMode == 5)
  { 
    if(SetFlag == 1)
    {
      if(++T1MinuteCnt >= 500)          //500ms   M500_mSECONDS
      {
        T1MinuteCnt = 0;    
        T1secFlag = 1;                  //��־λ��1
      }
      if(++T5secCnt >= FIVE_SECONDS)    //5s��   FIVE_SECONDS = 5000
      {
        T5secCnt = 0;    
        T5secFlag = 1;
      }
    }
  }
  else
  {
    T1MinuteCnt = 0;
    T1secFlag = 0;
    T5secCnt = 0;
    T5secFlag = 0;
  } 
}


/***********************************************************************************************************
* ��������: HealthModeON()
* �������: ��
* ����ֵ  : ��
* ��    ��: ��ʼ������ģʽ��صı���
************************************************************************************************************/
void HealthModeON(void)
{
  if(HealthMode == 1)
  {
    HealthMode = 5;
    DisplayMode = RemindMode;
  }
  else if(HealthMode == 0)      //��һ�ΰ���A���Ĳ���
  {
    //HealthMode = 1;
    HealthMode = 6;             //HealthMode = 6 
    //SetFlag = 1;
    T45MinuteCnt = TIMEcnt;
    MinValue = TimeNow; //TIMEVAL
    SecAlarmFlag = 0;
    T5MinuteCnt = 0;
    AlarmFlag = 0;
    T30sCnt = 0;
    T1sCnt = 0;
    T1sFlag = 0;
    T2sCnt = 0;
    BuzzerState=OFF;
    DisplayMode = RemindMode;             
    Dis_Char[0] = 0;
    Dis_Char[1] = Char_O;
    Dis_Char[2] = Char_N;
  }
}

/***********************************************************************************************************
* ��������: HealthModeReset()
* �������: ��
* ����ֵ  : ��
* ��    ��: ���ý���ģʽ�ļ�ʱ
************************************************************************************************************/
void HealthModeReset(void)
{
  T45MinuteCnt = TIMEcnt;   //TIMEcnt = 60000ms * 45
  MinValue = TimeNow;       //TimeNow = TIMEVAL;   //TIMEVAL = 45
  SecAlarmFlag = 0;
  T5MinuteCnt = 0;
  AlarmFlag = 0;
  T30sCnt = 0;
  if(HealthMode == 1)
    HealthMode = 2;
  DisplayMode = HeightMode;
  T2sCnt = 0;
  BuzzerState = OFF;
}


/***********************************************************************************************************
* ��������: HealthModeOFF()
* �������: ��
* ����ֵ  : ��
* ��    ��: �رս���ģʽ
************************************************************************************************************/
void HealthModeOFF(void)
{
  T45MinuteCnt = TIMEcnt;
  MinValue = TimeNow; 
  SecAlarmFlag = 0;
  T5MinuteCnt = 0;
  AlarmFlag = 0;
  T30sCnt = 0;
  BuzzerState=OFF;
  HealthMode = 3;
  T45MinuteFlag = 0;
  MinValue = 0;
  T2sCnt = 0;
  T1sCnt = 0;
  T1sFlag = 0;
  Dis_Char[0] = Char_O;
  Dis_Char[1] = Char_F;
  Dis_Char[2] = Char_F;
}


/***********************************************************************************************************
* ��������: HealthModeProc()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����ģʽ������
************************************************************************************************************/
void HealthModeProc(void)
{
  if(HealthMode==1)
  {
    if((T2sCnt>=1500)&&(T1sFlag==1)) //1s����һ�ε���ʱ��ֵ
    {
      T1sCnt = 0;
      T1sFlag = 0;  
      BlinkFlag ^= 1;  
      if(BlinkFlag == 1)
      {
        Dis_Char[0] = Char_Pt;;
      }
      else
      {
        Dis_Char[0] = 0;
      }
      Dis_Char[1] = Data_Char[MinValue/10];
      Dis_Char[2] = Data_Char[MinValue%10]; 
    }
    if(AlarmFlag == 1)
    {
      if(BuzzerFlag == 1)
      {
        BuzzerState = ON;
      }
      else
      {
        BuzzerState=OFF;
      }
    }
  }
}

/***********************************************************************************************************
* ��������: HealthModeTimerSet()
* �������: key������ֵ
* ����ֵ  : ��
* ��    ��: ����ģʽʱ�����ã���������ִ�У�ÿ10msִ��һ�Σ�
************************************************************************************************************/
void HealthModeTimerSet(u16 value)
{
  Key = KeyScan(value);
  
  switch(Key)
  {
    case KEY_UP:
      //T5secCnt = 0;
      SetFlag = 0;
      if(KeyPressDownCnt>=300)
      {
        KeyPressDownCnt = 0;
        if(++TimeNow >= 100)
        TimeNow = 1;
      }
      break;
    
    case KEY_DOWN:
      //T5secCnt = 0;
      SetFlag = 0;
      if(KeyPressDownCnt>=300)
      {
        KeyPressDownCnt = 0;
        if(TimeNow > 1)
          TimeNow--;
        else if(TimeNow == 1)
          TimeNow = 99;
      }
      break;
      
    case KEY_A_LONG:
      T5secCnt = 0;
      HealthModeOFF();
      break;
     
    case KEY_M1:
    case KEY_M2:
    case KEY_M3:
    case KEY_MEM:
      
    case KEY_A:
      if(Release == 1)
      {
        memcpy(&Buffer[34],&TimeNow, sizeof(TimeNow));
        EEPROM_Write();
        TIMEcnt = ONE_MINUTE* TimeNow;
        T45MinuteCnt = TIMEcnt;
        MinValue = TimeNow; // TIMEVAL
        SecAlarmFlag = 0;
        T5MinuteCnt = 0;
        AlarmFlag = 0;
        T30sCnt = 0;
        BuzzerState=OFF;	  
        HealthMode = 1;
        DisplayMode = RemindMode; 
        SetFlag = 0;
        Dis_Char[0] = Char_Pt;
        Dis_Char[1] = Data_Char[TimeNow/10];
        Dis_Char[2] = Data_Char[TimeNow%10];      
      }
      break;
    
    case KEY_NULL:
      Release = 1;
      if((SetFlag == 0)&&(T2sCnt>=1500))
      {
        SetFlag = 1;
        T1MinuteCnt= 0;
        T1secFlag = 0;
        FlashFlag = 0;
        T5secCnt = 0;
      }
      break;
  }
  if(T5secFlag==1)      //5s��ʱʱ�䵽�����붨ʱ����
  {
    memcpy(&Buffer[34],&TimeNow, sizeof(TimeNow));
    EEPROM_Write();
    TIMEcnt = ONE_MINUTE* TimeNow;
    T45MinuteCnt = TIMEcnt;
    MinValue = TimeNow; // TIMEVAL
    SecAlarmFlag = 0;
    T5MinuteCnt = 0;
    AlarmFlag = 0;
    T30sCnt = 0;
    BuzzerState=OFF;	  
    HealthMode = 1;
    DisplayMode = RemindMode;   	  
  }
  if((SetFlag == 1)&&(T2sCnt>=1500))
  {
    if((T1secFlag==1)) //1s����һ�ε���ʱ��ֵ
    {
      T1secFlag = 0;
      FlashFlag ^= 1;
      if(FlashFlag == 1)
      {
        Dis_Char[0] = Char_Pt;
        Dis_Char[1] = Data_Char[TimeNow/10];
        Dis_Char[2] = Data_Char[TimeNow%10];
      }
      else
      {
        Dis_Char[0] = Char_Pt;
        Dis_Char[1] = 0;
        Dis_Char[2] = 0;
      }
    }
  }
  else if(SetFlag == 0)
  {
    Dis_Char[0] = Char_Pt;
    Dis_Char[1] = Data_Char[TimeNow/10];
    Dis_Char[2] = Data_Char[TimeNow%10]; 
  }
}
