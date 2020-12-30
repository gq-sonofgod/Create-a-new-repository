#include "FaultDetection.h"
#include "Main.h"
#include "ADC.h"
#include "Key.h"
#include "LED.h"
#include "Motor.h"
#include "PID.h"
#include "Timer.h"
#include "stm8s_it.h"
#include "EXTI.h"
#include "Delay.h"
#include "EEPROM.h"
#include "string.h"
#include "Balance.h"

u8 M1HallN1ErrCnt = 0; //���1�����ź�1�������ֵ
u8 M1HallN2ErrCnt = 0; //���1�����ź�2�������ֵ
u8 M2HallN1ErrCnt = 0; //���2�����ź�1�������ֵ
u8 M2HallN2ErrCnt = 0; //���2�����ź�2�������ֵ

u16 ErrCodeOld = 0;
u8 M1ErrFlag = 0;
u8 M2ErrFlag = 0;
u8 T18minFlag = 0;
u8 M1OverCurFlag = 0;
u8 M2OverCurFlag = 0;
u8 OverCurFlag = 0;  


const u16 Pt[12]={18969,19849,20729,21609,22489,23369,24249,25129,26009,26889,27769,28649};


/***********************************************************************************************************
* ��������: FaultDetector()
* �������: ��
* ����ֵ  : ��
* ��    ��: ϵͳ���ֹ��ϼ��
************************************************************************************************************/
void FaultDetect(void)
{
  static u8 reach = 0;
  CurrentDetect();      //���������

  HallDetect();
  TemperatureDetect();
  if((ErrCode != 0)&&(ErrCode != Err_RESET))    //���Ϻ�һЩ��λ����
  {
    M1Up(OFF);
    M1Down(OFF);   
    M1Cmd = CmdNull;
    M1Dir = 0;
    M1_PWM_OFF;
    
    M2Up(OFF);   
    M2Down(OFF);
    M2Cmd = CmdNull;
    M2Dir = 0;
    M2_PWM_OFF;
    
    SysCmd = CmdNull;
    PID_Set(MINSPEED,BASEDUTYDOWN);
    M1OverCurFlag = 0;
    M2OverCurFlag = 0;
    OverCurFlag = 0;
    
    if((SysState == RESET)&&(reach == 0)&&(ErrCode!=Err_Fall)&&(ErrCode!=Err_LSM6DSL))
    KeyDisable = 1;
    reach = 1;
    
    if(ErrCode != ErrCodeOld)
    {
      if((ErrCode != Err_TimeEnd) &&        //����ʱ�䵽����
         (ErrCode != Err_Overheating) &&    //���¹���
         (ErrCode != Err_LSM6DSL))          //6�����
      {
        M1State.HallNow = BASEHALL;
        M1State.HallLast = BASEHALL;
        M2State.HallNow = BASEHALL;
        M2State.HallLast = BASEHALL;       
        
        memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));
        EEPROM_Write();
      }
    }
    
    ErrCodeOld = ErrCode;
    
    if((ErrCode != Err_TimeEnd) && (ErrCode != Err_Overheating))
    {
      SysState = RESET;     
    }
    if(DisplayMode != TestMode) 
      DisplayMode = ErrorMode;
    
    if(Release == 1)
      Release = 2;
  }
  if((SysState == RESET)&&(ErrCode==Err_RESET))
    reach = 0;
}

/***********************************************************************************************************
* ��������: CurrentDetect()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����������
************************************************************************************************************/
void CurrentDetect(void)
{ 
  u16 temp;
  u16 temp1;
  u8 i;
  u8 offset_hall_flag = 0;      //����ָʾ�ĸ��������λ�ø�  0����ʾM1�ߣ�1��ʾM2��
  
  if((SysState == NORMAL)&&(M1OverCurFlag == 0)&&(M2OverCurFlag == 0))
  { 
    if(ADCValue.M1Current > (MAXCURRENT)) //���1����
      M1OverCurFlag = 1;
    if(ADCValue.M2Current > (MAXCURRENT))  //���2����
      M2OverCurFlag = 1;
    
    
    /*
    if(ADCValue.M1Current > (200)) //���1����
      M1OverCurFlag = 1;
    if(ADCValue.M2Current > (200))  //���2����
      M2OverCurFlag = 1;
    */
    
    if(OverCurFlag == 1)
    {
      M1OverCurFlag = 1;
      M2OverCurFlag = 1;
    }
    
    /*
    if((M1OverCurFlag == 1)||(M2OverCurFlag == 1))
    {
      M1Up(OFF);
      M1Down(OFF);
      M1_PWM_OFF;
      M2Up(OFF);
      M2Down(OFF);
      M2_PWM_OFF;
      PID_Set(Speed,BASEDUTYUP);
      temp = (M1State.HallNow + M2State.HallNow)/2;
      if(Pt[0] > temp)
      {
        M1State.Record[3] = M1State.LimitDown;
        M2State.Record[3] = M2State.LimitDown;
      }
      else if(temp > Pt[11])
      {
        M1State.Record[3] = Pt[11];
        M2State.Record[3] = Pt[11];
      }
      else
      {
        for(i = 1; i < 12; i++)
        {
          if(Pt[i] >= temp)
          {
            M1State.Record[3] = Pt[i-1];
            M2State.Record[3] = Pt[i-1];
            break;
          }
        }
      }
      
      if((M1State.Record[3] < M1State.LimitDown)||(M2State.Record[3] < M2State.LimitDown))
      {
        M1State.Record[3] = M1State.LimitDown;
        M2State.Record[3] = M2State.LimitDown;
      }
      if(ErrCode == 0)
      {
        M1Cmd = CmdGoBack;
        M1Dir = DOWN;
        SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
        M2Cmd = CmdGoBack;
        M2Dir = DOWN;
        SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
        SysCmd = CmdGoBack;
        PID_Set(Speed,BASEDUTYDOWN);
      }
      
      if(M1OverCurFlag==1)
        M1OverCurFlag = 2;
      if(M2OverCurFlag==1)
        M2OverCurFlag = 2;
    }
    */
   
    if((M1OverCurFlag == 1)||(M2OverCurFlag == 1))
    {
      M1Up(OFF);
      M1Down(OFF);
      M1_PWM_OFF;
      M2Up(OFF);
      M2Down(OFF);
      M2_PWM_OFF;
      PID_Set(Speed,BASEDUTYUP);
      
      
      //temp = (M1State.HallNow + M2State.HallNow)/2;
      
      temp =(M1State.HallNow > M2State.HallNow)? M2State.HallNow:M1State.HallNow;
      
      if (GetTwoMotorOffsetHall() >= 0) //TwoMotorOffsetHall = M1 - M2
      {
        offset_hall_flag = 0;   //M1>= M2
      }
      else 
      {
        offset_hall_flag = 1;   //M1 < M2
      }
       
      /*
      if(Pt[0] > temp)
      {
        M1State.Record[3] = M1State.LimitDown;
        M2State.Record[3] = M2State.LimitDown;
      }
      else if(temp > Pt[11])
      {
        M1State.Record[3] = Pt[11];
        M2State.Record[3] = Pt[11];
      }
      else
      {
        for(i = 1; i < 12; i++)
        {
          if(Pt[i] >= temp)
          {
            M1State.Record[3] = Pt[i-1];
            M2State.Record[3] = Pt[i-1];
            break;
          }
        }
      }
      
      if((M1State.Record[3] < M1State.LimitDown)||(M2State.Record[3] < M2State.LimitDown))
      {
        M1State.Record[3] = M1State.LimitDown;
        M2State.Record[3] = M2State.LimitDown;
      }
      */
      
      if (offset_hall_flag == 0)    //M1>= M2
      {
            if(Pt[0] > temp)
            {
              M2State.Record[3] = M2State.LimitDown;
              M1State.Record[3] = M1State.LimitDown + GetTwoMotorOffsetHall();
            }
            else if (M1State.HallNow >= Pt[11])
            {
              M1State.Record[3] = Pt[11];
              temp1 = M1State.HallNow - Pt[11];
              M2State.Record[3] = M2State.HallNow - temp1;
              if (M2State.Record[3] < M2State.LimitDown)
              {
                M2State.Record[3] = M2State.LimitDown;
                M1State.Record[3] = M1State.HallNow - (M2State.HallNow - M2State.LimitDown);
              }
            }
            else
            {
                for(i = 1; i < 12; i++)
                {
                    if(Pt[i] >= temp)
                    {
                        M2State.Record[3] = Pt[i-1];
                        M1State.Record[3] = Pt[i-1] + GetTwoMotorOffsetHall();
                        
                        if (M1State.Record[3] > M1State.LimitUp)//�������ֵ����M2�ص���͵�
                        {
                          M2State.Record[3] = M2State.LimitDown;
                          M1State.Record[3] = M1State.LimitDown + GetTwoMotorOffsetHall();
                        }
                        break;
                    }
                }
            }
      }
      else  //(offset_hall_flag == 1)    //M1 < M2
      {
        if(Pt[0] > temp)
        {
          M1State.Record[3] = M1State.LimitDown;
          M2State.Record[3] = M2State.LimitDown - GetTwoMotorOffsetHall();
        }
        else if (M2State.HallNow >= Pt[11])
        {
          M2State.Record[3] = Pt[11];
          temp1 = M2State.HallNow - Pt[11];
          M1State.Record[3] = M1State.HallNow - temp1;
          if (M1State.Record[3] < M1State.LimitDown)
          {
            M1State.Record[3] = M1State.LimitDown;
            M2State.Record[3] = M2State.HallNow - (M1State.HallNow - M1State.LimitDown);
          }
        }
        else 
        {
            for(i = 1; i < 12; i++)
            {
                if(Pt[i] >= temp)
                {
                    M1State.Record[3] = Pt[i-1];
                    M2State.Record[3] = Pt[i-1] - GetTwoMotorOffsetHall();
                    if (M2State.Record[3] > M2State.LimitUp)    //�������ֵ����M1�ص���͵�
                    {
                        M1State.Record[3] = M1State.LimitDown;
                        M2State.Record[3] = M2State.LimitDown - GetTwoMotorOffsetHall();
                    }
                    break;
                 }
             }
        }
      }
      
      if(ErrCode == 0)
      {
        M1Cmd = CmdGoBack;
        M1Dir = DOWN;
        SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
        M2Cmd = CmdGoBack;
        M2Dir = DOWN;
        SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
        SysCmd = CmdGoBack;
        PID_Set(Speed,BASEDUTYDOWN);
      }
      
      if(M1OverCurFlag==1)
        M1OverCurFlag = 2;
      if(M2OverCurFlag==1)
        M2OverCurFlag = 2;
    }
    
  }
}

/***********************************************************************************************************
* ��������: HallDetect()
* �������: ��
* ����ֵ  : ��
* ��    ��: �����źż��
************************************************************************************************************/
void HallDetect(void)
{  
  if(M1HallN1ErrCnt >= 15)
  {
    M1HallN1ErrCnt = 0;
    if(ErrCode == Err_RESET)    //�ڸ�λʱ���ֻ�������
      ErrCode = Err_M1OneHall;
    else if(ErrCode==0)         //��������ʱ���ֻ�������
      ErrCode = Err_M1OneHall;
  }
  else if(M1HallN2ErrCnt >= 15)
  {
    M1HallN2ErrCnt = 0;
    if(ErrCode == Err_RESET) //�ڸ�λʱ���ֻ�������
      ErrCode = Err_M1OneHall;
    else if(ErrCode==0)  //��������ʱ���ֻ�������
      ErrCode = Err_M1OneHall;
  }
  if(M2HallN1ErrCnt >= 15)
  {
    M2HallN1ErrCnt = 0;
    if(ErrCode == Err_RESET) //�ڸ�λʱ���ֻ�������
      ErrCode = Err_M2OneHall;
    else if(ErrCode==0)  //��������ʱ���ֻ�������
      ErrCode = Err_M2OneHall;
  }
  else if(M2HallN2ErrCnt >= 15)
  {
    M2HallN2ErrCnt = 0;
    if(ErrCode == Err_RESET) //�ڸ�λʱ���ֻ�������
      ErrCode = Err_M2OneHall;
    else if(ErrCode==0)  //��������ʱ���ֻ�������
      ErrCode = Err_M2OneHall;
  }
  if(SysState==RESET)
  {
    if(M1Cmd == CmdDown)
    {
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>450)&&(ADCValue.M1Current<100)) 
      {
        ErrCode = Err_M1AllWire;  //M1��HALL�ߺ͵����ȫ����
      }
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>650)&&(ADCValue.M1Current<150)) 
      {
        ErrCode = Err_M1TwoHall;  //M1��HALL��ȫ����
      }
    }
    else if(M1Cmd == CmdUp)
    {
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>850)) 
      {
        ErrCode = Err_M1TwoHall;  //M1��HALL��ȫ����
      }
    }
    if(M2Cmd == CmdDown)
    {
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>450)&&(ADCValue.M2Current<100))
      {
        ErrCode = Err_M2AllWire;  //M2��HALL�ߺ͵����ȫ����
      }   
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>650)&&(ADCValue.M2Current<150))
      {
        ErrCode = Err_M2TwoHall;  //M2��HALL��ȫ����
      }
    }
    else if(M2Cmd == CmdUp)
    {
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>850))
      {
        ErrCode = Err_M2TwoHall;  //M2��HALL��ȫ����
      }
    }
  }
  //if((SysState == NORMAL)&&(SysCmd!=CmdStop)&&(SysCmd!=CmdNull)) //�����������״̬
  if((SysState == NORMAL)&&(SysCmd!=CmdStop)&& ((SysCmd!=CmdNull) || (GetBalaceState() != 0))) //�����������״̬
  {
    /*if((M1State.HallNow>M1State.LimitDown)&&(M1State.HallNow<M1State.LimitUp)) //M1��HALL����δ�ﵽ�˵�
    {
      if((M1Detect>=4)&&(M1PID.CurrSpeed+7<=SPEEDTHRESHOLD)&&(ADCValue.M1Current>380)&&(ErrCode==0)&&(M1PID.PWMDuty>800)) //���1�ٶȹ��ͣ�������
        ErrCode = Err_M1Position;
      else if(M1PID.CurrSpeed+5>SPEEDTHRESHOLD)
        M1Detect = 0;
    }*/
    if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>650))
    {
      if(M1ErrFlag==0)
      {
        M1ErrFlag = 1; 
      }
    }
    else
    {
      M1ErrFlag = 0;
      M1ErrCnt = 0;
    }
    
    if(M1ErrFlag == 2)
    {
      if(ADCValue.M1Current >= 100)
        ErrCode = Err_M1TwoHall;  //M1������HALL��ȫ���ϵ�
      else
        ErrCode = Err_M1AllWire;
    }
    /*if((M2State.HallNow>M2State.LimitDown)&&(M2State.HallNow<M2State.LimitUp)) //M2��HALL����δ�ﵽ�˵�
    {
      if((M2Detect>=4)&&(M2PID.CurrSpeed+7<=SPEEDTHRESHOLD)&&(ADCValue.M2Current>380)&&(ErrCode==0)&&(M2PID.PWMDuty>800)) //���2�ٶȹ��ͣ�������
        ErrCode = Err_M2Position;
      else if(M2PID.CurrSpeed+5>SPEEDTHRESHOLD)
        M2Detect = 0;
    }*/
    if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>650))
    {
      if(M2ErrFlag==0)
        M2ErrFlag = 1;      
    }
    else
    {
      M2ErrFlag = 0;
      M2ErrCnt = 0;
    }
    if(M2ErrFlag == 2)
    {
      if(ADCValue.M2Current >= 100)
        ErrCode = Err_M2TwoHall;  //M2������HALL��ȫ���ϵ�
      else
        ErrCode = Err_M2AllWire;
    }
  }
  if(SysState == NORMAL)
  {
    //if ()
    //if((M1State.HallNow > M2State.HallNow+350)||(M2State.HallNow > M1State.HallNow+350)) //������߶Ȳ�̫��
    
    if ((Balance_Data.HallCheckState == 0) && (ErrCode == 0)&&(Adjust_State==0) )         //��������״̬�½��е���߶Ȳ���
    {
      if(((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) > M2State.HallNow + 350)||
         (M2State.HallNow > ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) + 350))) //������߶Ȳ�̫��
      {
        ErrCode = Err_Unbalance;
        Balance_Data.BalanceAdjuseState = 0;
      }
    }
  }
  
    
  
  if(MotorRunTotal >= 120000) //��������ʱ��ﵽ2����
  {
    ErrCode = Err_TimeEnd;
    MotorRunTotal = 120000;
  }
  
  if((MotorStopTime >= 780000)&&(ErrCode == Err_TimeEnd)) //ͣЪʱ��ﵽ13���� 
  {
    ErrCode = 0;
    MotorRunTotal = 0;
    MotorStopTime = 780000;
    if(DisplayMode == ErrorMode)
      DisplayMode = HeightMode;
    if(Release == 3)
      Release = 1;
  }
  
  
  /*
  //���Գ���
  if(MotorRunTotal >= 10000) //��������ʱ��ﵽ4����
  {
    ErrCode = Err_TimeEnd;
    MotorRunTotal = 30000;
  }
  
  if((MotorStopTime >= 15000)&&(ErrCode == Err_TimeEnd)) //ͣЪʱ��ﵽ13���� 
  {
    ErrCode = 0;
    MotorRunTotal = 0;
    MotorStopTime = 30000;
    if(DisplayMode == ErrorMode)
      DisplayMode = HeightMode;
    if(Release == 3)
      Release = 1;
  }
  */ 
         
}


/***********************************************************************************************************
* ��������: TemperatureDetect()
* �������: ��
* ����ֵ  : ��
* ��    ��: �¶ȼ��
************************************************************************************************************/
void TemperatureDetect(void)
{
  
  if(ADCValue.Temperature > 780)//710
  {
    if(ErrCode==0)
    {
      ErrCode = Err_Overheating;
      T18minFlag = 1;
    }
  }
  else if((ADCValue.Temperature < 709)&&(T18minFlag == 1)&&(T18minCnt>=1080000))//1080000
  {
    if(ErrCode == Err_Overheating)
    {
      ErrCode=0;
      T18minFlag = 0;
      T18minCnt = 0;
      if(DisplayMode == ErrorMode)
        DisplayMode = HeightMode;
      if(Release == 3)
        Release = 1;
    }
  } 
  
  
  /*
  //���Գ���
  if(ADCValue.Temperature > 640)
  {
    if(ErrCode==0)
    {
      ErrCode = Err_Overheating;
      T18minFlag = 1;
    }
  }
  else if((T18minFlag == 1)&&(T18minCnt>=30000))
  {
    if(ErrCode == Err_Overheating)
    {
      ErrCode=0;
      T18minFlag = 0;
      T18minCnt = 0;
      if(DisplayMode == ErrorMode)
        DisplayMode = HeightMode;
      if(Release == 3)
        Release = 1;
    }
  }
  */
}

/***********************************************************************************************************
* ��������: AgingTestFaultDetector()
* �������: ��
* ����ֵ  : ��
* ��    ��: ϵͳ���ֹ��ϼ��
************************************************************************************************************/
void AgingTestFaultDetect(void)
{
  if(ADCValue.Temperature > 925)
  {
    AgingTest = 4;
    M1Up(OFF);
    M1Down(OFF);
    M1_PWM_OFF;
    M2Up(OFF);
    M2Down(OFF);
    M2_PWM_OFF; 
    Dis_Char[0] = Char_E;
    Dis_Char[1] = Data_Char[0];
    Dis_Char[2] = Data_Char[2];
  }
  if((ADCValue.M1Current>850)||(ADCValue.M2Current>850))
  {
    AgingTest = 4;
    M1Up(OFF);
    M1Down(OFF);
    M1_PWM_OFF;
    M2Up(OFF);
    M2Down(OFF);
    M2_PWM_OFF; 
    Dis_Char[0] = Char_E;
    Dis_Char[1] = Data_Char[0];
    Dis_Char[2] = Data_Char[3];
  }
  if((AgingTest == 1)&&(AgingTime<240000)&&(AgingTime>=2000))
  {
    if((M1TestPWMDuty > 1000)&&(M2TestPWMDuty > 1000))
    {
      if((ADCValue.M1Current < 50)||(ADCValue.M2Current < 50))
      {
        AgingTest = 4;
        M1Up(OFF);
        M1Down(OFF);
        M1_PWM_OFF;
        M2Up(OFF);
        M2Down(OFF);
        M2_PWM_OFF; 
        Dis_Char[0] = Char_E;
        Dis_Char[1] = Data_Char[0];
        Dis_Char[2] = Data_Char[3];
      }
    }
  }
}


