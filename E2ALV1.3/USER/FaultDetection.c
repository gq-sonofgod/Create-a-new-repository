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

u8 M1HallN1ErrCnt = 0; //电机1霍尔信号1错误计数值
u8 M1HallN2ErrCnt = 0; //电机1霍尔信号2错误计数值
u8 M2HallN1ErrCnt = 0; //电机2霍尔信号1错误计数值
u8 M2HallN2ErrCnt = 0; //电机2霍尔信号2错误计数值

u16 ErrCodeOld = 0;
u8 M1ErrFlag = 0;
u8 M2ErrFlag = 0;
u8 T18minFlag = 0;
u8 M1OverCurFlag = 0;
u8 M2OverCurFlag = 0;
u8 OverCurFlag = 0;  


const u16 Pt[12]={18969,19849,20729,21609,22489,23369,24249,25129,26009,26889,27769,28649};


/***********************************************************************************************************
* 函数名称: FaultDetector()
* 输入参数: 无
* 返回值  : 无
* 功    能: 系统各种故障检测
************************************************************************************************************/
void FaultDetect(void)
{
  static u8 reach = 0;
  CurrentDetect();      //检测电机过流

  HallDetect();
  TemperatureDetect();
  if((ErrCode != 0)&&(ErrCode != Err_RESET))    //故障后一些复位操作
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
      if((ErrCode != Err_TimeEnd) &&        //运行时间到故障
         (ErrCode != Err_Overheating) &&    //过温故障
         (ErrCode != Err_LSM6DSL))          //6轴故障
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
* 函数名称: CurrentDetect()
* 输入参数: 无
* 返回值  : 无
* 功    能: 电机电流检测
************************************************************************************************************/
void CurrentDetect(void)
{ 
  u16 temp;
  u16 temp1;
  u8 i;
  u8 offset_hall_flag = 0;      //用来指示哪个电机运行位置高  0：表示M1高，1表示M2高
  
  if((SysState == NORMAL)&&(M1OverCurFlag == 0)&&(M2OverCurFlag == 0))
  { 
    if(ADCValue.M1Current > (MAXCURRENT)) //电机1过流
      M1OverCurFlag = 1;
    if(ADCValue.M2Current > (MAXCURRENT))  //电机2过流
      M2OverCurFlag = 1;
    
    
    /*
    if(ADCValue.M1Current > (200)) //电机1过流
      M1OverCurFlag = 1;
    if(ADCValue.M2Current > (200))  //电机2过流
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
                        
                        if (M1State.Record[3] > M1State.LimitUp)//超过最高值，则M2回到最低点
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
                    if (M2State.Record[3] > M2State.LimitUp)    //超过最高值，则M1回到最低点
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
* 函数名称: HallDetect()
* 输入参数: 无
* 返回值  : 无
* 功    能: 霍尔信号检测
************************************************************************************************************/
void HallDetect(void)
{  
  if(M1HallN1ErrCnt >= 15)
  {
    M1HallN1ErrCnt = 0;
    if(ErrCode == Err_RESET)    //在复位时出现霍尔故障
      ErrCode = Err_M1OneHall;
    else if(ErrCode==0)         //正常运行时出现霍尔故障
      ErrCode = Err_M1OneHall;
  }
  else if(M1HallN2ErrCnt >= 15)
  {
    M1HallN2ErrCnt = 0;
    if(ErrCode == Err_RESET) //在复位时出现霍尔故障
      ErrCode = Err_M1OneHall;
    else if(ErrCode==0)  //正常运行时出现霍尔故障
      ErrCode = Err_M1OneHall;
  }
  if(M2HallN1ErrCnt >= 15)
  {
    M2HallN1ErrCnt = 0;
    if(ErrCode == Err_RESET) //在复位时出现霍尔故障
      ErrCode = Err_M2OneHall;
    else if(ErrCode==0)  //正常运行时出现霍尔故障
      ErrCode = Err_M2OneHall;
  }
  else if(M2HallN2ErrCnt >= 15)
  {
    M2HallN2ErrCnt = 0;
    if(ErrCode == Err_RESET) //在复位时出现霍尔故障
      ErrCode = Err_M2OneHall;
    else if(ErrCode==0)  //正常运行时出现霍尔故障
      ErrCode = Err_M2OneHall;
  }
  if(SysState==RESET)
  {
    if(M1Cmd == CmdDown)
    {
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>450)&&(ADCValue.M1Current<100)) 
      {
        ErrCode = Err_M1AllWire;  //M1的HALL线和电机线全断了
      }
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>650)&&(ADCValue.M1Current<150)) 
      {
        ErrCode = Err_M1TwoHall;  //M1的HALL线全断了
      }
    }
    else if(M1Cmd == CmdUp)
    {
      if((M1PID.CurrSpeed == 0)&&(M1PID.PWMDuty>850)) 
      {
        ErrCode = Err_M1TwoHall;  //M1的HALL线全断了
      }
    }
    if(M2Cmd == CmdDown)
    {
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>450)&&(ADCValue.M2Current<100))
      {
        ErrCode = Err_M2AllWire;  //M2的HALL线和电机线全断了
      }   
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>650)&&(ADCValue.M2Current<150))
      {
        ErrCode = Err_M2TwoHall;  //M2的HALL线全断了
      }
    }
    else if(M2Cmd == CmdUp)
    {
      if((M2PID.CurrSpeed == 0)&&(M2PID.PWMDuty>850))
      {
        ErrCode = Err_M2TwoHall;  //M2的HALL线全断了
      }
    }
  }
  //if((SysState == NORMAL)&&(SysCmd!=CmdStop)&&(SysCmd!=CmdNull)) //电机正常运行状态
  if((SysState == NORMAL)&&(SysCmd!=CmdStop)&& ((SysCmd!=CmdNull) || (GetBalaceState() != 0))) //电机正常运行状态
  {
    /*if((M1State.HallNow>M1State.LimitDown)&&(M1State.HallNow<M1State.LimitUp)) //M1的HALL计数未达到端点
    {
      if((M1Detect>=4)&&(M1PID.CurrSpeed+7<=SPEEDTHRESHOLD)&&(ADCValue.M1Current>380)&&(ErrCode==0)&&(M1PID.PWMDuty>800)) //电机1速度过低（卡死）
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
        ErrCode = Err_M1TwoHall;  //M1的两根HALL线全部断掉
      else
        ErrCode = Err_M1AllWire;
    }
    /*if((M2State.HallNow>M2State.LimitDown)&&(M2State.HallNow<M2State.LimitUp)) //M2的HALL计数未达到端点
    {
      if((M2Detect>=4)&&(M2PID.CurrSpeed+7<=SPEEDTHRESHOLD)&&(ADCValue.M2Current>380)&&(ErrCode==0)&&(M2PID.PWMDuty>800)) //电机2速度过低（卡死）
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
        ErrCode = Err_M2TwoHall;  //M2的两根HALL线全部断掉
      else
        ErrCode = Err_M2AllWire;
    }
  }
  if(SysState == NORMAL)
  {
    //if ()
    //if((M1State.HallNow > M2State.HallNow+350)||(M2State.HallNow > M1State.HallNow+350)) //两电机高度差太大
    
    if ((Balance_Data.HallCheckState == 0) && (ErrCode == 0)&&(Adjust_State==0) )         //正常运行状态下进行电机高度差检测
    {
      if(((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) > M2State.HallNow + 350)||
         (M2State.HallNow > ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) + 350))) //两电机高度差太大
      {
        ErrCode = Err_Unbalance;
        Balance_Data.BalanceAdjuseState = 0;
      }
    }
  }
  
    
  
  if(MotorRunTotal >= 120000) //连续运行时间达到2分钟
  {
    ErrCode = Err_TimeEnd;
    MotorRunTotal = 120000;
  }
  
  if((MotorStopTime >= 780000)&&(ErrCode == Err_TimeEnd)) //停歇时间达到13分钟 
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
  //测试程序
  if(MotorRunTotal >= 10000) //连续运行时间达到4分钟
  {
    ErrCode = Err_TimeEnd;
    MotorRunTotal = 30000;
  }
  
  if((MotorStopTime >= 15000)&&(ErrCode == Err_TimeEnd)) //停歇时间达到13分钟 
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
* 函数名称: TemperatureDetect()
* 输入参数: 无
* 返回值  : 无
* 功    能: 温度检测
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
  //测试程序
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
* 函数名称: AgingTestFaultDetector()
* 输入参数: 无
* 返回值  : 无
* 功    能: 系统各种故障检测
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


