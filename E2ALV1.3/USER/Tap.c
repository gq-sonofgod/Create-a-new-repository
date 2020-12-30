#include "Tap.h"
#include "Main.h"

#include "Motor.h"
#include "PID.h"
#include "Timer.h"
#include "Key.h"
#include "LED.h"
#include "ADC.h"
#include "EEPROM.h"
#include "string.h"
#include "Delay.h"
#include "stm8s_it.h"
#include "FaultDetection.h"
#include "HealthMode.h"
u16 Record_Temp[5]={0};
u16 Savebuff_Down[5]={0};
u16 Savebuff_Up[5]={0};
u16 Min_Up_Record=0;
u16 Max_Down_Record=0;
u16 Recordmax=0;
u16 Recordmin=0;
u8 Down_Position=0;
u8 Up_Position=0;
u8 TapControlFlag = 0;          //敲击控制开关   0：敲击控制关  1：敲击控制开
u8 SaveIndex_Sta = 0; 
u8 Lsm6dIntState = 0;
u8 i= 0; 
u8 Tap_nolmalflag=0;
TAP_PARAMETER Tap_Parameter = TAP_PARAMETER_DEFAULTS;

void Tap_Control()
{
 
  if ((SysState == NORMAL) && (AntiCollisionState == 0))
  {
    switch(Tap_Parameter.TapTriggerState)
    {
    case 0:
      break;
    case 1:             //单击实现上升操作
      {
        //if ((M2Cmd == CmdNull) && (M1Cmd == CmdNull) && (SysCmd == CmdNull))
        if ((SysCmd == CmdNull) && (Release == 1)&&(SaveIndex!=0))
        { 
            Release=0;
           Save_Position_Tap(); 
          if(Up_Position!=0x08)
          {
          
          HealthModeReset();                    
          M1Cmd = CmdToPreseting;
          SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
          M2Cmd = CmdToPreseting;
          SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
          SysCmd = CmdToPreseting;
          PID_Set(Speed,BASEDUTYDOWN);
          Position =Up_Position;
          //Tap_Parameter.TapTriggerState = 0;
          }
        }
          else if ((SysCmd == CmdNull) && (Release == 1)&&(SaveIndex==0))
          {
           Release = 0;
          DeleteSavedHeight();
          HealthModeReset();
          SysCmd = CmdUp;
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1, 1350-BASEDUTYUP); //设置初始PWM占空比
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); //设置初始PWM占空比        
          //PID_Set(Speed,BASEDUTYUP);
           PID_Set(40,BASEDUTYUP); 
          }   
         
         
        
        
        Tap_Parameter.TapControlTimerCnt = 0;
        /*
        else 
        {
          Tap_Parameter.TapTriggerState = 0;
        }
        */
      }
      break;
    case 2:             //单击实现停止操作
      {
          
        if ((SysCmd == CmdUp)||(SysCmd == CmdDown)||(SysCmd == CmdToPreseting))
        {
          KeyNullProcess();
           M1Up(OFF);
           M1Down(OFF);
           M2Up(OFF);
           M2Down(OFF);
           M1_PWM_OFF;
           M1Dir = STOP;
           M1Cmd = CmdStop;
           M2_PWM_OFF;
           M2Dir = STOP;
           M2Cmd = CmdStop; 
           SysCmd =CmdNull;
        }
        else if((M2Cmd == CmdNull) && (M1Cmd == CmdNull) && (SysCmd == CmdNull))
        {
          Tap_Parameter.TapTriggerState = 0;
          Tap_Parameter.TapControlFlag = 0;     //进入TAP控制状态 = 1;
          
          Tap_Parameter.TapFilterFlag = 0;
          Tap_Parameter.SingleTapFilterCnt = 0;
          Tap_Parameter.TapCheckOrderFlag = 0;
          Tap_Parameter.TapCnt = 0;
          
        }
      }
      break;
    case 3:             //双击实现下降操作
      {
          if((M2Cmd == CmdNull) && (Release == 1)&&(SaveIndex!=0))
          {   Release=0;
              Save_Position_Tap();
              if(Down_Position!=0x08)
              {
              
              HealthModeReset();                    
              M1Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
              M2Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
              SysCmd = CmdToPreseting;
              PID_Set(Speed,BASEDUTYDOWN);
              Position =Down_Position;
              Release = 0;
             // Tap_Parameter.TapTriggerState = 0;
              }
          }
          else if(((M2Cmd == CmdNull) && (Release == 1)&&(SaveIndex==0)))
          {
          if(((M1State.HallNow+M2State.HallNow) > (M1State.LimitDown+M2State.LimitDown+13)))
            {
              DeleteSavedHeight();
              HealthModeReset();
              SysCmd = CmdDown;
              M1Cmd = CmdDown;
              SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
              M2Cmd = CmdDown;
              SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN); 
              
              //PID_Set(Speed,BASEDUTYDOWN);
              PID_Set(40,BASEDUTYDOWN);
             
            }
          
          
          }
            
          
          
          Tap_Parameter.TapControlTimerCnt = 0;
      }
      break;
    case 4:             //双击实现停止操作
    {
        if ((SysCmd == CmdUp)||(SysCmd == CmdDown)||(SysCmd == CmdToPreseting))
        {
                    KeyNullProcess();
                    M1Up(OFF);
                    M1Down(OFF);
                    M2Up(OFF);
                    M2Down(OFF);
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
        }
        else if((M2Cmd == CmdNull) && (M1Cmd == CmdNull) && (SysCmd == CmdNull))
        {
          Tap_Parameter.TapTriggerState = 0;
          Tap_Parameter.TapControlFlag = 0;
          
          Tap_Parameter.TapFilterFlag = 0;
          Tap_Parameter.SingleTapFilterCnt = 0;
          Tap_Parameter.TapCheckOrderFlag = 0;
          Tap_Parameter.TapCnt = 0;
        }
    }
    break;
    default:break;
    }
  }
  else if((SysCmd == CmdNull) && (AntiCollisionState != 0))   //遇阻回退后敲击使能关闭
  {
     Tap_Parameter.TapFilterFlag = 0;
     Tap_Parameter.SingleTapFilterCnt = 0;
     Tap_Parameter.TapCheckOrderFlag = 0;
     Tap_Parameter.TapCnt = 0;
     Tap_Parameter.TapControlEN = 0;
     Tap_Parameter.TapControlFlag = 0;
  }
  
  if ((SysCmd == CmdNull) && (M2Cmd == CmdNull) && (M1Cmd == CmdNull))
  {
     Tap_Parameter.TapTriggerState = 0;
     Tap_Parameter.TapControlFlag = 0;;
  }
}

//判断在运行方向上最近的点，敲击移动到最近的点。

void Save_Position_Tap()
{
//M1,M2各有两个点是固定的点，既最高点126和最低点60.5，分别赋值给Mx.Record[3]和Mx.Record[4]
  M1State.Record[6]= M1State.RelativeLimitUp;
  M1State.Record[7]= M1State.RelativeLimitDown; 
  M2State.Record[6]= M2State.RelativeLimitUp;
  M2State.Record[7]= M2State.RelativeLimitDown; 
 if((M1Cmd == CmdStop )||(M1Cmd == CmdNull   )) //u8 Record_Temp[3]={0};
 {
  if(SaveIndex==0)//无记录值得情况
  { 
  Min_Up_Record=0;
  Up_Position=0x08; 
  Max_Down_Record=0;
  Down_Position=0x08; 
  }
  //记录值存入Record_Temp中
  if(SaveIndex&0x01) Record_Temp[0]=M1State.Record[0];
  else  Record_Temp[0]=0;
  if(SaveIndex&0x02) Record_Temp[1]=M1State.Record[1];
  else  Record_Temp[1]=0;
  if(SaveIndex&0x04) Record_Temp[2]=M1State.Record[2];
  else  Record_Temp[2]=0;
  Record_Temp[3]=M1State.RelativeLimitUp;
  Record_Temp[4]=M1State.RelativeLimitDown;
  
  //Savebuff_Up找出大于当前高度hall值的记录值，Savebuff_Down找出小于当前高度hall值的记录值
  for(i=0;i<5;i++)
  {
   if((Record_Temp[i]!=0)&&(Record_Temp[i] > (M1State.HallNow+50)))//&&(M1State.HallNow < M1State.LimitUp)
   {
    Savebuff_Up[i]=Record_Temp[i];
   }
   else if((Record_Temp[i]!=0)&&((Record_Temp[i]+50 )< M1State.HallNow))//&&(M1State.HallNow > M1State.LimitDown)
   {
   Savebuff_Down[i]=Record_Temp[i]; 
   }
   else 
   {
       Savebuff_Up[i]=0;
       Savebuff_Down[i]=0;
   }
   //Savebuff_Up[1]=0;
  // Savebuff_Down[i]=0;
  }
 ///////////////////////
  
  //找到高于当前高度hall值最近的记录值（一个最小值），Up_Position指示向上运行的位置////////////////////////////////////////////////////////////////////////////////////// 
  if(Savebuff_Up[0]>0) 
  { 
  Min_Up_Record=Savebuff_Up[0];
  Up_Position=0;
  }
  else Min_Up_Record=0;
  
  if(Savebuff_Up[1]>0)
  {
  if(Min_Up_Record==0)
  {
  Min_Up_Record=Savebuff_Up[1];
  Up_Position=1;
  }
  else 
  {
  if(Min_Up_Record>Savebuff_Up[1])
  {
  Min_Up_Record=Savebuff_Up[1];
  Up_Position=1;
  }
  
  }
  }
  
if(Savebuff_Up[2]>0)
{
    
  if(Min_Up_Record==0)
  {
  Min_Up_Record=Savebuff_Up[2];
  Up_Position=2;
  }
  else 
  {
  if(Min_Up_Record>Savebuff_Up[2])
  {
  Min_Up_Record=Savebuff_Up[2];
  Up_Position=2;
  }
  
  }  
} 
if(Savebuff_Up[3]>0)
{
    
  if(Min_Up_Record==0)
  {
  Min_Up_Record=Savebuff_Up[3];
  Up_Position=6;
  }
  else 
  {
  if(Min_Up_Record>Savebuff_Up[3])
  {
  Min_Up_Record=Savebuff_Up[3];
  Up_Position=6;
  }
  
  }  
} 



if((Savebuff_Up[0]==0)&&(Savebuff_Up[1]==0)&&(Savebuff_Up[2]==0))
{
 //Up_Position=8;
}
/////////////////////////////////////////////////////////////
//找到低于当前高度hall值最近的记录值（一个最大值），Up_Position指示向下运行的位置
if(Savebuff_Down[0]>Savebuff_Down[4])
{
Max_Down_Record=Savebuff_Down[0];
Down_Position=0;
}
else
{
Max_Down_Record=Savebuff_Down[4];
Down_Position=7;
}

if(Savebuff_Down[1]>Max_Down_Record)
{ 
Max_Down_Record=Savebuff_Down[1];
Down_Position=1;
}  

if(Savebuff_Down[2]>Max_Down_Record)
{ 
Max_Down_Record=Savebuff_Down[2];
Down_Position=2;
}

if(Max_Down_Record==0)Down_Position=8;
  
 }

}