#include "Balance.h"
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
u8 Ref_Para_MiniSpeed=0;
BALANCE_STR Balance_Data;

/*******************************************************************************
函数名称：GetBalaceState()  (全局函数)
函数功能：获得当前平衡调整的状态值
输入：无
输出：平衡调整当前状态值
*******************************************************************************/
u8 GetBalaceState()
{
  return Balance_Data.BalanceAdjuseState;
}

/*******************************************************************************
函数名称：SetBalaceState()  (全局函数)
函数功能：设置平衡调整的状态
输入：需设置的状态值
输出：无
*******************************************************************************/
void SetBalaceState(u8 state)
{
  Balance_Data.BalanceAdjuseState = state;
}

void SetBalance()
{
  static u8 Time1=0;
  static u8 Time2=0;
  static u8 Time3=0;
  static u8 Time4=0;
  static u8 Time5=0;
  
  
  if ((ErrCode !=0) && (Balance_Data.BalanceAdjuseState != 0))
  {
    Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //得到两电机间的HALL偏差值
        
    memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]，Buffer[63]存储因地形自适应而产生的两电机HALL差值
        
    memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //存储M1 HALL当前值
    memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //存储M2 HALL当前值
            
    EEPROM_Write();
        
    if (Balance_Data.TwoMotorOffsetHall >= 0)             //M1电机比M2电机高
    {
          M1State.RelativeLimitUp = M1State.LimitUp;
          M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
          M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
          M2State.RelativeLimitDown = M2State.LimitDown;
     }
     else  //Balance_Data.TwoMotorOffsetHall < 0           //M1电机比M2电机低
     {
          M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //减少M1最高位置设定值
          M1State.RelativeLimitDown = M1State.LimitDown;
            
          M2State.RelativeLimitUp = M2State.LimitUp;
          M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //升高M2最低位置设定值
     }

     EnableHPSlopeFilter();
            
            
     AnglePID.LastDiffValur = 0;
     AnglePID.PrevDiffValur = 0;    
     AnglePID.OutPut = 0;
            
     M1PID.SetSpeed = 0;
     M1PID.SetSpeed = 0;
            
     M1PID.CurrSpeed = 0;
     M2PID.CurrSpeed = 0;
            
     Balance_Data.DecideMotorRunState = 0;
                      
     Balance_Data.BalanceAdjuseState = 0;
            
     Balance_Data.HallCheckState = 0;
            
     Balance_Data.Acc_yTemp = 0;   
            
     M1PID.LastDev = 0;
     M1PID.PrevDev = 0;
     M1PID.SyncDev = 0;
            
     M2PID.LastDev = 0;
     M2PID.PrevDev = 0;
     M2PID.SyncDev = 0;
            
     M2Flag = 0;
     M1Flag = 0;           
            
     SaveIndex = 0;       //存储位置的标志位清0
            
     Buffer[32] = SaveIndex;
     
      Menu1Flag = 0;
      Menu2Flag = 0;
      //Menu2Num = 0;
     
  }
  else if (Balance_Data.BalanceAdjuseState != 0)    //进入调整模式
  {
    /*
    if((SysState == NORMAL)&&(Release == 1)&&(AntiCollisionState == 0) && 
       (Balance_Data.BalanceAdjuseState == 1) &&
     (AccDataBag.Full_State ==1) && (Balance_Data.DelayTimerCnt > 2000))    
    */
    
    if((SysState == NORMAL)&&(Release == 1)&&(AntiCollisionState == 0) && 
       (Balance_Data.BalanceAdjuseState == 1) &&
       (GetAccFullState() ==1))   
    { 
        
       if((AccDataBag.Acc_y >= 200) || (AccDataBag.Acc_y <= -200))
       {
         
          //Release = 0;
          DeleteSavedHeight();  //SaveFlag = 0;删除两个电机当前位置EEPROM中的存储值（M1State.HallNow，M2State.HallNow）                            
          
          Balance_Data.Acc_yTemp = AccDataBag.Acc_y;    //已除以8后的数据        
          
          Balance_Data.HallCheckState = 1;              //不使能两电机HALL差值检测
                   
          Balance_Data.MaxHallDif = (MAX_HALL_DIFF > DIF_HALL)? DIF_HALL:MAX_HALL_DIFF;
          
          
          //SysCmd = CmdSetBalance;
          //给两电机一个参考运行方向 
          if (Balance_Data.TwoMotorRunFlag == 0)
          {
            //if (Balance_Data.AccXl_yTemp > 0)
            if (Balance_Data.Acc_yTemp > 0)
            {
              M1Cmd = CmdUp;          
              M2Cmd = CmdDown;
            }
            else 
            {
              M2Cmd = CmdUp;          
              M1Cmd = CmdDown;
            }
          }
          else
          {
            //if (Balance_Data.AccXl_yTemp > 0)
            if (Balance_Data.Acc_yTemp > 0)
            {
              M2Cmd = CmdUp;          
              M1Cmd = CmdDown;
            }
            else 
            {
              M1Cmd = CmdUp;          
              M2Cmd = CmdDown;
            }
          }
          
          SetTIM1_PWMDuty(1, 1350 - 400);                  //设置初始PWM占空比 
          SetTIM1_PWMDuty(2, 1350 - 400);                  //设置初始PWM占空比
          
          PID_Set(8,BASEDUTYUP);                            //设置给到转速为8
          //PID_Set(20,BASEDUTYUP);
          //RunCntFlag = 1;
          
          Balance_Data.DecideMotorRunTimerCnt = 0;          //用于确定两电机运行方向的时间计数器
          Balance_Data.DecideMotorRunState = 0;
          Balance_Data.BalanceAdjuseState = 2;              //进行平衡调整（电机开始运行）         
          
          AnglePID.LastDiffValur = 0;
          AnglePID.PrevDiffValur = 0;  
          
          M2PID.LastDev = 0;
          M2PID.PrevDev = 0;
            
          M1PID.LastDev = 0;
          M1PID.PrevDev = 0;
       }
       else 
       {
         EnableHPSlopeFilter();
         
         Balance_Data.Acc_yTemp = 0;
         Balance_Data.BalanceAdjuseState = 4;
         SysCmd = CmdNull;
       }
    }
    else
    {   
      if(Balance_Data.BalanceAdjuseState == 2)  //进入确定两电机运行方向阶段
      {
        if ((Balance_Data.DecideMotorRunState == 0) && (Balance_Data.DecideMotorRunTimerCnt >= 1000))   //先运行1000ms，再来判断现在的调整方向是否正确
        {
          Balance_Data.DecideMotorRunTimerCnt = 500;    //每500ms再进行一次检测
          //Balance_Data.DecideMotorRunState = 1;
          //if (AccDataBag.Acc_y <= 100)          //直接调整完成，进入减速阶段及后续操作
          //if (AccDataBag.Acc_y <= 200)          //直接调整完成，进入减速阶段及后续操作
          if ((AccDataBag.Acc_y <= 100) && (AccDataBag.Acc_y >= -100))
          //if (abs(AccDataBag.Acc_y) < 100 )
          {
            Time1++;
            if(Time1>5)
            {
            M1Up(OFF);
            M1Down(OFF);
            M2Up(OFF);
            M2Down(OFF);
            M1_PWM_OFF;
            M1Dir = STOP;
            M1Cmd = CmdStop;                    //电机1进入停止状态           
            M2_PWM_OFF;
            M2Dir = STOP;
            M2Cmd = CmdStop;
            
            Balance_Data.BalanceAdjuseState = 4;
            Balance_Data.MotorSlideTimerCnt = 0;
            }
             Time2=0;
             Time3=0;
             Time4=0;
             Time5=0;
          }
          else     //判断当前两电机调整方向是否正确
          {
            //Balance_Data.Acc_yTemp = AccDataBag.Acc_y;
            //Balance_Data.Acc_yTemp
             Time1=0; 
            if (Balance_Data.Acc_yTemp >= 0)
            {
              
              
              if ((AccDataBag.Acc_y < 0) || (AccDataBag.Acc_y > Balance_Data.Acc_yTemp))
              {
                  
                    Time2++;
                 if(Time2>5)
                {
                   if (Balance_Data.TwoMotorRunFlag == 1)
                {
                  Balance_Data.TwoMotorRunFlag = 0;   //改变运行状态
                }
                else 
                {
                  Balance_Data.TwoMotorRunFlag = 1;
                }
                                      
                Buffer[61] = Balance_Data.TwoMotorRunFlag;
      
                EEPROM_Write();
                    
                M1Up(OFF);
                M1Down(OFF);
                M2Up(OFF);
                M2Down(OFF);
                M1_PWM_OFF;
                M1Dir = STOP;
                M1Cmd = CmdStop;                                    //电机1进入停止状态           
                M2_PWM_OFF;
                M2Dir = STOP;
                M2Cmd = CmdStop;
            
                Balance_Data.TwoMotorRunFaultFlag = 1;
                    
                Balance_Data.BalanceAdjuseState = 4;
                Balance_Data.MotorSlideTimerCnt = 0;  
                Time3=0;    
                 }//两电机运行方向错误
                  
               
              }
              else if (AccDataBag.Acc_y < Balance_Data.Acc_yTemp)       //方向调整正确
              {
                 Time3++;
                 if(Time3>5)
                 {
                 RunCntFlag = 1;
              
                 if((AccDataBag.Acc_y<=500)&&(AccDataBag.Acc_y>-500))
                 {
                 
                 Ref_Para_MiniSpeed=1;
                 } 
                 else
                 {
                 Ref_Para_MiniSpeed=0;
                 AnglePID.Ref_Para = (AccDataBag.Acc_y*1)/4;
                 }
                 
                    
                 //AnglePID.Ref_Para = (AccDataBag.Acc_y)/2;      //设置参考角度，用于PID计算
                 
                 AnglePID.LastDiffValur = 0;
                 AnglePID.PrevDiffValur = 0;               
                 
                 M2PID.Kp = KP2;
                 M2PID.Ki = KI2;
                 M2PID.Kd = KD2;
                    
                 M1PID.Kp = KP2;
                 M1PID.Ki = KI2;
                 M1PID.Kd = KD2;
                 
                 
                 AnglePID.OutPut = 8;

                 Balance_Data.BalanceAdjuseState = 3;   //设置成直接运行状态
                 Time2=0;
                  }
                 
              }
             Time5=0;
             Time4=0;
            }
            else  //(Balance_Data.Acc_yTemp < 0)
            {
                Time3=0;
                Time2=0;
                
              if ((AccDataBag.Acc_y > 0)||(AccDataBag.Acc_y < Balance_Data.Acc_yTemp)) //调整方向不对
              {
                  Time5=0;
                  Time4++;
                  if(Time4>5)
                  {
                  if (Balance_Data.TwoMotorRunFlag == 1)
                   {
                    Balance_Data.TwoMotorRunFlag = 0;                   //改变运行状态
                   }
                 else
                   {
                     Balance_Data.TwoMotorRunFlag = 1;
                   }
                    
                 Buffer[61] = Balance_Data.TwoMotorRunFlag;
      
                 EEPROM_Write();

                 M1Up(OFF);
                 M1Down(OFF);
                 M2Up(OFF);
                 M2Down(OFF);
                 M1_PWM_OFF;
                 M1Dir = STOP;
                 M1Cmd = CmdStop;                                       //电机1进入停止状态           
                 M2_PWM_OFF;
                 M2Dir = STOP;
                 M2Cmd = CmdStop;
            
                 Balance_Data.TwoMotorRunFaultFlag = 1;
                 Balance_Data.BalanceAdjuseState = 4;
                 Balance_Data.MotorSlideTimerCnt = 0;
                  
                  }
                      
                 if (Balance_Data.TwoMotorRunFlag == 1)
                 {
                    Balance_Data.TwoMotorRunFlag = 0;                   //改变运行状态
                 }
                 else
                 {
                    Balance_Data.TwoMotorRunFlag = 1;
                 }
                    
                 Buffer[61] = Balance_Data.TwoMotorRunFlag;
      
                 EEPROM_Write();

                 M1Up(OFF);
                 M1Down(OFF);
                 M2Up(OFF);
                 M2Down(OFF);
                 M1_PWM_OFF;
                 M1Dir = STOP;
                 M1Cmd = CmdStop;                                       //电机1进入停止状态           
                 M2_PWM_OFF;
                 M2Dir = STOP;
                 M2Cmd = CmdStop;
            
                 Balance_Data.TwoMotorRunFaultFlag = 1;
                 Balance_Data.BalanceAdjuseState = 4;
                 Balance_Data.MotorSlideTimerCnt = 0;
              }
              else if (AccDataBag.Acc_y > Balance_Data.Acc_yTemp)       //调整方向正确，继续进行同方向调整
              {
                 
                 Time4=0;
                 
                  Time5++;
                  if(Time5>5)
                  {
                  AnglePID.LastDiffValur = 0;
                  AnglePID.PrevDiffValur = 0;  
              
                  RunCntFlag = 1;
              
                 if((AccDataBag.Acc_y<=500)&&(AccDataBag.Acc_y>-500)) 
                 {
                 Ref_Para_MiniSpeed=1;    //设定成加速时间短，减速时间长
                 }
                 else  
                 {
                 AnglePID.Ref_Para = (AccDataBag.Acc_y*1)/4;  
                 Ref_Para_MiniSpeed=0;
                 }
                 //AnglePID.Ref_Para = (AccDataBag.Acc_y)/2;
                              
                 
                 M2PID.Kp = KP2;
                 M2PID.Ki = KI2;
                 M2PID.Kd = KD2;
                    
                 M1PID.Kp = KP2;
                 M1PID.Ki = KI2;
                 M1PID.Kd = KD2;
                         
                 AnglePID.OutPut = 8;
                 //AnglePID.OutPut = 15;
                 Balance_Data.BalanceAdjuseState = 3;
                  
                  
                  }
                 
               }
            }
          } 
        }    
      }
      if (Balance_Data.BalanceAdjuseState == 3)         //进入真正调整阶段
      {
        //if(AccDataBag.Acc_y <= 300)     //进入电机减速阶段 
        //if(AccDataBag.Acc_y <= 100)      //进入电机减速阶段
        //if(AccDataBag.Acc_y <= 200)      //进入电机减速阶段
        //if ((AccDataBag.Acc_y <= 100)&&(AccDataBag.Acc_y >= -100))      //一直运行，直到角度调整到一定位置
            
        if ((AccDataBag.Acc_y <= 100)&&(AccDataBag.Acc_y >= -100))      //一直运行，直到角度调整到一定位置
        {
          M1Up(OFF);
          M1Down(OFF);
          M2Up(OFF);
          M2Down(OFF);
          M1_PWM_OFF;
          M1Dir = STOP;
          M1Cmd = CmdStop;    //电机进入停止状态           
          M2_PWM_OFF;
          M2Dir = STOP;
          M2Cmd = CmdStop;
            
          Balance_Data.BalanceAdjuseState = 4;
          Balance_Data.MotorSlideTimerCnt = 0;
        } //快到极限高度时，
        /*
        else if ((M1State.HallNow > (M1State.LimitUp - 300)) || (M2State.HallNow > (M1State.LimitUp - 300)))
        {
           M1Up(OFF);
           M1Down(OFF);
           M2Up(OFF);
           M2Down(OFF);
           M1_PWM_OFF;
           M1Dir = STOP;
           M1Cmd = CmdStop;    //电机进入停止状态           
           M2_PWM_OFF;
           M2Dir = STOP;
           M2Cmd = CmdStop;
            
           Balance_Data.BalanceAdjuseState = 4;
           Balance_Data.MotorSlideTimerCnt = 0;
        }
        */
        else 
        {
          u16 hall_diff;
          if (M2State.HallNow > M1State.HallNow)
          {
                hall_diff = M2State.HallNow - M1State.HallNow;
          }
          else 
          {
                hall_diff = M1State.HallNow - M2State.HallNow ; 
          }
          
          //if (hall_diff > 6515)     //最大40cm高度差
            if (hall_diff > Balance_Data.MaxHallDif) 
          {
                M1Up(OFF);
                M1Down(OFF);
                M2Up(OFF);
                M2Down(OFF);
                M1_PWM_OFF;
                M1Dir = STOP;
                M1Cmd = CmdStop;    //电机进入停止状态           
                M2_PWM_OFF;
                M2Dir = STOP;
                M2Cmd = CmdStop;
            
                Balance_Data.BalanceAdjuseState = 4;
                Balance_Data.MotorSlideTimerCnt = 0;
          }
        }
        
      }  
      if (Balance_Data.BalanceAdjuseState == 4)                 //关闭电源，电机进入滑行阶段
      {
        if ((KEY_Stop_M_Flag==1)||((M2Cmd == CmdNull) && (M1Cmd == CmdNull)))
        {
          Balance_Data.BalanceAdjuseState = 5;
          Balance_Data.MotorSlideTimerCnt = 0; 
        }
      }
      
      if(Balance_Data.BalanceAdjuseState == 5)                  //进入惯性滑行阶段
      {
        if (Balance_Data.MotorSlideTimerCnt >= 200)             //达到规定滑行时间
        {   
          Balance_Data.BalanceAdjuseState = 6;                  //进入计算这次调整平衡电机运行的HALL数阶段
        }   
      }
      
      if (Balance_Data.BalanceAdjuseState == 6)
      {
         if ((Balance_Data.TwoMotorRunFaultFlag == 1)&&(KEY_Stop_M_Flag==0))            //调整平衡趋势反了，则从新开始进行调整
         {
            Balance_Data.BalanceAdjuseState = 1;            
            Balance_Data.TwoMotorRunFaultFlag = 0;
           
            //Release = 1;
         }
         else
         { 
            
            KEY_Stop_M_Flag=0;
            Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //得到两电机间的HALL偏差值
        
            memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]，Buffer[63]存储因地形自适应而产生的两电机HALL差值
        
            memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //存储M1 HALL当前值
            memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //存储M2 HALL当前值
            
            EEPROM_Write();
        
            if (Balance_Data.TwoMotorOffsetHall >= 0)             //M1电机比M2电机高
            {
              M1State.RelativeLimitUp = M1State.LimitUp;
              M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
              M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
              M2State.RelativeLimitDown = M2State.LimitDown;
            }
            else  //Balance_Data.TwoMotorOffsetHall < 0           //M1电机比M2电机低
            {
              M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //减少M1最高位置设定值
              M1State.RelativeLimitDown = M1State.LimitDown;
            
              M2State.RelativeLimitUp = M2State.LimitUp;
              M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //升高M2最低位置设定值
            }

            EnableHPSlopeFilter();
            
            
            AnglePID.LastDiffValur = 0;
            AnglePID.PrevDiffValur = 0;    
            AnglePID.OutPut = 0;
            
            M1PID.SetSpeed = 0;
            M1PID.SetSpeed = 0;
            
            M1PID.CurrSpeed = 0;
            M2PID.CurrSpeed = 0;
            
            Balance_Data.DecideMotorRunState = 0;
                      
            //Balance_Data.BuzzerOnTimerCnt = 0;
            
            BuzzerOnTimerCnt = 0;
            BuzzerState = ON;
            BuzzerWorkMode = 1;
            Balance_Data.BalanceAdjuseState = 0;
            
            Balance_Data.HallCheckState = 0;
            
            Balance_Data.Acc_yTemp = 0;   
            
            M1PID.LastDev = 0;
            M1PID.PrevDev = 0;
            M1PID.SyncDev = 0;
            
            M2PID.LastDev = 0;
            M2PID.PrevDev = 0;
            M2PID.SyncDev = 0;
            
            M2Flag = 0;
            M1Flag = 0;
            
            //Release = 1;
            
            SaveIndex = 0;       //存储位置的标志位清0
            
            Buffer[32] = SaveIndex;
            DisplayMode = HeightMode;
            //MenuTime = 5000;        //z
        }
      }
    }
    MenuTime = 4800;    //用于一直显示  SET
    LEDRest = 0;        //有可能由于自平衡调整时间过长，超过长时间显示时间设定值（10s）
  }
}

s16 GetTwoMotorOffsetHall()
{
  return Balance_Data.TwoMotorOffsetHall;   // TwoMotorOffsetHall = M1 - M2
}




void Adjust_Balance_StepByStepo()
{
  if( Adjust_State==1)
  {
  AnglePID.LastDiffValur = 0;
  AnglePID.PrevDiffValur = 0;  
              
  RunCntFlag = 1;
              
  AnglePID.Ref_Para = (AccDataBag.Acc_y*3)/4;    //设定成加速时间短，减速时间长
                    
                 //AnglePID.Ref_Para = (AccDataBag.Acc_y)/2;
                              
                 
  M2PID.Kp = KP2;
  M2PID.Ki = KI2;
  M2PID.Kd = KD2;
                    
  M1PID.Kp = KP2;
  M1PID.Ki = KI2;
  M1PID.Kd = KD2;
                         
  AnglePID.OutPut = 8;
  Adjust_State=2;
  }
  else if(Adjust_State==2)
  {
         
  // Balance_Data_Refresh();
       
  
  
  
  }








}
