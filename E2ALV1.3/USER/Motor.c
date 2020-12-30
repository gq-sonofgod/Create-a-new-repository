#include "Motor.h"
#include "Main.h"
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
#include "Balance.h"
#include "Tap.h"


MSTATE M1State = MSTATE_DEFAULTS;
MSTATE M2State = MSTATE_DEFAULTS;

u8 RstBalanceFlag = 0;

u8 DelayFlagForTap = 0;

u8 M1Detect = 0;
u8 M2Detect = 0;

u8 M1Dir = STOP;
u8 M2Dir = STOP;


u16 M1TestPWMDuty = 0;
u16 M2TestPWMDuty = 0;

u8 TurnFlag = 0;

u16 ADCBuffer1[40];
u16 ADCBuffer2[40];
u8 M1Flag = 0;
u8 M2Flag = 0;
u8 AntiCollisionState = 0;
u16 M1CurTemp1 = 0;
u16 M1CurTemp2 = 0;
u16 M1CurTemp3 = 0;
u16 M1CurMax = 0;
u16 M1CurFlag = 0;

u16 M2CurTemp1 = 0;
u16 M2CurTemp2 = 0;
u16 M2CurTemp3 = 0;
u16 M2CurMax = 0;
u16 M2CurFlag = 0;

u16 CurFlag = 0;
u16 M1HallTemp = 0;
u16 M2HallTemp = 0;
u8 LimitPowerFlag = 0;
u16 M1CurrBase = 0;
u16 M2CurrBase = 0;
u8 LimitPowerState = 0;
u8 hd_anti_up;
u8 hd_anti_down;
u8 st_anti_up;
u8 st_anti_down1;
u8 st_anti_down2;

extern s16 Acc_x,Acc_y,Acc_z;
extern s16 Ang_x,Ang_y,Ang_z;
u16 add_x,add_y,add_z;
s16 stopAcc_x,stopAcc_y,stopAcc_z;
s16 stopAng_x,stopAng_y,stopAng_z;
u16 motorStopTimer=1000;
u16 motorStartTimer=0;
u16 add_Acc;
u8 add_value;
u8 v33Flag = 0;
/***********************************************************************************************************
* 函数名称: Motor_Init()
* 输入参数: 无
* 返回值  : 无
* 功    能: 电机控制引脚和蜂鸣器控制引脚初始化
************************************************************************************************************/
void Motor_Init(void)
{
  GPIO_Init(GPIOB, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);//Motor1控制引脚
  GPIO_Init(GPIOE, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);                      //Motor2控制引脚
  //GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);                    // 蜂鸣器控制引脚
  GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);                      // 33v输出控制引脚
}


/***********************************************************************************************************
* 函数名称: M1Control()
* 输入参数: cmd，控制命令 //电机M1命令  (M1Cmd参数)
* 返回值  : 无
* 功    能: 电机1运行控制
************************************************************************************************************/
void M1Control(u8 cmd)
{
  if(SysState == NORMAL) //系统当前处于正常运行状态
  {
    switch(cmd)
    {
      case CmdUp:
        /*  //老程序
        if(M1PID.CurrSpeed > 0)
        {
          if((M1State.LimitUp <= M1State.HallNow)) //运动到电子的上顶点则停止运行
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop; //电机1进入停止状态 
          }
          else
          {
            M1Down(OFF);
            M1Up(ON);
          } 
        }
        else    //M1PID.CurrSpeed <= 0
        {
          if(M1State.LimitUp <= (M1State.HallNow+DELT_HALL))
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop; //电机1进入停止状态 
          }
          else
          {
            M1Down(OFF);
            M1Up(ON);
          }
        } 
        if((M1State.LimitUp <= M1State.HallNow+700)&&(M1PID.CurrSpeed>MINSPEED))
        {
          M1PID.SetSpeed = MINSPEED;
        }  
        break;  
        */  
        
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          if(M1PID.CurrSpeed > 0)
          {
            if((M1State.LimitUp <= M1State.HallNow)) //运动到电子的上顶点则停止运行
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop; //电机1进入停止状态 
            }
            else
            {
              M1Down(OFF);
              M1Up(ON);
            } 
          }
          else    //M1PID.CurrSpeed <= 0
          {
            if(M1State.LimitUp <= (M1State.HallNow+DELT_HALL))
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop; //电机1进入停止状态 
            }
            else
            {
              M1Down(OFF);
              M1Up(ON);
            }
          } 
          //快要运行到最高处时，进入减速阶段
          if((M1State.LimitUp <= M1State.HallNow + 700) && (M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          } 
        }
        else    //正常运行阶段，这时就要考虑到为了自适应，产生的两电机间HALL的偏差值
        {
                if(M1PID.CurrSpeed > 0)
                {
                    //if((M1State.LimitUp <= M1State.HallNow)) //运动到电子的上顶点则停止运行
                    if((M1State.RelativeLimitUp <= M1State.HallNow)) //运动到相对顶点则停止运行
                    {
                        M1Up(OFF);
                        M1Down(OFF);
                        M1_PWM_OFF;
                        M1Cmd = CmdStop; //电机1进入停止状态 
                        
                        //SetM1BeforeRunState(3);
                        
                    }
                    else
                    {
                        M1Down(OFF);
                        M1Up(ON);         //就是简单的对换向继电器进行操作（GPIO的宏定义操作）
                    } 
                }
                else    //M1PID.CurrSpeed <= 0
                {
                    //if(M1State.LimitUp <= (M1State.HallNow+DELT_HALL))
                    if(M1State.RelativeLimitUp <= (M1State.HallNow + DELT_HALL))
                    {
                        M1Up(OFF);
                        M1Down(OFF);
                        M1_PWM_OFF;
                        M1Cmd = CmdStop; //电机1进入停止状态 
                    }
                    else
                    {
                        M1Down(OFF);
                        M1Up(ON);
                    }
                } 
                //if((M1State.LimitUp <= M1State.HallNow+700)&&(M1PID.CurrSpeed>MINSPEED))
                if((M1State.RelativeLimitUp <= M1State.HallNow+700)&&(M1PID.CurrSpeed>MINSPEED))
                {
                    M1PID.SetSpeed = MINSPEED;
                }
        }
        break;
        
      case CmdDown:
        
        /*  //老程序
        M1Up(OFF);
        M1Down(ON); 
        if(M1State.LimitDown >= M1State.HallNow)//运动到电子的下顶点则停止运行
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Cmd = CmdStop;//电机1进入停止状态
        }
        else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
        {
          M1PID.SetSpeed = MINSPEED;
        }
        break;
        */
        
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          M1Up(OFF);
          M1Down(ON); 
          if(M1State.LimitDown >= M1State.HallNow)//运动到电子的下顶点则停止运行
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;//电机1进入停止状态
          }
          else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          }
        }
        else            //正常运行阶段
        {
          M1Up(OFF);
          M1Down(ON); 
          //if(M1State.LimitDown >= M1State.HallNow)//运动到电子的下顶点则停止运行
          if(M1State.RelativeLimitDown >= M1State.HallNow)//运动到电子的下顶点则停止运行
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;//电机1进入停止状态
          }
          //else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
          else if(((M1State.RelativeLimitDown+700) >= M1State.HallNow) && (M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          }
        }
        break;
        
        //升降平台没有此功能  
      case CmdToPreseting:      //运行到设定的位置
        /*   //运行到指定位置操作时，进行平衡度检测操作
        if (GetM1BeforeRunState() == 1)
        {
            if(GetAccFullState() == 1)      //加速度值已获得
            {
                if ((GetAcc_y() < 200) && (GetAcc_y() > -200))        //目前桌子处于平衡状态
                {
                    SetM1BeforeRunState(2);   
                    SetM2BeforeRunState(2);
                    
                    HealthModeReset();
              
                    SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
                    SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
                    PID_Set(Speed,BASEDUTYDOWN);
                    
                }
                else
                {
                    SetM1BeforeRunState(0);
                    SetM2BeforeRunState(0);
                    EnableHPSlopeFilter();
                    
                    M1Up(OFF);
                    M1Down(OFF);
                    M2Up(OFF);
                    M2Down(OFF);
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;//电机1进入停止状态           
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
                    
                    BuzzerWorkMode = 3;
                    BuzzerState = ON;
                    BuzzerOnTimerCnt = 0;
                }
            }
        }
        if (GetM1BeforeRunState() == 2)  
        {
            if((M1State.Record[Position] > (M1State.HallNow+10))&&(M1State.HallNow < M1State.LimitUp))//设定位置在当前位置上方则向上运行
            {
                if(M1State.Record[Position] + M2State.Record[Position] > M1State.HallNow + M2State.HallNow + 20)
                {
                    M1Dir = UP;
                    DeleteSavedHeight();        //删除存储的当前高度数据
                }
            }
            else if(((M1State.Record[Position]+10)< M1State.HallNow)&&(M1State.HallNow > M1State.LimitDown))//设定位置在当前位置下方则向下运行
            {
                if(M1State.Record[Position]+M2State.Record[Position]+20<M1State.HallNow+M2State.HallNow)
                {
                    M1Dir = DOWN;
                    DeleteSavedHeight();        //删除存储的当前高度数据
                }
            }
            if(M1Dir == UP)
            {
                if((M1State.Record[Position] - M1State.HallNow) < 500)//快到达设定位置时减速运行
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//到达指定位置后停止运行
                {
                    M1Up(OFF);
                    M1Down(OFF);
                    M2Up(OFF);
                    M2Down(OFF);
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;//电机1进入停止状态           
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
                    
                    SetM1BeforeRunState(3);
                    SetM2BeforeRunState(3);
                    
                }
                else
                {
                    M1Down(OFF);
                    M1Up(ON);
                }
            }
            else if(M1Dir == DOWN)
            {
                if(M1State.HallNow - M1State.Record[Position] < 500)//快到达设定位置时减速运行
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.Record[Position]+M2State.Record[Position]) >= (M1State.HallNow+M2State.HallNow+5))//到达指定位置后停止运行
                {
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
                    
                    SetM1BeforeRunState(3);
                    SetM2BeforeRunState(3);
                    
                }
                else
                {
                    M1Up(OFF);
                    M1Down(ON);
                }
            }
            else 
            {
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M1Cmd = CmdStop;    
                
                SetM1BeforeRunState(3);
                SetM2BeforeRunState(3);
            }
        }
        else if (GetM1BeforeRunState() == 3)
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Cmd = CmdStop;
        }
        */
        
         if((M1State.Record[Position] > (M1State.HallNow+10))&&(M1State.HallNow < M1State.LimitUp))//设定位置在当前位置上方则向上运行
         {
             if(M1State.Record[Position] + M2State.Record[Position] > M1State.HallNow + M2State.HallNow + 20)
             {
                 M1Dir = UP;
                 DeleteSavedHeight();        //删除存储的当前高度数据
              }
         }
         else if(((M1State.Record[Position]+10)< M1State.HallNow)&&(M1State.HallNow > M1State.LimitDown))//设定位置在当前位置下方则向下运行
         {
             if(M1State.Record[Position]+M2State.Record[Position]+20<M1State.HallNow+M2State.HallNow)
             {
                 M1Dir = DOWN;
                 DeleteSavedHeight();        //删除存储的当前高度数据
              }
          }
          if(M1Dir == UP)
          {
              if((M1State.Record[Position] - M1State.HallNow) < 500)//快到达设定位置时减速运行
              {
                  M1PID.SetSpeed = Speed/2;
               }
               if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//到达指定位置后停止运行
               {
                  M1Up(OFF);
                  M1Down(OFF);
                  M2Up(OFF);
                  M2Down(OFF);
                  M1_PWM_OFF;
                  M1Dir = STOP;
                  M1Cmd = CmdStop;//电机1进入停止状态           
                  M2_PWM_OFF;
                  M2Dir = STOP;
                  M2Cmd = CmdStop;
                    
                  //SetM1BeforeRunState(3);
                  //SetM2BeforeRunState(3);
                    
                }
                else
                {
                   M1Down(OFF);
                   M1Up(ON);
                }
            }
            else if(M1Dir == DOWN)
            {
                if(M1State.HallNow - M1State.Record[Position] < 500)//快到达设定位置时减速运行
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.Record[Position]+M2State.Record[Position]) >= (M1State.HallNow+M2State.HallNow+5))//到达指定位置后停止运行
                {
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
                    
                    //SetM1BeforeRunState(3);
                    //SetM2BeforeRunState(3);
                    
                }
                else
                {
                    M1Up(OFF);
                    M1Down(ON);
                }
            }
            else 
            {
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M1Cmd = CmdStop;    
                
                //SetM1BeforeRunState(3);
                //SetM2BeforeRunState(3);
            }
        
        break;
      
      
      case CmdGoBack:           //检测到过载后回退
        if(M1State.HallNow - M1State.Record[3] < 290)//快到达设定位置时减速运行
        {
          M1PID.SetSpeed = Speed/2;
        }
        /*
        if(M1State.Record[3]+M2State.Record[3]+15 >= M1State.HallNow+M2State.HallNow)//到达指定位置后停止运行
        {
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
          if(M1OverCurFlag == 2)
            ErrCode = Err_M1Overcurrent;
          if(M2OverCurFlag == 2)
            ErrCode = Err_M2Overcurrent;
        }*/
        if (M1State.Record[3]+15 >= M1State.HallNow)
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Dir = STOP;
          M1Cmd = CmdStop;
          
          /*
          if (M2Cmd == CmdNull)
          {
            if(M1OverCurFlag == 2)
            {
                ErrCode = Err_M1Overcurrent;
            }
            if(M2OverCurFlag == 2)
            {
                ErrCode = Err_M2Overcurrent;
            }
          }
          */
        }
        else
        {
          M1Up(OFF);
          M1Down(ON);
        }
        break;
        
        
      case CmdUpRetard: //从向上运行状态进入延时减速状态
        
        //if (Balance_Data.BalanceAdjuseState == 0)
        //{
        //if((FinalPosition <= M1State.HallNow)) //电机1延时减速到达指定位置后则停止运行
        /*      //保存老程序
          if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //把运行速度减到设定速度以下
             (M1State.LimitUp <= M1State.HallNow)||
             (RetardStop==1) ||
             (Balance_Data.RetardM1Stop == 1))
          {
            if (SysCmd != CmdSetBalance)
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop;
              RetardStop=0;
            }
            else
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop;
              Balance_Data.RetardM1Stop = 0;
            }
          }
          else
          {
            M1Down(OFF);
            M1Up(ON);          
          }
        //}    
        break;
        */
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //把运行速度减到设定速度以下
             (M1State.LimitUp <= M1State.HallNow)||
             (Balance_Data.RetardM1Stop == 1))
            {
               M1Up(OFF);
               M1Down(OFF);
               M1_PWM_OFF;
               M1Cmd = CmdStop;
               Balance_Data.RetardM1Stop = 0;
            }
            else        //没到设定目标，继续运行
            {
              M1Down(OFF);
              M1Up(ON);          
            }
        }
        else    //正常运行阶段
        {
            /*if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //把运行速度减到设定速度以下
             (M1State.LimitUp <= M1State.HallNow)||
             (RetardStop==1)）*/
             if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //把运行速度减到设定速度以下
             (M1State.RelativeLimitUp <= M1State.HallNow)||
             (RetardStop==1)) 
            {
               M1Up(OFF);
               M1Down(OFF);
               M1_PWM_OFF;
               M1Cmd = CmdStop;
               M2Up(OFF);
               M2Down(OFF);
               M2_PWM_OFF;
               M2Cmd = CmdStop;
               RetardStop=0;
            }
            else
            {
              M1Down(OFF);
              M1Up(ON);          
            }
        }
        break;
        
      case CmdDownRetard://从向下运行状态进入延时减速状态
        //if(FinalPosition >= M1State.HallNow)//电机1延时减速到达指定位置后则停止运行
        //RetardStop为进入减速阶段800ms后 被置位
        /*  保存老程序
        if((M1PID.CurrSpeed <= 25)|| 
           (M1State.LimitDown >= M1State.HallNow)|| 
           (RetardStop==1) ||
           (Balance_Data.RetardM1Stop == 1))
        {
          if (SysCmd != CmdSetBalance)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            RetardStop = 0;      
          } //   
          else
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;
            Balance_Data.RetardM1Stop = 0;
          }
        }
        else
        {
          M1Up(OFF);
          M1Down(ON);
        }
        break;
        */
        
        if (Balance_Data.BalanceAdjuseState != 0)       //平衡调整阶段
        {
          if((M1PID.CurrSpeed <= 25)|| 
             (M1State.LimitDown >= M1State.HallNow)|| 
             (Balance_Data.RetardM1Stop == 1))
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;
            Balance_Data.RetardM1Stop = 0;
          }
          else
          {
            M1Up(OFF);
            M1Down(ON);
          }
        }
        else    //正常运行阶段
        {
          if((M1PID.CurrSpeed <= 25)|| 
             (M1State.RelativeLimitDown >= M1State.HallNow)|| 
             (RetardStop==1))
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop;
              RetardStop = 0;       
            }
            else
            {
              M1Up(OFF);
              M1Down(ON);
            }
        }
        break;
        
      case CmdStop:
        M1Up(OFF);
        M1Down(OFF);
        M1PID_Set(Speed,BASEDUTYDOWN);
        //SetTIM1_PWMDuty(1,M1PID.PWMDuty);
        M1_PWM_OFF;
        if(M2Cmd == CmdNull)    //三个电机全部停止运行了
        {
          SysCmd = CmdNull;
          
          T400msCnt = 0;
          
          M1Cmd = CmdNull;       
          
          DelayFlagForTap = 1;
          DelayCntForTap = 0;
          
        }
        M1CurMax = 0;
        break;
        
      case CmdNull:
        M1State.HallLast = M1State.HallNow;
        M1CurMax = 0;
        if (M2Cmd == CmdNull)
        {
            if(M1OverCurFlag == 2)
            {
                ErrCode = Err_M1Overcurrent;
            }
            if(M2OverCurFlag == 2)
            {
                ErrCode = Err_M2Overcurrent;
            }
         }
        
        break;      
    }   
  }
  else if(SysState == RESET) //系统当前处于复位状态
  {
    switch(cmd)
    {
      case CmdUp:
        M1Down(OFF);
        M1Up(ON);
        if(M1State.HallNow >= (BOTTOM+(u16)((MinColumnHeight-BASEHEIGHT)*RATE))) //电机1到达底端位置则停止运行
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Cmd = CmdUpHold;//从上行状态转入暂停状态
        }
        break;
      
      case CmdDown:  //向下运动    
        M1Up(OFF);
        M1Down(ON);
        if(M1Detect>=2)//PWM调节达到一定值,电机速度仍然小于阈值，则认为已经到达结构底端
        {
          if((ADCValue.M1Current>(M1CurrBase + 100))||(M1PID.PWMDuty > 1000))
          {
            if(M1PID.PWMDuty>550)
            {
              M1Detect = 0;
              M1CurrBase = 0;
              M1Down(OFF);
              M1Up(OFF);
              M1_PWM_OFF;
              M1State.HallNow = BASEHALL;
              M1State.HallLast = BASEHALL;
              M1PID_Set(Speed,BASEDUTYDOWN);
              M1Cmd = CmdDownHold; //保持当前状态，等待其他电机复位到结构最底端
            }
          }
          else
          {
            M1CurrBase = (u16)ADCValue.M1Current;
          }
          M1Detect = 0;
        }
        break;
        
      case CmdStop:  //停止运动    
        M1Up(OFF);
        M1Down(OFF); 
        M1_PWM_OFF;
        M1Detect = 0;
        M1PID_Set(MINSPEED,BASEDUTYDOWN);
        if(M2Cmd == CmdNull)//三个电机全部停止运行了
        {
          SysCmd = CmdNull;
          M1Cmd = CmdNull;
        }
        break;
      
      case CmdDownHold: //从向下运行转入停止状态，等待其他电机复位
        if(M2Cmd == CmdDownHold)//三个电机都复位到结构最底端则一起向上运动
        {
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1,1350-BASEDUTYUP);
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2,1350-BASEDUTYUP);
          PID_Set(MINSPEED,BASEDUTYUP);
        }
        break; 
      //  
      case CmdUpHold://从向上运行转入停止状态，等待其他电机复位
        if(M2Cmd == CmdUpHold)//三个电机都复位结束则系统复位完成
        {
           if (AccDataBag.Y_OffsetFlag == 0)  
          {
              if (RstBalanceFlag == 0)
              {
                  RstBalanceFlag = 1;
                  ResetCnt = 0;
                  return ;
              }
              else
              {
                  if (ResetCnt <= 1500)
                  {
                      return;
                  }
              }
          }
            
          PID_Set(MINSPEED,BASEDUTYDOWN);
          
          M1Cmd = CmdNull;
          M2Cmd = CmdNull;
          SysCmd = CmdNull;
          M1Detect = 0;
          M2Detect = 0;
          Delay_ms(200);
          
          M1State.LimitDown = M1State.HallNow;
          M1State.LimitUp = M1State.LimitDown + DiffHall;
          M2State.LimitDown = M2State.HallNow;
          M2State.LimitUp = M2State.LimitDown + DiffHall;     
          
          
          //复位完成后，把两个电机上下运行的相对最大值和最小值都复位
          /*******************************************************************/
            M1State.RelativeLimitUp = M1State.LimitUp;          
            M2State.RelativeLimitUp = M2State.LimitUp;
  
            M1State.RelativeLimitDown = M1State.LimitDown;
            M2State.RelativeLimitDown = M2State.LimitDown;
          
          /*******************************************************************/
            
          
          ErrCode = 0;
          memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));                          //清除故障值
          
          memcpy(&Buffer[8],&M1State.LimitDown,sizeof(M1State.LimitDown));      //存入M1State.LimitDown值
          memcpy(&Buffer[10],&M2State.LimitDown,sizeof(M2State.LimitDown));     //存入M2State.LimitDown值
          //EEPROM_Write();
          
          DisplayMode = HeightMode;
          Release = 0;
          
          Balance_Data.TwoMotorOffsetHall = 0;          //把偏移量清零，并写入EEPROM中
          
          memcpy(&Buffer[62],& Balance_Data.TwoMotorOffsetHall,sizeof( Balance_Data.TwoMotorOffsetHall));   //Buffer[62]
          
          if(AccDataBag.Y_OffsetFlag == 0)      //未获取陀螺仪偏移量（第一次上电后，进行复位操作后，把陀螺仪的偏移量存进EEPROM中）
          {
            AccDataBag.Y_Offset = (s16)(AccDataBag.ALLAccXl_y >> 3);
          
            memcpy(&Buffer[64],&AccDataBag.Y_Offset,sizeof(AccDataBag.Y_Offset));
            
            AccDataBag.Y_OffsetFlag = 1;
            
            memcpy(&Buffer[66],&AccDataBag.Y_OffsetFlag,sizeof(AccDataBag.Y_OffsetFlag));       //已获取陀螺仪偏移量的标志位存入EEPROM中
          
            EnableHPSlopeFilter();
          }
          
          EEPROM_Write();
          
          SysState = NORMAL;            //系统进入正常运行状态
        }
        break;
        
      case CmdNull:

        break;
    }
  }
}


/***********************************************************************************************************
* 函数名称: M2Control()
* 输入参数: cmd，控制命令
* 返回值  : 无
* 功    能: 电机2运行控制
************************************************************************************************************/
void M2Control(u8 cmd)
{
  if(SysState == NORMAL) //系统当前处于正常运行状态
  {
    switch(cmd)
    {
      case CmdUp:
        /* 保存老程序
        if(M2PID.CurrSpeed > 0)
        {
          if((M2State.LimitUp <= M2State.HallNow)) //运动到电子的上顶点则停止运行
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop; //电机1进入停止状态
          }
          else
          {
            M2Down(OFF);
            M2Up(ON);
          }
        }
        else
        {
          if(M2State.LimitUp <= (M2State.HallNow+DELT_HALL))
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop; //电机1进入停止状态
          }
          else
          {
            M2Down(OFF);
            M2Up(ON);
          }
        }               
        if((M2State.LimitUp <= M2State.HallNow+700)&&(M2PID.CurrSpeed>MINSPEED))
        {
          M2PID.SetSpeed = MINSPEED;
        }
        break;
        */
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          if(M2PID.CurrSpeed > 0)
          {
            if((M2State.LimitUp <= M2State.HallNow)) 
            {
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop; //电机1进入停止状态
            }
            else
            {
              M2Down(OFF);
              M2Up(ON);
            }
          }
          else
          {
            if(M2State.LimitUp <= (M2State.HallNow+DELT_HALL))
            {
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop; //电机1进入停止状态
            }
            else
            {
              M2Down(OFF);
              M2Up(ON);
            }
          }               
          if((M2State.LimitUp <= M2State.HallNow+700)&&(M2PID.CurrSpeed>MINSPEED))
          {
            M2PID.SetSpeed = MINSPEED;
          }
        }
        else    //进入正常运行阶段
        {          
            //if (GetM2BeforeRunState() == 2)
            {          
                if(M2PID.CurrSpeed > 0)
                {
                    //if(M2State.LimitUp <= M2State.HallNow) //运动到电子的上顶点则停止运行
                    if((M2State.RelativeLimitUp <= M2State.HallNow)&&(Adjust_State==0)) 
                    {
                        M2Up(OFF);
                        M2Down(OFF);
                        M2_PWM_OFF;
                        M2Cmd = CmdStop; //电机1进入停止状态
                        
                        //SetM2BeforeRunState(3);
                    }
                   else if((Adjust_State==2)&&(M2State.LimitUp <= (M2State.HallNow)))
                   {
                    M2Up(OFF);
                    M2Down(OFF);
                    M2_PWM_OFF;
                    M2Cmd = CmdStop;
          
                   }
                    else
                    {
                        M2Down(OFF);
                        M2Up(ON);
                    }
                }
                else
                {
                    if((M2State.RelativeLimitUp <= (M2State.HallNow+DELT_HALL))&&(Adjust_State==0))
                    {
                        M2Up(OFF);
                        M2Down(OFF);
                        M2_PWM_OFF;
                        M2Cmd = CmdStop; //电机1进入停止状态
                        
                        //SetM2BeforeRunState(3);
                    }
                     else if((Adjust_State==2)&&(M2State.LimitUp <= (M2State.HallNow)))
                   {
                    M2Up(OFF);
                    M2Down(OFF);
                    M2_PWM_OFF;
                    M2Cmd = CmdStop;
          
                   }
                    else
                    {
                        M2Down(OFF);
                        M2Up(ON);
                    }
                }               
                if((M2State.RelativeLimitUp <= M2State.HallNow+700)&&(M2PID.CurrSpeed>MINSPEED))
                {
                    M2PID.SetSpeed = MINSPEED;
                }
            }
            /*
            else if (GetM2BeforeRunState() == 3)
            {
                M2_PWM_OFF;
                M2Cmd = CmdStop;
            }
            */
        }
        break;
        
      case CmdDown:
        /*      //保存老程序
        M2Up(OFF);
        M2Down(ON);
        if(M2State.LimitDown >= M2State.HallNow)//运动到电子的下顶点则停止运行
        {
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          M2Cmd = CmdStop;
        }
        else if(((M2State.LimitDown+700) >= M2State.HallNow)&&(M2PID.CurrSpeed>MINSPEED))
        {
          M2PID.SetSpeed = MINSPEED;
        }
        break;
        */
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          M2Up(OFF);
          M2Down(ON);
          if(M2State.LimitDown >= M2State.HallNow)//运动到电子的下顶点则停止运行
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
          }
          else if(((M2State.LimitDown+700) >= M2State.HallNow)&&(M2PID.CurrSpeed>MINSPEED))
          {
            M2PID.SetSpeed = MINSPEED;
          }
        }
        else    //正常运行阶段
        {
          M2Up(OFF);
          M2Down(ON);
          if((M2State.RelativeLimitDown >= M2State.HallNow)&&(Adjust_State==0))//运动到电子的下顶点则停止运行
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
          }
          else if((Adjust_State==2)&&(M2State.LimitDown >= M2State.HallNow))
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
          
          }
          else if(((M2State.RelativeLimitDown+700) >= M2State.HallNow)&&(M2PID.CurrSpeed>MINSPEED)&&(Adjust_State==0))
          {
            M2PID.SetSpeed = MINSPEED;
          }
        }
        break;
        
      //运行到指定高度指令 
      case CmdToPreseting: 
        //{
            /*
            if (GetM2BeforeRunState() == 2) 
            {
          
            if((M2State.Record[Position] > (M2State.HallNow + 10)) && (M2State.HallNow < M2State.LimitUp))//设定位置在当前位置上方
            {
                if((M1State.Record[Position] + M2State.Record[Position]) > (M1State.HallNow + M2State.HallNow + 20))
                {
                    M2Dir = UP;
                }
            }
            else if(((M2State.Record[Position] + 10) < M2State.HallNow) && (M2State.HallNow > M2State.LimitDown))//设定位置在当前位置下方
            { 
                if((M1State.Record[Position] + M2State.Record[Position] + 20) < (M1State.HallNow + M2State.HallNow))
                {
                    M2Dir = DOWN;
                }
            }
            if(M2Dir == UP)
            {
                if((M2State.Record[Position] - M2State.HallNow) < 500)//快到达设定位置时减速运行
                {
                    M2PID.SetSpeed = Speed/2;
                }
                if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//到达设定位置后停止运行
                {
                    M2Up(OFF);
                    M2Down(OFF);
                    M1Up(OFF);
                    M1Down(OFF);
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;  
                    
                    SetM2BeforeRunState(3);
                    SetM2BeforeRunState(3);
                    //EnableHPSlopeFilter();
                }
                else
                { 
                    M2Down(OFF);
                    M2Up(ON);
                }
            }
            else if(M2Dir == DOWN)
            {
                if((M2State.HallNow - M2State.Record[Position]) < 500)//快到达设定位置时减速运行
                {
                    M2PID.SetSpeed = Speed/2;
                }
                if((M2State.Record[Position] + M1State.Record[Position]) >= (M2State.HallNow + M1State.HallNow + 5))//到达设定位置后停止运行
                {
                    M2Up(OFF);
                    M2Down(OFF);
                    M1Up(OFF);
                    M1Down(OFF);
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;
                    
                    SetM2BeforeRunState(3);
                    SetM2BeforeRunState(3);
                    
                    //SetCmdToPresetState(0);
                    //EnableHPSlopeFilter();
                    
                    //SetM1BeforeRunState(3);
                    //SetM2BeforeRunState(3);
                }
                else
                {
                    M2Up(OFF);
                    M2Down(ON);
                }
            } 
            else
            { 
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                M2Cmd = CmdStop;
                
                SetM2BeforeRunState(3);
                SetM1BeforeRunState(3);
            }
          }
          else if (GetM2BeforeRunState() == 3)
          {
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                M2Cmd = CmdStop;
          }
        }
        */
          
          
        if((M2State.Record[Position] > (M2State.HallNow + 10)) && (M2State.HallNow < M2State.LimitUp))//设定位置在当前位置上方
        {
             if((M1State.Record[Position] + M2State.Record[Position]) > (M1State.HallNow + M2State.HallNow + 20))
             {
                 M2Dir = UP;
              }
         }
         else if(((M2State.Record[Position] + 10) < M2State.HallNow) && (M2State.HallNow > M2State.LimitDown))//设定位置在当前位置下方
         { 
              if((M1State.Record[Position] + M2State.Record[Position] + 20) < (M1State.HallNow + M2State.HallNow))
              {
                  M2Dir = DOWN;
              }
          }
          if(M2Dir == UP)
          {
              if((M2State.Record[Position] - M2State.HallNow) < 500)//快到达设定位置时减速运行
              {
                  M2PID.SetSpeed = Speed/2;
              }
              if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//到达设定位置后停止运行
              {
                  M2Up(OFF);
                  M2Down(OFF);
                  M1Up(OFF);
                  M1Down(OFF);
                  M2_PWM_OFF;
                  M2Dir = STOP;
                  M2Cmd = CmdStop;
                  M1_PWM_OFF;
                  M1Dir = STOP;
                  M1Cmd = CmdStop;  
                    
                  //SetM2BeforeRunState(3);
                  //SetM2BeforeRunState(3);
                  //EnableHPSlopeFilter();
               }
               else
               { 
                  M2Down(OFF);
                  M2Up(ON);
               }
            }
            else if(M2Dir == DOWN)
            {
              if((M2State.HallNow - M2State.Record[Position]) < 500)//快到达设定位置时减速运行
              {
                    M2PID.SetSpeed = Speed/2;
              }
              if((M2State.Record[Position] + M1State.Record[Position]) >= (M2State.HallNow + M1State.HallNow + 5))//到达设定位置后停止运行
              {
                    M2Up(OFF);
                    M2Down(OFF);
                    M1Up(OFF);
                    M1Down(OFF);
                    M2_PWM_OFF;
                    M2Dir = STOP;
                    M2Cmd = CmdStop;
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;
                    
                    //SetM2BeforeRunState(3);
                    //SetM2BeforeRunState(3);
                    
               }
               else
               {
                   M2Up(OFF);
                   M2Down(ON);
               }
            } 
            else
            { 
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                M2Cmd = CmdStop;
                
                //SetM2BeforeRunState(3);
                //SetM1BeforeRunState(3);
            }
        break;
        
       //无此功能  
      case CmdGoBack:       //遇阻回退功能
        if(M2State.HallNow - M2State.Record[3] < 290)//快到达设定位置时减速运行
        {
          M2PID.SetSpeed = Speed/2;
        }
        /*
        if(M2State.Record[3]+M1State.Record[3]+15 >= M2State.HallNow+M1State.HallNow)//到达设定位置后停止运行
        {
          M2Up(OFF);
          M2Down(OFF);
          M1Up(OFF);
          M1Down(OFF);
          M2_PWM_OFF;
          M2Dir = STOP;
          M2Cmd = CmdStop;
          M1_PWM_OFF;
          M1Dir = STOP;
          M1Cmd = CmdStop;
          if(M1OverCurFlag == 2)
            ErrCode = Err_M1Overcurrent;
          if(M2OverCurFlag == 2)
            ErrCode = Err_M2Overcurrent;
        }
        */
        if (M2State.Record[3]+15 >= M2State.HallNow)
        {
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          M2Dir = STOP;
          M2Cmd = CmdStop;
          
          if (M1Cmd == CmdNull)
          {
            if(M1OverCurFlag == 2)
            {
                ErrCode = Err_M1Overcurrent;
            }
            if(M2OverCurFlag == 2)
            {
                ErrCode = Err_M2Overcurrent;
            }
          }
        }
        else
        {
          M2Up(OFF);
          M2Down(ON);
        }
        break;
        
        
      case CmdUpRetard: //从向上运行状态进入延时减速状态
        //if((FinalPosition <= M2State.HallNow)) //减速运动到设定位置
        /* 保存老程序
        if((M2PID.CurrSpeed <= M2PID.SetSpeed)||
           (M2State.LimitUp <= M2State.HallNow)||
           (RetardStop == 1) ||
           (Balance_Data.RetardM2Stop == 1))
        {
          if(SysCmd != CmdSetBalance)
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;
            RetardStop = 0;
          }
          else
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            Balance_Data.RetardM2Stop = 0;
          }         
        }
        else
        {
          M2Down(OFF);
          M2Up(ON);         
        }
        break;
        */
        
         if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
         {
           if((M2PID.CurrSpeed <= M2PID.SetSpeed)||
           (M2State.LimitUp <= M2State.HallNow)||
           (Balance_Data.RetardM2Stop == 1))
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            Balance_Data.RetardM2Stop = 0;
          }
          else
          {
            M2Down(OFF);
            M2Up(ON);         
          }
         }
         else   //正常运行阶段
         {
           if((M2PID.CurrSpeed <= M2PID.SetSpeed)||
           (M2State.RelativeLimitUp <= M2State.HallNow)||
           (RetardStop == 1))
            {
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop;
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop;
              RetardStop = 0;
            }
            else
            {
              M2Down(OFF);
              M2Up(ON);         
            }
         }
         break;
        
        
      case CmdDownRetard://从向下运行状态进入延时减速状态
        //if(FinalPosition >= M2State.HallNow)//加速运动到设定位置
        /* //保存老程序
        if((M2PID.CurrSpeed <= 25)||
           (M2State.LimitDown >= M2State.HallNow)||
           (RetardStop == 1)||
           (Balance_Data.RetardM2Stop == 1))
        {
          if (SysCmd != CmdSetBalance)
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;
            RetardStop = 0;
          }
          else
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            Balance_Data.RetardM2Stop = 0;
          }
        }
        else
        {
          M2Up(OFF);
          M2Down(ON);
        }
        break;
        */
        
        if (Balance_Data.BalanceAdjuseState != 0)     //平衡调整阶段
        {
          if((M2PID.CurrSpeed <= 25)||
             (M2State.LimitDown >= M2State.HallNow)||
             (Balance_Data.RetardM2Stop == 1))
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop;
            Balance_Data.RetardM2Stop = 0;
          }
          else
          {
            M2Up(OFF);
            M2Down(ON);
          }
        }
        else    //正常运行阶段
        {
          if((M2PID.CurrSpeed <= 25)||
             (M2State.RelativeLimitDown >= M2State.HallNow)||
             (RetardStop == 1))
            {
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop;
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop;
              RetardStop = 0;
            }
           else
           {
              M2Up(OFF);
              M2Down(ON);
           }
        }
        break;
        
      case CmdStop:
        M2Up(OFF);
        M2Down(OFF);
        M2_PWM_OFF;
        M2PID_Set(Speed,BASEDUTYDOWN);
        M2Cmd = CmdNull;
        M2CurMax = 0;
        break;
        
      case CmdNull:
        M2State.HallLast = M2State.HallNow;
        M2CurMax = 0;
        
        if (M1Cmd == CmdNull)
        {
            if(M1OverCurFlag == 2)
            {
                ErrCode = Err_M1Overcurrent;
            }
            if(M2OverCurFlag == 2)
            {
                ErrCode = Err_M2Overcurrent;
            }
         }
        
        break;
    }   
  }
  else if(SysState == RESET) //系统当前处于复位状态
  {
    switch(cmd)
    {
      case CmdUp:
        M2Down(OFF);
        M2Up(ON);
        if(M2State.HallNow >= (BOTTOM+(u16)((MinColumnHeight - BASEHEIGHT)*RATE)))
        {
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          M2Cmd = CmdUpHold;
        }
        break;
        
      case CmdDown: //向下运行
        M2Up(OFF);
        M2Down(ON);
        if(M2Detect>=2)
        {
          if((ADCValue.M2Current>(M2CurrBase+100))||(M2PID.PWMDuty > 1000))
          {
            if(M2PID.PWMDuty>550)//PWM调节达到一定值,电机速度仍然小于阈值则认为已经到达结构底端
            {
              M2Detect = 0;
              M2CurrBase = 0;
              M2Down(OFF);
              M2Up(OFF); 
              M2_PWM_OFF;
              M2PID_Set(Speed,BASEDUTYDOWN);
              M2State.HallNow = BASEHALL;
              M2State.HallLast = BASEHALL;
              M2Cmd = CmdDownHold;
            }
          }
          else
          {
            M2CurrBase = (u16)ADCValue.M2Current;
          }
          M2Detect = 0;
        }
        break;
        
      case CmdStop: //停止运动
        M2Up(OFF);
        M2Down(OFF);
        M2_PWM_OFF;
        M2Detect = 0;
        if(SysState == NORMAL)
        {
          M2PID_Set(Speed,BASEDUTYDOWN);
        }
        else if(SysState == RESET)
        {
          M2PID_Set(MINSPEED,BASEDUTYDOWN);
        }
        //SetTIM1_PWMDuty(2,M2PID.PWMDuty);
        M2Cmd = CmdNull;
        break;
        
      case CmdNull:
        //if(ErrCode!=0)
        //{
          //M2State.HallNow = BASEHALL;
          //M2State.HallLast = BASEHALL;
        //}
        break;
    }
  }
}


/***********************************************************************************************************
* 函数名称: MotorControl()
* 输入参数: 无
* 返回值  : 无
* 功    能: 电机控制函数，封装三个电机控制函数
************************************************************************************************************/
void MotorControl(void)
{
  M1Control(M1Cmd);    //初始化时，M1Cmd，M2Cmd都为0
  M2Control(M2Cmd);
  
  if(SysCmd == CmdNull)   //初始化时，SysCmd = 0
  {
    CurrentOut(ON);     //使能输出的33V  //就一个关于GPIO管脚的宏定义操作
    v33Flag = 0;        //v33Flag = 0，使能输出33V，v33Flag = 1，不使能输出33V
  }
  else if((ADCValue.M1Current+ADCValue.M2Current)>700)  //250w
  {
    CurrentOut(OFF);    //就一个关于GPIO管脚的宏定义操作
    if((v33Flag == 0)&&(motorStartTimer > 500))             //6轴屏蔽500ms
    {
       v33Flag = 1;
       motorStartTimer = 500;
    }
  }
  /*
  if ((GetM2BeforeRunState() == 3) && (GetM1BeforeRunState() == 3))
  {
    SetM2BeforeRunState(0);
    SetM1BeforeRunState(0);
    
    M2_PWM_OFF;
    M1_PWM_OFF;
    EnableHPSlopeFilter();
  }
  */
}


/***********************************************************************************************************
* 函数名称: LimitPower()
* 输入参数: 无
* 返回值  : 无
* 功    能: 限定系统最大功率
************************************************************************************************************/
void LimitPower(void)
{
  if((SysCmd == CmdStop)||(SysCmd == CmdNull))
  {
    T2sCnt1 = 0;
    LimitPowerFlag = 0;
    LimitPowerState = 0;
  }
  if(LimitPowerFlag == 1) //电机运行500ms后，通过判断电机输出电流和电机当前速度，使电机进入LimitPowerFlag==2的运行状态
  {
    if((ADCValue.M1Current+ADCValue.M2Current)>890)
    {
      if((M1PID.CurrSpeed >= 5)&&(M2PID.CurrSpeed >= 5))
      {
        M1PID.SetSpeed = (M1PID.CurrSpeed+M2PID.CurrSpeed)/2+3;
        M2PID.SetSpeed = M1PID.SetSpeed;
      }
      LimitPowerFlag = 2;
      T500msCnt = 0;
      LimitPowerState = 1;
    }
    else
    {
      LimitPowerFlag = 1;
      T500msCnt = 0;     
    }
  }
  else if(LimitPowerFlag == 2)
  {
    if((ADCValue.M1Current+ADCValue.M2Current)>890)
    {
      if(T500msCnt == 600)
      {
        if((M1PID.CurrSpeed >=M1PID.SetSpeed)&&(M2PID.CurrSpeed>=M2PID.SetSpeed))
        {
          if(((M1PID.CurrSpeed + M2PID.CurrSpeed)/2) > 9)
          {
            M1PID.SetSpeed = (M1PID.CurrSpeed + M2PID.CurrSpeed)/2 - 3;
            M2PID.SetSpeed = M1PID.SetSpeed;
          }
        }
        T500msCnt = 0;
      }            
    }
    else if((ADCValue.M1Current + ADCValue.M2Current) < 810)
    {
      if(T500msCnt == 600)
      {
        if((((M1Cmd==CmdUp)&&(M1State.LimitUp > (M1State.HallNow+700)))||((M1Cmd==CmdToPreseting)&&(M1Dir==UP)&&((M1State.Record[Position] - M1State.HallNow) > 1500)))
          &&(((M2Cmd==CmdUp)&&(M2State.LimitUp > (M2State.HallNow+700)))||((M2Cmd==CmdToPreseting)&&(M2Dir==UP)&&((M2State.Record[Position] - M2State.HallNow) > 1500))))
        {
          if((M1PID.CurrSpeed <= (Speed-3))&&(M2PID.CurrSpeed <= (Speed-3)))
          {
            M1PID.SetSpeed = (M1PID.CurrSpeed+M2PID.CurrSpeed)/2+3;
            M2PID.SetSpeed = M1PID.SetSpeed;
          }          
        }
        T500msCnt = 0;
      }
    }
    else 
    {
      if(T500msCnt == 600)
      {
        if((((M1Cmd==CmdUp)&&(M1State.LimitUp > (M1State.HallNow+700)))||((M1Cmd==CmdToPreseting)&&(M1Dir==UP)&&((M1State.Record[Position] - M1State.HallNow) > 1500)))
          &&(((M2Cmd==CmdUp)&&(M2State.LimitUp > (M2State.HallNow+700)))||((M2Cmd==CmdToPreseting)&&(M2Dir==UP)&&((M2State.Record[Position] - M2State.HallNow) > 1500))))
        {          
          M1PID.SetSpeed = (M1PID.CurrSpeed+M2PID.CurrSpeed)/2+1;
          M2PID.SetSpeed = M1PID.SetSpeed;
        }
        T500msCnt = 0;
      }
    }
    if((SysCmd == CmdUp)||(SysCmd==CmdDown)||((M1Dir!=0)&&(M2Dir!=0)))
    {
      if((M1PID.CurrSpeed <= 12)&&(M2PID.CurrSpeed <= 12)&&(M1PID.SetSpeed <= 12)&&(M2PID.SetSpeed <= 12))
      {
        OverCurFlag = 1;
      }
    }
  }
}

/***********************************************************************************************************
* 函数名称: AntiCollision()
* 输入参数: 无
* 返回值  : 无
* 功    能: 防碰撞处理
************************************************************************************************************/
/***********************************************************************************************************
* 函数名称: AntiCollision()
* 输入参数: 无
* 返回值  : 无
* 功    能: 防碰撞处理
************************************************************************************************************/
void AntiCollision(void)
{
  if(Sensitivity > 0)
  {
    if(LSM6DSLFlag == 1)          //判断是否有安装6轴传感器 LSM6DSLFlag == 1，说明已安装
    {
      hd_anti_up = HD_ANTI_UP_1;        //HD_ANTI_UP_1 = 16    
      hd_anti_down = HD_ANTI_DOWN_1;    //HD_ANTI_DOWN_1 = 18
      st_anti_up = ST_ANTI_UP_1;        //ST_ANTI_UP_1 = 36
      st_anti_down1 = ST_ANTI_DOWN1_1;  //ST_ANTI_DOWN1_1 = 28
      st_anti_down2 = ST_ANTI_DOWN2_1;  //ST_ANTI_DOWN2_1 = 18
    }
    else
    {
      if(Sensitivity == 1)
      {
        hd_anti_up = HD_ANTI_UP_1;
        hd_anti_down = HD_ANTI_DOWN_1;
        st_anti_up = ST_ANTI_UP_1;
        st_anti_down1 = ST_ANTI_DOWN1_1;
        st_anti_down2 = ST_ANTI_DOWN2_1;
      }
      else if(Sensitivity == 2)
      {
        hd_anti_up = HD_ANTI_UP_2;
        hd_anti_down = HD_ANTI_DOWN_2;
        st_anti_up = ST_ANTI_UP_2;
        st_anti_down1 = ST_ANTI_DOWN1_2;
        st_anti_down2 = ST_ANTI_DOWN2_2;
      }
      else if(Sensitivity > 0)
      {
        hd_anti_up = HD_ANTI_UP_3;
        hd_anti_down = HD_ANTI_DOWN_3;
        st_anti_up = ST_ANTI_UP_3;
        st_anti_down1 = ST_ANTI_DOWN1_3;
        st_anti_down2 = ST_ANTI_DOWN2_3;
      }
    }
    //开始进行防碰撞处理
    if(SysState == NORMAL)
    {  
      //防碰撞处理第一种情况（单电机情况）
      
      //对电机1进行防碰撞处理，先对采集得到的电流值进行滤波处理
      if(M1ADCCnt == 40)  //判断AD采样达到40次
      {
        M1Cur = MiddleAVG(&ADCBuffer1[0], 40);  //对AD数据进行取平均处理（先进行排序，去除最大10个和最小10个数据，然后再对中间20个数据进行取平均计算）
        //对处理得到的AD值进行3级缓存保存操作
        if(M1CurTemp1 == 0)
        {
          M1CurTemp1 = M1Cur;   
          M1ADCCnt = 0;
        }
        else if(M1CurTemp2 == 0)
        {
          M1CurTemp2 = M1Cur;
          M1ADCCnt = 0;
        }
        else if(M1CurTemp3 == 0)
        {
          M1CurTemp3 = M1Cur;
          M1ADCCnt = 0;
        }
      }
      //对电机2进行防碰撞处理，先对采集得到的电流值进行滤波处理
      if(M2ADCCnt==40)
      {    
        M2Cur = MiddleAVG(&ADCBuffer2[0], 40);   
        if(M2CurTemp1 == 0)
        {
          M2CurTemp1 = M2Cur;
          M2ADCCnt = 0;
        }
        else if(M2CurTemp2 == 0)
        {
          M2CurTemp2 = M2Cur;
          M2ADCCnt = 0;
        }
        else if(M2CurTemp3 == 0)
        {
          M2CurTemp3 = M2Cur;
          M2ADCCnt = 0;
        }
      }
      
      //if((M1CurTemp1 != 0)&&(M1CurTemp2 != 0)&&(M1CurTemp3 != 0)&&(M2CurTemp1 != 0)&&(M2CurTemp2 != 0)&&(M2CurTemp3 != 0))
      if((((M1CurTemp1 != 0)&&(M1CurTemp2 != 0)&&(M1CurTemp3 != 0)) ||
          ((M2CurTemp1 != 0)&&(M2CurTemp2 != 0)&&(M2CurTemp3 != 0))))
      {
        //通过判断电机1的当前位置值与设定的最高及最低位置进行比较
        if((M1State.HallNow + 750 < M1State.LimitUp) &&
           (M1State.HallNow > M1State.LimitDown))
        {
          //若电机1的命令是向上，或者运行方向为向上且未进入防撞保护状态
          if(((M1Cmd==CmdUp)||(M1Dir==UP))&&(AntiCollisionState==0))
          {
            if((M1CurTemp3+M1CurTemp1>2*M1CurTemp2+hd_anti_up)&&
               (M1CurTemp2>=M1CurTemp1)&&
               (M1CurTemp3>M1CurTemp2)&&
               ((M1PID.CurrSpeed>35)||(M1PID.SetSpeed<=35)))
            {
              M1Flag = 1;
              M1Up(OFF);                        //向上关闭
              M1Down(OFF);
              M1_PWM_OFF;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              PID_Set(Speed,BASEDUTYUP);        //PID初始化   
            }           
          }//若电机1的命令是向上，或者运行方向为向上且未进入防撞保护状态
          else if(((M1Cmd==CmdDown)||((M1Dir==DOWN)&&(M1Cmd!=CmdGoBack)))&&(AntiCollisionState==0))
          {
            if((M1CurTemp3+M1CurTemp1>2*M1CurTemp2+hd_anti_down)&&(M1CurTemp2>=M1CurTemp1)&&(M1CurTemp3>M1CurTemp2)&&((M1PID.CurrSpeed>35)||(M1PID.SetSpeed<=35)))
            {
              M1Flag = 1;
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              PID_Set(Speed,BASEDUTYUP);         
            }           
          }
        }
        if((M2State.HallNow + 750 < M2State.LimitUp) &&
           (M2State.HallNow > M2State.LimitDown))
        {
          
          if(((M2Cmd==CmdUp)||(M2Dir==UP))&&(AntiCollisionState==0))
          {
            if((M2CurTemp3 + M2CurTemp1 > 2*M2CurTemp2 + hd_anti_up) &&
               (M2CurTemp2 >= M2CurTemp1) && 
               (M2CurTemp3>M2CurTemp2) && ((M2PID.CurrSpeed>35)||(M2PID.SetSpeed<=35)))
            {
              M2Flag = 1;
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              PID_Set(Speed,BASEDUTYUP);          
            }
          }
          else if(((M2Cmd==CmdDown)||((M2Dir==DOWN)&&(M2Cmd!=CmdGoBack)))&&(AntiCollisionState==0))
          {
            if((M2CurTemp3+M2CurTemp1>2*M2CurTemp2+hd_anti_down)&&(M2CurTemp2>=M2CurTemp1)&&(M2CurTemp3>M2CurTemp2)&&((M2PID.CurrSpeed>35)||(M2PID.SetSpeed<=35)))
            {
              M2Flag = 1;
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              PID_Set(Speed,BASEDUTYUP);         
            }     
          }
        }
        
       //防碰撞处理第二种情况（双电机情况）
        
        if((M1State.HallNow+750<M1State.LimitUp)&&
           (M1State.HallNow>M1State.LimitDown)&&
           (M2State.HallNow+750<M2State.LimitUp)&&
           (M2State.HallNow>M2State.LimitDown))
        {
          if((M1CurMax!=0)&&(M2CurMax!=0))      //这两个参数在电机控制上用到
          {
            if(M1CurFlag == 1)  
            {
              M1CurFlag = 0;
              M1CurMax = M1CurTemp3;
            }
            if(M2CurFlag == 1)
            {
              M2CurFlag = 0;
              M2CurMax = M2CurTemp3;
            }
            //系统命令往上升，或者电机1和电机2命令往上升，且未进入防撞保护状态
            if(((SysCmd==CmdUp)||((M1Dir==UP)&&(M2Dir==UP)))&&(AntiCollisionState==0))
            {
              if(((M1CurTemp3+M2CurTemp3)>(M1CurMax+M2CurMax+st_anti_up))&&((M1PID.CurrSpeed>35)||(M1PID.SetSpeed<=35))&&((M2PID.CurrSpeed>35)||(M2PID.SetSpeed<=35)))
              {
                M1Flag = 1;
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                PID_Set(Speed,BASEDUTYUP);      
              }
            }
            else if(((SysCmd==CmdDown)||((M1Dir==DOWN)&&(M2Dir==DOWN)&&(M1Cmd!=CmdGoBack)&&(M2Cmd!=CmdGoBack)))&&(AntiCollisionState==0))
            { 
              if(M1CurFlag == 1)
              {
                M1CurFlag = 0;
                M1CurMax = M1CurTemp3;
              }
              if(M2CurFlag == 1)
              {
                M2CurFlag = 0;
                M2CurMax = M2CurTemp3;
              }
              if(((M1CurTemp3+M2CurTemp3)>(M1CurMax+M2CurMax+st_anti_down1))&&((M1PID.CurrSpeed>35)||(M2PID.SetSpeed<=35))&&((M2PID.CurrSpeed>35)||(M2PID.SetSpeed<=35)))
              {
                M1Flag = 1;
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                PID_Set(Speed,BASEDUTYUP);      
              }
              else if(((M1CurTemp3+M2CurTemp3)>(M1CurMax+M2CurMax+st_anti_down2))&&(M1CurMax+M2CurMax<=100)&&((M1PID.CurrSpeed>35)||(M2PID.SetSpeed<=35))&&((M2PID.CurrSpeed>35)||(M2PID.SetSpeed<=35)))
              {
                M1Flag = 1;
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                PID_Set(Speed,BASEDUTYUP);
              }            
            }
          }
        }
        
        
        if((M1CurMax==0)&&(M1PID.CurrSpeed >=(Speed-1))&&(M1CurTemp3<(M1CurTemp1+10))&&(M1CurTemp1<(M1CurTemp3+10)))
        {
          M1CurMax = M1CurTemp3;      
        }
        if((M2CurMax==0)&&(M2PID.CurrSpeed >=(Speed-1))&&(M2CurTemp3<(M2CurTemp1+10))&&(M2CurTemp1<(M2CurTemp3+10)))
        {
          M2CurMax = M2CurTemp3;      
        }
        if((M1CurMax!=0)&&(M2CurMax!=0)&&(CurFlag==0)&&((SysCmd==CmdDown)||(SysCmd==CmdUp)||(SysCmd == CmdToPreseting)))
        {
          CurFlag = 1; //在向下运行时，当两个电机匀速运行的电流基准值都已经获取时，启动更新计时
        }
        
        M1CurTemp1 = M1CurTemp2;
        M1CurTemp2 = M1CurTemp3;
        M1CurTemp3 = 0;
        M1ADCCnt = 0;
        M2CurTemp1 = M2CurTemp2;
        M2CurTemp2 = M2CurTemp3;
        M2CurTemp3 = 0;
        M2ADCCnt = 0;      
      }     
    }
  }
}

u16 MiddleAVG(u16 *data, u16 len)
{
  u16 x, y, temp;
  u16 *p = 0;
  
  p = data;
  for(x=0; x<len;x++)
  {
    for(y=0; y<len-1;y++)
    {
      if(*(data+y)>*(data+y+1))
      {
        temp = *(data+y+1);
        *(data+y+1)=*(data+y);
        *(data+y) = temp;
      }
    }
  }
  for(x=10,temp = 0; x<30; x++)
  {
    temp += p[x];   
  }
  temp = temp/20;
  return temp;
}

/***************************************************************




***************************************************************/
void AntiLsm6dsl(void)
{
   AntiCollision();//电流遇阻回退判断
   
   if(((M1Cmd!=CmdStop)&&(M1Cmd!=CmdNull))||((M2Cmd!=CmdStop)&&(M2Cmd!=CmdNull)))//设备运行中
   {
      motorStopTimer = 0;                 //定时器数据清0
   }
   else//停止
   {
      motorStartTimer = 0;
      add_Acc=0;
      if(motorStopTimer>=1000)
      {
         stopAng_x = Ang_x;
         stopAng_y = Ang_y;
         stopAng_z = Ang_z;
      }
   }
   //系统处于正常运行模式
   if(SysState == NORMAL)  //系统状态分为 NORMAL 和 RESET     
   {
      
      if (GetBalaceState() == 0)
      {
        if(((Sensitivity == 3)||(Sensitivity == 2))&&(LSM6DSLFlag == 1))
        {
            if(Sensitivity == 3)     //碰撞
            {
                add_value = 9;//硬性碰撞10kg 
            }
            else if(Sensitivity == 2)
            {
                add_value = 12;//硬性碰撞15kg 
            }

            if(Ang_x >= stopAng_x) 
                add_x = Ang_x - stopAng_x;
            else 
                add_x = stopAng_x - Ang_x;
          
            if(Ang_y >= stopAng_y) 
                add_y = Ang_y - stopAng_y;
            else 
                add_y = stopAng_y - Ang_y;
         
            if(Ang_z >= stopAng_z) 
                add_z = Ang_z - stopAng_z;
            else 
                add_z = stopAng_z - Ang_z;
         
            add_Acc = add_x + add_y + add_z;
         
         
         
            if(((SysCmd == CmdUp)||((M1Dir == UP)&&(M2Dir == UP)))&&
               (AntiCollisionState == 0))
            { 
                if((add_Acc >= add_value) && (motorStartTimer > 1000))
                {
                    M1Flag=1; //what
                }
                
                //if(((Acc_z>150)||(Acc_z<-150))&&(motorStartTimer>1000))
                //{
                    //M1Flag=1;
                //}
                
                
                if(((Acc_x>4000)||(Acc_y>4000)||(Acc_x<-4000)||(Acc_y<-4000))&&(motorStartTimer>1000))
                {
                    M1Flag=1;
                }
                
                
                /*
                if ((((Acc_z>7000)||(Acc_z<-7000))&&(motorStartTimer>1000)) && (Tap_Parameter.TapControlFlag == 0))
                {
                    M1Flag=1;
                }
                */  
            }
            else if(((SysCmd == CmdDown)||((M1Dir==DOWN)&&(M2Dir==DOWN)&&(M1Cmd!=CmdGoBack)&&(M2Cmd!=CmdGoBack)))&&
                     (AntiCollisionState==0))
            {
                if((add_Acc >= add_value) && (motorStartTimer > 1000))
                {
                    M1Flag=1;
                }
                
               
                //if(((Acc_z>150)||(Acc_z<-150))&&(motorStartTimer>1000))
                //{
                    //M1Flag=1;
                //}
                
                
                 if(((Acc_x>4000)||(Acc_y>4000)||(Acc_x<-4000)||(Acc_y<-4000))&&(motorStartTimer>1000))
                {
                    M1Flag=1;
                }
                
                /*
                if ((((Acc_z>7000)||(Acc_z<-7000))&&(motorStartTimer>1000)) && (Tap_Parameter.TapControlFlag == 0))
                {
                    M1Flag=1;
                }
                */
            }
         }
      }
      
       if((M1Flag==1)||(M2Flag==1)||(BlockFlag))
       {
           if(BlockFlag==1)
           {
           //Adjust_State==0
           Adjust_State=0;
           Balance_Data_Refresh();
           DisplayMode=HeightMode;    
           
           }
        if((SysCmd==CmdUp)||((M1Dir==UP)&&(M2Dir==UP))) //向上运动
        {
          M1Dir = 0;
          M2Dir = 0;
          SysCmd = CmdDown;
          M1Cmd = CmdDown;
          SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
          M2Cmd = CmdDown;
          SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);            
          PID_Set(Speed,BASEDUTYDOWN);
          M1Flag = 0;
          M2Flag = 0;
          M1HallTemp = M1State.HallNow;
          M2HallTemp = M2State.HallNow;
          AntiCollisionState = DOWN;   
        }
        else if((SysCmd==CmdDown)|| (SysCmd == CmdDownRetard)||((M1Dir==DOWN)&&(M2Dir==DOWN)))//向下运动
        {
          M1Dir = 0;
          M2Dir = 0;
          SysCmd = CmdUp;
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1, 1350-BASEDUTYUP); //设置初始PWM占空比
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); //设置初始PWM占空比
          PID_Set(Speed,BASEDUTYUP);
          M1Flag = 0;
          M2Flag = 0;
          M1HallTemp = M1State.HallNow;
          M2HallTemp = M2State.HallNow;
          AntiCollisionState = UP;
        }
        BlockFlag = 0;
       }
      if(AntiCollisionState==DOWN)
      {
        if((M1HallTemp>(M1State.HallNow+700))||(M2HallTemp>(M2State.HallNow+700)))
        {
          KeyNullProcess();
        }
      }
      else if(AntiCollisionState==UP)
      {
        if((M1HallTemp+700<M1State.HallNow)||(M2HallTemp+700<M2State.HallNow))
        {
          KeyNullProcess();
        }
      }
      else if(AntiCollisionState==0)
      {
        M1HallTemp = 0;
        M2HallTemp = 0;
      }
   }
}

