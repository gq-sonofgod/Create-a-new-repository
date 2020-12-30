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
�������ƣ�GetBalaceState()  (ȫ�ֺ���)
�������ܣ���õ�ǰƽ�������״ֵ̬
���룺��
�����ƽ�������ǰ״ֵ̬
*******************************************************************************/
u8 GetBalaceState()
{
  return Balance_Data.BalanceAdjuseState;
}

/*******************************************************************************
�������ƣ�SetBalaceState()  (ȫ�ֺ���)
�������ܣ�����ƽ�������״̬
���룺�����õ�״ֵ̬
�������
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
    Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //�õ���������HALLƫ��ֵ
        
    memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]��Buffer[63]�洢���������Ӧ�������������HALL��ֵ
        
    memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //�洢M1 HALL��ǰֵ
    memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //�洢M2 HALL��ǰֵ
            
    EEPROM_Write();
        
    if (Balance_Data.TwoMotorOffsetHall >= 0)             //M1�����M2�����
    {
          M1State.RelativeLimitUp = M1State.LimitUp;
          M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
          M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
          M2State.RelativeLimitDown = M2State.LimitDown;
     }
     else  //Balance_Data.TwoMotorOffsetHall < 0           //M1�����M2�����
     {
          M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //����M1���λ���趨ֵ
          M1State.RelativeLimitDown = M1State.LimitDown;
            
          M2State.RelativeLimitUp = M2State.LimitUp;
          M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //����M2���λ���趨ֵ
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
            
     SaveIndex = 0;       //�洢λ�õı�־λ��0
            
     Buffer[32] = SaveIndex;
     
      Menu1Flag = 0;
      Menu2Flag = 0;
      //Menu2Num = 0;
     
  }
  else if (Balance_Data.BalanceAdjuseState != 0)    //�������ģʽ
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
          DeleteSavedHeight();  //SaveFlag = 0;ɾ�����������ǰλ��EEPROM�еĴ洢ֵ��M1State.HallNow��M2State.HallNow��                            
          
          Balance_Data.Acc_yTemp = AccDataBag.Acc_y;    //�ѳ���8�������        
          
          Balance_Data.HallCheckState = 1;              //��ʹ�������HALL��ֵ���
                   
          Balance_Data.MaxHallDif = (MAX_HALL_DIFF > DIF_HALL)? DIF_HALL:MAX_HALL_DIFF;
          
          
          //SysCmd = CmdSetBalance;
          //�������һ���ο����з��� 
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
          
          SetTIM1_PWMDuty(1, 1350 - 400);                  //���ó�ʼPWMռ�ձ� 
          SetTIM1_PWMDuty(2, 1350 - 400);                  //���ó�ʼPWMռ�ձ�
          
          PID_Set(8,BASEDUTYUP);                            //���ø���ת��Ϊ8
          //PID_Set(20,BASEDUTYUP);
          //RunCntFlag = 1;
          
          Balance_Data.DecideMotorRunTimerCnt = 0;          //����ȷ����������з����ʱ�������
          Balance_Data.DecideMotorRunState = 0;
          Balance_Data.BalanceAdjuseState = 2;              //����ƽ������������ʼ���У�         
          
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
      if(Balance_Data.BalanceAdjuseState == 2)  //����ȷ����������з���׶�
      {
        if ((Balance_Data.DecideMotorRunState == 0) && (Balance_Data.DecideMotorRunTimerCnt >= 1000))   //������1000ms�������ж����ڵĵ��������Ƿ���ȷ
        {
          Balance_Data.DecideMotorRunTimerCnt = 500;    //ÿ500ms�ٽ���һ�μ��
          //Balance_Data.DecideMotorRunState = 1;
          //if (AccDataBag.Acc_y <= 100)          //ֱ�ӵ�����ɣ�������ٽ׶μ���������
          //if (AccDataBag.Acc_y <= 200)          //ֱ�ӵ�����ɣ�������ٽ׶μ���������
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
            M1Cmd = CmdStop;                    //���1����ֹͣ״̬           
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
          else     //�жϵ�ǰ��������������Ƿ���ȷ
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
                  Balance_Data.TwoMotorRunFlag = 0;   //�ı�����״̬
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
                M1Cmd = CmdStop;                                    //���1����ֹͣ״̬           
                M2_PWM_OFF;
                M2Dir = STOP;
                M2Cmd = CmdStop;
            
                Balance_Data.TwoMotorRunFaultFlag = 1;
                    
                Balance_Data.BalanceAdjuseState = 4;
                Balance_Data.MotorSlideTimerCnt = 0;  
                Time3=0;    
                 }//��������з������
                  
               
              }
              else if (AccDataBag.Acc_y < Balance_Data.Acc_yTemp)       //���������ȷ
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
                 
                    
                 //AnglePID.Ref_Para = (AccDataBag.Acc_y)/2;      //���òο��Ƕȣ�����PID����
                 
                 AnglePID.LastDiffValur = 0;
                 AnglePID.PrevDiffValur = 0;               
                 
                 M2PID.Kp = KP2;
                 M2PID.Ki = KI2;
                 M2PID.Kd = KD2;
                    
                 M1PID.Kp = KP2;
                 M1PID.Ki = KI2;
                 M1PID.Kd = KD2;
                 
                 
                 AnglePID.OutPut = 8;

                 Balance_Data.BalanceAdjuseState = 3;   //���ó�ֱ������״̬
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
                
              if ((AccDataBag.Acc_y > 0)||(AccDataBag.Acc_y < Balance_Data.Acc_yTemp)) //�������򲻶�
              {
                  Time5=0;
                  Time4++;
                  if(Time4>5)
                  {
                  if (Balance_Data.TwoMotorRunFlag == 1)
                   {
                    Balance_Data.TwoMotorRunFlag = 0;                   //�ı�����״̬
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
                 M1Cmd = CmdStop;                                       //���1����ֹͣ״̬           
                 M2_PWM_OFF;
                 M2Dir = STOP;
                 M2Cmd = CmdStop;
            
                 Balance_Data.TwoMotorRunFaultFlag = 1;
                 Balance_Data.BalanceAdjuseState = 4;
                 Balance_Data.MotorSlideTimerCnt = 0;
                  
                  }
                      
                 if (Balance_Data.TwoMotorRunFlag == 1)
                 {
                    Balance_Data.TwoMotorRunFlag = 0;                   //�ı�����״̬
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
                 M1Cmd = CmdStop;                                       //���1����ֹͣ״̬           
                 M2_PWM_OFF;
                 M2Dir = STOP;
                 M2Cmd = CmdStop;
            
                 Balance_Data.TwoMotorRunFaultFlag = 1;
                 Balance_Data.BalanceAdjuseState = 4;
                 Balance_Data.MotorSlideTimerCnt = 0;
              }
              else if (AccDataBag.Acc_y > Balance_Data.Acc_yTemp)       //����������ȷ����������ͬ�������
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
                 Ref_Para_MiniSpeed=1;    //�趨�ɼ���ʱ��̣�����ʱ�䳤
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
      if (Balance_Data.BalanceAdjuseState == 3)         //�������������׶�
      {
        //if(AccDataBag.Acc_y <= 300)     //���������ٽ׶� 
        //if(AccDataBag.Acc_y <= 100)      //���������ٽ׶�
        //if(AccDataBag.Acc_y <= 200)      //���������ٽ׶�
        //if ((AccDataBag.Acc_y <= 100)&&(AccDataBag.Acc_y >= -100))      //һֱ���У�ֱ���Ƕȵ�����һ��λ��
            
        if ((AccDataBag.Acc_y <= 100)&&(AccDataBag.Acc_y >= -100))      //һֱ���У�ֱ���Ƕȵ�����һ��λ��
        {
          M1Up(OFF);
          M1Down(OFF);
          M2Up(OFF);
          M2Down(OFF);
          M1_PWM_OFF;
          M1Dir = STOP;
          M1Cmd = CmdStop;    //�������ֹͣ״̬           
          M2_PWM_OFF;
          M2Dir = STOP;
          M2Cmd = CmdStop;
            
          Balance_Data.BalanceAdjuseState = 4;
          Balance_Data.MotorSlideTimerCnt = 0;
        } //�쵽���޸߶�ʱ��
        /*
        else if ((M1State.HallNow > (M1State.LimitUp - 300)) || (M2State.HallNow > (M1State.LimitUp - 300)))
        {
           M1Up(OFF);
           M1Down(OFF);
           M2Up(OFF);
           M2Down(OFF);
           M1_PWM_OFF;
           M1Dir = STOP;
           M1Cmd = CmdStop;    //�������ֹͣ״̬           
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
          
          //if (hall_diff > 6515)     //���40cm�߶Ȳ�
            if (hall_diff > Balance_Data.MaxHallDif) 
          {
                M1Up(OFF);
                M1Down(OFF);
                M2Up(OFF);
                M2Down(OFF);
                M1_PWM_OFF;
                M1Dir = STOP;
                M1Cmd = CmdStop;    //�������ֹͣ״̬           
                M2_PWM_OFF;
                M2Dir = STOP;
                M2Cmd = CmdStop;
            
                Balance_Data.BalanceAdjuseState = 4;
                Balance_Data.MotorSlideTimerCnt = 0;
          }
        }
        
      }  
      if (Balance_Data.BalanceAdjuseState == 4)                 //�رյ�Դ��������뻬�н׶�
      {
        if ((KEY_Stop_M_Flag==1)||((M2Cmd == CmdNull) && (M1Cmd == CmdNull)))
        {
          Balance_Data.BalanceAdjuseState = 5;
          Balance_Data.MotorSlideTimerCnt = 0; 
        }
      }
      
      if(Balance_Data.BalanceAdjuseState == 5)                  //������Ի��н׶�
      {
        if (Balance_Data.MotorSlideTimerCnt >= 200)             //�ﵽ�涨����ʱ��
        {   
          Balance_Data.BalanceAdjuseState = 6;                  //���������ε���ƽ�������е�HALL���׶�
        }   
      }
      
      if (Balance_Data.BalanceAdjuseState == 6)
      {
         if ((Balance_Data.TwoMotorRunFaultFlag == 1)&&(KEY_Stop_M_Flag==0))            //����ƽ�����Ʒ��ˣ�����¿�ʼ���е���
         {
            Balance_Data.BalanceAdjuseState = 1;            
            Balance_Data.TwoMotorRunFaultFlag = 0;
           
            //Release = 1;
         }
         else
         { 
            
            KEY_Stop_M_Flag=0;
            Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //�õ���������HALLƫ��ֵ
        
            memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]��Buffer[63]�洢���������Ӧ�������������HALL��ֵ
        
            memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //�洢M1 HALL��ǰֵ
            memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //�洢M2 HALL��ǰֵ
            
            EEPROM_Write();
        
            if (Balance_Data.TwoMotorOffsetHall >= 0)             //M1�����M2�����
            {
              M1State.RelativeLimitUp = M1State.LimitUp;
              M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
              M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
              M2State.RelativeLimitDown = M2State.LimitDown;
            }
            else  //Balance_Data.TwoMotorOffsetHall < 0           //M1�����M2�����
            {
              M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //����M1���λ���趨ֵ
              M1State.RelativeLimitDown = M1State.LimitDown;
            
              M2State.RelativeLimitUp = M2State.LimitUp;
              M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //����M2���λ���趨ֵ
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
            
            SaveIndex = 0;       //�洢λ�õı�־λ��0
            
            Buffer[32] = SaveIndex;
            DisplayMode = HeightMode;
            //MenuTime = 5000;        //z
        }
      }
    }
    MenuTime = 4800;    //����һֱ��ʾ  SET
    LEDRest = 0;        //�п���������ƽ�����ʱ�������������ʱ����ʾʱ���趨ֵ��10s��
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
              
  AnglePID.Ref_Para = (AccDataBag.Acc_y*3)/4;    //�趨�ɼ���ʱ��̣�����ʱ�䳤
                    
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
