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
* ��������: Motor_Init()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����������źͷ������������ų�ʼ��
************************************************************************************************************/
void Motor_Init(void)
{
  GPIO_Init(GPIOB, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);//Motor1��������
  GPIO_Init(GPIOE, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);                      //Motor2��������
  //GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);                    // ��������������
  GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);                      // 33v�����������
}


/***********************************************************************************************************
* ��������: M1Control()
* �������: cmd���������� //���M1����  (M1Cmd����)
* ����ֵ  : ��
* ��    ��: ���1���п���
************************************************************************************************************/
void M1Control(u8 cmd)
{
  if(SysState == NORMAL) //ϵͳ��ǰ������������״̬
  {
    switch(cmd)
    {
      case CmdUp:
        /*  //�ϳ���
        if(M1PID.CurrSpeed > 0)
        {
          if((M1State.LimitUp <= M1State.HallNow)) //�˶������ӵ��϶�����ֹͣ����
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop; //���1����ֹͣ״̬ 
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
            M1Cmd = CmdStop; //���1����ֹͣ״̬ 
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
        
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
        {
          if(M1PID.CurrSpeed > 0)
          {
            if((M1State.LimitUp <= M1State.HallNow)) //�˶������ӵ��϶�����ֹͣ����
            {
              M1Up(OFF);
              M1Down(OFF);
              M1_PWM_OFF;
              M1Cmd = CmdStop; //���1����ֹͣ״̬ 
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
              M1Cmd = CmdStop; //���1����ֹͣ״̬ 
            }
            else
            {
              M1Down(OFF);
              M1Up(ON);
            }
          } 
          //��Ҫ���е���ߴ�ʱ��������ٽ׶�
          if((M1State.LimitUp <= M1State.HallNow + 700) && (M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          } 
        }
        else    //�������н׶Σ���ʱ��Ҫ���ǵ�Ϊ������Ӧ���������������HALL��ƫ��ֵ
        {
                if(M1PID.CurrSpeed > 0)
                {
                    //if((M1State.LimitUp <= M1State.HallNow)) //�˶������ӵ��϶�����ֹͣ����
                    if((M1State.RelativeLimitUp <= M1State.HallNow)) //�˶�����Զ�����ֹͣ����
                    {
                        M1Up(OFF);
                        M1Down(OFF);
                        M1_PWM_OFF;
                        M1Cmd = CmdStop; //���1����ֹͣ״̬ 
                        
                        //SetM1BeforeRunState(3);
                        
                    }
                    else
                    {
                        M1Down(OFF);
                        M1Up(ON);         //���Ǽ򵥵ĶԻ���̵������в�����GPIO�ĺ궨�������
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
                        M1Cmd = CmdStop; //���1����ֹͣ״̬ 
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
        
        /*  //�ϳ���
        M1Up(OFF);
        M1Down(ON); 
        if(M1State.LimitDown >= M1State.HallNow)//�˶������ӵ��¶�����ֹͣ����
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Cmd = CmdStop;//���1����ֹͣ״̬
        }
        else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
        {
          M1PID.SetSpeed = MINSPEED;
        }
        break;
        */
        
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
        {
          M1Up(OFF);
          M1Down(ON); 
          if(M1State.LimitDown >= M1State.HallNow)//�˶������ӵ��¶�����ֹͣ����
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;//���1����ֹͣ״̬
          }
          else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          }
        }
        else            //�������н׶�
        {
          M1Up(OFF);
          M1Down(ON); 
          //if(M1State.LimitDown >= M1State.HallNow)//�˶������ӵ��¶�����ֹͣ����
          if(M1State.RelativeLimitDown >= M1State.HallNow)//�˶������ӵ��¶�����ֹͣ����
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M1Cmd = CmdStop;//���1����ֹͣ״̬
          }
          //else if(((M1State.LimitDown+700) >= M1State.HallNow)&&(M1PID.CurrSpeed>MINSPEED))
          else if(((M1State.RelativeLimitDown+700) >= M1State.HallNow) && (M1PID.CurrSpeed>MINSPEED))
          {
            M1PID.SetSpeed = MINSPEED;
          }
        }
        break;
        
        //����ƽ̨û�д˹���  
      case CmdToPreseting:      //���е��趨��λ��
        /*   //���е�ָ��λ�ò���ʱ������ƽ��ȼ�����
        if (GetM1BeforeRunState() == 1)
        {
            if(GetAccFullState() == 1)      //���ٶ�ֵ�ѻ��
            {
                if ((GetAcc_y() < 200) && (GetAcc_y() > -200))        //Ŀǰ���Ӵ���ƽ��״̬
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
                    M1Cmd = CmdStop;//���1����ֹͣ״̬           
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
            if((M1State.Record[Position] > (M1State.HallNow+10))&&(M1State.HallNow < M1State.LimitUp))//�趨λ���ڵ�ǰλ���Ϸ�����������
            {
                if(M1State.Record[Position] + M2State.Record[Position] > M1State.HallNow + M2State.HallNow + 20)
                {
                    M1Dir = UP;
                    DeleteSavedHeight();        //ɾ���洢�ĵ�ǰ�߶�����
                }
            }
            else if(((M1State.Record[Position]+10)< M1State.HallNow)&&(M1State.HallNow > M1State.LimitDown))//�趨λ���ڵ�ǰλ���·�����������
            {
                if(M1State.Record[Position]+M2State.Record[Position]+20<M1State.HallNow+M2State.HallNow)
                {
                    M1Dir = DOWN;
                    DeleteSavedHeight();        //ɾ���洢�ĵ�ǰ�߶�����
                }
            }
            if(M1Dir == UP)
            {
                if((M1State.Record[Position] - M1State.HallNow) < 500)//�쵽���趨λ��ʱ��������
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//����ָ��λ�ú�ֹͣ����
                {
                    M1Up(OFF);
                    M1Down(OFF);
                    M2Up(OFF);
                    M2Down(OFF);
                    M1_PWM_OFF;
                    M1Dir = STOP;
                    M1Cmd = CmdStop;//���1����ֹͣ״̬           
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
                if(M1State.HallNow - M1State.Record[Position] < 500)//�쵽���趨λ��ʱ��������
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.Record[Position]+M2State.Record[Position]) >= (M1State.HallNow+M2State.HallNow+5))//����ָ��λ�ú�ֹͣ����
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
        
         if((M1State.Record[Position] > (M1State.HallNow+10))&&(M1State.HallNow < M1State.LimitUp))//�趨λ���ڵ�ǰλ���Ϸ�����������
         {
             if(M1State.Record[Position] + M2State.Record[Position] > M1State.HallNow + M2State.HallNow + 20)
             {
                 M1Dir = UP;
                 DeleteSavedHeight();        //ɾ���洢�ĵ�ǰ�߶�����
              }
         }
         else if(((M1State.Record[Position]+10)< M1State.HallNow)&&(M1State.HallNow > M1State.LimitDown))//�趨λ���ڵ�ǰλ���·�����������
         {
             if(M1State.Record[Position]+M2State.Record[Position]+20<M1State.HallNow+M2State.HallNow)
             {
                 M1Dir = DOWN;
                 DeleteSavedHeight();        //ɾ���洢�ĵ�ǰ�߶�����
              }
          }
          if(M1Dir == UP)
          {
              if((M1State.Record[Position] - M1State.HallNow) < 500)//�쵽���趨λ��ʱ��������
              {
                  M1PID.SetSpeed = Speed/2;
               }
               if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//����ָ��λ�ú�ֹͣ����
               {
                  M1Up(OFF);
                  M1Down(OFF);
                  M2Up(OFF);
                  M2Down(OFF);
                  M1_PWM_OFF;
                  M1Dir = STOP;
                  M1Cmd = CmdStop;//���1����ֹͣ״̬           
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
                if(M1State.HallNow - M1State.Record[Position] < 500)//�쵽���趨λ��ʱ��������
                {
                    M1PID.SetSpeed = Speed/2;
                }
                if((M1State.Record[Position]+M2State.Record[Position]) >= (M1State.HallNow+M2State.HallNow+5))//����ָ��λ�ú�ֹͣ����
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
      
      
      case CmdGoBack:           //��⵽���غ����
        if(M1State.HallNow - M1State.Record[3] < 290)//�쵽���趨λ��ʱ��������
        {
          M1PID.SetSpeed = Speed/2;
        }
        /*
        if(M1State.Record[3]+M2State.Record[3]+15 >= M1State.HallNow+M2State.HallNow)//����ָ��λ�ú�ֹͣ����
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
        
        
      case CmdUpRetard: //����������״̬������ʱ����״̬
        
        //if (Balance_Data.BalanceAdjuseState == 0)
        //{
        //if((FinalPosition <= M1State.HallNow)) //���1��ʱ���ٵ���ָ��λ�ú���ֹͣ����
        /*      //�����ϳ���
          if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //�������ٶȼ����趨�ٶ�����
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
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
        {
          if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //�������ٶȼ����趨�ٶ�����
             (M1State.LimitUp <= M1State.HallNow)||
             (Balance_Data.RetardM1Stop == 1))
            {
               M1Up(OFF);
               M1Down(OFF);
               M1_PWM_OFF;
               M1Cmd = CmdStop;
               Balance_Data.RetardM1Stop = 0;
            }
            else        //û���趨Ŀ�꣬��������
            {
              M1Down(OFF);
              M1Up(ON);          
            }
        }
        else    //�������н׶�
        {
            /*if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //�������ٶȼ����趨�ٶ�����
             (M1State.LimitUp <= M1State.HallNow)||
             (RetardStop==1)��*/
             if((M1PID.CurrSpeed <= M1PID.SetSpeed)||        //�������ٶȼ����趨�ٶ�����
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
        
      case CmdDownRetard://����������״̬������ʱ����״̬
        //if(FinalPosition >= M1State.HallNow)//���1��ʱ���ٵ���ָ��λ�ú���ֹͣ����
        //RetardStopΪ������ٽ׶�800ms�� ����λ
        /*  �����ϳ���
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
        
        if (Balance_Data.BalanceAdjuseState != 0)       //ƽ������׶�
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
        else    //�������н׶�
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
        if(M2Cmd == CmdNull)    //�������ȫ��ֹͣ������
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
  else if(SysState == RESET) //ϵͳ��ǰ���ڸ�λ״̬
  {
    switch(cmd)
    {
      case CmdUp:
        M1Down(OFF);
        M1Up(ON);
        if(M1State.HallNow >= (BOTTOM+(u16)((MinColumnHeight-BASEHEIGHT)*RATE))) //���1����׶�λ����ֹͣ����
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M1Cmd = CmdUpHold;//������״̬ת����ͣ״̬
        }
        break;
      
      case CmdDown:  //�����˶�    
        M1Up(OFF);
        M1Down(ON);
        if(M1Detect>=2)//PWM���ڴﵽһ��ֵ,����ٶ���ȻС����ֵ������Ϊ�Ѿ�����ṹ�׶�
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
              M1Cmd = CmdDownHold; //���ֵ�ǰ״̬���ȴ����������λ���ṹ��׶�
            }
          }
          else
          {
            M1CurrBase = (u16)ADCValue.M1Current;
          }
          M1Detect = 0;
        }
        break;
        
      case CmdStop:  //ֹͣ�˶�    
        M1Up(OFF);
        M1Down(OFF); 
        M1_PWM_OFF;
        M1Detect = 0;
        M1PID_Set(MINSPEED,BASEDUTYDOWN);
        if(M2Cmd == CmdNull)//�������ȫ��ֹͣ������
        {
          SysCmd = CmdNull;
          M1Cmd = CmdNull;
        }
        break;
      
      case CmdDownHold: //����������ת��ֹͣ״̬���ȴ����������λ
        if(M2Cmd == CmdDownHold)//�����������λ���ṹ��׶���һ�������˶�
        {
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1,1350-BASEDUTYUP);
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2,1350-BASEDUTYUP);
          PID_Set(MINSPEED,BASEDUTYUP);
        }
        break; 
      //  
      case CmdUpHold://����������ת��ֹͣ״̬���ȴ����������λ
        if(M2Cmd == CmdUpHold)//�����������λ������ϵͳ��λ���
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
          
          
          //��λ��ɺ󣬰���������������е�������ֵ����Сֵ����λ
          /*******************************************************************/
            M1State.RelativeLimitUp = M1State.LimitUp;          
            M2State.RelativeLimitUp = M2State.LimitUp;
  
            M1State.RelativeLimitDown = M1State.LimitDown;
            M2State.RelativeLimitDown = M2State.LimitDown;
          
          /*******************************************************************/
            
          
          ErrCode = 0;
          memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));                          //�������ֵ
          
          memcpy(&Buffer[8],&M1State.LimitDown,sizeof(M1State.LimitDown));      //����M1State.LimitDownֵ
          memcpy(&Buffer[10],&M2State.LimitDown,sizeof(M2State.LimitDown));     //����M2State.LimitDownֵ
          //EEPROM_Write();
          
          DisplayMode = HeightMode;
          Release = 0;
          
          Balance_Data.TwoMotorOffsetHall = 0;          //��ƫ�������㣬��д��EEPROM��
          
          memcpy(&Buffer[62],& Balance_Data.TwoMotorOffsetHall,sizeof( Balance_Data.TwoMotorOffsetHall));   //Buffer[62]
          
          if(AccDataBag.Y_OffsetFlag == 0)      //δ��ȡ������ƫ��������һ���ϵ�󣬽��и�λ�����󣬰������ǵ�ƫ�������EEPROM�У�
          {
            AccDataBag.Y_Offset = (s16)(AccDataBag.ALLAccXl_y >> 3);
          
            memcpy(&Buffer[64],&AccDataBag.Y_Offset,sizeof(AccDataBag.Y_Offset));
            
            AccDataBag.Y_OffsetFlag = 1;
            
            memcpy(&Buffer[66],&AccDataBag.Y_OffsetFlag,sizeof(AccDataBag.Y_OffsetFlag));       //�ѻ�ȡ������ƫ�����ı�־λ����EEPROM��
          
            EnableHPSlopeFilter();
          }
          
          EEPROM_Write();
          
          SysState = NORMAL;            //ϵͳ������������״̬
        }
        break;
        
      case CmdNull:

        break;
    }
  }
}


/***********************************************************************************************************
* ��������: M2Control()
* �������: cmd����������
* ����ֵ  : ��
* ��    ��: ���2���п���
************************************************************************************************************/
void M2Control(u8 cmd)
{
  if(SysState == NORMAL) //ϵͳ��ǰ������������״̬
  {
    switch(cmd)
    {
      case CmdUp:
        /* �����ϳ���
        if(M2PID.CurrSpeed > 0)
        {
          if((M2State.LimitUp <= M2State.HallNow)) //�˶������ӵ��϶�����ֹͣ����
          {
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            M2Cmd = CmdStop; //���1����ֹͣ״̬
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
            M2Cmd = CmdStop; //���1����ֹͣ״̬
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
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
        {
          if(M2PID.CurrSpeed > 0)
          {
            if((M2State.LimitUp <= M2State.HallNow)) 
            {
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              M2Cmd = CmdStop; //���1����ֹͣ״̬
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
              M2Cmd = CmdStop; //���1����ֹͣ״̬
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
        else    //�����������н׶�
        {          
            //if (GetM2BeforeRunState() == 2)
            {          
                if(M2PID.CurrSpeed > 0)
                {
                    //if(M2State.LimitUp <= M2State.HallNow) //�˶������ӵ��϶�����ֹͣ����
                    if((M2State.RelativeLimitUp <= M2State.HallNow)&&(Adjust_State==0)) 
                    {
                        M2Up(OFF);
                        M2Down(OFF);
                        M2_PWM_OFF;
                        M2Cmd = CmdStop; //���1����ֹͣ״̬
                        
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
                        M2Cmd = CmdStop; //���1����ֹͣ״̬
                        
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
        /*      //�����ϳ���
        M2Up(OFF);
        M2Down(ON);
        if(M2State.LimitDown >= M2State.HallNow)//�˶������ӵ��¶�����ֹͣ����
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
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
        {
          M2Up(OFF);
          M2Down(ON);
          if(M2State.LimitDown >= M2State.HallNow)//�˶������ӵ��¶�����ֹͣ����
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
        else    //�������н׶�
        {
          M2Up(OFF);
          M2Down(ON);
          if((M2State.RelativeLimitDown >= M2State.HallNow)&&(Adjust_State==0))//�˶������ӵ��¶�����ֹͣ����
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
        
      //���е�ָ���߶�ָ�� 
      case CmdToPreseting: 
        //{
            /*
            if (GetM2BeforeRunState() == 2) 
            {
          
            if((M2State.Record[Position] > (M2State.HallNow + 10)) && (M2State.HallNow < M2State.LimitUp))//�趨λ���ڵ�ǰλ���Ϸ�
            {
                if((M1State.Record[Position] + M2State.Record[Position]) > (M1State.HallNow + M2State.HallNow + 20))
                {
                    M2Dir = UP;
                }
            }
            else if(((M2State.Record[Position] + 10) < M2State.HallNow) && (M2State.HallNow > M2State.LimitDown))//�趨λ���ڵ�ǰλ���·�
            { 
                if((M1State.Record[Position] + M2State.Record[Position] + 20) < (M1State.HallNow + M2State.HallNow))
                {
                    M2Dir = DOWN;
                }
            }
            if(M2Dir == UP)
            {
                if((M2State.Record[Position] - M2State.HallNow) < 500)//�쵽���趨λ��ʱ��������
                {
                    M2PID.SetSpeed = Speed/2;
                }
                if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//�����趨λ�ú�ֹͣ����
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
                if((M2State.HallNow - M2State.Record[Position]) < 500)//�쵽���趨λ��ʱ��������
                {
                    M2PID.SetSpeed = Speed/2;
                }
                if((M2State.Record[Position] + M1State.Record[Position]) >= (M2State.HallNow + M1State.HallNow + 5))//�����趨λ�ú�ֹͣ����
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
          
          
        if((M2State.Record[Position] > (M2State.HallNow + 10)) && (M2State.HallNow < M2State.LimitUp))//�趨λ���ڵ�ǰλ���Ϸ�
        {
             if((M1State.Record[Position] + M2State.Record[Position]) > (M1State.HallNow + M2State.HallNow + 20))
             {
                 M2Dir = UP;
              }
         }
         else if(((M2State.Record[Position] + 10) < M2State.HallNow) && (M2State.HallNow > M2State.LimitDown))//�趨λ���ڵ�ǰλ���·�
         { 
              if((M1State.Record[Position] + M2State.Record[Position] + 20) < (M1State.HallNow + M2State.HallNow))
              {
                  M2Dir = DOWN;
              }
          }
          if(M2Dir == UP)
          {
              if((M2State.Record[Position] - M2State.HallNow) < 500)//�쵽���趨λ��ʱ��������
              {
                  M2PID.SetSpeed = Speed/2;
              }
              if((M1State.HallNow + M2State.HallNow + 10) >= (M1State.Record[Position] + M2State.Record[Position]))//�����趨λ�ú�ֹͣ����
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
              if((M2State.HallNow - M2State.Record[Position]) < 500)//�쵽���趨λ��ʱ��������
              {
                    M2PID.SetSpeed = Speed/2;
              }
              if((M2State.Record[Position] + M1State.Record[Position]) >= (M2State.HallNow + M1State.HallNow + 5))//�����趨λ�ú�ֹͣ����
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
        
       //�޴˹���  
      case CmdGoBack:       //������˹���
        if(M2State.HallNow - M2State.Record[3] < 290)//�쵽���趨λ��ʱ��������
        {
          M2PID.SetSpeed = Speed/2;
        }
        /*
        if(M2State.Record[3]+M1State.Record[3]+15 >= M2State.HallNow+M1State.HallNow)//�����趨λ�ú�ֹͣ����
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
        
        
      case CmdUpRetard: //����������״̬������ʱ����״̬
        //if((FinalPosition <= M2State.HallNow)) //�����˶����趨λ��
        /* �����ϳ���
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
        
         if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
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
         else   //�������н׶�
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
        
        
      case CmdDownRetard://����������״̬������ʱ����״̬
        //if(FinalPosition >= M2State.HallNow)//�����˶����趨λ��
        /* //�����ϳ���
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
        
        if (Balance_Data.BalanceAdjuseState != 0)     //ƽ������׶�
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
        else    //�������н׶�
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
  else if(SysState == RESET) //ϵͳ��ǰ���ڸ�λ״̬
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
        
      case CmdDown: //��������
        M2Up(OFF);
        M2Down(ON);
        if(M2Detect>=2)
        {
          if((ADCValue.M2Current>(M2CurrBase+100))||(M2PID.PWMDuty > 1000))
          {
            if(M2PID.PWMDuty>550)//PWM���ڴﵽһ��ֵ,����ٶ���ȻС����ֵ����Ϊ�Ѿ�����ṹ�׶�
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
        
      case CmdStop: //ֹͣ�˶�
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
* ��������: MotorControl()
* �������: ��
* ����ֵ  : ��
* ��    ��: ������ƺ�������װ����������ƺ���
************************************************************************************************************/
void MotorControl(void)
{
  M1Control(M1Cmd);    //��ʼ��ʱ��M1Cmd��M2Cmd��Ϊ0
  M2Control(M2Cmd);
  
  if(SysCmd == CmdNull)   //��ʼ��ʱ��SysCmd = 0
  {
    CurrentOut(ON);     //ʹ�������33V  //��һ������GPIO�ܽŵĺ궨�����
    v33Flag = 0;        //v33Flag = 0��ʹ�����33V��v33Flag = 1����ʹ�����33V
  }
  else if((ADCValue.M1Current+ADCValue.M2Current)>700)  //250w
  {
    CurrentOut(OFF);    //��һ������GPIO�ܽŵĺ궨�����
    if((v33Flag == 0)&&(motorStartTimer > 500))             //6������500ms
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
* ��������: LimitPower()
* �������: ��
* ����ֵ  : ��
* ��    ��: �޶�ϵͳ�����
************************************************************************************************************/
void LimitPower(void)
{
  if((SysCmd == CmdStop)||(SysCmd == CmdNull))
  {
    T2sCnt1 = 0;
    LimitPowerFlag = 0;
    LimitPowerState = 0;
  }
  if(LimitPowerFlag == 1) //�������500ms��ͨ���жϵ����������͵����ǰ�ٶȣ�ʹ�������LimitPowerFlag==2������״̬
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
* ��������: AntiCollision()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����ײ����
************************************************************************************************************/
/***********************************************************************************************************
* ��������: AntiCollision()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����ײ����
************************************************************************************************************/
void AntiCollision(void)
{
  if(Sensitivity > 0)
  {
    if(LSM6DSLFlag == 1)          //�ж��Ƿ��а�װ6�ᴫ���� LSM6DSLFlag == 1��˵���Ѱ�װ
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
    //��ʼ���з���ײ����
    if(SysState == NORMAL)
    {  
      //����ײ�����һ�����������������
      
      //�Ե��1���з���ײ�����ȶԲɼ��õ��ĵ���ֵ�����˲�����
      if(M1ADCCnt == 40)  //�ж�AD�����ﵽ40��
      {
        M1Cur = MiddleAVG(&ADCBuffer1[0], 40);  //��AD���ݽ���ȡƽ�������Ƚ�������ȥ�����10������С10�����ݣ�Ȼ���ٶ��м�20�����ݽ���ȡƽ�����㣩
        //�Դ���õ���ADֵ����3�����汣�����
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
      //�Ե��2���з���ײ�����ȶԲɼ��õ��ĵ���ֵ�����˲�����
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
        //ͨ���жϵ��1�ĵ�ǰλ��ֵ���趨����߼����λ�ý��бȽ�
        if((M1State.HallNow + 750 < M1State.LimitUp) &&
           (M1State.HallNow > M1State.LimitDown))
        {
          //�����1�����������ϣ��������з���Ϊ������δ�����ײ����״̬
          if(((M1Cmd==CmdUp)||(M1Dir==UP))&&(AntiCollisionState==0))
          {
            if((M1CurTemp3+M1CurTemp1>2*M1CurTemp2+hd_anti_up)&&
               (M1CurTemp2>=M1CurTemp1)&&
               (M1CurTemp3>M1CurTemp2)&&
               ((M1PID.CurrSpeed>35)||(M1PID.SetSpeed<=35)))
            {
              M1Flag = 1;
              M1Up(OFF);                        //���Ϲر�
              M1Down(OFF);
              M1_PWM_OFF;
              M2Up(OFF);
              M2Down(OFF);
              M2_PWM_OFF;
              PID_Set(Speed,BASEDUTYUP);        //PID��ʼ��   
            }           
          }//�����1�����������ϣ��������з���Ϊ������δ�����ײ����״̬
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
        
       //����ײ����ڶ��������˫��������
        
        if((M1State.HallNow+750<M1State.LimitUp)&&
           (M1State.HallNow>M1State.LimitDown)&&
           (M2State.HallNow+750<M2State.LimitUp)&&
           (M2State.HallNow>M2State.LimitDown))
        {
          if((M1CurMax!=0)&&(M2CurMax!=0))      //�����������ڵ���������õ�
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
            //ϵͳ���������������ߵ��1�͵��2��������������δ�����ײ����״̬
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
          CurFlag = 1; //����������ʱ������������������еĵ�����׼ֵ���Ѿ���ȡʱ���������¼�ʱ
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
   AntiCollision();//������������ж�
   
   if(((M1Cmd!=CmdStop)&&(M1Cmd!=CmdNull))||((M2Cmd!=CmdStop)&&(M2Cmd!=CmdNull)))//�豸������
   {
      motorStopTimer = 0;                 //��ʱ��������0
   }
   else//ֹͣ
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
   //ϵͳ������������ģʽ
   if(SysState == NORMAL)  //ϵͳ״̬��Ϊ NORMAL �� RESET     
   {
      
      if (GetBalaceState() == 0)
      {
        if(((Sensitivity == 3)||(Sensitivity == 2))&&(LSM6DSLFlag == 1))
        {
            if(Sensitivity == 3)     //��ײ
            {
                add_value = 9;//Ӳ����ײ10kg 
            }
            else if(Sensitivity == 2)
            {
                add_value = 12;//Ӳ����ײ15kg 
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
        if((SysCmd==CmdUp)||((M1Dir==UP)&&(M2Dir==UP))) //�����˶�
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
        else if((SysCmd==CmdDown)|| (SysCmd == CmdDownRetard)||((M1Dir==DOWN)&&(M2Dir==DOWN)))//�����˶�
        {
          M1Dir = 0;
          M2Dir = 0;
          SysCmd = CmdUp;
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1, 1350-BASEDUTYUP); //���ó�ʼPWMռ�ձ�
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); //���ó�ʼPWMռ�ձ�
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

