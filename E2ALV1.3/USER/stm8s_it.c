#include "stm8s_it.h"
#include "ADC.h"
#include "PID.h"
#include "Timer.h"
#include "Motor.h"
#include "Main.h"
#include "LED.h"
#include "Key.h"
#include "EEPROM.h"
#include "FaultDetection.h"
#include "HealthMode.h"
#include "Uart.h"
#include "string.h"
#include "Balance.h"
#include "Tap.h"

//#define BALANCE_MAXSPD  35
//#define BALANCE_MINSPD  6

extern u16 UartSendTimerCnt;

u16 DelayCntForTap = 0;

u16 TestMotorHallTimerCnt = 0;;

u16 ResetCnt = 0;

u8 T100msCnt = 0;
u8 T100msCnt1 = 0;      //���1�ٶȼ����ʱֵ
u8 T100msCnt2 = 0;      //���2�ٶȼ����ʱֵ
u16 T500msCnt = 0;
u16 T500msCnt1 = 0;
u16 T500msCnt2 = 0;
u16 T500msCnt3 = 0;
u16 M1T3sCnt = 0;
u16 M2T3sCnt = 0;
u8 TimeFlag = 0;
u16 T2sCnt = 0, T5secCnt = 0;
u32 T45MinuteCnt = 0, T1MinuteCnt = 0;
u8  T45MinuteFlag = 0;
u32  T5MinuteCnt = 0;
u16 T1sCnt = 0;
u8 T1sFlag = 0, T1secFlag = 0, T5secFlag = 0;
u16 T30sCnt = 0;
u8 BlinkFlag = 0;

u8 RetardStop = 0;
u16 RetardStopCnt = 0;

u16 MenuTime = 0;
u16 UpKeyDelay = 0;
u16 DownKeyDelay = 0;
/*
u8 RetardM1Stop = 0;
u16 RetardM1StopCnt = 0;

u8 RetardM2Stop = 0;
u16 RetardM2StopCnt = 0;
*/
u32 MotorRunTime = 0;
u32 MotorRunTotal = 0;
u32 MotorStopTime = 0;
u32 RunTimeTemp = 0;

u16 RSTCnt = 0;
u16 InitCnt = 0;

u8 T10msCnt = 0;
u8 T10msFlag = 0;
u16 T400msCnt = 0;
s16 TestValue = 0;
u16 BuzzerTestCnt = 0;
u16 AgingTurnTime = 0;
u8  AgingTurnFlag = 0;
u32 AgingTime = 0;
u32 AgingAllTime = 0;
u16 T2SecondCnt = 0;
u16 T100msCnt3 = 0;
u16 T100msCnt4 = 0;
u16 T100msCnt5 = 0;
u16 M1ErrCnt = 0;
u16 M2ErrCnt = 0;
u32 T18minCnt = 0;
u8 FallDetectFlag = 0;
u16 FallDetectBase1 =0;
u16 FallDetectBase2 =0;
u16 T2sCnt1 = 0;
u16 RunCnt = 0;
u16 Adc_Time=0;
u8 ADC_Scan_Flag=0;
extern u16 Com1LongCnt;
extern u16 Com3LongCnt;
extern u16 KeyPressDownCnt;
extern u16 motorStartTimer;
extern u16 motorStopTimer;
extern u16 lsm6dslTimer;
extern u16 sensTimer;



//extern u8 Menu1Flag;//һ���˵���־λ
//extern u8 Menu2Flag;//�����˵���־λ
//extern u8 Menu2Num;

//TIM2�ж� 1MS�ж�����
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);//�Ը���ADCͨ����������
        
  //Adc_Time++;
  if(Adc_Time == 0)
  {
   Adc_Time=1;
   ADC_Scan_Flag=1;           
  
  }
  
  if(Key==KEY_VER)
  {
  VER_Cnt++;
  
  }
  else VER_Cnt=0;
  
  if(DisplayMode==FLash_HeightMode)
  {
      Flash_Cnt++;
      if(Flash_Cnt>1000)
      {
      Flash_Cnt=0;
      
      }
  }
  else  Flash_Cnt=0;
 
 if(Key==KEY_UP) 
 {
 KeyUpCnt++;
 if(KeyUpCnt>200)
 {
 KeyUpCnt=201;
 
 }
 
 }
 else KeyUpCnt=0;
 
 if(Key==KEY_DOWN) 
  {
    KeyDownCnt++;
    if(KeyDownCnt>200)
   {
    KeyDownCnt=201;
 
    }
 
 }
 else KeyDownCnt=0;   

     
  
     
  
  UartSendTimerCnt++;
  
  if(Com1LongCnt < 0xff00) 
    Com1LongCnt++;
  if(Com3LongCnt < 0xff00) 
    Com3LongCnt++;
  if(KeyPressDownCnt < 0xff00) 
    KeyPressDownCnt++;
  if(motorStartTimer < 0xff00) 
    motorStartTimer++;
  if(motorStopTimer < 0xff00) 
    motorStopTimer++;
  if(lsm6dslTimer < 0xff00) 
    lsm6dslTimer++;
  if(sensTimer < 0xff00) 
    sensTimer++;
  
  if(MenuTime < 0xff00) 
    MenuTime++;
  
    UpKeyDelay++;
    DownKeyDelay++;
    ResetCnt++;
  
  
  if(++T10msCnt >= 10)
  {
    T10msCnt = 0;
    
    T10msFlag = 1;
  }
  if(++T100msCnt >= 30)
  {
    T100msCnt = 0; 
    DispFlag = 1;
  }
  
  if((Menu1Flag == 1)||(Menu1Flag == 2)||(Menu1Flag == 3)||(Menu1Flag == 4))
  {
    if (MenuTime >= 5000)//5���ް��� �������˵�
    {
      if(Adjust_State==0)MenuTime = 0;
      Menu1Flag = 0;
      Menu2Flag = 0;
      Menu2Num = 0;
      Menu3Num = 0;
      Rst_EN=0;
      //Check_EN=0;
      Balance_EN=0;
      DisplayMode = HeightMode;
      if((Check_EN==1)||(Check_EN==2))
      {
      DisplayRemind=ON; 
      HealthModeReset() ; 
      Check_EN=0;
      }
    }
  }
  
    Tap_Parameter.CheckTriggerTimerCnt++;
  Tap_Parameter.TapControlTimerCnt++;

  /*
  if ((Tap_Parameter.TapControlTimerCnt > 10000) && (Tap_Parameter.TapControlEN == 1))
  {
    Tap_Parameter.TapControlEN = 0;
    Tap_Parameter.TapControlFlag = 0;
    
    lsm6dsl_tap_mode_set(&dev_ctx,0x01);    //ֻ��ⵥ������
  }
  */
  
  Tap_Parameter.TapBuzzerTimerCnt++;
  /*
  if ((Tap_Parameter.TapControlEN == 1) && (Tap_Parameter.TapBuzzerTimerCnt >= 500) && (Tap_Parameter.TapBuzzerState == ON))  //��������500ms
  //if ((Tap_Parameter.TapControlEN == 1) && (Tap_Parameter.TapBuzzerTimerCnt >= 1000) && (Tap_Parameter.TapBuzzerState == ON))
  {
    Tap_Parameter.TapBuzzerState = OFF;
    BuzzerState = Tap_Parameter.TapBuzzerState;
  }
  */
  
  Tap_Parameter.TapCheckDelayTimerCnt++;
  
  /*
  if ((Tap_Parameter.TapCheckDelayTimerCnt > 1000) && (Tap_Parameter.TapCheckDelayFlag == 1))
  {
    Tap_Parameter.TapCheckDelayFlag = 0;
  }
  */
  
  Tap_Parameter.TapCheckOrderTimerCnt++;
  
  DelayCntForTap++;
  
  if ((DelayFlagForTap == 1) && (DelayCntForTap > 200))
  {
    DelayFlagForTap = 0;
  }
  
  
  
  if(RunCntFlag == 2)
  {
    //if(++RunCnt >= 100)
    if(++RunCnt >= 100)
    {
      RunCnt = 0;
      RunCntFlag = 0;
    }
  }
  if(CurFlag == 1)
  {
    if(++M1T3sCnt >= 750)
    {
      M1T3sCnt = 0;
      M1CurFlag = 1;
    }
    if(++M2T3sCnt >= 750)
    {
      M2T3sCnt = 0;
      M2CurFlag = 1;
    }
  }
  if((SysState==NORMAL)&&(LimitPowerFlag==0))    
  {
    //�����ʼ���к��Ƚ���500ms��ʱ��LimitPowerFlag��־λ���1
    if((SysCmd==CmdUp)||(SysCmd==CmdDown)||((M1Dir==UP)&&(M2Dir==UP))||((M1Dir==DOWN)&&(M2Dir==DOWN))) //���������2S��ʱ
    { 
      if(++T2sCnt1 >= 500)
      {
        T2sCnt1 = 0;
        LimitPowerFlag = 1;
      }
    }
  }
  else if((SysCmd==CmdStop)||(SysCmd==CmdNull))
  {
    T2sCnt1 = 0;
    LimitPowerFlag = 0;
  }
  if(LimitPowerFlag == 2)
  {
    if(T500msCnt < 600)
      T500msCnt++;
  }
  if(((Com3UartState == WAIT)||(Com3UartState == RECEIVEBUSY))&&(Com3DeviceState!=SLAVER))
  {
    if(++T100msCnt3>=200)
    {
      T100msCnt3 = 0;
      Com3UartState = IDLE;
      Com3ReceiveCount = 0;
      Com3KeyValue = 0;
      memset(&Com3ReceiveBuffer[0], 0, sizeof(Com3ReceiveBuffer));
    }
  } 
  
  if(((Com1UartState == WAIT)||(Com1UartState == RECEIVEBUSY))&&(Com1DeviceState!=SLAVER))
  {
    if(++T100msCnt5>=200)
    {
      T100msCnt5 = 0;
      Com1UartState = IDLE;
      Com1ReceiveCount = 0;
      Com1KeyValue = 0;
      memset(&Com1ReceiveBuffer[0], 0, sizeof(Com1ReceiveBuffer));
    }
  } 
  if(RSTFlag == 1)
  {
    RSTCnt++;
    if(RSTCnt>= 10000)
    {
      RSTCnt = 0;
      RSTFlag = 2;
      SysState = RESET;
      ErrCode = Err_RESET;
      DisplayMode = ErrorMode;
      T500msCnt3 = 0;
    }
    //if((M1State.HallNow>M1State.LimitDown)||(M2State.HallNow>M2State.LimitDown))
    //{
      //RSTFlag = 0;
    //}
      //Balance_Data.TwoMotorOffsetHall >= 0      //M1�����M2�����
      if (Balance_Data.TwoMotorOffsetHall >= 0)
      {
          if (M2State.HallNow > M2State.LimitDown)
          {
            RSTFlag = 0;
          }
          //top = ((float)(M1State.HallNow) + (float)(M2State.HallNow) + Balance_Data.TwoMotorOffsetHall)/2;
          
      }
      else
      {
          if (M1State.HallNow > M1State.LimitDown)
          {
            RSTFlag = 0;
          }
      }
  }
  
  if(InvalidFlag == 2)
  {
    if(++InvalidCnt>=500)       //��Ч����ʱʱ��500ms
    {
      InvalidCnt = 0;
      InvalidFlag = 0;
    }
  }
  if(KeyAValid == 2)
  {
    if(++T500msCnt1>=500)
    {
      T500msCnt1 = 0;
      KeyAValid = 0;
    }
  }
  if(KeyMValid == 2)
  {
    if(++T500msCnt2>=500)
    {
      T500msCnt2 = 0;
      KeyMValid = 0;
    }
  }
  if(SysCmd == CmdNull)
  {      
    if(SaveFlag == 0)
    {
      T400msCnt++;
      if(T400msCnt >= 400)
      {
        T400msCnt = 0;
        SaveFlag = 1;
      }
    }
  }
  
  
  if(InitFlag == 1)
  {
    if(++InitCnt >= 120)
    //if(++InitCnt >= 300)
    {
      InitFlag = 2;
      InitCnt = 0;
      BuzzerState = OFF;
    }
  }
  else if(InitFlag == 2)
  {
    if(++InitCnt >= 300)
    {
      InitFlag = 3;
      InitCnt = 0;
      BuzzerState = ON;
    }
  }
  else if(InitFlag == 3)
  {
    if(++InitCnt>=120)
    {
      InitFlag = 4;
      InitCnt = 0;
      BuzzerState = OFF;
     }
  }
else if(InitFlag == 5)
  {
    if(++InitCnt>=50)
    {
      InitCnt = 0;
      InitFlag = 0;
      ((void (*)(void))0x8000)();
    }
  }
  /*
  else if(InitFlag == 3)
  {
    if(++InitCnt>=120)
    //if(++InitCnt >= 300)
    {
      InitFlag = 4;
      InitCnt = 0;
      BuzzerState = OFF;
   // }
  //}
  //else if(InitFlag == 5)
  //{
    //if(++InitCnt >= 50)
    {  //���븴λ������RST��
      InitCnt = 0;
      InitFlag = 0;
      
      memset(&Buffer[0], 0, sizeof(Buffer));
      //��λ֮ǰ�������ݱ������
      Buffer[33] = INITIALIZED;
      Buffer[35] = RELEASE;
      Buffer[36] = Unit;
      Buffer[34] = TIMEVAL;
      memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));
      memcpy(&Buffer[41],&DiffHall, 2);
      memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));
      Buffer[47] = Speed;
      Buffer[48] = Sensitivity;
      memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));
      memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));
      
      Buffer[61] = Balance_Data.TwoMotorRunFlag;
      
      memcpy(&Buffer[64],&AccDataBag.Y_Offset,sizeof(AccDataBag.Y_Offset));
      memcpy(&Buffer[66],&AccDataBag.Y_OffsetFlag,sizeof(AccDataBag.Y_OffsetFlag));
        
      EEPROM_Write();

      ((void (*)(void))0x8000)();       //���������λ��ʵ��RST����
      }
    }
  }*/
  
  if((SysState == RESET)&&(CombinationKey==1))
  {
    if(++T2SecondCnt >= 3000)
    {
      CombinationKey = 0;
      T2SecondCnt = 0;
      DisplayMode = ErrorMode;        
    }
  }
  if(CombinationKey == 2)
  {
    if(++T2SecondCnt>=3000)
    {
      CombinationKey = 3;
      T2SecondCnt = 0;      
    }
  }
  else if(CombinationKey == 4)
  {
    if(++T2SecondCnt>=3000)
    {
      CombinationKey = 5;
      T2SecondCnt = 0;
    }
  }
  else if(CombinationKey == 6)
  {
    if(++T2SecondCnt>=3000)
    {
      CombinationKey = 7;
      T2SecondCnt = 0;
      if(SysState==NORMAL)
        DisplayMode = HeightMode;
      else if(SysState==RESET)
        DisplayMode = ErrorMode;
    }
  }
  HealthModeTimeProc();     //ÿ1msִ��һ��
  
  if((SysCmd == CmdDownRetard)||(SysCmd == CmdUpRetard))
  {
    if(++RetardStopCnt>800)
    {
      RetardStopCnt = 0;
      RetardStop = 1;
    }
  }
  
  if(M1ErrFlag==1)
  {
    if(++M1ErrCnt>=200)
    {
      M1ErrCnt = 0;
      M1ErrFlag = 2;
    }
  }
  if(M2ErrFlag==1)
  {
    if(++M2ErrCnt>=200)
    {
      M2ErrCnt = 0;
      M2ErrFlag = 2;
    }
  }
  
  TestMotorHallTimerCnt++;

  if (Balance_Data.BalanceAdjuseState == 2)
  {
    if((M1Cmd != CmdNull) && (M1Cmd != CmdStop))//���1��������״̬�����PID����
    { 
        if(++T100msCnt1 >= 100)
        { 
            T100msCnt1 = 0;     
            M1PID.CurrSpeed = ABS(M1State.HallNow,M1State.HallLast);//������1��ǰ�ٶ�  
            M2PID.CurrSpeed = ABS(M2State.HallNow,M2State.HallLast);//������2��ǰ�ٶ� 
            
            M1PID.SetSpeed = 8;
            M2PID.SetSpeed = 8;
     
            if ((M2Cmd != CmdNull) && (M2Cmd != CmdStop))
            {
                PIDCal(&M2PID);

                SetTIM1_PWMDuty(2,1350 - M2PID.PWMDuty);
            }
            
            if ((M1Cmd != CmdNull) && (M1Cmd != CmdStop))
            {
                PIDCal(&M1PID);

                SetTIM1_PWMDuty(1,1350 - M1PID.PWMDuty);
            }
                
            M1State.HallLast = M1State.HallNow;       //����һ��M1��ǰ��HALLֵ
            M2State.HallLast = M2State.HallNow;       //����һ��M1��ǰ��HALLֵ 
        }          
     } 
  }
  else if ((Balance_Data.BalanceAdjuseState == 3)||(Adjust_State==2))     //�ж�����ģʽ
  {
    if (TestMotorHallTimerCnt >= 100)       //ÿ20ms����һ�νǶ�PID��PID�����Ϊ���ת�ٲο�ֵ
    {
      TestMotorHallTimerCnt = 0; 
    
      //AnglePID.Fact_Para = AccDataBag.Acc_y;
      //AnglePIDCal(&AnglePID);
     //�����ٶ�ֵ��С������30������û��ʹ��PID���Ƹ���ת�٣�����ֱ�Ӱ��ٶȸ���ֵ�����󣬺���С��
      if (Balance_Data.Acc_yTemp > 0)
      {
        if ((AccDataBag.Acc_y >= AnglePID.Ref_Para)&&(Ref_Para_MiniSpeed==0)) 
        {
            //AnglePID.OutPut = 30;// BALANCE_MAXSPD;
            
            AnglePID.OutPut = 6;// BALANCE_MAXSPD;
        }
        else 
        {
            AnglePID.OutPut =  2;//BALANCE_MINSPD;
        }
      }
      else 
      {
        if ((AccDataBag.Acc_y <=AnglePID.Ref_Para)&&(Ref_Para_MiniSpeed==0))
        {
            //AnglePID.OutPut =  30;//BALANCE_MAXSPD;         
            AnglePID.OutPut = 6;// BALANCE_MAXSPD;
        }
        else 
        {
            AnglePID.OutPut = 2;//BALANCE_MINSPD;
        }
      }
      
    }
    //if (++Balance_Data.AnglePidTimerCnt >= 100)//;
    //if (++Balance_Data.AnglePidTimerCnt >= 50)//;
    if (++Balance_Data.AnglePidTimerCnt >= 20)
    {
      Balance_Data.AnglePidTimerCnt = 0;
      
      M1PID.SetSpeed = AnglePID.OutPut;
      M2PID.SetSpeed = AnglePID.OutPut;
      
      M1PID.CurrSpeed = ABS(M1State.HallNow,M1State.HallLast);//������1��ǰ�ٶ� 
      
      M2PID.CurrSpeed = ABS(M2State.HallNow,M2State.HallLast);//������2��ǰ�ٶ�       
      
      if ((M2Cmd != CmdNull) && (M2Cmd != CmdStop))
      {
          PIDCal(&M2PID);

          SetTIM1_PWMDuty(2,1350 - M2PID.PWMDuty);
      }
            
       if ((M1Cmd != CmdNull) && (M1Cmd != CmdStop))
       {
           PIDCal(&M1PID);

           SetTIM1_PWMDuty(1,1350 - M1PID.PWMDuty);
       }
      
      
      
      /*
      PIDCal(&M1PID);

      SetTIM1_PWMDuty(1,1350 - M1PID.PWMDuty);
      
      PIDCal(&M2PID);

      SetTIM1_PWMDuty(2,1350 - M2PID.PWMDuty);
      */
      M1State.HallLast = M1State.HallNow;       //����һ��M1��ǰ��HALLֵ
      M2State.HallLast = M2State.HallNow;       //����һ��M1��ǰ��HALLֵ     
    }
  }
  else if ((Balance_Data.BalanceAdjuseState == 0)&&(Adjust_State==0))
  {
  
    if((M1Cmd != CmdNull)&&(M1Cmd != CmdStop))//���1��������״̬�����PID����
    { 
        if(++T100msCnt1 >= 100)
        { 
            T100msCnt1 = 0;     
            M1PID.CurrSpeed = ABS(M1State.HallNow,M1State.HallLast);//������1��ǰ�ٶ�   
     
          if ((Balance_Data.TwoMotorOffsetHall != 0) && (SysState == RESET))
          {
            Balance_Data.TwoMotorOffsetHall = 0;
          }
              
          if((M1Cmd == CmdUp)||(M1Dir == UP))           //���1������������״̬
          {
            /*
            if(M2State.HallNow <= M1State.HallNow)    //M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ�ã�M1���ٿ죩       
              M1PID.SyncDev = (s8)((M1State.HallNow- M2State.HallNow));            //����M1ͬ��ƫ��ֵ      
            */

            if (M2State.HallNow <= (M1State.HallNow - Balance_Data.TwoMotorOffsetHall))
              M1PID.SyncDev = (s8)((M1State.HallNow - Balance_Data.TwoMotorOffsetHall - M2State.HallNow));            //����M1ͬ��ƫ��ֵ   
          }
          else if(M1Cmd == CmdUpRetard)     //M1�������������״̬������ʱ����״̬
          {
            /*
            if((M2State.HallNow <= M1State.HallNow)) //M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ�ã�M1���ٿ죩       
              M1PID.SyncDev = (s8)((M1State.HallNow- M2State.HallNow)); //����M1ͬ��ƫ��ֵ
            */
            if (M2State.HallNow <= (M1State.HallNow - Balance_Data.TwoMotorOffsetHall))
              M1PID.SyncDev = (s8)((M1State.HallNow - Balance_Data.TwoMotorOffsetHall - M2State.HallNow));            //����M1ͬ��ƫ��ֵ   
          }
          else if((M1Cmd == CmdDown)||(M1Dir == DOWN)) //���1������������״̬
          {
            /*
            if(M1State.HallNow >= M2State.HallNow) //M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ�ã�M1��������  
            {
              M1PID.SyncDev = (s8)((M1State.HallNow- M2State.HallNow)); //����M1ͬ��ƫ��ֵ 
              M1PID.SyncDev = 0 - M1PID.SyncDev;          
            }*/
            if ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) >= M2State.HallNow)         //��������������У���M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ��
            {
              M1PID.SyncDev = (s8)(M1State.HallNow - Balance_Data.TwoMotorOffsetHall - M2State.HallNow); //����M1ͬ��ƫ��ֵ 
              M1PID.SyncDev = 0 - M1PID.SyncDev;  
            }
          }
          else if(M1Cmd == CmdDownRetard)           //M1�������������״̬������ʱ����״̬
          {
            /*
            if((M2State.HallNow >= M1State.HallNow)) //M2�ĵ�ǰλ�ø���M1�ĵ�ǰλ�ã�M1��������       
              M1PID.SyncDev = (s8)((M2State.HallNow- M1State.HallNow)); //����M1ͬ��ƫ��ֵ  
            else 
            {
              M1PID.SyncDev = (s8)((M1State.HallNow- M2State.HallNow));
              M1PID.SyncDev = 0 - M1PID.SyncDev;
            }
            */
            if ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall)>= M2State.HallNow )         //ϵͳ�����ڵ�����£��ҽ�����ʱ���ٽ׶Σ�M2�ĵ�ǰλ�õ���M1�ĵ�ǰλ��
            {
              M1PID.SyncDev = (s8)(M2State.HallNow - (M1State.HallNow - Balance_Data.TwoMotorOffsetHall));                 //M1���Ҫ�ӿ������ٶ�  
            }
            else
            {
              M1PID.SyncDev = (s8)((M1State.HallNow - Balance_Data.TwoMotorOffsetHall)- M2State.HallNow); 
              M1PID.SyncDev = 0 - M1PID.SyncDev;
            }
          }
      
      PIDCal(&M1PID);

      SetTIM1_PWMDuty(1,1350 - M1PID.PWMDuty);
      if(SysState == RESET)
      {       
        M1Detect++;  
      }
      M1State.HallLast = M1State.HallNow; //ÿ100ms����һ��M1��ǰ��HALLֵ
    }
  }   
  if((M2Cmd != CmdNull)&&(M2Cmd != CmdStop)) //���2��������״̬�����PID����
  {
    /*
    if (GetM2BeforeRunState() == 1)
    {
      return;
    }
    */
    if(++T100msCnt2 >= 100)
    {
      T100msCnt2 = 0;     
      M2PID.CurrSpeed = ABS(M2State.HallNow,M2State.HallLast);//������2��ǰ�ٶ�  
     
      if ((Balance_Data.TwoMotorOffsetHall != 0) && (SysState == RESET))
      {
          Balance_Data.TwoMotorOffsetHall = 0;
      }
      
        if((M2Cmd == CmdUp)||(M2Dir == UP)) //���2������������״̬
        {
          /*
          if(M1State.HallNow <= M2State.HallNow) //M2�ĵ�ǰλ�ø���M1�ĵ�ǰλ�ã�M2���ٿ죩        
            M2PID.SyncDev = (s8)((M2State.HallNow- M1State.HallNow)); //����M2ͬ��ƫ��ֵ  
          */
          if ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) <= M2State.HallNow)                           //ϵͳ������������״̬����M1λ�õ���M2λ��
          {
            M2PID.SyncDev = (s8)((M2State.HallNow - (M1State.HallNow - Balance_Data.TwoMotorOffsetHall)));      //M2Ҫ���е���һ��
          }  
        }
        else if(M2Cmd == CmdUpRetard)
        {
          /*
          if((M1State.HallNow <= M2State.HallNow)) //M2�ĵ�ǰλ�ø���M1�ĵ�ǰλ�ã�M2���ٿ죩        
            M2PID.SyncDev = (s8)((M2State.HallNow- M1State.HallNow)); //����M2ͬ��ƫ��ֵ
          */
          if ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) <= M2State.HallNow)   //ϵͳ����������ʱ���н׶Σ���M1λ�õ���M2λ��
          {
            M2PID.SyncDev = (s8)((M2State.HallNow - (M1State.HallNow - Balance_Data.TwoMotorOffsetHall)));      //M2Ҫ���е���һ��
          }
        }
        else if((M2Cmd == CmdDown)||(M2Dir == DOWN)) //���2������������״̬
        {
          /*
          if(M2State.HallNow >= M1State.HallNow) //M2�ĵ�ǰλ�ø���M1�ĵ�ǰλ�ã�M2�������� 
          {
            M2PID.SyncDev = (s8)((M2State.HallNow- M1State.HallNow)); //����M2ͬ��ƫ��ֵ 
            M2PID.SyncDev = 0 - M2PID.SyncDev;            
          }
          */
          if(M2State.HallNow >= (M1State.HallNow - Balance_Data.TwoMotorOffsetHall)) //M2�ĵ�ǰλ�ø���M1�ĵ�ǰλ�ã�M2��������
          {
            M2PID.SyncDev = (s8)((M2State.HallNow - (M1State.HallNow - Balance_Data.TwoMotorOffsetHall))); //����M2ͬ��ƫ��ֵ 
            M2PID.SyncDev = 0 - M2PID.SyncDev; 
          }
        }
        else if(M2Cmd == CmdDownRetard)
        {
          /*
          if((M1State.HallNow >= M2State.HallNow)) //M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ�ã�M2���ٿ죩   
          {
            M2PID.SyncDev = (s8)((M1State.HallNow- M2State.HallNow)); //����M2ͬ��ƫ��ֵ
          }
          else 
          {
            M2PID.SyncDev = (s8)((M2State.HallNow- M1State.HallNow));
            M2PID.SyncDev = 0 - M2PID.SyncDev;
          }
          */
          if ((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) >= M2State.HallNow)   //M1�ĵ�ǰλ�ø���M2�ĵ�ǰλ�ã�M2���ٿ죩 
          {
            M2PID.SyncDev = (s8)(((M1State.HallNow - Balance_Data.TwoMotorOffsetHall) - M2State.HallNow)); //����M2ͬ��ƫ��ֵ
          }
          else
          {
            M2PID.SyncDev = (s8)((M2State.HallNow- (M1State.HallNow - Balance_Data.TwoMotorOffsetHall)));
            M2PID.SyncDev = 0 - M2PID.SyncDev;
          }
        }
      
      PIDCal(&M2PID);    
     
      SetTIM1_PWMDuty(2,1350 - M2PID.PWMDuty);
      if(SysState == RESET)
      {
        M2Detect++;    
      }
      M2State.HallLast = M2State.HallNow;//ÿ100ms����һ��M2��ǰ��HALLֵ  
    
    }
   }   
  }
  
  if(SaveState != NULL) //M�����º�ʼ3���ʱ
  {
    MemCnt++;
  }
  if(SysState == NORMAL)
  {
    if(((M1Cmd!=CmdStop)&&(M1Cmd!=CmdNull))||((M2Cmd!=CmdStop)&&(M2Cmd!=CmdNull)))//���1�͵��2û��ȫ��ֹͣ
    {
      if(MotorStopTime > (9*RunTimeTemp))
      {
        if(MotorRunTotal>=RunTimeTemp)
          MotorRunTotal -= RunTimeTemp;
        else
          MotorRunTotal = 0;
      }
      MotorRunTime++;
      MotorRunTotal++;
      MotorStopTime = 0;
      //RunTimeTemp = 0;
    }
    if(((M1Cmd==CmdStop)||(M1Cmd==CmdNull))&&((M2Cmd==CmdStop)||(M2Cmd==CmdNull)))//���1�͵��2ȫ��ֹͣ
    {
      if(MotorRunTime!=0)
        RunTimeTemp = MotorRunTime;
      MotorStopTime++;
      MotorRunTime = 0;
    }
    if((MotorStopTime > (9*MotorRunTotal))||(MotorStopTime>=1080000))
      MotorRunTotal = 0;
  }
  if(T18minFlag == 1)
  {
    if(T18minCnt < 1080000)
      T18minCnt++;
  }
  if(SysCmd == CmdNull)
  {
    if((ErrCode==0)||((SysState==RESET)&&(ErrCode==Err_RESET)))
    if(LEDRest<10000)
      LEDRest++;
  }
  if(((M1Cmd!=CmdStop)&&(M1Cmd!=CmdNull))||((M2Cmd!=CmdStop)&&(M2Cmd!=CmdNull)))//���1�͵��2û��ȫ��ֹͣ
  {
    T500msCnt3 = 0;
    FallDetectBase1 = 0;
    FallDetectBase2 = 0;
    FallDetectFlag = 0;
  }
  if(((M1Cmd==CmdStop)||(M1Cmd==CmdNull))&&((M2Cmd==CmdStop)||(M2Cmd==CmdNull)))//���1�͵��2ȫ��ֹͣ
  {
    if(T500msCnt3 < 500)
    {
      T500msCnt3++;
    }
    else if(T500msCnt3 == 500)
    {
      if(FallDetectFlag == 0)
      {
        FallDetectFlag = 1;
        FallDetectBase1 = M1State.HallNow;
        FallDetectBase2 = M2State.HallNow;
      }          
      else if(FallDetectFlag == 1)
      {
        if((FallDetectBase1>(M1State.HallNow+((u16)(RATE))))||(FallDetectBase2>(M2State.HallNow+((u16)(RATE)))))
        {
          ErrCode = Err_Fall;
        }
      }
    }
  }
  if(ErrCode == Err_Fall)
  {
    if(++T100msCnt4 >= 100)
    {
      T100msCnt4 = 0;
      if(BuzzerState == OFF)
        BuzzerState = ON;
      else
        BuzzerState = OFF;
    }
  }  
  
  Balance_Data.DecideMotorRunTimerCnt++;
  
  Balance_Data.MotorSlideTimerCnt++;          //����ʱ���������0
  
  Balance_Data.HallCheckDelayTimerCnt++;
    
  //Balance_Data.BuzzerOnTimerCnt++;
  BuzzerOnTimerCnt++;
  
  Balance_Data.DelayTimerCnt++;
  
  if(BuzzerWorkMode == 3)
  {
    if (BuzzerOnTimerCnt > 1000)
    {
      BuzzerState = OFF;
      BuzzerWorkMode = 0;
    }
  }
  
  if (BuzzerWorkMode == 2)
  {
    if (BuzzerOnTimerCnt > 1500)
    {
      BuzzerState = OFF;
      BuzzerWorkMode = 0;
    }
  }
  if (BuzzerWorkMode ==1)
  {
     if (BuzzerOnTimerCnt > 500)
    {
      BuzzerState = OFF;
      BuzzerWorkMode = 0;
    }
  }

  
}

INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
  if((UART3->SR & UART3_SR_OR) == UART3_SR_OR)
  {		
    Com3RX_Data = UART3_ReceiveData8();
    Com3RX_Data = 0;
    
    T10msFlag = 0;
    T10msCnt = 0;
    Com3UartState = IDLE;
    memset(&Com3ReceiveBuffer[0],0,64);
    Com3ReceiveCount = 0;
  }	
  else if((UART3->SR & UART3_SR_RXNE) == UART3_SR_RXNE)
  {
    Com3RX_Data = UART3_ReceiveData8();
    if((Com3UartState == WAIT)&&(Com3RX_Data == FrameHeader)) //֡ͷ�ж�
    {
      Com3UartState = RECEIVEBUSY;
    }
    else if(Com3UartState == RECEIVEBUSY)
    {
      if(Com3DeviceState!=SLAVER)
      {
      if(((Com3RX_Data==FrameEnd)||(Com3RX_Data==FrameHeader))&&
         ((Com3ReceiveCount < Com3ReceiveBuffer[0])&&(Com3ReceiveBuffer[Com3ReceiveCount-1]=='\\'))) //ת���ַ��ж�
      {     
        Com3ReceiveBuffer[Com3ReceiveCount-1] = Com3RX_Data;
      }
      else if(Com3ReceiveCount==0)
      {
        Com3ReceiveBuffer[0] = Com3RX_Data;
        Com3ReceiveCount++;
      }
      else if(Com3ReceiveCount<Com3ReceiveBuffer[0])
      {
        Com3ReceiveBuffer[Com3ReceiveCount] = Com3RX_Data;
        Com3ReceiveCount++;
      }
      else
      {
        Com3UartState = FINISHED;
        if(Com3RX_Data == FrameEnd)
           T100msCnt3 = 0;
      }
      }
      else
      {
        if((Com3RX_Data==FrameEnd)&&(Com3ReceiveCount==Com3ReceiveBuffer[0]))
        {
          Com3UartState = FINISHED; //����֡�������
        }
        else
        {
          Com3ReceiveBuffer[Com3ReceiveCount] = Com3RX_Data;
          Com3ReceiveCount++;
          if(Com3ReceiveCount>63)
            Com3UartState = FINISHED;
        }
      }
    }
    else if((Com3UartState == WAIT)&&(Com3RX_Data != FrameHeader))
    {
      Com3UartState = IDLE;
      Com3ReceiveCount = 0;
      memset(&Com3ReceiveBuffer[0], 0, sizeof(Com3ReceiveBuffer));
    }
  }
  UART3_ClearITPendingBit(UART3_IT_RXNE);  
}

INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
  if((UART1->SR & UART1_SR_OR) == UART1_SR_OR)
  {		
    Com1RX_Data = UART1_ReceiveData8();
    Com1RX_Data = 0;
    
    T10msFlag = 0;
    T10msCnt = 0;
    Com1UartState = IDLE;
    memset(&Com1ReceiveBuffer[0],0,64);
    Com1ReceiveCount = 0;
  }	
  else if((UART1->SR & UART1_SR_RXNE) == UART1_SR_RXNE) //����1���յ�1���ֽڵ�����
  {
    Com1RX_Data = UART1_ReceiveData8();                 //������յ�������
    
    if((Com1UartState == WAIT)&&(Com1RX_Data == FrameHeader)) //֡ͷ�ж� (����һ�����յ���������Э��ͷ)
    {
      Com1UartState = RECEIVEBUSY;      //�ı䴮��1��״̬������1�������״̬
    }
    else if(Com1UartState == RECEIVEBUSY)       //
    {
      if(Com1DeviceState != SLAVER)     //���ƺ���Ϊ���豸
      {
        if(((Com1RX_Data == FrameEnd)||(Com1RX_Data == FrameHeader))&&
           ((Com1ReceiveCount < Com1ReceiveBuffer[0]) && (Com1ReceiveBuffer[Com1ReceiveCount-1]=='\\'))) //ת���ַ��ж�
        {     
          Com1ReceiveBuffer[Com1ReceiveCount-1] = Com1RX_Data;
        }
        else if(Com1ReceiveCount == 0)  //Com1ReceiveBuffer[0] ��ʾ �洢���Ƿ��͹��������ݳ���
        {
          Com1ReceiveBuffer[0] = Com1RX_Data;   //�������ݴ洢
          Com1ReceiveCount++;
        }
        else if(Com1ReceiveCount < Com1ReceiveBuffer[0])
        {
          Com1ReceiveBuffer[Com1ReceiveCount] = Com1RX_Data;
          Com1ReceiveCount++;
        }
        else
        {
          Com1UartState = FINISHED;
          if(Com1RX_Data == FrameEnd)
          {
            T100msCnt5 = 0;
          }
        }
      }
      else      //���ƺ���Ϊ���豸
      {
        if((Com1RX_Data==FrameEnd)&&(Com1ReceiveCount==Com1ReceiveBuffer[0]))
        {
          Com1UartState = FINISHED; //����֡�������
        }
        else
        {
          Com1ReceiveBuffer[Com1ReceiveCount] = Com1RX_Data;
          Com1ReceiveCount++;
          if(Com1ReceiveCount>63)
            Com1UartState = FINISHED;
        }
      }
    }
    else if((Com1UartState == WAIT)&&(Com1RX_Data != FrameHeader))
    {
      Com1UartState = IDLE;
      Com1ReceiveCount = 0;
      memset(&Com1ReceiveBuffer[0], 0, sizeof(Com1ReceiveBuffer));
    }
  }
  UART1_ClearITPendingBit(UART1_IT_RXNE);  
}