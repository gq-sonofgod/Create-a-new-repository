#include "Key.h"
#include "Main.h"
#include "PID.h"
#include "Motor.h"
#include "EEPROM.h"
#include "LED.h"
#include "Timer.h"
#include "FaultDetection.h"
#include "stm8s_it.h"
#include "HealthMode.h"
#include "Delay.h"
#include "string.h"
#include "Uart.h"
#include "ADC.h"
#include "math.h"
#include "LSM6DSL.h"
#include "Balance.h"

#include "Tap.h"

/*
extern u16 MenuTime;
extern u16 UpKeyDelay;
extern u16 DownKeyDelay;
*/
u8 KEY_Stop_M_Flag=0;
u8 MenuKeyM = 0; 
u8 MenuKeyUp = 0;  
u8 DisplayRemind=1;
//�ϰ�����־λ
u8 MenuKeyDown = 0;         //�°�����־λ
u8 Menu1Flag = 0;           //����һ���˵���־λ     0��δ����    1���ѽ���
u8 Menu2Flag = 0;           //��������˵���־λ     0��δ����    1���ѽ���
u8 Menu2Num = 0;//��ǰ��ʾ�Ķ����˵����Ӳ˵�
u8 Menu1Num=0;
u8 Menu3Num=0;
u8 Menu2_0 = 0;             //�����˵���0����    0Ϊoff   1ΪON
u8 Rst_EN=0;
u8 Check_EN=0;
u8 Balance_EN=0;
u16 VER_Cnt=0;
//extern u16 Acc_x,Acc_y,Acc_z;
u8 SaveState = 0;
u16 MemCnt = 0;
u8 SavePosition = 0; //�洢���ǵڼ���λ��
u8 Position;
u16 Key;
u16 Keyolder=0;
u8 RSTFlag = 0;
u8 Release = 1;         //
u8 InvalidFlag = 0;     //������Ч��־λ��0��ʾ������Ч����0��ʾ������Ч
u16 InvalidCnt = 0;     //������Чʱ�����ֵ
u16 LastKey = 0;
u8 InitFlag = 0;        //���ڸ�λ������RST���ı�־λ��ͬʱ���Ǹ�λ����ʱ���Ʒ������ı�־λ
u8 BuzzerTest = 0;
double TempRes = 0;
u8 AgingTest = 0;
u8 KeyDisable = 0;
u8 CombinationKey = 0; //��ϼ���־λ
u8 KeyAValid = 0;  //A����Ч��־
u8 KeyMValid = 0;  //M����Ч��־

u16 Com3KeyValue = 0;
u16 Com1KeyValue = 0;
u16 KeyUpCnt=0;
u16 KeyDownCnt=0;

u8 M1BeforeRunState = 0;
u8 M2BeforeRunState = 0;

u8 CmdToPresetState = 0;

extern u16 T100msCnt5;

u32  dheight;
#define  ESP32_RESTART      0x81
u8 Downflag=0;
u16 com1key;
u16 com3key;
u16 Com1LongCnt=0;
u16 Com3LongCnt=0;
u16 sensTimer=0;
u8 sensKeyFlag=0;
u8 RunCntFlag = 0;
u8 BlockFlag = 0;
extern u8 DisMode=0;
u8 RST_Sat=0;
/***********************************************************************************************************
* ��������: KeyScan()
* �������: value  ��õİ�����ADCֵ
* ����ֵ  : keytemp, ��ǰ��ֵ
* ��    ��: ɨ�谴�����ж�������ֵ��״̬
************************************************************************************************************/
u16 Com3GetValue(u16 value)
{
    u8 cnt;

    static uint16_t  com3send_num = 0;

    if(Com3UartState == IDLE)
    {
        com3send_num++;

        memset(&SendBuffer[0], 0, sizeof(SendBuffer));

        if(com3send_num > 2000)
        {
            SendBuffer[0] = ESP32_RESTART;

            com3send_num = 0;
        }
        else
        {
            SendBuffer[0] = GetKeyValue;
        }

        cnt = 1;

        PackageSendData(&SendBuffer[0], &cnt);

        Uart3SendData(&SendBuffer[0], cnt);

        Com3UartState = WAIT;

    }
    else if(Com3UartState == FINISHED)
    {
        if(UnpackReceivedData(&Com3ReceiveBuffer[0], &Com3ReceiveCount))
        {
          if(Com3ReceiveBuffer[1]!=0x04)
          {  
            if(Com3ReceiveBuffer[1] == 0x02) //�ж��ǰ�������ƻ������ܶ˿���
            {
               com3Link=LinkKey;
               if(Com3ReceiveCount==5)
               {
                 Com3KeyValue = Com3ReceiveBuffer[2];
               }
               else
               {
                 Com3KeyValue = Com3ReceiveBuffer[3]<<8|Com3ReceiveBuffer[2];
               }
            }
            else if(Com3ReceiveBuffer[1] == 0x03)//APP����
            {
               com3Link=LinkApp;
               Com3KeyValue = Com3ReceiveBuffer[2];
            }

            com3send_num = 0;

            if(Com3KeyValue == KEY12) //�յ��ܵ�ָ���߶�����
            {             
                dheight = (((u16)Com3ReceiveBuffer[4]) << 8) + Com3ReceiveBuffer[3];

                 if(dheight >= (BaseHeight * 10))//������յ��ĸ߶�
                  {
                    dheight -= (BaseHeight * 10);

                    dheight = (u32)(dheight * RATE / 10); //

                    M1State. Record[4] = M1State.LimitDown + dheight;

                    M2State. Record[4] = M2State.LimitDown + dheight;
                    
                    
                    //Balance_Data.TwoMotorOffsetHall >= 0      //M1�����M2�����
                    if (Balance_Data.TwoMotorOffsetHall >= 0)
                    {  
                        M2State.Record[4] = M2State.Record[4] - Balance_Data.TwoMotorOffsetHall;
                    }
                    else
                    {
                        M1State.Record[4] = M1State.Record[4] + Balance_Data.TwoMotorOffsetHall;
                    }
                    
                    
                    if((M1State. Record[4] > M1State.LimitUp)||(M1State. Record[4] < M1State.LimitDown))
                    {
                        Com3KeyValue = KEY_NULL;
                    }

                    if((M2State. Record[4] > M2State.LimitUp)||(M2State. Record[4] < M2State.LimitDown))
                    {
                        Com3KeyValue = KEY_NULL;
                    }

                    M1State. Record[4] += 9;//����9��������
                    
                    if(M1State. Record[4] > M1State.LimitUp)
                    {
                        M1State. Record[4] = M1State.LimitUp;
                    }

                    M2State. Record[4] += 9;
                    if(M2State. Record[4] > M2State.LimitUp)
                    {
                        M2State. Record[4] = M2State.LimitUp;
                    }

                }
		else
		{
                   Com3KeyValue = KEY_NULL;
		}
            }
            else if(Com3KeyValue==KEY_DEVINFO)
            {
                SendBuffer[0] = KEY_DEVINFO;

                SendBuffer[1] = UNIT; //��λ
                
                if(UNIT==0)
                {
                SendBuffer[2] = (u16)(MaxHeight*10) & 0xff; //��߸߶ȵ�λ

                SendBuffer[3] = (u16)(MaxHeight*10) >> 8; //��߸߶ȸ�λ

                SendBuffer[4] = (u16)(BaseHeight*10) & 0xff; //��͸߶ȵ�λ

                SendBuffer[5] = (u16)(BaseHeight*10) >> 8; //��͸߶ȸ�λ
                }
                else
                {
                SendBuffer[2] = (u16)(MaxHeight*10/2.54) & 0xff; //��߸߶ȵ�λ

                SendBuffer[3] = (u16)(MaxHeight*10/2.54) >> 8; //��߸߶ȸ�λ

                SendBuffer[4] = (u16)(BaseHeight*10/2.54) & 0xff; //��͸߶ȵ�λ

                SendBuffer[5] = (u16)(BaseHeight*10/2.54) >> 8; //��͸߶ȸ�λ
                }
                SendBuffer[6] = 0;	 //�������������Ԥ��

                cnt = 7;

                PackageSendData(&SendBuffer[0], &cnt);

                Uart3SendData(&SendBuffer[0], cnt);
                
                Com3KeyValue=KEY_NULL;
            }
            else if(Com3KeyValue == KEY_ANTI)
            {
              if(((SysCmd == CmdDown)||((M1Dir==DOWN)&&(M2Dir==DOWN)&&(M1Cmd!=CmdGoBack)&&(M2Cmd!=CmdGoBack))))
              {
                BlockFlag = 1;
                Tap_Parameter.TapTriggerState = 0;
                Tap_Parameter.TapControlFlag = 0;
              }
              Com3KeyValue = KEY_NULL;
            }
          }
          else
          {
              Com3DeviceState = SLAVER;  //�豸״̬תΪ��״̬������������λ��������
              Com3UartState = WAIT;  
          }
          T10msFlag = 0;
        }

        if(Com3DeviceState!=SLAVER)
           Com3UartState = IDLE;

        T100msCnt3 = 0;

        Com3ReceiveCount = 0;

        memset(&Com3ReceiveBuffer[0], 0, sizeof(Com3ReceiveBuffer));
    }
    return Com3KeyValue;
}

u16 Com1GetValue(u16 value)
{
    u8 cnt;

    static uint16_t  com1send_num = 0;

    if(Com1UartState == IDLE)           //
    {
        com1send_num++;

        memset(&SendBuffer[0], 0, sizeof(SendBuffer));  //SendBuffer������0����

        if(com1send_num > 2000)
        {
            SendBuffer[0] = ESP32_RESTART;      //#define  ESP32_RESTART      0x81
  
            com1send_num = 0;
        }
        else
        {
            SendBuffer[0] = GetKeyValue;        //#define  GetKeyValue        0x11
        }

        cnt = 1;

        PackageSendData(&SendBuffer[0], &cnt);  //����ͨ��Э�飬��װҪ���͵�����
        Uart1SendData(&SendBuffer[0], cnt);     //��������
        
        /*
        RST_Sat=RST-> SR&0x1f;
        SendBuffer[0] =RST_Sat|0xa0;
        Uart1SendData(&SendBuffer[0],1);
        */
        Com1UartState = WAIT;                   //����ȴ��ⲿ����״̬

    }
    else if(Com1UartState == FINISHED)
    {
        if(UnpackReceivedData(&Com1ReceiveBuffer[0], &Com1ReceiveCount))        //������յ������ݲ��ж�������Ч�� (CRCУ��)
        {
          if(Com1ReceiveBuffer[1] != 0x04)
          {
            if(Com1ReceiveBuffer[1] == 0x02)    //�ж��ǰ�������ƻ������ܶ˿���
            {
               com1Link = LinkKey;              //�жϴ���1�˿ڴ��ӵ��ǰ����壬com1Link������ʼ��ʱLinkNull
               if(Com1ReceiveCount==5)
               {
                 Com1KeyValue = Com1ReceiveBuffer[2];
               }
               else
               {
                 Com1KeyValue = Com1ReceiveBuffer[3]<<8|Com1ReceiveBuffer[2];
               }
            }
            else if(Com1ReceiveBuffer[1] == 0x03)//APP����
            {
               com1Link = LinkApp;
               Com1KeyValue = Com1ReceiveBuffer[2];
            }

            com1send_num = 0;

            if(Com1KeyValue == KEY12) //�յ��ܵ�ָ���߶�����
            {             
                dheight = (((u16)Com1ReceiveBuffer[4]) << 8) + Com1ReceiveBuffer[3];

		  if(dheight >= (BaseHeight * 10))//������յ��ĸ߶�
                  {
                    dheight -= (BaseHeight * 10);

                    dheight = (u32)(dheight * RATE/10); //

                    M1State.Record[4] = M1State.LimitDown + dheight;

                    M2State.Record[4] = M2State.LimitDown + dheight; 
                    
                    
                    //Balance_Data.TwoMotorOffsetHall >= 0      //M1�����M2�����
                    if (Balance_Data.TwoMotorOffsetHall >= 0)
                    {  
                        M2State.Record[4] = M2State.Record[4] - Balance_Data.TwoMotorOffsetHall;
                    }
                    else
                    {
                        M1State.Record[4] = M1State.Record[4] + Balance_Data.TwoMotorOffsetHall;
                    }
                    
                    
                    if((M1State.Record[4] > M1State.LimitUp)||(M1State.Record[4] < M1State.LimitDown))
                    {
                        Com1KeyValue = KEY_NULL;
                    }

                    if((M2State.Record[4] > M2State.LimitUp)||(M2State.Record[4] < M2State.LimitDown))
                    {
                        Com1KeyValue = KEY_NULL;
                    }
                    
                    M1State.Record[4] += 9;//����9��������
                    
                    if(M1State.Record[4] > M1State.LimitUp)
                    {
                        M1State.Record[4] = M1State.LimitUp;
                    }

                    M2State.Record[4] += 9;
                    if(M2State.Record[4] > M2State.LimitUp)
                    {
                        M2State.Record[4] = M2State.LimitUp;
                    }
                }
		else
		{
                    Com1KeyValue = KEY_NULL;
		}
            }
            else if(Com1KeyValue==KEY_DEVINFO)          //APP��ȡ�豸��Ϣ�����
            {
                SendBuffer[0] = KEY_DEVINFO;

                SendBuffer[1] = UNIT; //��λ

                if(UNIT==0)
                {
                  SendBuffer[2] = (u16)(MaxHeight*10) & 0xff;           //��߸߶ȵ�λ

                  SendBuffer[3] = (u16)(MaxHeight*10) >> 8;             //��߸߶ȸ�λ

                  SendBuffer[4] = (u16)(BaseHeight*10) & 0xff;          //��͸߶ȵ�λ

                  SendBuffer[5] = (u16)(BaseHeight*10) >> 8;            //��͸߶ȸ�λ
                }
                else
                {
                  SendBuffer[2] = (u16)(MaxHeight*10/2.54) & 0xff;        //��߸߶ȵ�λ

                  SendBuffer[3] = (u16)(MaxHeight*10/2.54) >> 8;          //��߸߶ȸ�λ

                  SendBuffer[4] = (u16)(BaseHeight*10/2.54) & 0xff;       //��͸߶ȵ�λ

                  SendBuffer[5] = (u16)(BaseHeight*10/2.54) >> 8;         //��͸߶ȸ�λ
                }

                SendBuffer[6] = 0;	 //�������������Ԥ��

                cnt = 7;

                PackageSendData(&SendBuffer[0], &cnt);

                Uart1SendData(&SendBuffer[0], cnt);
                
                Com1KeyValue = KEY_NULL;
            }
            else if(Com1KeyValue == KEY_ANTI)
            {
              if(((SysCmd == CmdDown)||((M1Dir==DOWN)&&(M2Dir==DOWN)&&(M1Cmd!=CmdGoBack)&&(M2Cmd!=CmdGoBack)))) //�������
              {
                BlockFlag = 1;
                Tap_Parameter.TapTriggerState = 0;
                Tap_Parameter.TapControlFlag = 0;
              }
              Com1KeyValue = KEY_NULL;
            }
            
          }
          else
          {
              Com1DeviceState = SLAVER;         //�豸״̬תΪ��״̬������������λ��������
              Com1UartState = WAIT;  
          }
          T10msFlag = 0;
        }
        if(Com1DeviceState!=SLAVER)
           Com1UartState = IDLE;

        T100msCnt5 = 0;

        Com1ReceiveCount = 0;

        memset(&Com1ReceiveBuffer[0], 0, sizeof(Com1ReceiveBuffer));
    }
    return Com1KeyValue;
}



/***********************************************************************************************************
* ��������: KeyScan()
* �������: value  ��õİ�����ADCֵ
* ����ֵ  : keytemp, ��ǰ��ֵ
* ��    ��: ɨ�谴�����ж�������ֵ��״̬
************************************************************************************************************/
u16 Com3KeyScan(u16 value)
{
  static u16 Com3KeyState = STATE_NULL;
  static u16 Com3WobbleCnt = 0; //��������
  static u16 com3keytemp = 0;
  u16 key;
  
  key = Com3GetValue(value);    //��ȡ����ֵ
  
  switch(Com3KeyState)
  {
    case STATE_NULL:    //STATE_NULL = 0
      if(key != KEY_NULL) //�а�������
      {
        com3keytemp = key;
        Com3KeyState = STATE_WOBBLE; //��������״̬   //STATE_WOBBLE = 1
      }
      else
      {
        com3keytemp = 0;
      }
      break;
    
    case STATE_WOBBLE:          //����״̬
      if(key == com3keytemp)    //����ͬһ����������
      {
        if(++Com3WobbleCnt >= 1)
        {
          Com3WobbleCnt = 0;
          if((com3keytemp == KEY1)||(com3keytemp==KEY2)||(com3keytemp==KEY14))
          {
            if((Balance_Data.BalanceAdjuseState != 0)&&(Balance_Data.BalanceAdjuseState != 1))
            {
            Com3KeyState = STATE_PRESS;
             Com3LongCnt = 0;
             }
            else return (com3keytemp|KEYPRESS);
           //return (com3keytemp|KEYPRESS);
          }
          else if((com3keytemp == KEY6)||(com3keytemp == KEY8)||(com3keytemp==KEY7)||(com3keytemp==KEY9)||(com3keytemp==KEY10)||(com3keytemp==KEY11)||(com3keytemp == KEY15))
          {
            Com3KeyState = STATE_PRESS;
            Com3LongCnt = 0;
          }
          else
          {
            Com3KeyState = STATE_HOLD;
            return (com3keytemp|KEYPRESS);
          }
        }
      }
      else
      {
        Com3WobbleCnt = 0;
        com3keytemp = KEY_NULL;
        Com3KeyState = STATE_NULL;   //����û��ͨ����״̬�ع��ʼ̬
      }
      break;
      
    case STATE_HOLD:
      if(key==KEY_NULL)
      {
        com3keytemp = KEY_NULL;
        Com3KeyState = STATE_NULL;
        return com3keytemp;
      }
      else if((key == KEY8)||(key==KEY9)||(key==KEY10)||(key==KEY11)||(key == KEY15))
      {
        com3keytemp = KEY_NULL;
        Com3KeyState = STATE_NULL;
        return com3keytemp;
      }
      else if(key!=com3keytemp)
      {
        com3keytemp = KEY_NULL;
        Com3KeyState = STATE_NULL;
        return com3keytemp;
      }
      break;
      
    case STATE_PRESS: //���������Ѿ�ȷ��״̬
      if(key==com3keytemp)//����û���ɿ�
      {
        if((Com3LongCnt >= 3000)&&(Balance_Data.BalanceAdjuseState== 0))
        {
          Com3LongCnt = 0;
          Com3KeyState = STATE_LONG;
          return (com3keytemp|KEYLONG);
        }
        else if((com3keytemp==KEY2)&&(Balance_Data.BalanceAdjuseState!= 0))
        {
        Downflag=1; 
        Com3LongCnt = 0;
        Com3KeyState = STATE_PRESS;
        return KEY_NULL;
        }
      }
      else if((key == KEY_NULL)&&(Downflag==0))  //�������û��ͨ����ص���ʼ̬
      {
        Com3LongCnt = 0;
        Com3KeyState = STATE_NULL;
        return (com3keytemp|KEYPRESS);
      }
      else if((key == KEY_NULL)&&(Downflag==1))  //�������û��ͨ����ص���ʼ̬
      {
        Downflag=0;
        Com3LongCnt = 0;
        Com3KeyState = STATE_NULL;
        return KEY_NULL;;
      }
      else
      {
        Com3LongCnt = 0;
        Com3KeyState = STATE_NULL;
        com3keytemp = KEY_NULL;
        return (com3keytemp|KEYPRESS);
      }
      break;
           
    case STATE_LONG:
      if(((key == KEY8)&&(key==com3keytemp))||((key == KEY15)&&(key==com3keytemp)))
      {
        return (com3keytemp|KEYLONG);
      }
      else if(key != com3keytemp)
      {
        com3keytemp = KEY_NULL;
        Com3KeyState = STATE_NULL;
        return com3keytemp;
      }
      break;
  }
  return com3keytemp;
}

/****************************************************************
�������ܣ�ʵ�ְ�������������
���룺u16 value ���˲�����������
�����u16�Ͳ�����������յ��İ���ֵ 
****************************************************************/
u16 Com1KeyScan(u16 value)
{
  static u16 Com1KeyState = STATE_NULL;
  static u16 Com1WobbleCnt = 0; //��������
  static u16 com1keytemp = 0;
  u16 key;
  
  key = Com1GetValue(value);    //ͨ�����ڻ�ȡ����ֵ
  switch(Com1KeyState)
  {
    case STATE_NULL:
      if(key != KEY_NULL) //�а�������
      {
        com1keytemp = key;
        Com1KeyState = STATE_WOBBLE; //��������״̬
      }
      else
      {
        com1keytemp = 0;
      }
      break;
    
    case STATE_WOBBLE: //����״̬
      if(key == com1keytemp) //����ͬһ����������
      {
        if(++Com1WobbleCnt >= 1)
        {
          Com1WobbleCnt = 0;
          if((com1keytemp == KEY1)||(com1keytemp==KEY2)||(com1keytemp==KEY14))
          {
            return (com1keytemp|KEYPRESS);
          }
          else if((com1keytemp == KEY6)||(com1keytemp == KEY8)||(com1keytemp==KEY7)||(com1keytemp==KEY9)||(com1keytemp==KEY10)||(com1keytemp==KEY11)||(com1keytemp == KEY15))
          {
            Com1KeyState = STATE_PRESS;
            Com1LongCnt=0;
          }
          else
          {
            Com1KeyState = STATE_HOLD;
            return (com1keytemp|KEYPRESS);
          }
        }
      }
      else
      {
        Com1WobbleCnt = 0;
        com1keytemp = KEY_NULL;
        Com1KeyState = STATE_NULL;   //����û��ͨ����״̬�ع��ʼ̬
      }
      break;
      
    case STATE_HOLD:
      if(key == KEY_NULL)
      {
        com1keytemp = KEY_NULL;
        Com1KeyState = STATE_NULL;
        return com1keytemp;
      }
      else if((key == KEY8)||(key==KEY9)||(key==KEY10)||(key==KEY11)||(key == KEY15))
      {
        com1keytemp = KEY_NULL;
        Com1KeyState = STATE_NULL;
        return com1keytemp;
      }
      else if(key!=com1keytemp)
      {
        com1keytemp = KEY_NULL;
        Com1KeyState = STATE_NULL;
        return com1keytemp;
      }
      break;
      
    case STATE_PRESS:                   //���������Ѿ�ȷ��״̬
      if(key == com1keytemp)            //����û���ɿ�
      {
        if(Com1LongCnt >= 3000)
        {
          Com1LongCnt = 0;
          Com1KeyState = STATE_LONG;
          return (com1keytemp|KEYLONG);
        }
      }
      else if(key == KEY_NULL)          //�������û��ͨ����ص���ʼ̬
      {
        Com1LongCnt = 0;
        Com1KeyState = STATE_NULL;
        return (com1keytemp|KEYPRESS);
      }
      else
      {
        Com1LongCnt = 0;
        Com1KeyState = STATE_NULL;
        com1keytemp = KEY_NULL;
        return (com1keytemp|KEYPRESS);
      }
      break;
      
      
    case STATE_LONG:
      if(((key == KEY8)&&(key==com1keytemp))||((key == KEY15)&&(key==com1keytemp)))
      {
        return (com1keytemp|KEYLONG);
      }
      else if(key != com1keytemp)
      {
        com1keytemp = KEY_NULL;
        Com1KeyState = STATE_NULL;
        return com1keytemp;
      }
      break;
  }
  return com1keytemp;
}

/**********************************************************************


**********************************************************************/

u16 KeyScan(u16 value)
{
   com1key=Com1KeyScan(value);  //��ȡ����1�İ���ֵ���������������ܣ�
   com3key=Com3KeyScan(value);  //��ȡ����3�İ���ֵ���������������ܣ�
   
   if(com3Link == LinkApp)
   {
      if(com3key != KEY_NULL)
      {
         return com3key;
      }
      else if(com1Link != LinkNull)
      {
         return com1key;
      }
      else return KEY_NULL;
   }
   else if(com1Link==LinkApp)
   {
      if(com1key!=KEY_NULL)
      {
         return com1key;
      }
      else if(com3Link!=LinkNull)
      {
         return com3key;
      }
      else return KEY_NULL;
   }
   else if(com3Link==LinkKey)
   {
      if(com3key!=KEY_NULL)
      {
        return com3key;
      }
      else if(com1Link==LinkKey)
      {
        return com1key;
      }
      else return KEY_NULL;
   }
   else if(com1Link==LinkKey)
   {
      if(com1key!=KEY_NULL)
      {
        return com1key;
      }
      else if(com3Link==LinkKey)
      {
         return com3key;
      }
      else return KEY_NULL;
   }
   return KEY_NULL;
}


/***********************************************************************************************************
* ��������: KeyRespond()
* �������: value, �ò�����ʵ������
* ����ֵ  : ��
* ��    ��: ������Ӧ����
************************************************************************************************************/
void KeyRespond(u16 value)
{  
  Key = KeyScan(value);
  
  if(InitFlag!= 0)
  {
  Key = KEY_NULL;
  
  }
  if(Key != KEY_NULL)
  {
    LEDRest = 0;
    
  }
  if((sensKeyFlag != 0) && (Key != KEY_SENS)&&(Key != KEY_NULL))
  {
    Key = 0x99;                         //��Ч
    Menu1Flag=0;//�޸ĵģ����������Ȱ����󣬿��ٵ㰴A�����ڰ����¼�û��Ӧ������
    Menu2Flag=0;
    Menu1Num=0;
    Menu2Num=0;
    Menu3Num=0;
    
    
  }
  if((ErrCode == Err_TimeEnd)||(ErrCode == Err_Overheating))    //����ʱ�䵽(��������2����) �͵����������£�������Ч
  {
    Key = KEY_NULL;
  }
  if((ErrCode == Err_LSM6DSL)&&(Key != KEY_SENS))               //�����ǹ����Ұ���Ϊ�ǵ��������Եİ������򰴼���Ч
  {
    Key = KEY_NULL;
  }
  if((LastKey != 0)&&(Key != KEY_UP)&&(Key != KEY_DOWN)&&(SysState == NORMAL))
  {
    Key = KEY_NULL;
  }
  if(KeyDisable == 1)
  {
    Key = KEY_NULL;
  }
  if((HealthMode == 6)||(HealthMode == 5))
  {
    Key = KEY_NULL;
  }
   if((Balance_Data.BalanceAdjuseState != 0)&&(Keyolder!= KEY_NULL)&&(Key!=KEY_NULL))
  {
  // Key = KEY_Stop_M;
  // Balance_Data.BalanceAdjuseState=0;
   if(Balance_Data.BalanceAdjuseState != 0)
     {
     // if(Key!=Keyolder)
      //{
         
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
      KEY_Stop_M_Flag=1;      
      Balance_Data.BalanceAdjuseState = 4;
      Balance_Data.MotorSlideTimerCnt = 200;
      //}
      HealthModeReset();  
        
     }   
  }
  if (((Balance_Data.BalanceAdjuseState != 0) || (InitFlag != 0 ))&&(Key==KEY_NULL))
  {
   return;
  }
 
  
  if (Key != KEY_NULL) 
  {
    if ((Tap_Parameter.TapControlEN == 1) &&(Tap_Parameter.TapControlFlag != 0))
    {
      if ((Tap_Parameter.TapTriggerState == 3) || (Tap_Parameter.TapTriggerState == 1))
      {
        Tap_Parameter.TapTriggerState = 2;
      }
    }
  }
  
  if ((Tap_Parameter.TapControlEN == 0) || 
      ((Tap_Parameter.TapControlEN == 1) && (Tap_Parameter.TapControlFlag == 0)))
{
  switch(Key)
      
  {
    case KEY_UP:  //���ϼ�
       
      if((SysState != RESET)&&(AntiCollisionState == 0))        //SysState = NORMAL
      {       
        if(SysCmd == CmdToPreseting) //(���ж��Ƿ���Ҫ��ͣ)   //�˶���Ԥ���λ������
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          PID_Set(Speed,BASEDUTYUP);
          M1Cmd = CmdNull;
          M2Cmd = CmdNull;
          SysCmd = CmdNull;
          M1Dir = 0;
          M2Dir = 0;
          Release = 0;         
          
        }//�����ս���UP������
        else if(((SysCmd == CmdNull)&&(Release==1))||(Adjust_State==2)/*||(LastKey==KEY_DOWN)*/)   //Release = 1��ϵͳ���ڿ���״̬  
        {
          if ((SaveState == BEGIN) || (Menu1Flag == 1)||((Menu1Flag == 2)&&(Menu2Num==2))||((Menu1Flag == 2)&&(Menu2Num==1)))
          {
            SaveState = NULL;
            MenuTime = 0;
            InvalidFlag = 0;
                  
            if((Menu1Flag == 1)||((Menu1Flag == 2)&&(Menu2Num==2))||((Menu1Flag == 2)&&(Menu2Num==1))) 
            {
              if ((UpKeyDelay >= 400) && (MenuKeyUp == 0) )
              {
                 MenuKeyUp = 1;
                 UpKeyDelay = 0;
               }
             }
             Key = KEY_NULL;
          }
          else
          {  
            if(Adjust_State == 2)  
            {
              //DeleteSavedHeight();                            //SaveFlag = 0;ɾ�����������ǰλ��EEPROM�еĴ洢ֵ��M1State.HallNow��M2State.HallNow��
           // HealthModeReset();  
            //���ý���ģʽ�ļ�ʱ
              if(KeyUpCnt < 200)
              {
                u16 hall_diff;
                if (M2State.HallNow > M1State.HallNow)
                { 
                  hall_diff = M2State.HallNow - M1State.HallNow;
                }
                else 
                {
                  hall_diff = 0 ; 
                } 
                if (hall_diff < MAX_HALL_DIFF)  //���40cm�߶Ȳ�
                {
                  DeleteSavedHeight(); 
                    
                  SysCmd = CmdUp;
                            //�Ӷ�ʱ�����棬ֱ�����ó�ʼPWMռ�ձ�
                  M2Cmd = CmdUp;
                  SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); 
                  PID_Set(8,BASEDUTYUP);                      //����PID��������������ת�٣�PID�����ʼPWMռ�ձ�
                  M2PID.SetSpeed = MINSPEED;
            
                  if(InvalidFlag == 0)                            //�˱�־λ��ʾ�Ƿ��а���ֵ��Ϊ0��˵���ް�����
                    InvalidFlag = 1;
            
                  RunCntFlag = 1;                                 //���б�־λ
            
                  if(HealthMode == 4)
                  {
                    HealthMode = 2;
                    T2sCnt = 1000;
                  }  
                }  
              }
              else
              {
                M1Up(OFF);
                M1Down(OFF);
                M1_PWM_OFF;
                M2Up(OFF);
                M2Down(OFF);
                M2_PWM_OFF;
                PID_Set(Speed,BASEDUTYDOWN);
                M1Cmd = CmdNull;
                M2Cmd = CmdNull; 
                SysCmd = CmdNull;
                M1Dir = 0;
                M2Dir = 0;
                Release = 0;  
              }           //�Ӷ�ʱ�����棬ֱ�����ó�ʼPWMռ�ձ� 
            }
            else //������������
            {
              Release = 0;
              DeleteSavedHeight();                            //SaveFlag = 0;ɾ�����������ǰλ��EEPROM�еĴ洢ֵ��M1State.HallNow��M2State.HallNow��
              HealthModeReset();                              //���ý���ģʽ�ļ�ʱ
              SysCmd = CmdUp;
              M1Cmd = CmdUp;
              SetTIM1_PWMDuty(1, 1350-BASEDUTYUP);            //�Ӷ�ʱ�����棬ֱ�����ó�ʼPWMռ�ձ�
          
              M2Cmd = CmdUp;
              SetTIM1_PWMDuty(2, 1350-BASEDUTYUP);            //�Ӷ�ʱ�����棬ֱ�����ó�ʼPWMռ�ձ� 
          
              PID_Set(Speed,BASEDUTYUP);                      //����PID��������������ת�٣�PID�����ʼPWMռ�ձ�

              if(InvalidFlag == 0)                            //�˱�־λ��ʾ�Ƿ��а���ֵ��Ϊ0��˵���ް�����
              {
                InvalidFlag = 1;
              }
            
              RunCntFlag = 1;                                 //���б�־λ   
              if(HealthMode == 4)
              {
                  HealthMode = 2;
                  T2sCnt = 1000;
              } 
            }     
          }
        }
        else if((LastKey == Key)&&(InvalidFlag == 2))       //���ϴΰ����ϼ��ͷź��ʱ�����ٴΰ������ϼ�
        {
          InvalidFlag = 1;
          InvalidCnt = 0;                                   //��ʱ���ʱֵ����          
        }
        
        if((SysCmd == CmdDown)&&(SysState==NORMAL))         //����ִ��DOWN����ʱ�����յ�UP����
        {
          KeyNullProcess();
        }
      
        LastKey = Key;
      }
      break;
      
      case KEY_DOWN: //���¼�
       
      if(AntiCollisionState==0) //û��������״̬
      {  
        if(SysCmd == CmdToPreseting)
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          PID_Set(Speed,BASEDUTYDOWN);
          M1Cmd = CmdNull;
          M2Cmd = CmdNull; 
          SysCmd = CmdNull;
          M1Dir = 0;
          M2Dir = 0;
          Release = 0;
        }
        else if((Menu1Flag == 1)||((Menu1Flag == 2)&&(Menu2Num==2))||((Menu1Flag == 2)&&(Menu2Num==1)))//�Ѿ�����һ���˵�ģʽ
        {
          MenuTime = 0;
          SaveState = NULL;
                 
          InvalidFlag = 0;
                  
          if((Menu1Flag == 1)||((Menu1Flag == 2)&&(Menu2Num==2))||((Menu1Flag == 2)&&(Menu2Num==1))) 
          {
            if ((DownKeyDelay >= 400) && (MenuKeyDown == 0) )
            {
              MenuKeyDown = 1;          
              DownKeyDelay = 0;
             }
           }
           Key = KEY_NULL;        
        }
        else if((SysCmd == CmdNull)||(Adjust_State==2)/*||(LastKey==KEY_UP)*/)
        {         
          u16 halltemp;
          if((SysState == NORMAL)&&(Release == 1)&&(AntiCollisionState==0))
          {
              //HealthModeReset();
            //if(((M1State.HallNow + M2State.HallNow)>(M1State.LimitDown+M2State.LimitDown+13)))
            halltemp = MIN(M1State.HallNow,M2State.HallNow); // (a<=b? a: b)
            if ((halltemp > ((M1State.LimitDown + M2State.LimitDown)/2 +13))||(Adjust_State==2))
            {
              if(Adjust_State == 2)
              {
                if(KeyDownCnt < 200)
                {
                  u16 hall_diff;
                  if (M2State.HallNow > M1State.HallNow)
                  { 
                    hall_diff =0;
                  }
                  else 
                  {
                    hall_diff = M1State.HallNow - M2State.HallNow ; 
                  }
                  //if (hall_diff > 6515)     //���40cm�߶Ȳ�
                  if (hall_diff < MAX_HALL_DIFF) 
                  {        
                    DeleteSavedHeight();
                    //HealthModeReset();
                    SysCmd = CmdDown;
                    M2Cmd = CmdDown;
                    SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN); 
                    PID_Set(8,BASEDUTYDOWN);
                    //Release = 0;
                    if(InvalidFlag==0)
                    {
                      InvalidFlag = 1;
                    }
                    RunCntFlag = 1;
                    if(HealthMode == 4)
                    {
                      HealthMode = 2;
                      T2sCnt = 1000;
                    }
                  }
                }
                else   
                {
                  M1Up(OFF);
                  M1Down(OFF);
                  M1_PWM_OFF;
                  M2Up(OFF);
                  M2Down(OFF);
                  M2_PWM_OFF;
                  PID_Set(Speed,BASEDUTYUP);
                  M1Cmd = CmdNull;
                  M2Cmd = CmdNull;
                  SysCmd = CmdNull;
                  M1Dir = 0;
                  M2Dir = 0;
                  InvalidFlag = 1; 
                }
              }
              else //������������
              {
                DeleteSavedHeight();
                HealthModeReset();
                SysCmd = CmdDown;
                M1Cmd = CmdDown;
                SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
                M2Cmd = CmdDown;
                SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN); 
              
                PID_Set(Speed,BASEDUTYDOWN);
                Release = 0;
                if(InvalidFlag==0)
                {
                  InvalidFlag = 1;
                }
                RunCntFlag = 1;
                if(HealthMode == 4)
                {
                  HealthMode = 2;
                  T2sCnt = 1000;
                }
              }
            }
            else if(RSTFlag == 0)
            {
              RSTFlag = 1;
            }
            else 
            {
                HealthModeReset();
            }
          }
          else if((SysState == RESET)&&((Release==3)||(Release==1)))
          {
            BuzzerState = OFF;
            if(Release == 3)
              Release = 1;
            if((ErrCode != 0)&&(ErrCode != Err_RESET))
            {
              ErrCode = Err_RESET;
              memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));
              EEPROM_Write();
            }   
            SysCmd = CmdDown;
            M1Cmd = CmdDown;
            SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
            M2Cmd = CmdDown;
            SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
            PID_Set(MINSPEED,BASEDUTYDOWN);         
            Dis_Char[0] = Char_R;
            Dis_Char[1] = Char_S;
            Dis_Char[2] = Char_T;
            M1State.HallNow = BASEHALL;
            M1State.HallLast = BASEHALL;
            M2State.HallNow = BASEHALL;
            M2State.HallLast = BASEHALL;
            RunCntFlag = 1;
          }
        }
        else if((LastKey==Key)&&(InvalidFlag==2))       //���ϴΰ����ϼ��ͷź��ʱ�����ٴΰ������ϼ�
        {
          InvalidFlag = 1;
          InvalidCnt = 0;  //��ʱ���ʱֵ����          
        }
        if((SysCmd == CmdUp)&&(SysState==NORMAL))
        {
          KeyNullProcess();             //����ȫ���ͷ�ʱ������
        }
        
        LastKey = Key;
      }
      break;
    
    case KEY_M1: //λ�ô洢1��
       if(((SysState != RESET)&&((Menu1Flag ==4)||(Menu1Flag ==0)))&&(Adjust_State==0))
      {
          //HealthModeReset();
        if(SysCmd == CmdToPreseting)
        {
          if(InvalidFlag==0)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            PID_Set(Speed,BASEDUTYUP);
            M1Cmd = CmdNull;
            M2Cmd = CmdNull;
            SysCmd = CmdNull;
            M1Dir = 0;
            M2Dir = 0;
            InvalidFlag = 1;
          }            
          else if(InvalidFlag == 2) //����ȥ����Ч��
          {
            InvalidCnt = 0;        
          }
        }
        else if(SaveState == BEGIN)//���ڴ洢       //BEGIN  = 1  
        {
          SavePosition = M1;        //�洢��һ��λ��
          SaveState = CONFIRM;
          Menu1Flag =4;//�洢λ��ȷ��
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x01)              //��һ���洢λ�ô���
            {    
              HealthModeReset();                    
              M1Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
              M2Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
              SysCmd = CmdToPreseting;
              PID_Set(Speed,BASEDUTYDOWN);
              Position = 0;
              InvalidFlag = 1;          
             }
          }
          else if(InvalidFlag == 2)
          {
            InvalidCnt = 0;
          }
        }    
      }
      break;
      
    case KEY_M2: //λ�ô洢2��
       if(((SysState != RESET)&&((Menu1Flag ==4)||(Menu1Flag ==0)))&&(Adjust_State==0))
      {
         // HealthModeReset();
        if(SysCmd == CmdToPreseting)
        {
          if(InvalidFlag==0)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            PID_Set(Speed,BASEDUTYDOWN);
            M1Cmd = CmdNull;
            M2Cmd = CmdNull;
            SysCmd = CmdNull;
            M1Dir = 0;
            M2Dir = 0;
            InvalidFlag = 1;
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }
        else if(SaveState == BEGIN)//���ڴ洢
        {
          SavePosition = M2; //�洢�ڶ���λ��
          SaveState = CONFIRM; //�洢λ��ȷ��  
          Menu1Flag =4;//�洢λ��ȷ��
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x02) //�ڶ����洢λ�ô���
            {       
                 HealthModeReset();             
                 M1Cmd = CmdToPreseting;
                 SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
                 M2Cmd = CmdToPreseting;
                 SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
                 SysCmd = CmdToPreseting;
                 PID_Set(Speed,BASEDUTYDOWN);
                 Position = 1;
                 InvalidFlag = 1;
             }
          }
          else if(InvalidFlag == 2)
          {
            InvalidCnt = 0;
          }
        }      
      }
      break;
      
    case KEY_M3: //λ�ô洢3��
       if(((SysState != RESET)&&((Menu1Flag ==4)||(Menu1Flag ==0)))&&(Adjust_State==0))
      {
          //HealthModeReset();
        if(SysCmd == CmdToPreseting)
        {
          if(InvalidFlag == 0)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            PID_Set(Speed,BASEDUTYDOWN);
            M1Cmd = CmdNull;
            M2Cmd = CmdNull;
            SysCmd = CmdNull;
            M1Dir = 0;
            M2Dir = 0;
            InvalidFlag = 1;
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }
        else if(SaveState == BEGIN)//���ڴ洢
        {
          SavePosition = M3;
          SaveState = CONFIRM; //�洢λ��ȷ��
          Menu1Flag =4;//�洢λ��ȷ��
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x04) //�������洢λ�ô���
            { 
                HealthModeReset();
                M1Cmd = CmdToPreseting;
                SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
                M2Cmd = CmdToPreseting;
                SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
                SysCmd = CmdToPreseting;
                PID_Set(Speed,BASEDUTYDOWN);
                Position = 2;
                InvalidFlag = 1;
             }
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }       
      }
      break;
    
      
    case KEY_M4: //λ�ô洢4��
      if((SysState != RESET)&&((Menu1Flag ==4)||(Menu1Flag ==0)))
      {
        if(SysCmd == CmdToPreseting)
        {
          if(InvalidFlag == 0)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            PID_Set(Speed,BASEDUTYDOWN);
            M1Cmd = CmdNull;
            M2Cmd = CmdNull;
            SysCmd = CmdNull;
            M1Dir = 0;
            M2Dir = 0;
            InvalidFlag = 1; 
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }
        else if(SaveState == BEGIN)//���ڴ洢
        {
          SavePosition = M4;
          SaveState = CONFIRM; //�洢λ��ȷ��
         
          Menu1Flag =4;//�洢λ��ȷ��
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x08) //���ĸ��洢λ�ô���
            {
              HealthModeReset();
              Position = 5;
              M1Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
              M2Cmd = CmdToPreseting;
              SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
              SysCmd = CmdToPreseting;
              PID_Set(Speed,BASEDUTYDOWN);
              InvalidFlag = 1;
            }
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }       
      }
      break;
    
    case KEY_SPECPOS: //APP�ܵ�ָ��λ��ָ��
      if((SysState != RESET)&&(Adjust_State==0))
      {
        if(SysCmd == CmdToPreseting)
        {
          if(InvalidFlag == 0)
          {
            M1Up(OFF);
            M1Down(OFF);
            M1_PWM_OFF;
            M2Up(OFF);
            M2Down(OFF);
            M2_PWM_OFF;
            PID_Set(Speed,BASEDUTYDOWN);
            M1Cmd = CmdNull;
            M2Cmd = CmdNull;
            SysCmd = CmdNull;
            M1Dir = 0;
            M2Dir = 0;
            InvalidFlag = 1;
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }       
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {  
            
            HealthModeReset();
            Position = 4;
            M1Cmd = CmdToPreseting;
            SetTIM1_PWMDuty(1,1350-BASEDUTYDOWN);
            M2Cmd = CmdToPreseting;
            SetTIM1_PWMDuty(2,1350-BASEDUTYDOWN);
            SysCmd = CmdToPreseting;
            PID_Set(Speed,BASEDUTYDOWN);
            InvalidFlag = 1;          
          }
          else if(InvalidFlag == 2)
            InvalidCnt = 0;
        }       
      }
      break;
      
    case KEY_MENU: 
     if((SysState != RESET)&&(Adjust_State==0))
      {
       DisplayMode = MenuMode;
       //Menu2Flag = 1;  
       Menu1Flag = 2;
       Menu2Num = 2;
       MenuTime = 0;
       DisplayRemind = OFF;
       Check_EN=2;
       //HealthModeReset();
      }
     ;   
    break;            
              
    case KEY_MEM: //�洢��
       
      if((SysState != RESET))
      {
        if(SysCmd == CmdToPreseting)
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          PID_Set(Speed,BASEDUTYDOWN);
          M1Cmd = CmdNull;
          M2Cmd = CmdNull;
          SysCmd = CmdNull;
          M1Dir = 0;
          M2Dir = 0;
        }
       
        else if((Menu1Flag == 1))
        {
          Menu1Flag =2;         
          SaveState = NULL;
          MenuTime = 0;
          InvalidFlag = 0;

          if ((UpKeyDelay >= 400) && (MenuKeyM == 0) )
          {
             MenuKeyM = 1;
                        
             UpKeyDelay = 0;
           }     
        
        }
        else if(((Menu1Flag ==2)&&(Menu2Num==2))||((Menu1Flag ==2)&&(Menu2Num==1)))
        {
          Menu1Flag =3;         
          SaveState = NULL;
          MenuTime = 0;
          InvalidFlag = 0;

         if ((UpKeyDelay >= 400) && (MenuKeyM == 0) )
            {
             MenuKeyM = 1;
                        
             UpKeyDelay = 0;
             }     
        }
        else if(Adjust_State == 2)    
        {
        SaveState = NULL;
        MenuTime = 0;
        InvalidFlag = 0;

         if ((UpKeyDelay >= 400) && (MenuKeyM == 0) )
            {
              MenuKeyM = 1;
                        
              UpKeyDelay = 0;
             }     
        }
        else if((SysCmd != CmdToPreseting)&&(KeyMValid!=2)&&(Adjust_State==0))
        {     
          SaveState = 1;
          Menu1Flag =4;
          MemCnt = 0;
          MenuTime = 0;
          if(HealthMode==1)
          {
            HealthMode = 4;
          }
          HealthModeReset();
          DisplayMode = SaveMode;
          Dis_Char[0] = Char_S;
          Dis_Char[1] = Char_;
          Dis_Char[2] = 0;
        }      
      }
      break;
    
      
    case KEY_A: //�Զ����Ѱ���
      if((SysState != RESET)&&(Adjust_State==0))
      {   Menu1Flag =0;
          Check_EN=0;
          BuzzerState = OFF;
        if(SysCmd == CmdToPreseting)
        {
          M1Up(OFF);
          M1Down(OFF);
          M1_PWM_OFF;
          M2Up(OFF);
          M2Down(OFF);
          M2_PWM_OFF;
          PID_Set(Speed,BASEDUTYDOWN);
          M1Cmd = CmdNull;
          M2Cmd = CmdNull;
          SysCmd = CmdNull;
          M1Dir = 0;
          M2Dir = 0;        
        }
       
        else if(KeyAValid!=2)   //KeyAValid = 0;  //A����Ч��־ ��ʼΪ0
        {
          HealthModeON();
          DisplayRemind=ON;
        }     
      }
      break;
      
    case KEY_A_LONG:
      if((SysState != RESET)&&(KeyMValid!=2)&&(Adjust_State==0))
      {
        T5secCnt = 0;
        if(HealthMode != 5)
        {
          HealthModeOFF();
        }
      }
      break;   
    
    case KEY_RST: //��λ��ϼ�M+3
   
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
      memcpy(&Buffer[73],&AccDataBag.Y_Offset_Spec,sizeof(AccDataBag.Y_Offset_Spec));
                     
       
      memcpy(&Buffer[75],&AccDataBag.Y_OffsetFlag_Spec,sizeof(AccDataBag.Y_OffsetFlag_Spec)); 
      EnableHPSlopeFilter();
     //Y_Off_EN_Fleg=0;  
     // DisableHPSlopeFilter(); 
     // Clear_AccDateBuffer();
      EEPROM_Write();
      
    
    
    
    
      /*  ��λ�����ϳ���
      memset(&Buffer[0], 0, sizeof(Buffer));
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
      EEPROM_Write();
      InitFlag = 1;
      BuzzerState = ON;    
      */
        InitCnt = 0;
        InitFlag = 1;
        //BuzzerWorkMode = 2;
        BuzzerState = ON; 
      break;
    
    case KEY_VER: //�汾��ʾ 
        if((Adjust_State==0))   
        {
         if( DisplayMode != TestMode) DisMode=DisplayMode; 
         if(VER_Cnt<2000)  
         {
         Dis_Char[0] =Data_Char[8];
         Dis_Char[1] =Data_Char[0];
         Dis_Char[2] =Data_Char[9];
         CombinationKey = 2;
         DisplayMode = TestMode;
         }
         else if(VER_Cnt<4000)
         {
         Dis_Char[0] =Char_L;
         Dis_Char[1] =Data_Char[1]|Char_Dot;
         Dis_Char[2] =Data_Char[3];
         CombinationKey = 2;
         DisplayMode = TestMode;
         
         }
         else if(VER_Cnt<6000)
         {
          CombinationKey=6;
         }
         else 
         {
         //DisplayMode=HeightMode;
         CombinationKey = 0;
         DisplayMode = DisMode;
         }
        }
         
    break;
      /*
    case KEY_VER: //�汾��ʾ
      KeyAValid = 1;
      KeyMValid = 1;
      if(CombinationKey==0) //��ʾ��λ����͸߶�
      {
        Dis_Char[0] = MainBoardSoftNumChar1;
        Dis_Char[1] = MainBoardSoftNumChar2;
        Dis_Char[2] = MainBoardSoftNumChar3;
        CombinationKey = 2;
        DisplayMode = TestMode;
      }
      else if(CombinationKey==3)
      {
        Dis_Char[0] = MainBoardSoftVerChar1;
        Dis_Char[1] = MainBoardSoftVerChar2;
        Dis_Char[2] = MainBoardSoftVerChar3;
        DisplayMode = TestMode;
        T2SecondCnt = 0;
        CombinationKey = 4;
      }
      else if(CombinationKey==5)
      {
        T2SecondCnt = 0;
        CombinationKey = 6;
      }
      break;
      
    case KEY_VER2: //�汾��ʾ
      KeyAValid = 1;
      KeyMValid = 1;
      if(CombinationKey==0) //��ʾ��λ����͸߶�
      {
        Dis_Char[0] = MainBoardSoftNumChar1;
        Dis_Char[1] = MainBoardSoftNumChar2;
        Dis_Char[2] = MainBoardSoftNumChar3;
        CombinationKey = 2;
        DisplayMode = TestMode;
      }
      else if(CombinationKey==3)
      {
        Dis_Char[0] = MainBoardSoftVerChar1;
        Dis_Char[1] = MainBoardSoftVerChar2;
        Dis_Char[2] = MainBoardSoftVerChar3;
        DisplayMode = TestMode;
        T2SecondCnt = 0;
        CombinationKey = 4;
      }
      else if(CombinationKey==5)
      {
        T2SecondCnt = 0;
        CombinationKey = 6;
      }
      break;
    */  
    case KEY_SENS:     
      
      if((sensKeyFlag==0)&&(Adjust_State==0))
      {
         sensTimer=0;
         InvalidFlag=1;
         sensKeyFlag=1;
         if(SysCmd!=CmdNull)
         {
           M1Up(OFF);
           M1Down(OFF);
           M1_PWM_OFF;
           M2Up(OFF);
           M2Down(OFF);
           M2_PWM_OFF;
           PID_Set(Speed,BASEDUTYDOWN);
           M1Cmd = CmdNull;
           M2Cmd = CmdNull;
           SysCmd = CmdNull;
           M1Dir = 0;
           M2Dir = 0;
         }
      }
      else if((sensKeyFlag==1)&&(Adjust_State == 0))
      {
         if(sensTimer>5050)
         {
            if(LSM6DSLFlag==1)
            {
               if(Sensitivity > 1)
               {
                 Sensitivity--;
               }
               else
               {
                 //Sensitivity = SENS;
                 Sensitivity = 3;
               }
               
               if((ErrCode==Err_LSM6DSL)&&(Sensitivity==1))
               {
                  ErrCode = Err_RESET;
                  memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));
               }
               
               Buffer[48] = Sensitivity;

               EEPROM_Write();
              // DisplayRemind=OFF;
               Dis_Char[0]=Char_R;
               Dis_Char[1]=Char_;
               Dis_Char[2]=Data_Char[Sensitivity];
               DisplayMode = Test_Sens;
           }
           sensKeyFlag=2;
         }
      }
      break;
      
     
      /*
      //���븴λ����ǰ�����ݱ���
      if((InitFlag == 0) && (SysState != RESET))
      {
        
//        memset(&Buffer[0], 0, sizeof(Buffer));
//        Buffer[33] = INITIALIZED;
//        Buffer[35] = RELEASE;
//        Buffer[36] = Unit;
//        Buffer[34] = TIMEVAL;
//        memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));
//        memcpy(&Buffer[41],&DiffHall, 2);
//        memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));
//        Buffer[47] = Speed;
//        Buffer[48] = Sensitivity;
//        memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));
//        memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));
//      
//        Buffer[61] = Balance_Data.TwoMotorRunFlag;
//      
//        memcpy(&Buffer[64],&AccDataBag.Y_Offset,sizeof(AccDataBag.Y_Offset));
//        memcpy(&Buffer[66],&AccDataBag.Y_OffsetFlag,sizeof(AccDataBag.Y_OffsetFlag));
//        
//        EEPROM_Write();
        
        InitCnt = 0;
        InitFlag = 1;
        //BuzzerWorkMode = 2;
        BuzzerState = ON;  
      }
      break;
      */
      
    case KEY_NULL:
      if(RunCntFlag == 1)
      {
        RunCntFlag = 2;
        /*
        if(Adjust_State == 2)
            Balance_Data_Refresh();
        */
        RunCnt = 0;
      }
      else if(RunCntFlag == 0)  //���м����µ��ް���״̬������ʱ100ms����˲���
      {
        if(AntiCollisionState==0)
        {
          KeyNullProcess();             //����Ч��״̬��������ʱ���� 
          RunCntFlag = 0;
        }
      }
      
      if(Menu1Flag == 1)//�Ѿ�����˵�ģʽ
      {
        Menu2Num=2;        
      } 
      else if((Menu1Flag == 2))
      {
        if(Menu2Num == 2) 
        {
          if(MenuKeyUp == 1)    //���ϰ�������
          {
            Menu1Num++;         //�ò����������� ACT,dOT,CHE��ʾ֮����л�        
            MenuKeyUp = 0;
            if(Menu1Num > Menu1_Max - 1)
            {
                Menu1Num = 0;
            }
          }
          else if(MenuKeyDown==1)
          {
            MenuKeyDown = 0;
            if(Menu1Num<= 0)
            {
                Menu1Num = Menu1_Max;
            }
              Menu1Num--;
          }
        }
        else if(Menu2Num==1)
        {
          if(MenuKeyUp == 1)//���ϰ�������
          {
            Menu3Num++;
            MenuKeyUp = 0;
            if(Menu3Num > Menu3_Max - 1)
            {
                Menu3Num = 0;
            }
          }
          else if(MenuKeyDown==1)
          {
            MenuKeyDown = 0;
            if(Menu3Num<= 0)
            {
                Menu3Num = Menu3_Max;
            }
              Menu3Num--;
          }  
        }
      }
      else if(Menu1Flag == 3)
      {
        if(Menu2Num==2) 
        {
           if(MenuKeyM==1)
        {
        MenuKeyM=0;  
        switch(Menu1Num) 
        { 
          case 0:
            Balance_EN=1;
            //Check_EN=0;
            Rst_EN=0;
            break;
            
            case 1:
            Check_EN=1;
            Balance_EN=0;
            Rst_EN=0;
            if(Y_Off_EN_Fleg==0)
            {
                Y_Off_EN_Fleg=1;
                DisableHPSlopeFilter();
                Clear_AccDateBuffer();
            }      
            break;
            
            case 2:
            if(Adjust_State==0)
            {
              Adjust_State=1;
              DisplayMode=FLash_HeightMode;
              Menu1Flag=0;
            }
            break;
            
          default:break;
           }  
          }  
        }
        else if(Menu2Num==1)
        {
          if(Menu3Num == 0)
          {
            if(MenuKeyM == 1)
            {
              MenuKeyM = 0;  
            
              if(TapControlFlag==0)
              {
                TapControlFlag=1;
              }
            }
          }
          else if(Menu3Num==1)
          {
            if(MenuKeyM==1)
            {
              MenuKeyM=0;  
            
              if(TapControlFlag==1)
              {
                TapControlFlag=0;
              }
            }
          } 
        }
      }
      if( (Adjust_State==2) && ( DisplayMode == FLash_HeightMode))
      {    
        if(MenuKeyM == 1)
        {
          MenuKeyM=0;      
          Adjust_State=0;
          Balance_Data_Refresh();//��¼�������ǰHALLֵ
          DisplayMode = HeightMode;
          DisplayRemind=ON; 
          HealthModeReset() ; 
        } 
      }
      if(InvalidFlag == 0)              //�Ӽ��ſ���ʼ����ʱ500ms�����޼����£�������ϴΰ��µļ����йز������������
      {
        LastKey = 0;                    //��ʱ����֮������ϴεİ���ֵ
        if(sensKeyFlag != 0)
        {
           sensKeyFlag = 0;
           if(SysState == RESET)
           {
              DisplayMode = ErrorMode;
           }
           else if(SysState == NORMAL)
           {
              DisplayMode = HeightMode;
           }
        }
      }
      else if(InvalidFlag == 1)         //��������������֮��ʼ��ʱ
      {
        InvalidFlag = 2;
        InvalidCnt = 0;
      }
      
      if((SysCmd==CmdNull)&&(AntiCollisionState!=0)) 
      {
        AntiCollisionState = 0;
      }
      
      RSTFlag = 0;
      RSTCnt = 0;
      
      if(Release == 0)          //�޹�������½���NORMAL״̬
      {
        Release = 1;
        SysState = NORMAL;      //ϵͳ������������״̬
      }
      if(Release == 2)  //Release == 2;˵��ϵͳ�й��ϣ�������ɹ��Ϻ����Ӧ�����������
      {
        Release = 3;
        if((ErrCode!=Err_TimeEnd)&&(ErrCode!=Err_Overheating))  //ֻҪ����ϵͳ���»�ϵͳ��ʱ���еĹ���
         SysState = RESET;                                      //ϵͳ״̬���RESET����ʾ�û����и�λ����
      }
      
#ifdef NE26
      
      if(SysCmd == CmdToPreseting)
      {
        if((M1Dir==UP)&&(M2Dir==UP))
        {
          M1Dir = 0;
          M2Dir = 0;
          SysCmd = CmdUp;
          M1Cmd = CmdUp;
          M2Cmd = CmdUp;
          KeyNullProcess();
        }
        else if((M1Dir==DOWN)&&(M2Dir==DOWN))
        {
          M1Dir = 0;
          M2Dir = 0;
          SysCmd = CmdDown;
          M1Cmd = CmdDown;
          M2Cmd = CmdDown;
          KeyNullProcess();
        }
      }
#endif
      if(CombinationKey>1) //M+A��ϼ�����
      {
        CombinationKey = 0;
        T2SecondCnt = 0;
        if(SysState == RESET)
          DisplayMode = ErrorMode;
        else if(SysState == NORMAL)
          DisplayMode = HeightMode;
      }
      break;
  default:
      break;
  }
  if((KeyAValid==1)&&(KeyMValid==1)&&(Key!= KEY_VER))
  {
    KeyAValid = 2;
    KeyMValid = 2;
  }
}
Keyolder=Key;

}


/***********************************************************************************************************
* ��������: KeyNullProcess()
* �������: ��
* ����ֵ  : ��
* ��    ��: ����ȫ���ͷ�ʱ������
************************************************************************************************************/
void KeyNullProcess(void)
{
    if (Balance_Data.BalanceAdjuseState != 0)
    {
        if (M2Cmd == CmdUp)
        {
          M2Cmd = CmdUpRetard;
        }
        else 
        {
          M2Cmd = CmdDownRetard;
        }
        if (M1Cmd == CmdUp)
        {
          M1Cmd = CmdUpRetard;
        }
        else
        {
          M1Cmd = CmdDownRetard;
        }
        
        M1PID.SetSpeed = 12;//(MINSPEED+(temp - M1State.HallNow))/4;
        M2PID.SetSpeed = 12;//(MINSPEED+(temp - M2State.HallNow))/4;
        
        M1PID.LastDev = 0;
        M1PID.PrevDev = 0;
        M1PID.SyncDev = 0;
        M1State.HallLast = M1State.HallNow; 
        M1CurTemp1 = 0;
        M1CurTemp2 = 0;
        M1CurTemp3 = 0;
        M1CurMax = 0;
        M1Flag = 0;
        M1ADCCnt = 0;  
        M1CurFlag = 0;
        M1T3sCnt = 0;
        memset(&ADCBuffer1[0], 0, 40);
        
        M2PID.LastDev = 0;
        M2PID.PrevDev = 0;
        M2PID.SyncDev = 0;
        M2State.HallLast = M2State.HallNow;
        M2CurTemp1 = 0;
        M2CurTemp2 = 0;
        M2CurTemp3 = 0;
        M2CurMax = 0;
        M2ADCCnt = 0; 
        M2Flag = 0;
        M2CurFlag = 0;
        M2T3sCnt = 0;
        memset(&ADCBuffer2[0], 0, 40);
        
        CurFlag = 0;
        T100msCnt1 = 0;
        T100msCnt2 = 0;
        M1PID.Kp = 0.50;
        M1PID.Ki = 0.09;
        M1PID.Kd = 0.10;
        M2PID.Kp = 0.50;
        M2PID.Ki = 0.09;
        M2PID.Kd = 0.10;
        
        //SysCmd = CmdUpRetard;  
        Balance_Data.RetardM1Stop = 0;
        Balance_Data.RetardM2Stop = 0;
        
        Balance_Data.RetardM1StopCnt = 0;
        Balance_Data.RetardM2StopCnt = 0;
        //
        Balance_Data.BalanceAdjuseState = 0;   
   }
  
  else if((SysCmd==CmdUp)||(SysCmd==CmdDown))   
  //if((SysCmd==CmdUp)||(SysCmd==CmdDown) || (SysCmd == CmdSetBalance))   
  {
    if((SysState == RESET)) //��λ״̬ʱ�������ɿ�����ֹͣ����
    {
      SysCmd = CmdStop;
      M1Cmd = CmdStop;
      M1Up(OFF);
      M1Down(OFF);
      M2Up(OFF);
      M2Down(OFF);
      M1_PWM_OFF;
      M2Cmd = CmdStop;
      M2_PWM_OFF;
      SysState = RESET;  
      //Balance_Data_Refresh();
    }
    else if(SysState == NORMAL) //����������״̬�������ɿ��������ʱ������״̬
    { 
      if(SysCmd == CmdUp)      //����UP�ļ��ٵ�ֹͣ����
      {
        //temp = MAX(M1State.HallNow,M2State.HallNow);
        //FinalPosition = temp+MINSPEED*6;
        //if(FinalPosition > M1State.LimitUp)
          //FinalPosition = M1State.LimitUp;
        M1Cmd = CmdUpRetard;            //����ı�״̬
        M2Cmd = CmdUpRetard;
        M1PID.SetSpeed = 12;//(MINSPEED+(temp - M1State.HallNow))/4;
        M2PID.SetSpeed = 12;//(MINSPEED+(temp - M2State.HallNow))/4;
        
        M1PID.LastDev = 0;
        M1PID.PrevDev = 0;
        M1PID.SyncDev = 0;
        M1State.HallLast = M1State.HallNow; 
        M1CurTemp1 = 0;
        M1CurTemp2 = 0;
        M1CurTemp3 = 0;
        M1CurMax = 0;
        M1Flag = 0;
        M1ADCCnt = 0;  
        M1CurFlag = 0;
        M1T3sCnt = 0;
        memset(&ADCBuffer1[0], 0, 40);
        
        M2PID.LastDev = 0;
        M2PID.PrevDev = 0;
        M2PID.SyncDev = 0;
        M2State.HallLast = M2State.HallNow;
        M2CurTemp1 = 0;
        M2CurTemp2 = 0;
        M2CurTemp3 = 0;
        M2CurMax = 0;
        M2ADCCnt = 0; 
        M2Flag = 0;
        M2CurFlag = 0;
        M2T3sCnt = 0;
        memset(&ADCBuffer2[0], 0, 40);
        
        CurFlag = 0;
        T100msCnt1 = 0;
        T100msCnt2 = 0;
        M1PID.Kp = 0.50;
        M1PID.Ki = 0.09;
        M1PID.Kd = 0.10;
        M2PID.Kp = 0.50;
        M2PID.Ki = 0.09;
        M2PID.Kd = 0.10;
        SysCmd = CmdUpRetard;  
        RetardStopCnt = 0;      //
      }
      else if(SysCmd == CmdDown)
      {
        //temp = MIN(M1State.HallNow,M2State.HallNow);
        //FinalPosition = temp - MINSPEED*8;
       // if(FinalPosition < M1State.LimitDown)
          //FinalPosition = M1State.LimitDown;
        M1Cmd = CmdDownRetard;
        M2Cmd = CmdDownRetard;
        M1PID.SetSpeed = 12;//(MINSPEED+(M1State.HallNow-temp))/4;
        M2PID.SetSpeed = 12;//(MINSPEED+(M2State.HallNow-temp))/4;
        M1PID.LastDev = 0;
        M1PID.PrevDev = 0;
        M1PID.SyncDev = 0;
        M1State.HallLast = M1State.HallNow;
        M1CurTemp1 = 0;
        M1CurTemp2 = 0;
        M1CurTemp3 = 0;
        M1CurMax = 0;
        M1Flag = 0;
        M1ADCCnt = 0;
        M1CurFlag = 0;
        M1T3sCnt = 0;
        memset(&ADCBuffer1[0], 0, 40);
        M2PID.LastDev = 0;
        M2PID.PrevDev = 0;
        M2PID.SyncDev = 0;
        M2State.HallLast = M2State.HallNow;
        M2CurTemp1 = 0;
        M2CurTemp2 = 0;
        M2CurTemp3 = 0;
        M2CurMax = 0;
        M2ADCCnt = 0; 
        M2Flag = 0;
        M2CurFlag = 0;
        M2T3sCnt = 0;
        memset(&ADCBuffer2[0], 0, 40);
        CurFlag = 0;
        T100msCnt1 = 0;
        T100msCnt2 = 0;
        M1PID.Kp = 0.45;
        M1PID.Ki = 0.24;
        M1PID.Kd = 0.20;
        M2PID.Kp = 0.45;
        M2PID.Ki = 0.24;
        M2PID.Kd = 0.20;
        SysCmd = CmdDownRetard;
        RetardStopCnt = 0;                      //RetardStop״̬��ʱ800
      }
    }
  } 
}
/******************************************************************************************************/
/***********************************************************************************************************
* ��������: KeyRespondTest()
* �������: value, ����������ADCֵ
* ����ֵ  : ��
* ��    ��: ������Ӧ����
************************************************************************************************************/
void KeyRespondTest(u16 value)
{  
  Key = KeyScan(value);
  if(((ErrCode != 0)&&(Key!=KEY_DOWN))||(SysCmd!=CmdNull)) //�����ֻ�����¼���Ч
    Key = KEY_NULL;
  if((AgingTest!=0)&&(Key!=KEY_RST))
    Key = KEY_NULL;
  switch(Key)
  {
    case KEY_UP:  //���ϼ�
      if(SysState != RESET)
      {       
        if((SysCmd == CmdNull)&&(Release==1))
        {
          BuzzerState = OFF;
          BuzzerTest = 0;
          BuzzerTestCnt = 0;
          DisplayMode = HeightMode;
          SysCmd = CmdUp;
          M1Cmd = CmdUp;
          SetTIM1_PWMDuty(1, 1350-BASEDUTYUP); //���ó�ʼPWMռ�ձ�
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); //���ó�ʼPWMռ�ձ�        
          PID_Set(Speed,BASEDUTYUP);               
        }      
      }
      break;
      
    case KEY_DOWN: //���¼�
      if((ErrCode == 0)&&(Release == 1))
      {
        Release = 0;
        BuzzerState = ON;
        BuzzerTest = 1;
        BuzzerTestCnt = 0;
        DisplayMode = TestMode;
        Dis_Char[0] = Char_S;
        Dis_Char[1] = Char_T;
        Dis_Char[2] = Char_R;
      }
      else if(ErrCode!=0)
      {
        Release = 0;
        ErrCode=0;
        SysState = NORMAL;
        DisplayMode = HeightMode;
        M1State.HallNow = BOTTOM;
        M1State.HallLast = BOTTOM;
        M1State.LimitDown = BOTTOM;
        M1State.LimitUp = M1State.LimitDown + DIF_HALL;
        M2State.HallNow = BOTTOM;
        M2State.HallLast = BOTTOM;
        M2State.LimitDown = BOTTOM;
        M2State.LimitUp = M2State.LimitDown + DIF_HALL;
      }
      break;
      
    case KEY_M1: //λ�ô洢1��
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[1];
      break;
      
    case KEY_M2: //λ�ô洢2��
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[2];
      break;
      
    case KEY_M3: //λ�ô洢3��
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[3];
      break;
      
    case KEY_MEM: //�洢��
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0xff;
      Dis_Char[1] = 0xff;
      Dis_Char[2] = 0xff;
      break;
      
    case KEY_A: //�Զ����Ѱ���
      DisplayMode = TestMode;
      TempRes = (10240.0/ADCValue.Temperature)-10.0;
      TempRes = (-153.6845510733126+396.02175*pow(TempRes,-0.2817181))/(1.6458186+pow(TempRes,-0.2817181));
      if(ErrCode==0)
      {
        Disp_Temprature(TempRes);
      }
      break;
             
    case KEY_RST: //���״̬�л���M+3
      if(AgingTest == 0)
      {
        DisplayMode = TestMode;
        Dis_Char[0] = Char_T;
        Dis_Char[1] = Char_E;
        Dis_Char[2] = Char_S;
        AgingTest = 1;
        AgingTime = 0;
        AgingAllTime = 0;
        AgingTurnFlag = 1;
        AgingTurnTime = 0;
        M1TestPWMDuty = 200;
        M2TestPWMDuty = 200;
        InitFlag = 1;
        BuzzerState = ON;
        Release = 0;
        Buffer[39] = AgingTest;
        EEPROM_Write();
      }
      else if((AgingTest != 4)&&(Release==1))
      {
        AgingTest = 0;
        AgingTime = 0;
        AgingAllTime = 0;
        AgingTurnFlag = 1;
        AgingTurnTime = 0;
        memset(&Buffer[0], 0, sizeof(Buffer));
        Buffer[35] = RELEASE;
        EEPROM_Write();
        InitFlag = 1;
        BuzzerState = ON;
      }
      break;
            
    case KEY_NULL: 
      Release = 1;
      break;
  }
  
}


void Balance_Data_Refresh()
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
            
  Buffer[32] = SaveIndex;
}