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
//上按键标志位
u8 MenuKeyDown = 0;         //下按键标志位
u8 Menu1Flag = 0;           //进入一级菜单标志位     0：未进入    1：已进入
u8 Menu2Flag = 0;           //进入二级菜单标志位     0：未进入    1：已进入
u8 Menu2Num = 0;//当前显示的二级菜单的子菜单
u8 Menu1Num=0;
u8 Menu3Num=0;
u8 Menu2_0 = 0;             //二级菜单第0个数    0为off   1为ON
u8 Rst_EN=0;
u8 Check_EN=0;
u8 Balance_EN=0;
u16 VER_Cnt=0;
//extern u16 Acc_x,Acc_y,Acc_z;
u8 SaveState = 0;
u16 MemCnt = 0;
u8 SavePosition = 0; //存储的是第几个位置
u8 Position;
u16 Key;
u16 Keyolder=0;
u8 RSTFlag = 0;
u8 Release = 1;         //
u8 InvalidFlag = 0;     //按键无效标志位，0表示按键有效，非0表示按键无效
u16 InvalidCnt = 0;     //按键无效时间计数值
u16 LastKey = 0;
u8 InitFlag = 0;        //用于复位操作（RST）的标志位，同时还是复位操作时控制蜂鸣器的标志位
u8 BuzzerTest = 0;
double TempRes = 0;
u8 AgingTest = 0;
u8 KeyDisable = 0;
u8 CombinationKey = 0; //组合键标志位
u8 KeyAValid = 0;  //A键无效标志
u8 KeyMValid = 0;  //M键无效标志

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
* 函数名称: KeyScan()
* 输入参数: value  测得的按键的ADC值
* 返回值  : keytemp, 当前键值
* 功    能: 扫描按键，判定按键键值和状态
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
            if(Com3ReceiveBuffer[1] == 0x02) //判断是按键板控制还是智能端控制
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
            else if(Com3ReceiveBuffer[1] == 0x03)//APP控制
            {
               com3Link=LinkApp;
               Com3KeyValue = Com3ReceiveBuffer[2];
            }

            com3send_num = 0;

            if(Com3KeyValue == KEY12) //收到跑到指定高度命令
            {             
                dheight = (((u16)Com3ReceiveBuffer[4]) << 8) + Com3ReceiveBuffer[3];

                 if(dheight >= (BaseHeight * 10))//解码接收到的高度
                  {
                    dheight -= (BaseHeight * 10);

                    dheight = (u32)(dheight * RATE / 10); //

                    M1State. Record[4] = M1State.LimitDown + dheight;

                    M2State. Record[4] = M2State.LimitDown + dheight;
                    
                    
                    //Balance_Data.TwoMotorOffsetHall >= 0      //M1电机比M2电机高
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

                    M1State. Record[4] += 9;//补偿9个霍尔数
                    
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

                SendBuffer[1] = UNIT; //单位
                
                if(UNIT==0)
                {
                SendBuffer[2] = (u16)(MaxHeight*10) & 0xff; //最高高度低位

                SendBuffer[3] = (u16)(MaxHeight*10) >> 8; //最高高度高位

                SendBuffer[4] = (u16)(BaseHeight*10) & 0xff; //最低高度低位

                SendBuffer[5] = (u16)(BaseHeight*10) >> 8; //最低高度高位
                }
                else
                {
                SendBuffer[2] = (u16)(MaxHeight*10/2.54) & 0xff; //最高高度低位

                SendBuffer[3] = (u16)(MaxHeight*10/2.54) >> 8; //最高高度高位

                SendBuffer[4] = (u16)(BaseHeight*10/2.54) & 0xff; //最低高度低位

                SendBuffer[5] = (u16)(BaseHeight*10/2.54) >> 8; //最低高度高位
                }
                SendBuffer[6] = 0;	 //遇阻回退灵敏度预留

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
              Com3DeviceState = SLAVER;  //设备状态转为从状态，接收来自上位机的命令
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

        memset(&SendBuffer[0], 0, sizeof(SendBuffer));  //SendBuffer缓存清0操作

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

        PackageSendData(&SendBuffer[0], &cnt);  //根据通信协议，封装要发送的数据
        Uart1SendData(&SendBuffer[0], cnt);     //发送数据
        
        /*
        RST_Sat=RST-> SR&0x1f;
        SendBuffer[0] =RST_Sat|0xa0;
        Uart1SendData(&SendBuffer[0],1);
        */
        Com1UartState = WAIT;                   //进入等待外部数据状态

    }
    else if(Com1UartState == FINISHED)
    {
        if(UnpackReceivedData(&Com1ReceiveBuffer[0], &Com1ReceiveCount))        //解包接收到的数据并判断数据有效性 (CRC校验)
        {
          if(Com1ReceiveBuffer[1] != 0x04)
          {
            if(Com1ReceiveBuffer[1] == 0x02)    //判断是按键板控制还是智能端控制
            {
               com1Link = LinkKey;              //判断串口1端口处接的是按键板，com1Link参数初始化时LinkNull
               if(Com1ReceiveCount==5)
               {
                 Com1KeyValue = Com1ReceiveBuffer[2];
               }
               else
               {
                 Com1KeyValue = Com1ReceiveBuffer[3]<<8|Com1ReceiveBuffer[2];
               }
            }
            else if(Com1ReceiveBuffer[1] == 0x03)//APP控制
            {
               com1Link = LinkApp;
               Com1KeyValue = Com1ReceiveBuffer[2];
            }

            com1send_num = 0;

            if(Com1KeyValue == KEY12) //收到跑到指定高度命令
            {             
                dheight = (((u16)Com1ReceiveBuffer[4]) << 8) + Com1ReceiveBuffer[3];

		  if(dheight >= (BaseHeight * 10))//解码接收到的高度
                  {
                    dheight -= (BaseHeight * 10);

                    dheight = (u32)(dheight * RATE/10); //

                    M1State.Record[4] = M1State.LimitDown + dheight;

                    M2State.Record[4] = M2State.LimitDown + dheight; 
                    
                    
                    //Balance_Data.TwoMotorOffsetHall >= 0      //M1电机比M2电机高
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
                    
                    M1State.Record[4] += 9;//补偿9个霍尔数
                    
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
            else if(Com1KeyValue==KEY_DEVINFO)          //APP获取设备信息虚拟键
            {
                SendBuffer[0] = KEY_DEVINFO;

                SendBuffer[1] = UNIT; //单位

                if(UNIT==0)
                {
                  SendBuffer[2] = (u16)(MaxHeight*10) & 0xff;           //最高高度低位

                  SendBuffer[3] = (u16)(MaxHeight*10) >> 8;             //最高高度高位

                  SendBuffer[4] = (u16)(BaseHeight*10) & 0xff;          //最低高度低位

                  SendBuffer[5] = (u16)(BaseHeight*10) >> 8;            //最低高度高位
                }
                else
                {
                  SendBuffer[2] = (u16)(MaxHeight*10/2.54) & 0xff;        //最高高度低位

                  SendBuffer[3] = (u16)(MaxHeight*10/2.54) >> 8;          //最高高度高位

                  SendBuffer[4] = (u16)(BaseHeight*10/2.54) & 0xff;       //最低高度低位

                  SendBuffer[5] = (u16)(BaseHeight*10/2.54) >> 8;         //最低高度高位
                }

                SendBuffer[6] = 0;	 //遇阻回退灵敏度预留

                cnt = 7;

                PackageSendData(&SendBuffer[0], &cnt);

                Uart1SendData(&SendBuffer[0], cnt);
                
                Com1KeyValue = KEY_NULL;
            }
            else if(Com1KeyValue == KEY_ANTI)
            {
              if(((SysCmd == CmdDown)||((M1Dir==DOWN)&&(M2Dir==DOWN)&&(M1Cmd!=CmdGoBack)&&(M2Cmd!=CmdGoBack)))) //将组回退
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
              Com1DeviceState = SLAVER;         //设备状态转为从状态，接收来自上位机的命令
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
* 函数名称: KeyScan()
* 输入参数: value  测得的按键的ADC值
* 返回值  : keytemp, 当前键值
* 功    能: 扫描按键，判定按键键值和状态
************************************************************************************************************/
u16 Com3KeyScan(u16 value)
{
  static u16 Com3KeyState = STATE_NULL;
  static u16 Com3WobbleCnt = 0; //消抖计数
  static u16 com3keytemp = 0;
  u16 key;
  
  key = Com3GetValue(value);    //获取按键值
  
  switch(Com3KeyState)
  {
    case STATE_NULL:    //STATE_NULL = 0
      if(key != KEY_NULL) //有按键按下
      {
        com3keytemp = key;
        Com3KeyState = STATE_WOBBLE; //进入消抖状态   //STATE_WOBBLE = 1
      }
      else
      {
        com3keytemp = 0;
      }
      break;
    
    case STATE_WOBBLE:          //消抖状态
      if(key == com3keytemp)    //还是同一个键被按下
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
        Com3KeyState = STATE_NULL;   //消抖没有通过，状态回归初始态
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
      
    case STATE_PRESS: //按键按下已经确认状态
      if(key==com3keytemp)//按键没有松开
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
      else if((key == KEY_NULL)&&(Downflag==0))  //长按检测没有通过则回到起始态
      {
        Com3LongCnt = 0;
        Com3KeyState = STATE_NULL;
        return (com3keytemp|KEYPRESS);
      }
      else if((key == KEY_NULL)&&(Downflag==1))  //长按检测没有通过则回到起始态
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
函数功能：实现按键的消抖功能
输入：u16 value 但此参数不起作用
输出：u16型参数，输出接收到的按键值 
****************************************************************/
u16 Com1KeyScan(u16 value)
{
  static u16 Com1KeyState = STATE_NULL;
  static u16 Com1WobbleCnt = 0; //消抖计数
  static u16 com1keytemp = 0;
  u16 key;
  
  key = Com1GetValue(value);    //通过串口获取按键值
  switch(Com1KeyState)
  {
    case STATE_NULL:
      if(key != KEY_NULL) //有按键按下
      {
        com1keytemp = key;
        Com1KeyState = STATE_WOBBLE; //进入消抖状态
      }
      else
      {
        com1keytemp = 0;
      }
      break;
    
    case STATE_WOBBLE: //消抖状态
      if(key == com1keytemp) //还是同一个键被按下
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
        Com1KeyState = STATE_NULL;   //消抖没有通过，状态回归初始态
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
      
    case STATE_PRESS:                   //按键按下已经确认状态
      if(key == com1keytemp)            //按键没有松开
      {
        if(Com1LongCnt >= 3000)
        {
          Com1LongCnt = 0;
          Com1KeyState = STATE_LONG;
          return (com1keytemp|KEYLONG);
        }
      }
      else if(key == KEY_NULL)          //长按检测没有通过则回到起始态
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
   com1key=Com1KeyScan(value);  //获取串口1的按键值（附加了消抖功能）
   com3key=Com3KeyScan(value);  //获取串口3的按键值（附加了消抖功能）
   
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
* 函数名称: KeyRespond()
* 输入参数: value, 该参数无实际意义
* 返回值  : 无
* 功    能: 按键响应函数
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
    Key = 0x99;                         //无效
    Menu1Flag=0;//修改的，长按灵敏度按键后，快速点按A键，在按上下键没响应的问题
    Menu2Flag=0;
    Menu1Num=0;
    Menu2Num=0;
    Menu3Num=0;
    
    
  }
  if((ErrCode == Err_TimeEnd)||(ErrCode == Err_Overheating))    //运行时间到(连续运行2分钟) 和电机过热情况下，按键无效
  {
    Key = KEY_NULL;
  }
  if((ErrCode == Err_LSM6DSL)&&(Key != KEY_SENS))               //陀螺仪故障且按键为非调整灵敏性的按键，则按键无效
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
      M1Cmd = CmdStop;    //电机进入停止状态           
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
    case KEY_UP:  //向上键
       
      if((SysState != RESET)&&(AntiCollisionState == 0))        //SysState = NORMAL
      {       
        if(SysCmd == CmdToPreseting) //(先判断是否需要急停)   //运动到预设的位置命令
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
          
        }//正常刚进入UP键功能
        else if(((SysCmd == CmdNull)&&(Release==1))||(Adjust_State==2)/*||(LastKey==KEY_DOWN)*/)   //Release = 1，系统处于空闲状态  
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
              //DeleteSavedHeight();                            //SaveFlag = 0;删除两个电机当前位置EEPROM中的存储值（M1State.HallNow，M2State.HallNow）
           // HealthModeReset();  
            //重置健康模式的计时
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
                if (hall_diff < MAX_HALL_DIFF)  //最大40cm高度差
                {
                  DeleteSavedHeight(); 
                    
                  SysCmd = CmdUp;
                            //从定时器方面，直接设置初始PWM占空比
                  M2Cmd = CmdUp;
                  SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); 
                  PID_Set(8,BASEDUTYUP);                      //设置PID参数，包括给到转速，PID方面初始PWM占空比
                  M2PID.SetSpeed = MINSPEED;
            
                  if(InvalidFlag == 0)                            //此标志位表示是否有按键值，为0这说明无按键，
                    InvalidFlag = 1;
            
                  RunCntFlag = 1;                                 //运行标志位
            
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
              }           //从定时器方面，直接设置初始PWM占空比 
            }
            else //正常按键操作
            {
              Release = 0;
              DeleteSavedHeight();                            //SaveFlag = 0;删除两个电机当前位置EEPROM中的存储值（M1State.HallNow，M2State.HallNow）
              HealthModeReset();                              //重置健康模式的计时
              SysCmd = CmdUp;
              M1Cmd = CmdUp;
              SetTIM1_PWMDuty(1, 1350-BASEDUTYUP);            //从定时器方面，直接设置初始PWM占空比
          
              M2Cmd = CmdUp;
              SetTIM1_PWMDuty(2, 1350-BASEDUTYUP);            //从定时器方面，直接设置初始PWM占空比 
          
              PID_Set(Speed,BASEDUTYUP);                      //设置PID参数，包括给到转速，PID方面初始PWM占空比

              if(InvalidFlag == 0)                            //此标志位表示是否有按键值，为0这说明无按键，
              {
                InvalidFlag = 1;
              }
            
              RunCntFlag = 1;                                 //运行标志位   
              if(HealthMode == 4)
              {
                  HealthMode = 2;
                  T2sCnt = 1000;
              } 
            }     
          }
        }
        else if((LastKey == Key)&&(InvalidFlag == 2))       //在上次按向上键释放后短时间内再次按下向上键
        {
          InvalidFlag = 1;
          InvalidCnt = 0;                                   //短时间计时值重置          
        }
        
        if((SysCmd == CmdDown)&&(SysState==NORMAL))         //正在执行DOWN命令时，接收到UP命令
        {
          KeyNullProcess();
        }
      
        LastKey = Key;
      }
      break;
      
      case KEY_DOWN: //向下键
       
      if(AntiCollisionState==0) //没触发遇阻状态
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
        else if((Menu1Flag == 1)||((Menu1Flag == 2)&&(Menu2Num==2))||((Menu1Flag == 2)&&(Menu2Num==1)))//已经进入一级菜单模式
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
                  //if (hall_diff > 6515)     //最大40cm高度差
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
              else //正常按键操作
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
        else if((LastKey==Key)&&(InvalidFlag==2))       //在上次按向上键释放后短时间内再次按下向上键
        {
          InvalidFlag = 1;
          InvalidCnt = 0;  //短时间计时值重置          
        }
        if((SysCmd == CmdUp)&&(SysState==NORMAL))
        {
          KeyNullProcess();             //按键全部释放时处理函数
        }
        
        LastKey = Key;
      }
      break;
    
    case KEY_M1: //位置存储1键
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
          else if(InvalidFlag == 2) //按键去抖动效果
          {
            InvalidCnt = 0;        
          }
        }
        else if(SaveState == BEGIN)//正在存储       //BEGIN  = 1  
        {
          SavePosition = M1;        //存储第一个位置
          SaveState = CONFIRM;
          Menu1Flag =4;//存储位置确定
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x01)              //第一个存储位置存在
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
      
    case KEY_M2: //位置存储2键
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
        else if(SaveState == BEGIN)//正在存储
        {
          SavePosition = M2; //存储第二个位置
          SaveState = CONFIRM; //存储位置确定  
          Menu1Flag =4;//存储位置确定
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x02) //第二个存储位置存在
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
      
    case KEY_M3: //位置存储3键
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
        else if(SaveState == BEGIN)//正在存储
        {
          SavePosition = M3;
          SaveState = CONFIRM; //存储位置确定
          Menu1Flag =4;//存储位置确定
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x04) //第三个存储位置存在
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
    
      
    case KEY_M4: //位置存储4键
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
        else if(SaveState == BEGIN)//正在存储
        {
          SavePosition = M4;
          SaveState = CONFIRM; //存储位置确定
         
          Menu1Flag =4;//存储位置确定
          MenuTime = 0;
        }
        else if((SysCmd == CmdNull)&&(AntiCollisionState==0))
        {
          if(InvalidFlag == 0)
          {
            if(SaveIndex&0x08) //第四个存储位置存在
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
    
    case KEY_SPECPOS: //APP跑到指定位置指令
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
              
    case KEY_MEM: //存储键
       
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
    
      
    case KEY_A: //自动提醒按键
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
       
        else if(KeyAValid!=2)   //KeyAValid = 0;  //A键无效标志 初始为0
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
    
    case KEY_RST: //复位组合键M+3
   
      memset(&Buffer[0], 0, sizeof(Buffer));
      //复位之前有用数据保存操作
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
      
    
    
    
    
      /*  复位操作老程序
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
    
    case KEY_VER: //版本显示 
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
    case KEY_VER: //版本显示
      KeyAValid = 1;
      KeyMValid = 1;
      if(CombinationKey==0) //显示单位和最低高度
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
      
    case KEY_VER2: //版本显示
      KeyAValid = 1;
      KeyMValid = 1;
      if(CombinationKey==0) //显示单位和最低高度
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
      //进入复位操作前的数据保存
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
      else if(RunCntFlag == 0)  //从有键按下到无按键状态后再延时100ms进入此操作
      {
        if(AntiCollisionState==0)
        {
          KeyNullProcess();             //非无效键状态才允许延时减速 
          RunCntFlag = 0;
        }
      }
      
      if(Menu1Flag == 1)//已经进入菜单模式
      {
        Menu2Num=2;        
      } 
      else if((Menu1Flag == 2))
      {
        if(Menu2Num == 2) 
        {
          if(MenuKeyUp == 1)    //向上按键处理
          {
            Menu1Num++;         //该参数用来进行 ACT,dOT,CHE显示之间的切换        
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
          if(MenuKeyUp == 1)//向上按键处理
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
          Balance_Data_Refresh();//记录两电机当前HALL值
          DisplayMode = HeightMode;
          DisplayRemind=ON; 
          HealthModeReset() ; 
        } 
      }
      if(InvalidFlag == 0)              //从键放开开始，延时500ms后仍无键按下，则进入上次按下的键的有关参数的清零操作
      {
        LastKey = 0;                    //延时减速之后清除上次的按键值
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
      else if(InvalidFlag == 1)         //按键按下再松手之后开始计时
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
      
      if(Release == 0)          //无故障情况下进入NORMAL状态
      {
        Release = 1;
        SysState = NORMAL;      //系统进入正常运行状态
      }
      if(Release == 2)  //Release == 2;说明系统有故障，并已完成故障后的相应参数清零操作
      {
        Release = 3;
        if((ErrCode!=Err_TimeEnd)&&(ErrCode!=Err_Overheating))  //只要不是系统过温或系统过时运行的故障
         SysState = RESET;                                      //系统状态设成RESET，提示用户进行复位操作
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
      if(CombinationKey>1) //M+A组合键松手
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
* 函数名称: KeyNullProcess()
* 输入参数: 无
* 返回值  : 无
* 功    能: 按键全部释放时处理函数
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
    if((SysState == RESET)) //复位状态时，按键松开立即停止运行
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
    else if(SysState == NORMAL) //在正常运行状态，按键松开则进入延时并减速状态
    { 
      if(SysCmd == CmdUp)      //进入UP的减速到停止操作
      {
        //temp = MAX(M1State.HallNow,M2State.HallNow);
        //FinalPosition = temp+MINSPEED*6;
        //if(FinalPosition > M1State.LimitUp)
          //FinalPosition = M1State.LimitUp;
        M1Cmd = CmdUpRetard;            //命令改变状态
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
        RetardStopCnt = 0;                      //RetardStop状态延时800
      }
    }
  } 
}
/******************************************************************************************************/
/***********************************************************************************************************
* 函数名称: KeyRespondTest()
* 输入参数: value, 采样按键的ADC值
* 返回值  : 无
* 功    能: 按键响应函数
************************************************************************************************************/
void KeyRespondTest(u16 value)
{  
  Key = KeyScan(value);
  if(((ErrCode != 0)&&(Key!=KEY_DOWN))||(SysCmd!=CmdNull)) //出错后只有向下键有效
    Key = KEY_NULL;
  if((AgingTest!=0)&&(Key!=KEY_RST))
    Key = KEY_NULL;
  switch(Key)
  {
    case KEY_UP:  //向上键
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
          SetTIM1_PWMDuty(1, 1350-BASEDUTYUP); //设置初始PWM占空比
          M2Cmd = CmdUp;
          SetTIM1_PWMDuty(2, 1350-BASEDUTYUP); //设置初始PWM占空比        
          PID_Set(Speed,BASEDUTYUP);               
        }      
      }
      break;
      
    case KEY_DOWN: //向下键
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
      
    case KEY_M1: //位置存储1键
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[1];
      break;
      
    case KEY_M2: //位置存储2键
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[2];
      break;
      
    case KEY_M3: //位置存储3键
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0;
      Dis_Char[1] = 0;
      Dis_Char[2] = Data_Char[3];
      break;
      
    case KEY_MEM: //存储键
      BuzzerState = OFF;
      BuzzerTest = 0;
      BuzzerTestCnt = 0;
      DisplayMode = TestMode;
      Dis_Char[0] = 0xff;
      Dis_Char[1] = 0xff;
      Dis_Char[2] = 0xff;
      break;
      
    case KEY_A: //自动提醒按键
      DisplayMode = TestMode;
      TempRes = (10240.0/ADCValue.Temperature)-10.0;
      TempRes = (-153.6845510733126+396.02175*pow(TempRes,-0.2817181))/(1.6458186+pow(TempRes,-0.2817181));
      if(ErrCode==0)
      {
        Disp_Temprature(TempRes);
      }
      break;
             
    case KEY_RST: //软件状态切换键M+3
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