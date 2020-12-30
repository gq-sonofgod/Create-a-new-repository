#include "LED.h"
#include "Delay.h"
#include "Motor.h"
#include "stdio.h"
#include "Main.h"
#include "FaultDetection.h"
#include "Key.h"
#include "Uart.h"
#include "string.h"
#include "Timer.h"
#include "Main.h"
#include "Balance.h"
#include "Tap.h"
#include "EEPROM.h"
u32 LEDRest = 0;   //LED关闭计数值
u8 Dis_Char[3] = {0,0,0};
const u8 Data_Char[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
const u8 SelfCheckCmd[7] = {0x9B, 0x05, 0x88, 0x99, 0xAB, 0xC7, 0x9D};
u16 Flash_Cnt=0;
u8 DispFlag = 0;
u8 DisplayMode;
u8 ledFlag = 0;         //这个标志位的意思是对两个串口轮流进行操作
u8 Y_Off_EN_Fleg=0;
u8 Adjust_State=0;
/***********************************************************************************************************
* 函数名称: LED_Init()
* 输入参数: 无
* 返回值  : 无
* 功    能: LED 驱动初始化
************************************************************************************************************/
/*void LED_Init(void)
{
  GPIO_Init(GPIOE,GPIO_PIN_3,GPIO_MODE_OUT_OD_HIZ_FAST ); //SDA
  GPIO_Init(GPIOG,GPIO_PIN_0,GPIO_MODE_OUT_OD_HIZ_FAST ); //SCL
  GPIO_Init(GPIOG,GPIO_PIN_1,GPIO_MODE_IN_FL_NO_IT); //独立按键M键
  GPIO_Init(GPIOE,GPIO_PIN_0,GPIO_MODE_IN_FL_NO_IT); //独立按键A键
  TM1650_Set(0x48,0x01);
}

//I2C开始
void IIC_Start(void)
{
  SCL_H;
  SDA_H;
  Delay_us(4);
  SDA_L;
  Delay_us(4);
  SCL_L;
}

//I2C停止
void IIC_Stop(void)
{
  SDA_L;
  Delay_us(4);
  SCL_H;
  Delay_us(4);
  SDA_H;
  Delay_us(4);
  SCL_L;
  SDA_L;
}

//I2C写字节
void IIC_WriteByte(u8 data)
{
  u8 i;
  SCL_L;
  for(i = 0; i < 8; i++)
  {    
    if(data&0x80)
      SDA_H;
    else
      SDA_L;
    Delay_us(4);
    data <<= 1;
    SCL_H;   
    Delay_us(4);
    SCL_L;
  } 
  SDA_H;
  Delay_us(4);
  SCL_H;
  Delay_us(5);
  SCL_L;
  Delay_us(2);
}
*/

/***********************************************************************************************************
* 函数名称: TM1650_Set()
* 输入参数: addr， 要写入TM1650的地址；data，要写入的数据
* 返回值  : 无
* 功    能: 向TM1650指定地址写入数据
************************************************************************************************************/
/*void TM1650_Set(u8 addr, u8 data)
{
  IIC_Start();
  IIC_WriteByte(addr);
  IIC_WriteByte(data);
  IIC_Stop();
}*/

/***********************************************************************************************************
* 函数名称: DispNum()
* 输入参数: num, 要显示的数据
* 返回值  : 无
* 功    能: 显示数据，数据小于100，保留一位小数，否则整数显示,显示的数据必须小于1000
************************************************************************************************************/
void DispNum(float num)
{
  u16 i,j,k,m;
  u8  chartemp;

  if(num >= 100)
    num = (u16)((num+0.5)*10);
  else if(Unit==1)
    num = (u16)((num+0.05)*10);
  else
    num = (u16)((num)*10); 
  num = num/10;
  if(Unit == 0)
  {
    if(BaseHeight < 100)
    {
      if(num <BaseHeight)
        num = BaseHeight;
    }
    else
    {
      if(num <((u16)(BaseHeight+0.5)))
        num = (u16)(BaseHeight+0.5);
    }
    if(MaxHeight < 100)
    {
      if(num > MaxHeight)
        num = MaxHeight;
    }
    else
    {
      if(num > ((u16)(MaxHeight+0.5)))
        num = (u16)(MaxHeight+0.5);
    }
  }
  else if(Unit == 1)
  {
    if(num <(((u16)((BaseHeight/2.54)*10+0.5))/10.0f))
      num = ((u16)((BaseHeight/2.54)*10+0.5))/10.0f;
    if(num > ((u16)(((MaxHeight/2.54)*10+0.5))/10.0f))
      num = ((u16)(((MaxHeight/2.54)*10+0.5)))/10.0f;
  }
  num=num*100.0;
  j=10000;
  m=0;
  for(i=0;i<=4;i++)
  {
    chartemp=0;
    k=(u16)(num/j);
    if(k!=0)
       chartemp=Data_Char[k];
    else if((j==100)||(m!=0))
       chartemp=Data_Char[k];
    if((j==100)&&(m!=2))
       chartemp=chartemp|Char_Dot;
    if(chartemp!=0)
    {
        Dis_Char[m]=chartemp;
        m++;
    }
    if(m==3)
      break;
    num=num-j*k;
    j=j/10;
  }
}


/***********************************************************************************************************
* 函数名称: LED_Display()
* 输入参数: 无
* 返回值  : 无
* 功    能: LED显示函数
************************************************************************************************************/
void LED_Display(void)
{
  float Height,top,bottom;
  u8 Cnt;
  u16 ErrTemp;
  
  if(LEDRest < 10000)           //显示10s倒计时
  { 
    switch(DisplayMode)
    {
      case HeightMode:          //显示高度模式
        SaveState = NULL;
        MemCnt = 0;
        
        //top = (((float)M1State.HallNow)+((float)M2State.HallNow))/2;       
        
        //Balance_Data.TwoMotorOffsetHall >= 0      //M1电机比M2电机高
        if (Balance_Data.TwoMotorOffsetHall >= 0)
        {
          top = ((float)(M1State.HallNow) + (float)(M2State.HallNow) + Balance_Data.TwoMotorOffsetHall)/2;
          //bottom = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
        }
        else
        {
          top = ((float)(M1State.HallNow) + (float)(M2State.HallNow) - Balance_Data.TwoMotorOffsetHall)/2;
          //bottom = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;
        }
       
        bottom = (M1State.LimitDown + M2State.LimitDown)/2;
        //bottom = BOTTOM;    //18114
        if(top < bottom)
          top = bottom;
        Height = (top - bottom)/RATE + BaseHeight;  //RATE 为HALL脉冲数与实际设备高度之间的换算系数
#ifdef INCH
        Height = Height/2.54;
#endif
        DispNum(Height);
        break;
        
        
        case FLash_HeightMode:
        SaveState = NULL;
        MemCnt = 0;
        
        //top = (((float)M1State.HallNow)+((float)M2State.HallNow))/2;       
        
        //Balance_Data.TwoMotorOffsetHall >= 0      //M1电机比M2电机高
        if (Balance_Data.TwoMotorOffsetHall >= 0)
        {
          top = ((float)(M1State.HallNow) + (float)(M2State.HallNow) + Balance_Data.TwoMotorOffsetHall)/2;
          //bottom = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
        }
        else
        {
          top = ((float)(M1State.HallNow) + (float)(M2State.HallNow) - Balance_Data.TwoMotorOffsetHall)/2;
          //bottom = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;
        }
       
        //bottom = (M1State.LimitDown + M2State.LimitDown)/2;
        bottom = BOTTOM;    //18114
        if(top < bottom)
          top = bottom;
        Height = (top - bottom)/RATE + BaseHeight;  //RATE 为HALL脉冲数与实际设备高度之间的换算系数
#ifdef INCH
        Height = Height/2.54;
#endif  
        if(Flash_Cnt < 500)
        {
         Dis_Char[0] = Char_S;
         Dis_Char[1] = Char_E;
         Dis_Char[2] = Char_T;
        }
        else if (Flash_Cnt < 1000)
        {
          Dis_Char[0] = 0;
          Dis_Char[1] = 0;
          Dis_Char[2] = 0;
        }
        break;
        
      case SaveMode: //存储显示模式
        break;
        
      case RemindMode: //提醒模式显示
        break;
      
      case Test_Sens: //提醒模式显示
        Dis_Char[0]=Char_R;
        Dis_Char[1]=Char_;
        Dis_Char[2]=Data_Char[Sensitivity];
        
        break; 
      case ErrorMode: //错误显示
        if(ErrCode == 0xffff)
        {
          Dis_Char[0] = Char_R;
          Dis_Char[1] = Char_S;
          Dis_Char[2] = Char_T;
        }
        else if(ErrCode == Err_Fall)
        {
          Dis_Char[0] = Char_;
          Dis_Char[1] = Char_O;
          Dis_Char[2] = Char_L;
        }
        else
        {          
          ErrTemp = ErrCode;      
          Dis_Char[0] = Char_E;
          Dis_Char[1] = Data_Char[ErrTemp/10];
          Dis_Char[2] = Data_Char[ErrTemp%10];
        }
        break;
        
        case MenuMode://菜单选择模式
        if(Menu1Flag == 1)//进入一级菜单      //程序中没有Menu1Flag = 1设置
        {
          switch(Menu2Num) 
          {
            case 0:
            Dis_Char[0]=Char_P;
            Dis_Char[1]=Char_R;
            Dis_Char[2]=Char_E;   
            break;
           
            case 1:
            Dis_Char[0]=Char_T;
            Dis_Char[1]=Char_A;
            Dis_Char[2]=Char_P;    
            break;
           
            case 2:
            Dis_Char[0]=Char_b;
            Dis_Char[1]=Char_A;
            Dis_Char[2]=Char_L;    
            break;
           
            default:break;
          
          }
          if(Menu1Flag != 1)//进入二级菜单
          {
            switch(Menu2Num)
            {
            case 0:
                Dis_Char[0]=Char_O;
                Dis_Char[1]=Char_F;
                Dis_Char[2]=Char_F;
              break;
              
            case 1:
            case 2:
                  if(TapControlFlag == 0)
                  {
                    Tap_Parameter.TapCtlThreshold = 1600;//2600;
                    TapControlFlag = 1;
                    Buffer[70] = TapControlFlag; 
                    Buffer[72] =Tap_Parameter.TapCtlThreshold; 
                    Buffer[71] =Tap_Parameter.TapCtlThreshold>>8; 
                 
                    EEPROM_Write();
                    Dis_Char[0]=0;
                    Dis_Char[1]=Char_O;
                    Dis_Char[2]=Char_N;
                  }
                break;

            case 3:
                Dis_Char[0] = Char_S;
                Dis_Char[1] = Char_E;
                Dis_Char[2] = Char_T;
                break;
                
            case 4: 
            if ((SysState == NORMAL) && (M2Cmd == CmdNull) && (M1Cmd == CmdNull))
                 {
                   Uart1SendData((u8*)SelfCheckCmd, 7);
                   Uart3SendData((u8*)SelfCheckCmd, 7);
                   
                   Dis_Char[0] = Char_S;
                   Dis_Char[1] = Char_E;
                   Dis_Char[2] = Char_T;
                 }
              break;
            
              default:
                    break;
            }
          }
        }
        else if(Menu1Flag == 2) //长按M键后，Menu1Flag = 2,进入此段程序
        {
          switch(Menu2Num)      //长按M键后，Menu1Flag = 2,Menu2Num = 2进入此段程序
          {
          case 0:
           if ((SysState == NORMAL) && (M2Cmd == CmdNull) && (M1Cmd == CmdNull))
                 {
                   Uart1SendData((u8*)SelfCheckCmd, 7);
                   Uart3SendData((u8*)SelfCheckCmd, 7);
                   
                   Dis_Char[0] = Char_S;
                   Dis_Char[1] = Char_E;
                   Dis_Char[2] = Char_T;
                 }   
           break;
           
           case 1:
           switch(Menu3Num)
           {
               case 0:
               Dis_Char[0]=0;
               Dis_Char[1]=Char_O;
               Dis_Char[2]=Char_N;
               break;
               
               case 1:
               Dis_Char[0]=Char_O;
               Dis_Char[1]=Char_F;
               Dis_Char[2]=Char_F;
               break;  
               
             default:break;
           }
           break;
           
           case 2:      //进入这里执行
           
           switch(Menu1Num) 
           {
            case 0:  
              Dis_Char[0] = Char_A;
              Dis_Char[1] = Char_C;
              Dis_Char[2] = Char_T;      
            break;
           
            case 1: 
              Dis_Char[0] = Char_C;
              Dis_Char[1] = Char_H;
              Dis_Char[2] = Char_E;
            break;
            
            case 2:  
              Dis_Char[0] = Char_d;
              Dis_Char[1] = Char_O;
              Dis_Char[2] = Char_T; 
            break;
            
             default:break;
           }
           break;
           
            default:break;
          }
        } 
        else if(Menu1Flag == 3)         //长按M键后，又短按M键，进入Menu1Flag == 3模式
        {
          if(Menu2Num == 2)                 //长按M键后，Menu2Num == 2
          {
           switch(Menu1Num) 
           {
            case 0:     //显示ACT后，短按M键，进入自平衡执行阶段，显示SET
            if ((SysState == NORMAL) && (AntiCollisionState == 0) && (Balance_Data.BalanceAdjuseState == 0) &&
               (M2Cmd == CmdNull) && (M1Cmd == CmdNull))
                {     
                  Menu2_0 = 0;      
                  Tap_Parameter.TapControlEN = 0;
                  Tap_Parameter.TapControlFlag = 0;
                  SetBalaceState(1);
                  Balance_ON = 1;
                  Buffer[76]= Balance_ON;
                  EEPROM_Write();
                  DisableHPSlopeFilter();
                  Clear_AccDateBuffer();   
                }
              Dis_Char[0] = Char_S;
              Dis_Char[1] = Char_E;
              Dis_Char[2] = Char_T;
           break;
           
           case 1:      //显示CHE后，短按M键，进入自检阶段，显示SET
             Dis_Char[0] = Char_S;
             Dis_Char[1] = Char_E;
             Dis_Char[2] = Char_T; 
             
           break;       
           
           case 2:      //显示dOT后，短按M键，进入点动状态，显示SET
            Dis_Char[0] = Char_S;
            Dis_Char[1] = Char_E;
            Dis_Char[2] = Char_T;
           break;
           
             default:break;
           }
          }
          
          else if(Menu2Num==1)
          {
            if(TapControlFlag==0)
            {
              Buffer[70] = TapControlFlag;
              Buffer[71] =0; 
              Buffer[72] =0;  
              EEPROM_Write();
                  
              Dis_Char[0] = Char_S;
              Dis_Char[1] = Char_E;
              Dis_Char[2] = Char_T; 
            }
            else if(TapControlFlag==1)
            {
              Tap_Parameter.TapCtlThreshold = 1600;//2600;
                 
              Buffer[70] = TapControlFlag; 
              Buffer[72] =Tap_Parameter.TapCtlThreshold; 
              Buffer[71] =Tap_Parameter.TapCtlThreshold>>8; 
                 
              EEPROM_Write();
              Dis_Char[0] = Char_S;
              Dis_Char[1] = Char_E;
              Dis_Char[2] = Char_T;
           }
          }
        }  
        break;
        
      default:break;
    }
    
    if(CombinationKey != 6)
    {
      if(ledFlag == 0)  //这个标志位的意思是对两个串口轮流进行操作
      {
         if((com3Link != LinkNull)&&(Com3UartState == IDLE))
         {
           Com3UartState = SENDBUSY;
           memset(&SendBuffer[0], 0, sizeof(SendBuffer));
           SendBuffer[0] = DisplayLed;
           memcpy(&SendBuffer[1], &Dis_Char[0], 3);
           Cnt = 4;
           PackageSendData(&SendBuffer[0], &Cnt);
           Uart3SendData(&SendBuffer[0], Cnt);
           DispFlag = 0;
           Com3UartState = IDLE;
           ledFlag=1;
         }
         else if(com3Link == LinkNull)
         {
           DispFlag=0;
           ledFlag=1;
         }
      }
      else
      {
         if((com1Link!=LinkNull)&&(Com1UartState==IDLE))
         {
           Com1UartState = SENDBUSY;
           memset(&SendBuffer[0], 0, sizeof(SendBuffer));
           SendBuffer[0] = DisplayLed;
           
           memcpy(&SendBuffer[1], &Dis_Char[0], 3);
           Cnt = 4;
           PackageSendData(&SendBuffer[0], &Cnt);
           Uart1SendData(&SendBuffer[0], Cnt);
           DispFlag = 0;
           Com1UartState = IDLE;
           ledFlag=0;
         }
         else if(com1Link==LinkNull)
         {
           DispFlag=0;
           ledFlag=0;
         }
      }
    }
  }
  else if(HealthMode == 0)                //非健康模式下10秒钟无任何操作，则关闭显示
  {
    if(((com3Link == LinkNull)&&(com1Link == LinkKey))||((com3Link == LinkKey)&&(com1Link == LinkNull)))  
    {
       if((com3Link == LinkKey)&&(Com3UartState == IDLE))//非健康模式下，COM1,COM3两个串口中，一个接了KEY，另一个没接，则进入关电模式
       {
         Com3UartState = SENDBUSY;
         memset(&SendBuffer[0], 0, sizeof(SendBuffer));
         SendBuffer[0] = MainBoardPowerOff;
         Cnt = 1;              
         PackageSendData(&SendBuffer[0], &Cnt);
         Uart3SendData(&SendBuffer[0], Cnt);
         DispFlag = 0;
         Com3UartState = IDLE;
       }
       if((com1Link == LinkKey)&&(Com1UartState == IDLE))
       {
         Com1UartState = SENDBUSY;
         memset(&SendBuffer[0], 0, sizeof(SendBuffer));
        SendBuffer[0] = MainBoardPowerOff;
         Cnt = 1;        
         PackageSendData(&SendBuffer[0], &Cnt);
         Uart1SendData(&SendBuffer[0], Cnt);
         DispFlag = 0;
         Com1UartState = IDLE;
       }
    }
    else
    {
        if(ledFlag == 0)
        {
          if((com3Link == LinkKey)&&(Com3UartState == IDLE))
          {
            Com3UartState = SENDBUSY;
            memset(&SendBuffer[0], 0, sizeof(SendBuffer));
           
            SendBuffer[0] = DisplayLed;
            memcpy(&SendBuffer[1], &Dis_Char[0], 3);
            Cnt = 4;
            PackageSendData(&SendBuffer[0], &Cnt);
            Uart3SendData(&SendBuffer[0], Cnt);
            DispFlag = 0;
            Com3UartState = IDLE;
            ledFlag=1;
          }
          else if(com3Link != LinkKey)
          {
            DispFlag=0;
            ledFlag=1;
          }
        }
        else    //if(ledFlag == 1)      //  ledFlag
        {
          if((com1Link == LinkKey)&&(Com1UartState == IDLE))
          {
            Com1UartState = SENDBUSY;
            memset(&SendBuffer[0], 0, sizeof(SendBuffer));
           
            SendBuffer[0] = DisplayLed;
            memcpy(&SendBuffer[1], &Dis_Char[0], 3);
            Cnt = 4;
            PackageSendData(&SendBuffer[0], &Cnt);
            Uart1SendData(&SendBuffer[0], Cnt);
            DispFlag = 0;
            Com1UartState = IDLE;
            ledFlag=0;
          }
          else if(com1Link != LinkKey)
          {
            DispFlag=0;
            ledFlag=0;
          }
        }
    }
  }
  else
  {
    LEDRest = 0;        //健康模式下，不关闭LED显示，计数器则继续重新开始计时
  }
}

/***********************************************************************************************************
* 函数名称: KeyBoardSoftVersionDisplay()
* 输入参数: 无
* 返回值  : 无
* 功    能: 发送命令让按键板显示按键板的软件版本
************************************************************************************************************/
void KeyBoardSoftVersionDisplay(void)
{
  u8 Cnt;
  
  if(CombinationKey==6)
  {
    
    if(ledFlag==0)
    {
      if((com3Link==LinkKey)&&(Com3UartState == IDLE))
      {
        Com3UartState = SENDBUSY;
        memset(&SendBuffer[0], 0, sizeof(SendBuffer));
        SendBuffer[0] = SoftVersionDisplay;
        Cnt = 1;
        PackageSendData(&SendBuffer[0], &Cnt);
        Uart3SendData(&SendBuffer[0], Cnt);
        DispFlag = 0;
        Com3UartState = IDLE;
        ledFlag=1;
      }
      else if(com3Link!=LinkKey)
      {
        DispFlag=0;
        ledFlag=1;
      }
    }
    else
    {
      if((com1Link==LinkKey)&&(Com1UartState == IDLE))
      {
        Com1UartState = SENDBUSY;
        memset(&SendBuffer[0], 0, sizeof(SendBuffer));
        SendBuffer[0] = SoftVersionDisplay;
        Cnt = 1;
        PackageSendData(&SendBuffer[0], &Cnt);
        Uart1SendData(&SendBuffer[0], Cnt);
        DispFlag = 0;
        Com1UartState = IDLE;
        ledFlag=0;
      }
      else if(com1Link!=LinkKey)
      {
        DispFlag=0;
        ledFlag=0;
      }
    }
  }
}