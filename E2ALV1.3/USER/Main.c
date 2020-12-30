#include "Main.h"
#include "ADC.h"
#include "Key.h"
#include "Timer.h"
#include "EXTI.h"
#include "Motor.h"
#include "EEPROM.h"
#include "LED.h"
#include "PID.h"
#include "Delay.h"
#include "EEPROM.h"
#include "string.h"
#include "FaultDetection.h"
#include "HealthMode.h"
#include "Uart.h"
#include "stm8s_it.h"
#include "PortToPC.h"

#include "Balance.h"

#include "lsm6dsl_reg.h"
#include "LSM6DSL.h"
#include "Tap.h"

#include "math.h"

u8 SysState;                    //当前系统状态，分为NORMAL和RESET
u8 SysCmd;
u8 M1Cmd;                       //电机1命令
u8 M2Cmd;                       //电机2命令

u16 ErrCode;
u16 FinalPosition;              //延迟减速后的最终位置

//健康模式标志位 
//初始化为0,按下A键后，HealthMode = 6，显示ON(显示1.5S)，
//1.5s后，HealthMode = 5，进入设定定时时间模式，(这时LED第一位常亮，后两位设定的时间表现为闪烁)
//5s后，HealthMode = 1进入定时模式(这时LED第一位闪烁，后两位设定的时间表现为常亮)

u8 HealthMode = 0;               

u16  BuzzerOnTimerCnt = 0;
u8   BuzzerState = OFF;
u8   BuzzerWorkMode = 0;          //蜂鸣器工作时有两种模式  ,  //为3时，则表示非平衡状态下试图进行运行，连续响1000ms， 
                                                            //为2时，则表示复位状态下连续工作500ms，  
                                                            //为1时，则表示平衡调整完毕，连续响1500ms   
                                                            //为0时，则表示不工作   
u8 SaveFlag = 1;                //当前高度存储标志位，当值为1时，进行存储
u8 SoftState = 0;
u8 Unit = UNIT;
float BaseHeight = BASEHEIGHT;
float MaxHeight = MAXHEIGHT;
float MinColumnHeight = BASEHEIGHT;
float MaxColumnHeight = MAXHEIGHT;
u16 DiffHall = DIF_HALL;
u8 Sensitivity = SENS;
u8 Speed = SPEED;
u32 AgingAllTimeLast = 0;
u8 Com1DeviceState = MASTER;
u8 Com3DeviceState = MASTER;
//s16 ddd=0;


lsm6dsl_ctx_t dev_ctx;
lsm6dsl_status_reg_t  StatusReg;
/*
typedef struct {
   uint8_t xlda                     : 1;
   uint8_t gda                      : 1;
   uint8_t tda                      : 1;
   uint8_t not_used_01              : 5;
} lsm6dsl_status_reg_t;
*/
extern int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
extern int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
u8 SensorOut[6];

s16 Acc_x,Acc_y,Acc_z;


ACC_DATEA_STR  AccDataBag = ACC_DATEA_DEFAULTS;

//s16 AccXl_x[16],AccXl_y[16],AccXl_z[16];

s16 xValue,yValue,zValue;
s16 Ang_x,Ang_y,Ang_z;

u8 buffer_temp[5] = {0x55,0xaa,0x55,0xaa,0xff};
u16 UartSendTimerCnt = 0;
u16 lsm6dslTimer;
u8 lsm6dslErrNum=0;
u8 Lsm6dslErrFlg = 0;

u8 HPSlopeFilterFlag = 0;   //获取HPSlopeFilter状态，1则开启，0则不开启

void HSE_CLK_INIT(void)
{
 CLK_DeInit();  
 //CLK_HSECmd(ENABLE);                                  
 //while(SET != CLK_GetFlagStatus(CLK_FLAG_HSERDY));
 CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
 //CLK_ClockSwitchCmd(ENABLE); 
 CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO , CLK_SOURCE_HSE , DISABLE , CLK_CURRENTCLOCKSTATE_DISABLE);
}

void IWDG_Config(void)
{ 
  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_128);
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();   
}

void EnableHPSlopeFilter()
{
  u8 temp;
  u8 enHPSlopeFilter = 0x04;
  
  lsm6dsl_read_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  temp |=  enHPSlopeFilter;
  
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  HPSlopeFilterFlag = 1;
  
 // lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);//没有敲击的情况下频率只有12.5HZ
  
}


void DisableHPSlopeFilter()
{
  u8 temp;
  u8 disHPSlopeFilter = 0xf9;
  
  lsm6dsl_read_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  temp &=  disHPSlopeFilter;
  
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  HPSlopeFilterFlag = 0;
  
 // lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5); //没有敲击的情况下频率只有12.5HZ
  
}

//获取HPSlopeFilter状态，1则开启，0则不开启

u8 GetHPSlopeFilterState()      
{
    return HPSlopeFilterFlag;
}

u8 ctr8_val=0;



u8 LSM6DSL_Reset(void)
{
  u8 pro_val;
  //u8 ctr8_val=0;
  lsm6dsl_odr_xl_t   xl_ord_val;
  lsm6dsl_odr_g_t    gy_ord_val;
  lsm6dsl_fs_xl_t    xl_fs_val;
  lsm6dsl_fs_g_t     g_fs_val;
  
  if(lsm6dsl_reset_set(&dev_ctx,1)!=0)  //复位传感器芯片（设置CTRL3_C寄存器中的SW_RESET位（CTRL3_C第0位设置为1））
  {
     return 1; 
  }
  //功能：output registers not updated until MSB and LSB have been read
  //设置CTRL3_C寄存器中的BDU位
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);    
  lsm6dsl_block_data_update_get(&dev_ctx, &pro_val);
  
  if(pro_val!=PROPERTY_ENABLE)
  {
     return 2;
  }
  //设定CTRL1_XL寄存器中的ODR_XL[3:0]位，实现重力加速度输出数据频率选择（Output data rate selection）
  //由于设置的high performance模式
  //设定3轴重力加速度采样频率为12.5
  
  
  /*
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5);     
  lsm6dsl_xl_data_rate_get(&dev_ctx, &xl_ord_val);
  if(xl_ord_val!=LSM6DSL_XL_ODR_12Hz5)  
  {
     return 2;
  }
  */
  
  
  
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5); 
  lsm6dsl_xl_data_rate_get(&dev_ctx, &xl_ord_val);  
  if(xl_ord_val != LSM6DSL_XL_ODR_12Hz5) //滤波频率
  {
     return 2;
  }
  
  
  

  //设定CTRL2_G寄存器中的ODR_G [3:0][3:0]位，实现旋转加速度输出数据频率选择（Output data rate selection）
  //由于设置的high performance模式
  //设定3轴重力加速度采样频率为12.5
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
  lsm6dsl_gy_data_rate_get(&dev_ctx, &gy_ord_val);
  if(gy_ord_val!=LSM6DSL_GY_ODR_12Hz5)
  {
     return 2;
  }
    
  //设置CTRL1_XL寄存器中的FS_XL [1:0]位，实现加速度满量程为+-2g
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_xl_full_scale_get(&dev_ctx, &xl_fs_val);
  if(xl_fs_val!=LSM6DSL_2g)
  {
    return 2;
  }

  //设置CTRL2_G寄存器中的FS_G [1:0]位，实现角加速度满量程为2000 dps
  lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
  lsm6dsl_gy_full_scale_get(&dev_ctx, &g_fs_val);
  if(g_fs_val!=LSM6DSL_2000dps)
  {
    return 2;
  }
    
  //CTRL8_XL = 0x14;
  //
  //ctr8_val = 0x14;
  
  ctr8_val = 0x14;
  //ctr8_val = 0x10;
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&ctr8_val,1);
  ctr8_val = 0;
  lsm6dsl_read_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&ctr8_val,1);
  if(ctr8_val!=0x14)
  {
    return 2;
  }
  HPSlopeFilterFlag = 1;
  
  return 0;
  
}


/*
u8 LSM6DSL_Reset(void)
{
  u8 temp;
  u8 pro_val;
  u8 ctr8_val=0;
  lsm6dsl_odr_xl_t xl_ord_val;
  //lsm6dsl_odr_g_t gy_ord_val;
  lsm6dsl_fs_xl_t xl_fs_val;
  //lsm6dsl_fs_g_t g_fs_val;
  
  if(lsm6dsl_reset_set(&dev_ctx,1)!=0)
  {
     return 1; 
  }
  
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsl_block_data_update_get(&dev_ctx, &pro_val);
  if(pro_val!=PROPERTY_ENABLE)
  {
     return 2;
  }
  //Output Data Rate selection 
  //LSM6DSL_ACC_GYRO_W_ODR_XL
  
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz); 
  lsm6dsl_xl_data_rate_get(&dev_ctx, &xl_ord_val);  
  if(xl_ord_val != LSM6DSL_XL_ODR_416Hz) //滤波频率
  {
     return 2;
  }
    // Full scale selection. 
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_xl_full_scale_get(&dev_ctx, &xl_fs_val);
  if(xl_fs_val!=LSM6DSL_2g)
  {
    return 2;
  }
  
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
  lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
  

  //Enable X direction in tap recognition
  //lsm6dsl_tap_cfg_t
  //lsm6dsl_tap_detection_on_x_set(&dev_ctx,1);
 // lsm6dsl_tap_detection_on_y_set(&dev_ctx,1);
  lsm6dsl_tap_detection_on_z_set(&dev_ctx,1);//
  
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_CFG,&Lsm6dIntState,1);
  
  //Set tap threshold. (LSM6DSL_TAP_THRESHOLD_MID_LOW )
  //lsm6dsl_tap_threshold_x_set(&dev_ctx,0x08); 
  //lsm6dsl_tap_threshold_x_set(&dev_ctx,0x06);  //
  //lsm6dsl_tap_threshold_x_set(&dev_ctx,0x05);
  
  lsm6dsl_tap_threshold_x_set(&dev_ctx,0x03);   //精度调试1
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_THS_6D,&Lsm6dIntState,1);
  
  
  // Set tap shock time window (INT_DUR2[1:0] SHOCK) //Maximum duration of overthreshold event 超限时间最长持续时间
  //lsm6dsl_tap_shock_set(&dev_ctx,0x03); 
  lsm6dsl_tap_shock_set(&dev_ctx,0x02); //精度调试2  5A

  //Set tap quiet time window (INT_DUR2[3:2] QUIET) //Expected quiet time after a tap detection 敲击检测后的预期安静时间
  lsm6dsl_tap_quiet_set(&dev_ctx,0x03);
  //lsm6dsl_tap_quiet_set(&dev_ctx,0x00);
  
  //Set tap duration time window (INT_DUR2[7:4] DUR) //Duration of maximum time gap for double tap recognition 超限事件的最长持续时间
  lsm6dsl_tap_dur_set(&dev_ctx,0x08);
  //lsm6dsl_tap_dur_set(&dev_ctx,0x0f);
  
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_INT_DUR2,&Lsm6dIntState,1);
  
  //Single and double tap enabled  (WAKE_UP_THS [7] SINGLE_DOUBLE_TAP)
  //lsm6dsl_tap_mode_set(&dev_ctx,0x01);
  //lsm6dsl_tap_mode_set(&dev_ctx,LSM6DSL_BOTH_SINGLE_DOUBLE);  
  lsm6dsl_tap_mode_set(&dev_ctx,LSM6DSL_ONLY_SINGLE);//敲击模式
  
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_WAKE_UP_THS,&Lsm6dIntState,1);
  
  
  //Enable basic Interrupts 
  //tap_cfg.interrupts_enable = PROPERTY_ENABLE; //TAP_CFG Enable basic interrupt(6D/4D, free-fall, wake-up, tap, inactivity)
  //lsm6dsl_int_notification_set(&dev_ctx,1); 
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_CFG,&Lsm6dIntState,1);
  temp = 0x87;          //TAP_CFG Enable basic interrupt(6D/4D, free-fall, wake-up, tap, inactivity)
  //temp = 0x83;        //设置成0x83后，遇阻回退会非常灵敏。
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_TAP_CFG,&temp,1);//
  
  //Lsm6dIntState = 0;
  
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_CFG,&Lsm6dIntState,1);
  
  //Enable double tap on either INT1 
   
  temp = 0x04;
  
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_MD1_CFG,&temp,1);//6D事件中断使能
  

  ctr8_val=0x14;
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&ctr8_val,1);//低通配置
  ctr8_val=0;
  lsm6dsl_read_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&ctr8_val,1);
  if(ctr8_val!=0x14)
  {
    return 2;
  }
  
  return 0;

}
*/

void LSM6DSL_Init(void)
{
   u8 j=0;
   u8 i=0;
   lsm6dslErrNum=0;
   do
   {
      i=LSM6DSL_Reset();
      if(i!=0)
      {
         j++;
         if(j>=3)
         {
            if(Lsm6dslErrFlg == 0)
            {
              Lsm6dslErrFlg = 1;
            }
            return;
         }
         Delay_ms(10);
      }
      else return;
   }while(1);
}


void CheckTapOrder()
{
    if ((Tap_Parameter.TapCnt  == 1) && (Tap_Parameter.TapControlEN == 1)) //进入单击控制状态
        
    {
        
        if (Tap_Parameter.TapTriggerState == 0)
        {
            Tap_Parameter.TapTriggerState = 1;
        }
        else if ((Tap_Parameter.TapTriggerState == 1) || (Tap_Parameter.TapTriggerState == 3))
        {
            Tap_Parameter.TapTriggerState = 2;
        }
                
        Tap_Parameter.TapCheckDelayFlag = 1;
        Tap_Parameter.TapCheckDelayTimerCnt = 0;
                
        Tap_Parameter.TapControlTimerCnt = 0;
        Tap_Parameter.TapControlFlag = 1;     //进入TAP控制状态
        LEDRest = 0;
   
     }
     else if (Tap_Parameter.TapCnt >= 2) //进入双击控制状态
     {
       if (Tap_Parameter.TapControlEN == 0)
       {
         Tap_Parameter.TapBuzzerTimerCnt = 0;
         Tap_Parameter.TapBuzzerState = ON;
         BuzzerState = Tap_Parameter.TapBuzzerState;
          
         Tap_Parameter.TapControlTimerCnt = 0;           //
         Tap_Parameter.TapControlEN = 1;                 //使能TAP控制
         Tap_Parameter.TapControlFlag = 0;
                
         Tap_Parameter.TapCheckDelayFlag = 1;
         Tap_Parameter.TapCheckDelayTimerCnt = 0;     
         LEDRest=0;
         DisplayMode = HeightMode;
       }
       else
       {
          if (Tap_Parameter.TapTriggerState == 0)
          {
                Tap_Parameter.TapTriggerState = 3;
          }
          else if ((Tap_Parameter.TapTriggerState == 3) ||
                    (Tap_Parameter.TapTriggerState == 1))
          {       
            Tap_Parameter.TapTriggerState = 4;
          }
          
          Tap_Parameter.TapCheckDelayFlag = 1;
          Tap_Parameter.TapCheckDelayTimerCnt = 0;
                
          Tap_Parameter.TapControlTimerCnt = 0;
          Tap_Parameter.TapControlFlag = 1;     //进入TAP控制状态
          LEDRest = 0;
       }
    }
}


void GetTapTriggerState()
{
    if (ErrCode != 0)
  {
    Tap_Parameter.TapControlEN = 0;
    Tap_Parameter.TapControlFlag = 0;
                            
    Tap_Parameter.TapFilterFlag = 0;
    Tap_Parameter.SingleTapFilterCnt = 0;
    Tap_Parameter.TapCheckOrderFlag = 0;
    Tap_Parameter.TapCnt = 0;
  }
  else if (SysState == NORMAL)
  {
    //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_SRC,&Lsm6dIntState,1);          //读取敲击控制状态
   
    if (TapControlFlag == 1)
    
    //if (SysCmd != CmdToPreseting)        //系统非运行到指定位置命令
    if (((SysCmd == CmdToPreseting)&& ((Tap_Parameter.TapTriggerState == 1)||(Tap_Parameter.TapTriggerState == 3)))||((SysCmd == CmdNull) && (DelayFlagForTap == 0)) || \
        ((SysCmd == CmdUp)   && (Tap_Parameter.TapTriggerState == 1)) || \
        ((SysCmd == CmdDown) && (Tap_Parameter.TapTriggerState == 3)))  //系统非运行到指定位置命令
    {
        if (Tap_Parameter.TapCheckDelayFlag == 0)                         //非处于单次敲击检测后的延时状态
        {
            //if (Tap_Parameter.TapControlEN == 0)
            //if (Lsm6dIntState & 0x20)                       //单击触发   
            //if((Acc_z >= 2000) || (Acc_z <= -2000))
          
            if (((abs(Acc_z) >= Tap_Parameter.TapCtlThreshold) && (Tap_Parameter.TapControlFlag == 0)) ||   //判断是否超过设定的阈值
               (Tap_Parameter.TapControlFlag == 1) && (abs(Acc_z) >= (Tap_Parameter.TapCtlThreshold * 0.8)))
            {
               
                    
                if (Tap_Parameter.TapFilterFlag == 0)           //敲击检测滤波标志位
                {
                    Tap_Parameter.TapFilterFlag = 1;            //敲击检测滤波标志位置1
                    Tap_Parameter.CheckTriggerTimerCnt = 0;     //单次敲击检测时间计数器清0
                
                    Tap_Parameter.SingleTapFilterCnt = 1;       //单击计数器置1
                }
                else 
                {
                    if ((Tap_Parameter.TapFilterFlag == 1) &&                   //敲击控制未使能的情况下，检测时间拉长到200ms，避免误启动
                        (Tap_Parameter.CheckTriggerTimerCnt <= 200) && 
                        (Tap_Parameter.CheckTriggerTimerCnt >= 3) && 
                        (Tap_Parameter.TapControlEN == 0))
                    {
                        Tap_Parameter.SingleTapFilterCnt++;                     //200ms内，滤波阶段，进行单击计数器累加操作
                    }
                    else if ((Tap_Parameter.TapFilterFlag == 1) &&             //敲击控制使能的情况下，检测时间变成100ms，增加敲击灵敏度
                              (Tap_Parameter.CheckTriggerTimerCnt <= 100) && 
                              (Tap_Parameter.CheckTriggerTimerCnt >= 3) &&
                              (Tap_Parameter.TapControlEN == 1))
                    {
                        Tap_Parameter.SingleTapFilterCnt++;                     //100ms内，滤波阶段，进行单击计数器累加操作
                    }
                }
            
            }
           
            
            //else if ((Tap_Parameter.TapFilterFlag == 1) && (Tap_Parameter.CheckTriggerTimerCnt > 250))
            if ((Tap_Parameter.TapFilterFlag == 1) && 
                (Tap_Parameter.CheckTriggerTimerCnt > 100) &&
                (Tap_Parameter.TapControlEN == 1))
            {
                if (Tap_Parameter.SingleTapFilterCnt >= 2)
                {
                    Tap_Parameter.TapCnt++;
                    
                    if ((Tap_Parameter.TapTriggerState == 1) || (Tap_Parameter.TapTriggerState == 3))
                    {
                        CheckTapOrder(); 
                        /*
                        Tap_Parameter.TapFilterFlag = 1;
                        Tap_Parameter.TapCheckOrderFlag = 0;
                        Tap_Parameter.SingleTapFilterCnt = 0;
                        Tap_Parameter.TapCnt = 0;
                        */
                        return;
                    }
                    else if (Tap_Parameter.TapCheckOrderFlag == 0)
                    {
                        Tap_Parameter.TapCheckOrderFlag = 1;    //用于判断命令的标志位置1
                    }
                    
                    Tap_Parameter.TapCheckOrderTimerCnt = 0;    //进行命令判断的时间计数器清0
                }
                
                Tap_Parameter.SingleTapFilterCnt = 0;
                Tap_Parameter.TapFilterFlag = 0;
            }
            else if ((Tap_Parameter.TapFilterFlag == 1) &&          //敲击控制未使能状态下，每次进行敲击检测时间为200ms
                      (Tap_Parameter.CheckTriggerTimerCnt > 200) &&
                      (Tap_Parameter.TapControlEN == 0))
            {
                if (Tap_Parameter.SingleTapFilterCnt >= 2)
                {
                    Tap_Parameter.TapCnt++;
                                        
                    if (Tap_Parameter.TapCheckOrderFlag == 0)
                    {
                        Tap_Parameter.TapCheckOrderFlag = 1;    //用于判断命令的标志位置1
                    }
                    
                    Tap_Parameter.TapCheckOrderTimerCnt = 0;    //进行命令判断的时间计数器清0
                }
                
                Tap_Parameter.SingleTapFilterCnt = 0;
                Tap_Parameter.TapFilterFlag = 0;
            }
            
            if ((Tap_Parameter.TapControlEN == 0) && 
                (Tap_Parameter.TapCheckOrderFlag == 1) && 
                (Tap_Parameter.TapCheckOrderTimerCnt > 400))
            {
                CheckTapOrder();

                Tap_Parameter.TapCheckOrderFlag = 0;
                      
                Tap_Parameter.TapCnt = 0;
            }
            
         //}        //在电机未运行的情况下，若连续600ms没接收到敲击命令，则进入命令控制阶段
            else if ((Tap_Parameter.TapTriggerState == 0 ) && 
                      (Tap_Parameter.TapCheckOrderFlag == 1) &&  
                      (Tap_Parameter.TapCheckOrderTimerCnt > 500))   
            {
                CheckTapOrder();

                Tap_Parameter.TapCheckOrderFlag = 0;
                      
                Tap_Parameter.TapCnt = 0;
            } 
            /*
            //在电机运行的情况下，若连续600ms没接收到敲击命令，则进入命令控制阶段
            else if (((Tap_Parameter.TapTriggerState == 1 ) || (Tap_Parameter.TapTriggerState == 3)) &&
                      (Tap_Parameter.TapCheckOrderFlag == 1) &&  
                      (Tap_Parameter.TapCheckOrderTimerCnt > 400))
            {
                CheckTapOrder(); 
           
                Tap_Parameter.TapCheckOrderFlag = 0;
                      
                Tap_Parameter.TapCnt = 0;
            }
            */
        }
        
        //每一次实现敲击控制后，需再延时500ms，再进入下一次的检测周期
    //else if ((Tap_Parameter.TapCheckDelayTimerCnt > 500) && (Tap_Parameter.TapCheckDelayFlag == 1))   
        if ((Tap_Parameter.TapCheckDelayFlag == 1) && (Tap_Parameter.TapCheckDelayTimerCnt > 200))   
        {
            Tap_Parameter.TapCheckDelayFlag = 0;
        } 
  
        if ((Tap_Parameter.TapControlTimerCnt > 10000) && (Tap_Parameter.TapControlEN == 1))
        {
            Tap_Parameter.TapControlEN = 0;
            Tap_Parameter.TapControlFlag = 0;
            
        }

        if ((Tap_Parameter.TapControlEN == 1) && (Tap_Parameter.TapBuzzerTimerCnt >= 500) && (Tap_Parameter.TapBuzzerState == ON))  //蜂鸣器响500ms
        {
            Tap_Parameter.TapBuzzerState = OFF;
            BuzzerState = Tap_Parameter.TapBuzzerState;
        }
    }
    else if (SysCmd == CmdToPreseting)   //正在执行运行到指定位置的操作,或者碰到遇阻回退操作
    {
      /*Tap_Parameter.TapFilterFlag = 0;
      Tap_Parameter.SingleTapFilterCnt = 0;
      Tap_Parameter.TapCheckOrderFlag = 0;
      Tap_Parameter.TapCnt = 0;
      Tap_Parameter.TapControlEN = 0;
      Tap_Parameter.TapControlFlag = 0;*/
    }
  }
}


/*******************************************************************************
函数名称：Clear_AccDateBuffer()（全局函数）
函数功能：清除加速度采样计数值和数据存储状态（是否可以进行滤波操作）
输入：无
输出：无
*******************************************************************************/
void Clear_AccDateBuffer()
{
  AccDataBag.Number_Cnt = 0;
  AccDataBag.Full_State = 0;
  AccDataBag.ALLAccXl_x = 0;
  AccDataBag.ALLAccXl_y = 0;
  AccDataBag.ALLAccXl_z = 0;
}

u8 GetAccFullState()
{
  return AccDataBag.Full_State;
}

s16 GetAcc_y()
{
  return AccDataBag.Acc_y;
}

/******************************************************************************
函数名称：G_Acc_Filter()
函数功能：对采样得到的重力加速度值进行滤波处理
输入：无
输出：无
******************************************************************************/
void G_Acc_Filter()
{
    s16 acc_x,acc_y,acc_z;
    
    acc_x = (SensorOut[1]<<8)|SensorOut[0];     //得到16位数据
    acc_y = (SensorOut[3]<<8)|SensorOut[2];
    acc_z = (SensorOut[5]<<8)|SensorOut[4];
    
     //ddd=acc_y;
     
    if((AccDataBag.Y_OffsetFlag==1)&&(AccDataBag.Y_OffsetFlag_Spec==0)&&(Y_Off_EN_Fleg==0))
    {
    acc_y = acc_y -   AccDataBag.Y_Offset;        //得到Y轴值真实值
    }
    else if((AccDataBag.Y_OffsetFlag_Spec==1)&&(Y_Off_EN_Fleg==0))
    {
     acc_y = acc_y -   AccDataBag.Y_Offset_Spec;        //得到Y轴值真实值
    
    }
    else  acc_y = (SensorOut[3]<<8)|SensorOut[2];                  
    AccDataBag.AccXl_x_temp = AccDataBag.AccXl_x[AccDataBag.Number_Cnt];
    AccDataBag.AccXl_x[AccDataBag.Number_Cnt] = (SensorOut[1]<<8)|SensorOut[0];
              
    AccDataBag.AccXl_y_temp = AccDataBag.AccXl_y[AccDataBag.Number_Cnt];
    //AccDataBag.AccXl_y[AccDataBag.Number_Cnt] = (SensorOut[3]<<8)|SensorOut[2];
    AccDataBag.AccXl_y[AccDataBag.Number_Cnt] = acc_y;
              
    AccDataBag.AccXl_z_temp = AccDataBag.AccXl_z[AccDataBag.Number_Cnt];
    AccDataBag.AccXl_z[AccDataBag.Number_Cnt] = (SensorOut[5]<<8)|SensorOut[4];
       
    /*    
    AccDataBag.Number_Cnt++;
    
    if (AccDataBag.Full_State == 1)
    {
        AccDataBag.Number_Cnt = AccDataBag.Number_Cnt%8;
    }
    */
    
    if ((AccDataBag.Number_Cnt == 7) && (AccDataBag.Full_State == 0))         //第一次数据满了
    {
      u8 i;
      AccDataBag.Full_State = 1;
         
      for(i = 0;i < 8; i++)
      {
         AccDataBag.ALLAccXl_x += AccDataBag.AccXl_x[i];
         AccDataBag.ALLAccXl_y += AccDataBag.AccXl_y[i];
         AccDataBag.ALLAccXl_z += AccDataBag.AccXl_z[i];
       }
       
      
     }
     else if (AccDataBag.Full_State == 1)
     {
        //AccDataBag.Y_Offset_Spec = (s16)(AccDataBag.ALLAccXl_y >> 3);
        
        
        
        AccDataBag.ALLAccXl_x -= AccDataBag.AccXl_x_temp; 
        AccDataBag.ALLAccXl_x += AccDataBag.AccXl_x[AccDataBag.Number_Cnt];
                  
        AccDataBag.ALLAccXl_y -= AccDataBag.AccXl_y_temp;
        AccDataBag.ALLAccXl_y += AccDataBag.AccXl_y[AccDataBag.Number_Cnt];
                
        AccDataBag.ALLAccXl_z -= AccDataBag.AccXl_z_temp;
        AccDataBag.ALLAccXl_z += AccDataBag.AccXl_z[AccDataBag.Number_Cnt]; 
        if((Y_Off_EN_Fleg==1))
       {
        static u8 Che_cnt=0;
        Che_cnt++;
        if(Che_cnt>10)  
        {
         Che_cnt=0;
         Y_Off_EN_Fleg=0;
         AccDataBag.Y_Offset_Spec = (s16)(AccDataBag.ALLAccXl_y >> 3);
      
         memcpy(&Buffer[73],&AccDataBag.Y_Offset_Spec,sizeof(AccDataBag.Y_Offset_Spec));
                     
         AccDataBag.Y_OffsetFlag_Spec = 1;
         memcpy(&Buffer[75],&AccDataBag.Y_OffsetFlag_Spec,sizeof(AccDataBag.Y_OffsetFlag_Spec));       //已获取陀螺仪偏移量的标志位存入EEPROM中
         EnableHPSlopeFilter();              
         EEPROM_Write();
        }
        }
      } 
        
      AccDataBag.Number_Cnt++;
   
      AccDataBag.Number_Cnt = AccDataBag.Number_Cnt%8;
             
      s32 ang_temp = 0;
      ang_temp = AccDataBag.ALLAccXl_y >> 3;
              
      AccDataBag.Acc_y = (s16)(ang_temp);
}

//u8 tempp1 = 0;

void main(void)
{ 
  HSE_CLK_INIT();                       //24M外部晶振
  Delay_Init(24);
  EXTI_Init();
  Delay_ms(200);
 
  Motor_Power_ON;                       //GPIOC4 高电平（PIN29）                      
  Hall_Power_ON;                        //GPIOC5 高电平 (PIN30)
  GetLevel();                           //获取三个电机的HALL信号电平状态
  Motor_Init();                         //电机控制引脚和蜂鸣器控制引脚初始化
  TIM1_PWM_Init();
  TIM2_Init();                 
  //用于产生1ms的定时中断
  Uart_Config();                        //进行串口1和串口3初始化操作
  IIC2_Init();                          //SDA-PC5,SCL-PG0
  dev_ctx.read_reg  = platform_read;
  dev_ctx.write_reg = platform_write;
  LSM6DSL_Init();
  Delay_ms(300);
  EEPROM_Init();
  SysDataRead();                        //进行EEPROM读取操作 
  enableInterrupts();                   //使能中断
  
  //进行PID参数初始化，包括参考速度，PID参数和初始占空比
  PID_Set(MINSPEED,BASEDUTYDOWN);       //PID初始化  MINSPEED = 35 (MAXSPEED = 45)    
  AnglePID.LastDiffValur = 0;
  AnglePID.PrevDiffValur = 0;
  
  AnglePID.Kp = 0.03;
  //AnglePID.Kp = 0.003;//0.005;
  //AnglePID.Ki = 0.03;//0.06;
  AnglePID.Ki = 0.02;//0.06;
  //AnglePID.Kd = 0.03;
  AnglePID.Kd = 0.02;
  
  IWDG_Config();
  while(1)
  {  
    if((Com1DeviceState!=SLAVER)&&(Com3DeviceState!=SLAVER)) //当控制盒不作为通信从设备时才能正常使用
    {   
      //#define  LSM6DSLFlag 1  //带六轴传感器标志位   //Sensitivity=SENS = 3  遇阻回退等级
      //if(((Sensitivity ==3)||(Sensitivity ==2))&&(LSM6DSLFlag==1))    
      if((Sensitivity != 0)&&(LSM6DSLFlag==1))
      {
        if(lsm6dsl_status_reg_get(&dev_ctx, &StatusReg) == 0)     //获得传感器3个模拟量更新状态值（3个模拟量分别是温度，陀螺仪和加速度器）
        {                       
          if(StatusReg.xlda==1)                                 //加速度器的值有更新     
          {
            lsm6dslTimer = 0;
            //判断是否读取成功
            if(lsm6dsl_acceleration_raw_get(&dev_ctx, &SensorOut[0]) == 0)      //加速度   Linear acceleration
            { 
              //if ((AccDataBag.Y_OffsetFlag == 1) && (GetBalaceState() == 0) && ((GetM1BeforeRunState() != 1) && (GetM2BeforeRunState() != 1)))    //已完成出厂复位且没有进入平衡调整状态
              if ((AccDataBag.Y_OffsetFlag == 1) && (GetHPSlopeFilterState() == 1)) //已完成出厂复位且使能 HPSlopeFilter
              {
                Acc_x = (SensorOut[1]<<8)|SensorOut[0];
                Acc_y = (SensorOut[3]<<8)|SensorOut[2];
                Acc_z = (SensorOut[5]<<8)|SensorOut[4];
              }
              else if (((AccDataBag.Y_OffsetFlag == 0) ||(GetHPSlopeFilterState() == 0)))
              //else if ((AccDataBag.Y_OffsetFlag == 0) || (GetBalaceState() != 0) || ((GetM1BeforeRunState() == 1) && (GetM2BeforeRunState() == 1)))   //出厂复位 或 进入平衡调整状态 或进入到运行到指定位置操作
              {
                 if (GetHPSlopeFilterState() ==1)
                 {
                    DisableHPSlopeFilter();
                     
                 }
                 else
                 {
                    G_Acc_Filter();           //重力加速度采样值进行滤波处理
                    
                 }
              }
             
              
              lsm6dslErrNum = 0;
            }
            else
            { 
              lsm6dslErrNum++;
            }
            //判断是否读取成功
            if(lsm6dsl_angular_rate_raw_get(&dev_ctx, &SensorOut[0])==0)//角速度   Angular rate sensor
            {
              xValue = (SensorOut[1]<<8)|SensorOut[0];
              yValue = (SensorOut[3]<<8)|SensorOut[2];
              zValue = (SensorOut[5]<<8)|SensorOut[4];
              if((xValue==yValue)&&(xValue==zValue))
              {
                lsm6dslErrNum++;
              }
              else
              {
                 Ang_x=xValue;
                 Ang_y=yValue;
                 Ang_z=zValue;
                 lsm6dslErrNum=0;
              }
            }
            else
            {
              lsm6dslErrNum++;
            }
            
           /*******************************************************
                        增加获取TAP状态寄存器操作
            ******************************************************/

            if(Adjust_State== 0)
            {
                GetTapTriggerState();
            }

          }
          else if(lsm6dslTimer>200)     //200ms加速度器的值没更新，说明传感器故障
          {
            ErrCode=Err_LSM6DSL; 
          }
        }
        else
        {
          lsm6dslErrNum++;
        }

        if(lsm6dslErrNum>=5)            //连续5次无法与传感器通讯，说明传感器故障
        {
           ErrCode = Err_LSM6DSL; 
        }
      }
            
      {
        if(Sensitivity > 0)                //
        {
          AntiLsm6dsl();                  //该函数包含 电流遇阻回退判断功能
        }
        
        //这个是关于复位操作，先不去看
        if(RSTFlag == 2)           
        {
          RSTFlag = 3;
          M1State.HallNow = BASEHALL;     //BASEHALL =18000   // 机械最底端底端HALL值 
          M1State.HallLast = BASEHALL;
          M2State.HallNow = BASEHALL;
          M2State.HallLast = BASEHALL;
          memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));
          EEPROM_Write();
        }
  
        if(DispFlag == 1)                         //LED每30ms刷新一次
        { 
         LED_Display();          
        }
         if(T10msFlag == 1)                //10ms进行一个采样
        { 
          if(HealthMode != 5)   
          {
            KeyRespond(ADCValue.KeyADCValue);           //按键响应程序
          }
          else if(HealthMode == 5)
          {
            HealthModeTimerSet(ADCValue.KeyADCValue);   //健康模式时间设置
          }
        } 
       
        if(ADC_Scan_Flag == 1)
        {
         ADCSample(&ADCValue);  
         ADC_Scan_Flag=0;
         Adc_Time=0;
        }
        Adjust_Balance_StepByStepo();
       //Save_Position_Tap();
        SetBalance();
       //Tap_Control();
        LimitPower();                       //限定系统最大功率
        MotorControl();                     //电机控制函数，封装三个电机控制函数
        PositionSave();                     //存储当前位置（用于M键+1/2/3记录当前位置）
        
        IWDG_ReloadCounter();
        
        FaultDetect();                      //系统各种故障检测，主要包括电机过流故障，电机HALL开关故障，过温故障及各故障后的一些标志位的清零操作
        LowPowerSave();                     //低电压存储操作(不知道什么用)
        HealthModeProc();                   //健康模式处理函数  HealthMode=0(初始化)，所以刚上电此函数没进行操作
        BuzzerControl(BuzzerState);         //控制蜂鸣器开关,(state，要设置的蜂鸣器状态) (初始化 BuzzerState = 0) 
        KeyBoardSoftVersionDisplay();       //发送命令让按键板显示按键板的软件版本
      }
    }
    else//从设备模式下
    {
      RespondCmdOfPC();         //接收处理来自上位机软件的命令
    }
    
    IWDG_ReloadCounter();
  }
}