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

u8 SysState;                    //��ǰϵͳ״̬����ΪNORMAL��RESET
u8 SysCmd;
u8 M1Cmd;                       //���1����
u8 M2Cmd;                       //���2����

u16 ErrCode;
u16 FinalPosition;              //�ӳټ��ٺ������λ��

//����ģʽ��־λ 
//��ʼ��Ϊ0,����A����HealthMode = 6����ʾON(��ʾ1.5S)��
//1.5s��HealthMode = 5�������趨��ʱʱ��ģʽ��(��ʱLED��һλ����������λ�趨��ʱ�����Ϊ��˸)
//5s��HealthMode = 1���붨ʱģʽ(��ʱLED��һλ��˸������λ�趨��ʱ�����Ϊ����)

u8 HealthMode = 0;               

u16  BuzzerOnTimerCnt = 0;
u8   BuzzerState = OFF;
u8   BuzzerWorkMode = 0;          //����������ʱ������ģʽ  ,  //Ϊ3ʱ�����ʾ��ƽ��״̬����ͼ�������У�������1000ms�� 
                                                            //Ϊ2ʱ�����ʾ��λ״̬����������500ms��  
                                                            //Ϊ1ʱ�����ʾƽ�������ϣ�������1500ms   
                                                            //Ϊ0ʱ�����ʾ������   
u8 SaveFlag = 1;                //��ǰ�߶ȴ洢��־λ����ֵΪ1ʱ�����д洢
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

u8 HPSlopeFilterFlag = 0;   //��ȡHPSlopeFilter״̬��1������0�򲻿���

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
  
 // lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);//û���û��������Ƶ��ֻ��12.5HZ
  
}


void DisableHPSlopeFilter()
{
  u8 temp;
  u8 disHPSlopeFilter = 0xf9;
  
  lsm6dsl_read_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  temp &=  disHPSlopeFilter;
  
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&temp,1);
  
  HPSlopeFilterFlag = 0;
  
 // lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5); //û���û��������Ƶ��ֻ��12.5HZ
  
}

//��ȡHPSlopeFilter״̬��1������0�򲻿���

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
  
  if(lsm6dsl_reset_set(&dev_ctx,1)!=0)  //��λ������оƬ������CTRL3_C�Ĵ����е�SW_RESETλ��CTRL3_C��0λ����Ϊ1����
  {
     return 1; 
  }
  //���ܣ�output registers not updated until MSB and LSB have been read
  //����CTRL3_C�Ĵ����е�BDUλ
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);    
  lsm6dsl_block_data_update_get(&dev_ctx, &pro_val);
  
  if(pro_val!=PROPERTY_ENABLE)
  {
     return 2;
  }
  //�趨CTRL1_XL�Ĵ����е�ODR_XL[3:0]λ��ʵ���������ٶ��������Ƶ��ѡ��Output data rate selection��
  //�������õ�high performanceģʽ
  //�趨3���������ٶȲ���Ƶ��Ϊ12.5
  
  
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
  if(xl_ord_val != LSM6DSL_XL_ODR_12Hz5) //�˲�Ƶ��
  {
     return 2;
  }
  
  
  

  //�趨CTRL2_G�Ĵ����е�ODR_G [3:0][3:0]λ��ʵ����ת���ٶ��������Ƶ��ѡ��Output data rate selection��
  //�������õ�high performanceģʽ
  //�趨3���������ٶȲ���Ƶ��Ϊ12.5
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
  lsm6dsl_gy_data_rate_get(&dev_ctx, &gy_ord_val);
  if(gy_ord_val!=LSM6DSL_GY_ODR_12Hz5)
  {
     return 2;
  }
    
  //����CTRL1_XL�Ĵ����е�FS_XL [1:0]λ��ʵ�ּ��ٶ�������Ϊ+-2g
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_xl_full_scale_get(&dev_ctx, &xl_fs_val);
  if(xl_fs_val!=LSM6DSL_2g)
  {
    return 2;
  }

  //����CTRL2_G�Ĵ����е�FS_G [1:0]λ��ʵ�ֽǼ��ٶ�������Ϊ2000 dps
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
  if(xl_ord_val != LSM6DSL_XL_ODR_416Hz) //�˲�Ƶ��
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
  
  lsm6dsl_tap_threshold_x_set(&dev_ctx,0x03);   //���ȵ���1
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_THS_6D,&Lsm6dIntState,1);
  
  
  // Set tap shock time window (INT_DUR2[1:0] SHOCK) //Maximum duration of overthreshold event ����ʱ�������ʱ��
  //lsm6dsl_tap_shock_set(&dev_ctx,0x03); 
  lsm6dsl_tap_shock_set(&dev_ctx,0x02); //���ȵ���2  5A

  //Set tap quiet time window (INT_DUR2[3:2] QUIET) //Expected quiet time after a tap detection �û������Ԥ�ڰ���ʱ��
  lsm6dsl_tap_quiet_set(&dev_ctx,0x03);
  //lsm6dsl_tap_quiet_set(&dev_ctx,0x00);
  
  //Set tap duration time window (INT_DUR2[7:4] DUR) //Duration of maximum time gap for double tap recognition �����¼��������ʱ��
  lsm6dsl_tap_dur_set(&dev_ctx,0x08);
  //lsm6dsl_tap_dur_set(&dev_ctx,0x0f);
  
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_INT_DUR2,&Lsm6dIntState,1);
  
  //Single and double tap enabled  (WAKE_UP_THS [7] SINGLE_DOUBLE_TAP)
  //lsm6dsl_tap_mode_set(&dev_ctx,0x01);
  //lsm6dsl_tap_mode_set(&dev_ctx,LSM6DSL_BOTH_SINGLE_DOUBLE);  
  lsm6dsl_tap_mode_set(&dev_ctx,LSM6DSL_ONLY_SINGLE);//�û�ģʽ
  
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_WAKE_UP_THS,&Lsm6dIntState,1);
  
  
  //Enable basic Interrupts 
  //tap_cfg.interrupts_enable = PROPERTY_ENABLE; //TAP_CFG Enable basic interrupt(6D/4D, free-fall, wake-up, tap, inactivity)
  //lsm6dsl_int_notification_set(&dev_ctx,1); 
  //Lsm6dIntState = 0;
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_CFG,&Lsm6dIntState,1);
  temp = 0x87;          //TAP_CFG Enable basic interrupt(6D/4D, free-fall, wake-up, tap, inactivity)
  //temp = 0x83;        //���ó�0x83��������˻�ǳ�������
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_TAP_CFG,&temp,1);//
  
  //Lsm6dIntState = 0;
  
  //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_CFG,&Lsm6dIntState,1);
  
  //Enable double tap on either INT1 
   
  temp = 0x04;
  
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_MD1_CFG,&temp,1);//6D�¼��ж�ʹ��
  

  ctr8_val=0x14;
  lsm6dsl_write_reg(&dev_ctx,LSM6DSL_CTRL8_XL,&ctr8_val,1);//��ͨ����
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
    if ((Tap_Parameter.TapCnt  == 1) && (Tap_Parameter.TapControlEN == 1)) //���뵥������״̬
        
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
        Tap_Parameter.TapControlFlag = 1;     //����TAP����״̬
        LEDRest = 0;
   
     }
     else if (Tap_Parameter.TapCnt >= 2) //����˫������״̬
     {
       if (Tap_Parameter.TapControlEN == 0)
       {
         Tap_Parameter.TapBuzzerTimerCnt = 0;
         Tap_Parameter.TapBuzzerState = ON;
         BuzzerState = Tap_Parameter.TapBuzzerState;
          
         Tap_Parameter.TapControlTimerCnt = 0;           //
         Tap_Parameter.TapControlEN = 1;                 //ʹ��TAP����
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
          Tap_Parameter.TapControlFlag = 1;     //����TAP����״̬
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
    //lsm6dsl_read_reg(&dev_ctx,LSM6DSL_TAP_SRC,&Lsm6dIntState,1);          //��ȡ�û�����״̬
   
    if (TapControlFlag == 1)
    
    //if (SysCmd != CmdToPreseting)        //ϵͳ�����е�ָ��λ������
    if (((SysCmd == CmdToPreseting)&& ((Tap_Parameter.TapTriggerState == 1)||(Tap_Parameter.TapTriggerState == 3)))||((SysCmd == CmdNull) && (DelayFlagForTap == 0)) || \
        ((SysCmd == CmdUp)   && (Tap_Parameter.TapTriggerState == 1)) || \
        ((SysCmd == CmdDown) && (Tap_Parameter.TapTriggerState == 3)))  //ϵͳ�����е�ָ��λ������
    {
        if (Tap_Parameter.TapCheckDelayFlag == 0)                         //�Ǵ��ڵ����û��������ʱ״̬
        {
            //if (Tap_Parameter.TapControlEN == 0)
            //if (Lsm6dIntState & 0x20)                       //��������   
            //if((Acc_z >= 2000) || (Acc_z <= -2000))
          
            if (((abs(Acc_z) >= Tap_Parameter.TapCtlThreshold) && (Tap_Parameter.TapControlFlag == 0)) ||   //�ж��Ƿ񳬹��趨����ֵ
               (Tap_Parameter.TapControlFlag == 1) && (abs(Acc_z) >= (Tap_Parameter.TapCtlThreshold * 0.8)))
            {
               
                    
                if (Tap_Parameter.TapFilterFlag == 0)           //�û�����˲���־λ
                {
                    Tap_Parameter.TapFilterFlag = 1;            //�û�����˲���־λ��1
                    Tap_Parameter.CheckTriggerTimerCnt = 0;     //�����û����ʱ���������0
                
                    Tap_Parameter.SingleTapFilterCnt = 1;       //������������1
                }
                else 
                {
                    if ((Tap_Parameter.TapFilterFlag == 1) &&                   //�û�����δʹ�ܵ�����£����ʱ��������200ms������������
                        (Tap_Parameter.CheckTriggerTimerCnt <= 200) && 
                        (Tap_Parameter.CheckTriggerTimerCnt >= 3) && 
                        (Tap_Parameter.TapControlEN == 0))
                    {
                        Tap_Parameter.SingleTapFilterCnt++;                     //200ms�ڣ��˲��׶Σ����е����������ۼӲ���
                    }
                    else if ((Tap_Parameter.TapFilterFlag == 1) &&             //�û�����ʹ�ܵ�����£����ʱ����100ms�������û�������
                              (Tap_Parameter.CheckTriggerTimerCnt <= 100) && 
                              (Tap_Parameter.CheckTriggerTimerCnt >= 3) &&
                              (Tap_Parameter.TapControlEN == 1))
                    {
                        Tap_Parameter.SingleTapFilterCnt++;                     //100ms�ڣ��˲��׶Σ����е����������ۼӲ���
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
                        Tap_Parameter.TapCheckOrderFlag = 1;    //�����ж�����ı�־λ��1
                    }
                    
                    Tap_Parameter.TapCheckOrderTimerCnt = 0;    //���������жϵ�ʱ���������0
                }
                
                Tap_Parameter.SingleTapFilterCnt = 0;
                Tap_Parameter.TapFilterFlag = 0;
            }
            else if ((Tap_Parameter.TapFilterFlag == 1) &&          //�û�����δʹ��״̬�£�ÿ�ν����û����ʱ��Ϊ200ms
                      (Tap_Parameter.CheckTriggerTimerCnt > 200) &&
                      (Tap_Parameter.TapControlEN == 0))
            {
                if (Tap_Parameter.SingleTapFilterCnt >= 2)
                {
                    Tap_Parameter.TapCnt++;
                                        
                    if (Tap_Parameter.TapCheckOrderFlag == 0)
                    {
                        Tap_Parameter.TapCheckOrderFlag = 1;    //�����ж�����ı�־λ��1
                    }
                    
                    Tap_Parameter.TapCheckOrderTimerCnt = 0;    //���������жϵ�ʱ���������0
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
            
         //}        //�ڵ��δ���е�����£�������600msû���յ��û���������������ƽ׶�
            else if ((Tap_Parameter.TapTriggerState == 0 ) && 
                      (Tap_Parameter.TapCheckOrderFlag == 1) &&  
                      (Tap_Parameter.TapCheckOrderTimerCnt > 500))   
            {
                CheckTapOrder();

                Tap_Parameter.TapCheckOrderFlag = 0;
                      
                Tap_Parameter.TapCnt = 0;
            } 
            /*
            //�ڵ�����е�����£�������600msû���յ��û���������������ƽ׶�
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
        
        //ÿһ��ʵ���û����ƺ�������ʱ500ms���ٽ�����һ�εļ������
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

        if ((Tap_Parameter.TapControlEN == 1) && (Tap_Parameter.TapBuzzerTimerCnt >= 500) && (Tap_Parameter.TapBuzzerState == ON))  //��������500ms
        {
            Tap_Parameter.TapBuzzerState = OFF;
            BuzzerState = Tap_Parameter.TapBuzzerState;
        }
    }
    else if (SysCmd == CmdToPreseting)   //����ִ�����е�ָ��λ�õĲ���,��������������˲���
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
�������ƣ�Clear_AccDateBuffer()��ȫ�ֺ�����
�������ܣ�������ٶȲ�������ֵ�����ݴ洢״̬���Ƿ���Խ����˲�������
���룺��
�������
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
�������ƣ�G_Acc_Filter()
�������ܣ��Բ����õ����������ٶ�ֵ�����˲�����
���룺��
�������
******************************************************************************/
void G_Acc_Filter()
{
    s16 acc_x,acc_y,acc_z;
    
    acc_x = (SensorOut[1]<<8)|SensorOut[0];     //�õ�16λ����
    acc_y = (SensorOut[3]<<8)|SensorOut[2];
    acc_z = (SensorOut[5]<<8)|SensorOut[4];
    
     //ddd=acc_y;
     
    if((AccDataBag.Y_OffsetFlag==1)&&(AccDataBag.Y_OffsetFlag_Spec==0)&&(Y_Off_EN_Fleg==0))
    {
    acc_y = acc_y -   AccDataBag.Y_Offset;        //�õ�Y��ֵ��ʵֵ
    }
    else if((AccDataBag.Y_OffsetFlag_Spec==1)&&(Y_Off_EN_Fleg==0))
    {
     acc_y = acc_y -   AccDataBag.Y_Offset_Spec;        //�õ�Y��ֵ��ʵֵ
    
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
    
    if ((AccDataBag.Number_Cnt == 7) && (AccDataBag.Full_State == 0))         //��һ����������
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
         memcpy(&Buffer[75],&AccDataBag.Y_OffsetFlag_Spec,sizeof(AccDataBag.Y_OffsetFlag_Spec));       //�ѻ�ȡ������ƫ�����ı�־λ����EEPROM��
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
  HSE_CLK_INIT();                       //24M�ⲿ����
  Delay_Init(24);
  EXTI_Init();
  Delay_ms(200);
 
  Motor_Power_ON;                       //GPIOC4 �ߵ�ƽ��PIN29��                      
  Hall_Power_ON;                        //GPIOC5 �ߵ�ƽ (PIN30)
  GetLevel();                           //��ȡ���������HALL�źŵ�ƽ״̬
  Motor_Init();                         //����������źͷ������������ų�ʼ��
  TIM1_PWM_Init();
  TIM2_Init();                 
  //���ڲ���1ms�Ķ�ʱ�ж�
  Uart_Config();                        //���д���1�ʹ���3��ʼ������
  IIC2_Init();                          //SDA-PC5,SCL-PG0
  dev_ctx.read_reg  = platform_read;
  dev_ctx.write_reg = platform_write;
  LSM6DSL_Init();
  Delay_ms(300);
  EEPROM_Init();
  SysDataRead();                        //����EEPROM��ȡ���� 
  enableInterrupts();                   //ʹ���ж�
  
  //����PID������ʼ���������ο��ٶȣ�PID�����ͳ�ʼռ�ձ�
  PID_Set(MINSPEED,BASEDUTYDOWN);       //PID��ʼ��  MINSPEED = 35 (MAXSPEED = 45)    
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
    if((Com1DeviceState!=SLAVER)&&(Com3DeviceState!=SLAVER)) //�����ƺв���Ϊͨ�Ŵ��豸ʱ��������ʹ��
    {   
      //#define  LSM6DSLFlag 1  //�����ᴫ������־λ   //Sensitivity=SENS = 3  ������˵ȼ�
      //if(((Sensitivity ==3)||(Sensitivity ==2))&&(LSM6DSLFlag==1))    
      if((Sensitivity != 0)&&(LSM6DSLFlag==1))
      {
        if(lsm6dsl_status_reg_get(&dev_ctx, &StatusReg) == 0)     //��ô�����3��ģ��������״ֵ̬��3��ģ�����ֱ����¶ȣ������Ǻͼ��ٶ�����
        {                       
          if(StatusReg.xlda==1)                                 //���ٶ�����ֵ�и���     
          {
            lsm6dslTimer = 0;
            //�ж��Ƿ��ȡ�ɹ�
            if(lsm6dsl_acceleration_raw_get(&dev_ctx, &SensorOut[0]) == 0)      //���ٶ�   Linear acceleration
            { 
              //if ((AccDataBag.Y_OffsetFlag == 1) && (GetBalaceState() == 0) && ((GetM1BeforeRunState() != 1) && (GetM2BeforeRunState() != 1)))    //����ɳ�����λ��û�н���ƽ�����״̬
              if ((AccDataBag.Y_OffsetFlag == 1) && (GetHPSlopeFilterState() == 1)) //����ɳ�����λ��ʹ�� HPSlopeFilter
              {
                Acc_x = (SensorOut[1]<<8)|SensorOut[0];
                Acc_y = (SensorOut[3]<<8)|SensorOut[2];
                Acc_z = (SensorOut[5]<<8)|SensorOut[4];
              }
              else if (((AccDataBag.Y_OffsetFlag == 0) ||(GetHPSlopeFilterState() == 0)))
              //else if ((AccDataBag.Y_OffsetFlag == 0) || (GetBalaceState() != 0) || ((GetM1BeforeRunState() == 1) && (GetM2BeforeRunState() == 1)))   //������λ �� ����ƽ�����״̬ ����뵽���е�ָ��λ�ò���
              {
                 if (GetHPSlopeFilterState() ==1)
                 {
                    DisableHPSlopeFilter();
                     
                 }
                 else
                 {
                    G_Acc_Filter();           //�������ٶȲ���ֵ�����˲�����
                    
                 }
              }
             
              
              lsm6dslErrNum = 0;
            }
            else
            { 
              lsm6dslErrNum++;
            }
            //�ж��Ƿ��ȡ�ɹ�
            if(lsm6dsl_angular_rate_raw_get(&dev_ctx, &SensorOut[0])==0)//���ٶ�   Angular rate sensor
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
                        ���ӻ�ȡTAP״̬�Ĵ�������
            ******************************************************/

            if(Adjust_State== 0)
            {
                GetTapTriggerState();
            }

          }
          else if(lsm6dslTimer>200)     //200ms���ٶ�����ֵû���£�˵������������
          {
            ErrCode=Err_LSM6DSL; 
          }
        }
        else
        {
          lsm6dslErrNum++;
        }

        if(lsm6dslErrNum>=5)            //����5���޷��봫����ͨѶ��˵������������
        {
           ErrCode = Err_LSM6DSL; 
        }
      }
            
      {
        if(Sensitivity > 0)                //
        {
          AntiLsm6dsl();                  //�ú������� ������������жϹ���
        }
        
        //����ǹ��ڸ�λ�������Ȳ�ȥ��
        if(RSTFlag == 2)           
        {
          RSTFlag = 3;
          M1State.HallNow = BASEHALL;     //BASEHALL =18000   // ��е��׶˵׶�HALLֵ 
          M1State.HallLast = BASEHALL;
          M2State.HallNow = BASEHALL;
          M2State.HallLast = BASEHALL;
          memcpy(&Buffer[0],&ErrCode,sizeof(ErrCode));
          EEPROM_Write();
        }
  
        if(DispFlag == 1)                         //LEDÿ30msˢ��һ��
        { 
         LED_Display();          
        }
         if(T10msFlag == 1)                //10ms����һ������
        { 
          if(HealthMode != 5)   
          {
            KeyRespond(ADCValue.KeyADCValue);           //������Ӧ����
          }
          else if(HealthMode == 5)
          {
            HealthModeTimerSet(ADCValue.KeyADCValue);   //����ģʽʱ������
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
        LimitPower();                       //�޶�ϵͳ�����
        MotorControl();                     //������ƺ�������װ����������ƺ���
        PositionSave();                     //�洢��ǰλ�ã�����M��+1/2/3��¼��ǰλ�ã�
        
        IWDG_ReloadCounter();
        
        FaultDetect();                      //ϵͳ���ֹ��ϼ�⣬��Ҫ��������������ϣ����HALL���ع��ϣ����¹��ϼ������Ϻ��һЩ��־λ���������
        LowPowerSave();                     //�͵�ѹ�洢����(��֪��ʲô��)
        HealthModeProc();                   //����ģʽ������  HealthMode=0(��ʼ��)�����Ը��ϵ�˺���û���в���
        BuzzerControl(BuzzerState);         //���Ʒ���������,(state��Ҫ���õķ�����״̬) (��ʼ�� BuzzerState = 0) 
        KeyBoardSoftVersionDisplay();       //���������ð�������ʾ�����������汾
      }
    }
    else//���豸ģʽ��
    {
      RespondCmdOfPC();         //���մ���������λ�����������
    }
    
    IWDG_ReloadCounter();
  }
}