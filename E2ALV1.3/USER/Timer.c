#include "Timer.h"
#include "Motor.h"
#include "Uart.h"
#include "Key.h"
#include "string.h"
#include "Delay.h"
#include "stm8s_it.h"


/***********************************************************************************************************
* ��������: TIM1_PWM_Init()
* �������: ��
* ����ֵ  : ��
* ��    ��: TIM1��·ͬƵ��PWM���  CH1��CH2��CH3
************************************************************************************************************/
void TIM1_PWM_Init(void)
{
  TIM1_DeInit(); 
  TIM1_TimeBaseInit(0,TIM1_COUNTERMODE_UP,1350,0);//���ϼ���  24M����Ƶ  ����Ƶ��Ϊ17.7K
 // TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,\
    1350, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_SET);
  
  TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,\
    1350, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_SET);
  
   TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,\
    1350, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET, TIM1_OCNIDLESTATE_SET);
 // TIM1_OC1PreloadConfig(ENABLE);           //Ԥװ��ʹ��
  TIM1_OC2PreloadConfig(ENABLE);
  TIM1_OC3PreloadConfig(ENABLE);
  TIM1_CtrlPWMOutputs(ENABLE);
  TIM1_Cmd(ENABLE);
}

/***********************************************************************************************************
* ��������: SetTIM1_PWMDuty()
* �������: channel����ʱ��ͨ����TIM1_Pulse������ֵ
* ����ֵ  : ��
* ��    ��: ����TIM1��·PWM����ռ�ձ�
************************************************************************************************************/
void SetTIM1_PWMDuty( uint16_t channel,uint16_t TIM1_Pulse)
{
  if(channel==1)
  {
    TIM1->CCR3H = (uint8_t)(TIM1_Pulse >> 8);
    TIM1->CCR3L = (uint8_t)(TIM1_Pulse);
  }
  else if(channel==2)
  {
    TIM1->CCR2H = (uint8_t)(TIM1_Pulse >> 8);
    TIM1->CCR2L = (uint8_t)(TIM1_Pulse);
  }
}

/***********************************************************************************************************
* ��������: TIM2_Init()
* �������: ��
* ����ֵ  : ��
* ��    ��: TIM2��ʼ������ʱ1ms
************************************************************************************************************/
void TIM2_Init()
{
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_8,2999);       //8��Ƶ 1ms��ʱ
  TIM2_PrescalerConfig(TIM2_PRESCALER_8,TIM2_PSCRELOADMODE_IMMEDIATE);
  TIM2_ARRPreloadConfig(ENABLE);
  TIM2_ITConfig(TIM2_IT_UPDATE , ENABLE);
  TIM2_Cmd(ENABLE);
}

/***********************************************************************************************************
* ��������: BuzzerControl()
* �������: state��Ҫ���õķ�����״̬
* ����ֵ  : ��
* ��    ��: ���Ʒ���������
************************************************************************************************************/
void BuzzerControl(u8 state)
{
  u8 Cnt;
  
  if((Com3UartState == IDLE)&&(com3Link == LinkKey))
  {
    Com3UartState = SENDBUSY;
    memset(&SendBuffer[0], 0, sizeof(SendBuffer));
    if(state == ON)
      SendBuffer[0] = TurnOnBuzzer;
    else
      SendBuffer[0] = TurnOffBuzzer;
    Cnt = 1;
    PackageSendData(&SendBuffer[0], &Cnt);
    Uart3SendData(&SendBuffer[0], Cnt);
    Com3UartState = IDLE;
    if((InitFlag == 4)&&( BuzzerState == OFF))
    {
      InitCnt = 0;
      InitFlag = 5;
    }
  }
  if((Com1UartState == IDLE)&&(com1Link==LinkKey))
  {
    Com1UartState = SENDBUSY;
    memset(&SendBuffer[0], 0, sizeof(SendBuffer));
    if(state == ON)
      SendBuffer[0] = TurnOnBuzzer;
    else
      SendBuffer[0] = TurnOffBuzzer;
    Cnt = 1;
    PackageSendData(&SendBuffer[0], &Cnt);
    Uart1SendData(&SendBuffer[0], Cnt);
    Com1UartState = IDLE;
    if((InitFlag ==4)&&(BuzzerState==OFF))
    {
      InitCnt = 0;
      InitFlag = 5;
    }
  }
}



