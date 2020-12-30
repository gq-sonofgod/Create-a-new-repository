#include "ADC.h"
#include "Motor.h"
#include "Main.h"


ADC ADCValue;
u16 M1Cur;
u8 M1ADCCnt =0;
u16 M2Cur;
u8 M2ADCCnt =0;

/***********************************************************************************************************
* 函数名称: GetADCValue()
* 输入参数: ADC_Channel,对应的ADC通道
* 返回值  : 该通道的ADC采样值
* 功    能: 获取指定通道的ADC采样值
************************************************************************************************************/
u16 GetADCValue(ADC2_Channel_TypeDef ADC_Channel)
{
  ADC2_Init(ADC2_CONVERSIONMODE_CONTINUOUS,ADC_Channel, ADC2_PRESSEL_FCPU_D4,\
  ADC2_EXTTRIG_TIM, DISABLE, ADC2_ALIGN_RIGHT, ADC2_SCHMITTTRIG_ALL, DISABLE);          //ADC时钟频率6M，连续采样模式
  ADC2_ConversionConfig(ADC2_CONVERSIONMODE_CONTINUOUS, ADC_Channel,ADC2_ALIGN_RIGHT);
  ADC2_ITConfig(DISABLE);   //禁止中断
  ADC2_Cmd(ENABLE);   
  ADC2->CR1 |= 0x01;;                   //启动ADC转换
  while(ADC2_GetITStatus()==SET);       //等待ADC转换结束
  ADC2->CSR &= 0x7F; ;                  //清除转换完成标志位
  return ADC2_GetConversionValue();     //返回ADC结果
}


/***********************************************************************************************************
* 函数名称: ADCSample()
* 输入参数: adcval
* 返回值  : 无
* 功    能: 对各个ADC通道轮流采样
************************************************************************************************************/
void ADCSample(ADC* adcval)
{
  static ADC2_Channel_TypeDef ADCChannel = ADC2_CHANNEL_2;      //ADC2_CHANNEL_2 = 0x02
  //static u8 time = 0; //按键通道检测次数
  //static u16 KeyADC = 0;
  u16 ADCTemp;
  
  ADCTemp = GetADCValue(ADCChannel);
  switch(ADCChannel)
  {
    case ADC2_CHANNEL_6:        //电机1过流检测通道 PIN16
      if((M1Cmd==CmdUp)||(M1Cmd==CmdDown)||(M1Dir!=STOP))
      {
        if(M1ADCCnt < 40)       //M1ADCCnt = 0（初始化）
        {
          ADCBuffer1[M1ADCCnt] = ADCTemp;
          M1ADCCnt++;
        }
      }
      else
      {
        M1CurTemp1 = 0;
        M1CurTemp2 = 0;
        M1CurTemp3 = 0;
        M1ADCCnt = 0;
      }
      adcval->M1Current = adcval->M1Current * 0.9 + ADCTemp * 0.1;    //低通滤波
      
      //adcval->M1Current = ADCTemp;    //低通滤波
      break;
      
    case ADC2_CHANNEL_5:        //电机2过流检测通道  PIN17
      if((M2Cmd==CmdUp)||(M2Cmd==CmdDown)||(M2Dir!=STOP))
      {
        if(M2ADCCnt < 40)
        {
          ADCBuffer2[M2ADCCnt] = ADCTemp;
          M2ADCCnt++;
        }
      }
      else
      {
        M2CurTemp1 = 0;
        M2CurTemp2 = 0;
        M2CurTemp3 = 0;
        M2ADCCnt = 0;
      }
      adcval->M2Current = adcval->M2Current * 0.9 + ADCTemp * 0.1;    //低通滤波
      
      //adcval->M2Current = ADCTemp;    //低通滤波
      break;
 
    case ADC2_CHANNEL_7:        //系统过热检测通道 PIN15
      adcval->Temperature = adcval->Temperature * 0.9 + ADCTemp * 0.1;
      
      //adcval->Temperature = ADCTemp;
      break;
 
    case ADC2_CHANNEL_3:        //按键检测通道  PIN19 (该硬件无此功能)
      adcval->KeyADCValue = ADCTemp;    //无滤波
      break;     
  }
  if(ADCChannel < ADC2_CHANNEL_7)
    ADCChannel++;
  else
    ADCChannel = ADC2_CHANNEL_3;
  if(ADCChannel == ADC2_CHANNEL_4)
    ADCChannel = ADC2_CHANNEL_5;
  //if(ADCChannel == ADC2_CHANNEL_8)
  //  ADCChannel = ADC2_CHANNEL_9;
}
