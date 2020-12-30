#include "ADC.h"
#include "Motor.h"
#include "Main.h"


ADC ADCValue;
u16 M1Cur;
u8 M1ADCCnt =0;
u16 M2Cur;
u8 M2ADCCnt =0;

/***********************************************************************************************************
* ��������: GetADCValue()
* �������: ADC_Channel,��Ӧ��ADCͨ��
* ����ֵ  : ��ͨ����ADC����ֵ
* ��    ��: ��ȡָ��ͨ����ADC����ֵ
************************************************************************************************************/
u16 GetADCValue(ADC2_Channel_TypeDef ADC_Channel)
{
  ADC2_Init(ADC2_CONVERSIONMODE_CONTINUOUS,ADC_Channel, ADC2_PRESSEL_FCPU_D4,\
  ADC2_EXTTRIG_TIM, DISABLE, ADC2_ALIGN_RIGHT, ADC2_SCHMITTTRIG_ALL, DISABLE);          //ADCʱ��Ƶ��6M����������ģʽ
  ADC2_ConversionConfig(ADC2_CONVERSIONMODE_CONTINUOUS, ADC_Channel,ADC2_ALIGN_RIGHT);
  ADC2_ITConfig(DISABLE);   //��ֹ�ж�
  ADC2_Cmd(ENABLE);   
  ADC2->CR1 |= 0x01;;                   //����ADCת��
  while(ADC2_GetITStatus()==SET);       //�ȴ�ADCת������
  ADC2->CSR &= 0x7F; ;                  //���ת����ɱ�־λ
  return ADC2_GetConversionValue();     //����ADC���
}


/***********************************************************************************************************
* ��������: ADCSample()
* �������: adcval
* ����ֵ  : ��
* ��    ��: �Ը���ADCͨ����������
************************************************************************************************************/
void ADCSample(ADC* adcval)
{
  static ADC2_Channel_TypeDef ADCChannel = ADC2_CHANNEL_2;      //ADC2_CHANNEL_2 = 0x02
  //static u8 time = 0; //����ͨ��������
  //static u16 KeyADC = 0;
  u16 ADCTemp;
  
  ADCTemp = GetADCValue(ADCChannel);
  switch(ADCChannel)
  {
    case ADC2_CHANNEL_6:        //���1�������ͨ�� PIN16
      if((M1Cmd==CmdUp)||(M1Cmd==CmdDown)||(M1Dir!=STOP))
      {
        if(M1ADCCnt < 40)       //M1ADCCnt = 0����ʼ����
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
      adcval->M1Current = adcval->M1Current * 0.9 + ADCTemp * 0.1;    //��ͨ�˲�
      
      //adcval->M1Current = ADCTemp;    //��ͨ�˲�
      break;
      
    case ADC2_CHANNEL_5:        //���2�������ͨ��  PIN17
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
      adcval->M2Current = adcval->M2Current * 0.9 + ADCTemp * 0.1;    //��ͨ�˲�
      
      //adcval->M2Current = ADCTemp;    //��ͨ�˲�
      break;
 
    case ADC2_CHANNEL_7:        //ϵͳ���ȼ��ͨ�� PIN15
      adcval->Temperature = adcval->Temperature * 0.9 + ADCTemp * 0.1;
      
      //adcval->Temperature = ADCTemp;
      break;
 
    case ADC2_CHANNEL_3:        //�������ͨ��  PIN19 (��Ӳ���޴˹���)
      adcval->KeyADCValue = ADCTemp;    //���˲�
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
