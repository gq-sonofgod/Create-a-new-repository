#ifndef ADC_H
#define ADC_H
#include "stm8s.h"


typedef struct 
{

  u16   KeyADCValue;   //�������ֵ
 // float LowPower;      //�͵�ѹ���ֵ
  float Temperature;   //ϵͳ�¶ȼ��ֵ
  float M1Current;     //���1�������ֵ
  float M2Current;     //���2�������ֵ
 // float M3Current;     //���3�������ֵ

  
  /*
    u16   KeyADCValue;   //�������ֵ
 // float LowPower;      //�͵�ѹ���ֵ
  u16 Temperature;   //ϵͳ�¶ȼ��ֵ
  u16 M1Current;     //���1�������ֵ
  u16 M2Current;     //���2�������ֵ
 // float M3Current;     //���3�������ֵ
  */
  
}ADC;



  
void ADCSample(ADC* adcval);

extern ADC ADCValue;
extern u16 M1Cur;
extern u16 M2Cur;
extern u8 M1ADCCnt;
extern u8 M2ADCCnt;




#endif