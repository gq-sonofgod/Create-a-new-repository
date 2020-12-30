#ifndef ADC_H
#define ADC_H
#include "stm8s.h"


typedef struct 
{

  u16   KeyADCValue;   //按键检测值
 // float LowPower;      //低电压检测值
  float Temperature;   //系统温度检测值
  float M1Current;     //电机1过流检测值
  float M2Current;     //电机2过流检测值
 // float M3Current;     //电机3过流检测值

  
  /*
    u16   KeyADCValue;   //按键检测值
 // float LowPower;      //低电压检测值
  u16 Temperature;   //系统温度检测值
  u16 M1Current;     //电机1过流检测值
  u16 M2Current;     //电机2过流检测值
 // float M3Current;     //电机3过流检测值
  */
  
}ADC;



  
void ADCSample(ADC* adcval);

extern ADC ADCValue;
extern u16 M1Cur;
extern u16 M2Cur;
extern u8 M1ADCCnt;
extern u8 M2ADCCnt;




#endif