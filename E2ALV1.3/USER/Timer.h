#ifndef TIMER_H
#define TIMER_H
#include "stm8s.h"
#include "Main.h"




#define BUZZER_ON   BuzzerControl(ON)
#define BUZZER_OFF  BuzzerControl(OFF)

#define M1_PWM_OFF  SetTIM1_PWMDuty(1, 1350)
#define M2_PWM_OFF  SetTIM1_PWMDuty(2, 1350)
#define M3_PWM_OFF  SetTIM1_PWMDuty(3, 1350)

void TIM1_PWM_Init(void);
void SetTIM1_PWMDuty( uint16_t channel,uint16_t TIM1_Pulse);
void TIM2_Init();
void BuzzerControl(u8 state);



#endif