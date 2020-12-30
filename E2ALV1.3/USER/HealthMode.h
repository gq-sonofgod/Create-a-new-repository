#ifndef HEALTH_H
#define HEALTH_H
#include "stm8s.h"

#define TIMEVAL   45
//#define TIMECNT   (TIMEVAL*60000)

#define  TWO_MINUTES	120000
#define  ONE_MINUTE	  60000

#define  TEN_SECONDS	10000
#define  FIVE_SECONDS	5000
#define  M250_mSECONDS	250

extern u8 MinValue;
extern u8 SecAlarmFlag;
extern u8 AlarmFlag;
extern u32 TIMEcnt; 
extern u8 TimeNow;

void HealthModeTimeProc(void);
void HealthModeON(void);
void HealthModeProc(void);
void HealthModeReset(void);
void HealthModeOFF(void);
void HealthModeTimerSet(u16 value);



#endif