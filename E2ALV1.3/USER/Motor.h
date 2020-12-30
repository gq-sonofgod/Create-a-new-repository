#ifndef MOTOR_H
#define MOTOT_H
#include "stm8s.h"


typedef struct
{
  u8 MotorState;        //当前电机状态：UP,正转；DOWN,反转；STOP，停止
  //u8 Endpoint;        //电机是否运行到端点，1表示上端点，2表示下端点，0表示未到达任何端点
  u8 HallState;         //两个HALL信号的电平状态分为10,11,01,00四态
  u16 HallLast;         //上次的HALL计数值，这个值每200MS更新一次
  u16 HallNow;          //当前的HALL计数值, 这个值每200MS更新一次 
  u16 LimitUp;          //电机运行的HALL上限值
  u16 LimitDown;        //电机运行的HALL下限值
  
  //电机运行中，由于在适应地面不平时，使两电机的绝对位置值已经不统一，
  //在两个电机同时运行时，就会产生两个电机相对运行高度的最大值和最小值
  u16 RelativeLimitUp;          
  u16 RelativeLimitDown;        
  
  u16 Record[8];       //设备设定的运行高度记录值
}MSTATE;


#define MINSPEED   35  
#define MAXSPEED   45

#define DELT_HALL  45

#define BASEDUTYUP      200
#define BASEDUTYDOWN    100
#define CURRENTTHRESHOLD 600
#define SPEEDTHRESHOLD 10



  
#define MSTATE_DEFAULTS  {0,0,0,0,0,0,0,0,{0}}


//M1上
#define M1Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_0);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_0)
//M1下
#define M1Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_1);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_1) 
//M2上
#define M2Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_4);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_4)
//M2下
#define M2Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOE,GPIO_PIN_7);\
                         else GPIO_WriteHigh(GPIOE,GPIO_PIN_7)
//M3上
//#define M3Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_1);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_1)
//M3下
//#define M3Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_0);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_0)

//33V通
#define CurrentOut(ON_OFF) if(ON_OFF==ON) GPIO_WriteLow(GPIOB,GPIO_PIN_2);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_2);


extern u8 DelayFlagForTap;

extern MSTATE M1State;
extern MSTATE M2State;

extern u8 M1Detect;
extern u8 M2Detect;

extern u8 M1Dir;
extern u8 M2Dir;
extern u8 TurnFlag;

extern u16 M1TestPWMDuty;
extern u16 M2TestPWMDuty;

extern u8 M1Flag;
extern u8 M2Flag;
extern u8 AntiCollisionState;
extern u16 M1CurFlag;
extern u16 M2CurFlag;

extern u16 ADCBuffer1[40];
extern u16 ADCBuffer2[40];
extern u16 M1CurTemp1;
extern u16 M1CurTemp2;
extern u16 M1CurTemp3;
extern u16 M2CurTemp1;
extern u16 M2CurTemp2;
extern u16 M2CurTemp3;
extern u8 LimitPowerFlag;
extern u16 M1CurMax;
extern u16 M2CurMax; 
extern u16 CurFlag;
extern u8 LimitPowerState;  
  
void MotorControl(void);
void Motor_Init(void);
void M1Control(u8 cmd);
void M1ControlTest(u8 cmd);
void M2Control(u8 cmd);
void M2ControlTest(u8 cmd);
void LimitPower(void);
void MotorControlTest(void);
void AgingTestFunc(void);
void AntiCollision(void);
u16 MiddleAVG(u16 *data, u16 len);
void AntiLsm6dsl(void);

#endif