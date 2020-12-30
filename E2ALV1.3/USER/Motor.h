#ifndef MOTOR_H
#define MOTOT_H
#include "stm8s.h"


typedef struct
{
  u8 MotorState;        //��ǰ���״̬��UP,��ת��DOWN,��ת��STOP��ֹͣ
  //u8 Endpoint;        //����Ƿ����е��˵㣬1��ʾ�϶˵㣬2��ʾ�¶˵㣬0��ʾδ�����κζ˵�
  u8 HallState;         //����HALL�źŵĵ�ƽ״̬��Ϊ10,11,01,00��̬
  u16 HallLast;         //�ϴε�HALL����ֵ�����ֵÿ200MS����һ��
  u16 HallNow;          //��ǰ��HALL����ֵ, ���ֵÿ200MS����һ�� 
  u16 LimitUp;          //������е�HALL����ֵ
  u16 LimitDown;        //������е�HALL����ֵ
  
  //��������У���������Ӧ���治ƽʱ��ʹ������ľ���λ��ֵ�Ѿ���ͳһ��
  //���������ͬʱ����ʱ���ͻ�����������������и߶ȵ����ֵ����Сֵ
  u16 RelativeLimitUp;          
  u16 RelativeLimitDown;        
  
  u16 Record[8];       //�豸�趨�����и߶ȼ�¼ֵ
}MSTATE;


#define MINSPEED   35  
#define MAXSPEED   45

#define DELT_HALL  45

#define BASEDUTYUP      200
#define BASEDUTYDOWN    100
#define CURRENTTHRESHOLD 600
#define SPEEDTHRESHOLD 10



  
#define MSTATE_DEFAULTS  {0,0,0,0,0,0,0,0,{0}}


//M1��
#define M1Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_0);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_0)
//M1��
#define M1Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_1);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_1) 
//M2��
#define M2Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_4);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_4)
//M2��
#define M2Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOE,GPIO_PIN_7);\
                         else GPIO_WriteHigh(GPIOE,GPIO_PIN_7)
//M3��
//#define M3Up(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_1);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_1)
//M3��
//#define M3Down(ON_OFF) if(ON_OFF==OFF) GPIO_WriteLow(GPIOB,GPIO_PIN_0);\
                         else GPIO_WriteHigh(GPIOB,GPIO_PIN_0)

//33Vͨ
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