#ifndef MAIN_H
#define MAIN_H
#include "stm8s.h"

#define Motor_Power_ON   GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST)
#define Motor_Power_OFF  GPIO_WriteLow(GPIOC,GPIO_PIN_4)
#define Hall_Power_ON    GPIO_WriteHigh(GPIOC,GPIO_PIN_5)
#define Hall_Power_OFF   GPIO_WriteLow(GPIOC,GPIO_PIN_5)


#define  ON   1
#define  OFF  0

#define  MASTER  0
#define  SLAVER  1


//ϵͳ����״̬����
#define  UP         1
#define  DOWN       2
#define  STOP       0
#define  NORMAL     0x10
#define  RESET      0x11
#define  ERROR      0x12

//������������
#define  CmdNull    0x00                //���κ�����
#define  CmdUp      0x20                //�����˶�����
#define  CmdDown    0x21                //�����˶�����
#define  CmdStop    0x22
#define  CmdSave    0x23                //�洢����
#define  CmdToPreseting  0x24           //�˶���Ԥ���λ������
#define  CmdHealthModeOn  0x25          //��������ģʽ
#define  CmdHealthModeOFF  0x26         //�رս���ģʽ

//#define  CmdSyStemReset    0x27         //ϵͳ��ʼ��

#define  CmdDownHold    0x28            //������״̬ת����ͣ״̬
#define  CmdUpHold      0x29            //������״̬ת����ͣ״̬
//#define  CmdRetard      0x2A            //�ӳټ���״̬

#define  CmdUpRetard    0x2B            //����������״̬�������ʱ����״̬
#define  CmdDownRetard  0x2C            //����������״̬�������ʱ����״̬
#define  CmdGoBack      0x2D            //��⵽���غ����

#define  CmdSetBalance  0x30            //�������ƽ��ģʽ

//#define  CM    0
//#define  INCH  1


#define  ABS(a,b)  (((u32)a)>=((u32)b)?(((u32)a)-((u32)b)):(((u32)b)-((u32)a)))
#define  MAX(a,b)  (a>=b? a: b)
#define  MIN(a,b)  (a<=b? a: b)

#define  IDLE              0
#define  RECEIVEBUSY       1
#define  SENDBUSY          2
#define  WAIT              3
#define  FINISHED          4


#define  GetKeyValue        0x11
#define  DisplayLed         0x12
#define  MainBoardPowerOff  0x13
#define  TurnOnBuzzer       0x14
#define  TurnOffBuzzer      0x15
#define  SoftVersionDisplay 0x16

#define  TEST     0
#define  RELEASE  1



#define  MainBoardSoftVerChar2   (Data_Char[1]|Char_Dot)
#define  MainBoardSoftVerChar3   (Data_Char[3])



#define  LSM6DSLFlag 1                          //�����ᴫ����


#if defined (ET223_CM_ANTI)|| defined(ET223H_CM_ANTI)


  #define MINHEIGHT 60.5 //60.5       //Ҫ��
  #define MAXHEIGHT 126.0f//126.0f    //Ҫ��

  //#define MAXHEIGHT 100.0f


  #define BASEHEIGHT 60.5f            //Ҫ��
  #define UNIT 0
  #define SENS 3
  #define SPEED 60                      //����  100ms����60��HALL���壬���е��תһȦ����8��HALL����
  #define RATE 155.8742f
  #define MINHEIGHT1  23.8
  #define MAXHEIGHT1  49.4
  #define BOTTOM  18114                 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000                // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10210                //Ҫ��

    
  /*
  #define MINHEIGHT 40.0f
  #define MAXHEIGHT 70.0f

  #define BASEHEIGHT 40.0f
  #define UNIT 0
  #define SENS 3
  #define SPEED 60                      //����  100ms����60��HALL���壬���е��תһȦ����8��HALL����
  #define RATE 155.8742f
  #define MINHEIGHT1  23.8
  #define MAXHEIGHT1  49.4
  #define BOTTOM  18114                 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000                // ��е��׶˵׶�HALLֵ 

  #define DIF_HALL 4720//4680//5510//5460
  */

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14

  #define  MainBoardSoftVerChar1  (Char_C)
  #define  MainBoardSoftNumChar1  (Data_Char[1])
  #define  MainBoardSoftNumChar2  (Data_Char[2])
  #define  MainBoardSoftNumChar3  (Char_F)

#elif defined (ET223_INCH_ANTI)|| defined(ET223H_INCH_ANTI)
  #define MINHEIGHT  57.9f
  #define MAXHEIGHT  123.0f
  #define BASEHEIGHT 57.9f
  #define UNIT 1
  #define SENS 3
  #define SPEED 60
  #define RATE 155.8742f
  #define MINHEIGHT1  22.8
  #define MAXHEIGHT1  48.4
  #define BOTTOM  18114 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000  // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10135

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14

  #define  MainBoardSoftVerChar1  (Char_C)
  #define  MainBoardSoftNumChar1  (Data_Char[0])
  #define  MainBoardSoftNumChar2  (Data_Char[2])
  #define  MainBoardSoftNumChar3  (Data_Char[7])

#elif defined (ET223FT_CM_ANTI)
  #define MINHEIGHT 61.5f
  #define MAXHEIGHT 127.0f
  #define BASEHEIGHT 61.5f
  #define UNIT 0
  #define SENS 3
  #define SPEED 60
  #define RATE 155.8742f
  #define MINHEIGHT1  24.2
  #define MAXHEIGHT1  49.8
  #define BOTTOM  18114                 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000                // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10163

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14

  #define  MainBoardSoftVerChar1  (Char_L)
  #define  MainBoardSoftNumChar1  (Data_Char[0])
  #define  MainBoardSoftNumChar2  (Data_Char[3])
  #define  MainBoardSoftNumChar3  (Char_F)

#elif defined (ET223FT_INCH_ANTI)
  #define MINHEIGHT 61.5f
  #define MAXHEIGHT 126.5f
  #define BASEHEIGHT 61.5f
  #define UNIT 1
  #define SENS 3
  #define SPEED 60
  #define RATE 155.8742f
  #define MINHEIGHT1  24.2
  #define MAXHEIGHT1  49.8
  #define BOTTOM  18114 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000  // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10163

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14

  #define  MainBoardSoftVerChar1  (Char_L)
  #define  MainBoardSoftNumChar1  (Data_Char[0])
  #define  MainBoardSoftNumChar2  (Data_Char[4])
  #define  MainBoardSoftNumChar3  (Data_Char[0])

#elif defined (ET223BZ_CM_ANTI)||defined (ET223HBZ_CM_ANTI)
  #define MINHEIGHT 62.5f
  #define MAXHEIGHT 128.0f
  #define BASEHEIGHT 62.5f
  #define UNIT 0
  #define SENS 3
  #define SPEED 60
  #define RATE 155.8742f
  #define MINHEIGHT1  24.6
  #define MAXHEIGHT1  50.2
  #define BOTTOM  18114 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000  // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10163

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14

  #define  MainBoardSoftVerChar1  (Char_L)
  #define  MainBoardSoftNumChar1  (Data_Char[0])
  #define  MainBoardSoftNumChar2  (Data_Char[4])
  #define  MainBoardSoftNumChar3  (Data_Char[1])

#elif defined (ET223BZ_INCH_ANTI) || defined (ET223HBZ_INCH_ANTI)
  #define MINHEIGHT 62.5f
  #define MAXHEIGHT 128.0f
  #define BASEHEIGHT 62.5f
  #define UNIT 1
  #define SENS 3
  #define SPEED 60
  #define RATE 155.8742f
  #define MINHEIGHT1  24.6
  #define MAXHEIGHT1  50.2
  #define BOTTOM  18114 //��λ��ɺ��HALLֵ
  #define BASEHALL 18000  // ��е��׶˵׶�HALLֵ 
  #define DIF_HALL 10163

  #define HD_ANTI_UP_1  16
  #define HD_ANTI_DOWN_1 18
  #define ST_ANTI_UP_1  36
  #define ST_ANTI_DOWN1_1 28
  #define ST_ANTI_DOWN2_1 18

  #define HD_ANTI_UP_2  14
  #define HD_ANTI_DOWN_2 16
  #define ST_ANTI_UP_2  32
  #define ST_ANTI_DOWN1_2 26
  #define ST_ANTI_DOWN2_2 16

  #define HD_ANTI_UP_3  12
  #define HD_ANTI_DOWN_3 14
  #define ST_ANTI_UP_3  30
  #define ST_ANTI_DOWN1_3 24
  #define ST_ANTI_DOWN2_3 14
  
  #define  MainBoardSoftVerChar1  (Char_L)
  #define  MainBoardSoftNumChar1  (Data_Char[0])
  #define  MainBoardSoftNumChar2  (Data_Char[4])
  #define  MainBoardSoftNumChar3  (Data_Char[2])

#endif


typedef struct
{
  s16 AccXl_x[10];
  
  s16 AccXl_y[10];
 
  s16 AccXl_z[10];
  
  float Y_arctan;
  
  s32 ALLAccXl_x;
  s32 ALLAccXl_y;
  s32 ALLAccXl_z;
  
  s16 AccXl_x_temp;
  s16 AccXl_y_temp;
  s16 AccXl_z_temp;
  
  /*
  u16 Acc_x;
  u16 Acc_y;
  u16 Acc_z;
  */
  
  s16 Acc_x;
  s16 Acc_y;
  s16 Acc_z;
  
  s16 Y_Offset;
  s16 Y_Offset_Spec;
  u8 Y_OffsetFlag;
  u8 Y_OffsetFlag_Spec;
  u8  Operation_State;
  u8  Number_Cnt;
  u8  Full_State;

}ACC_DATEA_STR;

extern ACC_DATEA_STR  AccDataBag;

#define ACC_DATEA_DEFAULTS  {{0},{0},{0},0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

extern u8 SysState;             //��ǰϵͳ״̬����ΪNORMAL��RESET
extern u8 SysCmd;
extern u8 M1Cmd;                //���1����
extern u8 M2Cmd;                //���2����
extern u16 ErrCode;
extern u16 FinalPosition;       //�ӳټ��ٺ������λ��
extern u8 HealthMode;
extern u8 SaveFlag;             //��ǰ�߶ȴ洢��־λ����ֵΪ1ʱ�����д洢
extern u8 BuzzerState;
extern u8 BuzzerWorkMode;
extern u16 BuzzerOnTimerCnt;

extern u8 SoftState;
extern u8 Unit;
extern float BaseHeight;
extern u16 DiffHall;
extern u8 Sensitivity;
extern u8 Speed;
extern u8 Com1DeviceState;
extern u8 Com3DeviceState;
extern float MaxHeight;
extern float MinColumnHeight;
extern float MaxColumnHeight;
extern u8 Lsm6dslErrFlg;

extern void Clear_AccDateBuffer();
extern void EnableHPSlopeFilter();
extern void DisableHPSlopeFilter();
extern u8 GetAccFullState();
extern s16 GetAcc_y();
extern s16 ddd;

extern u8 GetHPSlopeFilterState();
extern void G_Acc_Filter();
extern void LSM6DSL_Init(void);
#endif