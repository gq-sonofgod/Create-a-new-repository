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


//系统运行状态定义
#define  UP         1
#define  DOWN       2
#define  STOP       0
#define  NORMAL     0x10
#define  RESET      0x11
#define  ERROR      0x12

//电机控制命令定义
#define  CmdNull    0x00                //无任何命令
#define  CmdUp      0x20                //向上运动命令
#define  CmdDown    0x21                //向下运动命令
#define  CmdStop    0x22
#define  CmdSave    0x23                //存储命令
#define  CmdToPreseting  0x24           //运动到预设的位置命令
#define  CmdHealthModeOn  0x25          //开启健康模式
#define  CmdHealthModeOFF  0x26         //关闭健康模式

//#define  CmdSyStemReset    0x27         //系统初始化

#define  CmdDownHold    0x28            //从下行状态转入暂停状态
#define  CmdUpHold      0x29            //从上行状态转入暂停状态
//#define  CmdRetard      0x2A            //延迟减速状态

#define  CmdUpRetard    0x2B            //从向上运行状态进入的延时减速状态
#define  CmdDownRetard  0x2C            //从向下运行状态进入的延时减速状态
#define  CmdGoBack      0x2D            //检测到过载后回退

#define  CmdSetBalance  0x30            //进入调整平衡模式

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



#define  LSM6DSLFlag 1                          //带六轴传感器


#if defined (ET223_CM_ANTI)|| defined(ET223H_CM_ANTI)


  #define MINHEIGHT 60.5 //60.5       //要改
  #define MAXHEIGHT 126.0f//126.0f    //要改

  //#define MAXHEIGHT 100.0f


  #define BASEHEIGHT 60.5f            //要改
  #define UNIT 0
  #define SENS 3
  #define SPEED 60                      //定义  100ms产生60个HALL脉冲，其中电机转一圈产生8个HALL脉冲
  #define RATE 155.8742f
  #define MINHEIGHT1  23.8
  #define MAXHEIGHT1  49.4
  #define BOTTOM  18114                 //复位完成后的HALL值
  #define BASEHALL 18000                // 机械最底端底端HALL值 
  #define DIF_HALL 10210                //要改

    
  /*
  #define MINHEIGHT 40.0f
  #define MAXHEIGHT 70.0f

  #define BASEHEIGHT 40.0f
  #define UNIT 0
  #define SENS 3
  #define SPEED 60                      //定义  100ms产生60个HALL脉冲，其中电机转一圈产生8个HALL脉冲
  #define RATE 155.8742f
  #define MINHEIGHT1  23.8
  #define MAXHEIGHT1  49.4
  #define BOTTOM  18114                 //复位完成后的HALL值
  #define BASEHALL 18000                // 机械最底端底端HALL值 

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
  #define BOTTOM  18114 //复位完成后的HALL值
  #define BASEHALL 18000  // 机械最底端底端HALL值 
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
  #define BOTTOM  18114                 //复位完成后的HALL值
  #define BASEHALL 18000                // 机械最底端底端HALL值 
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
  #define BOTTOM  18114 //复位完成后的HALL值
  #define BASEHALL 18000  // 机械最底端底端HALL值 
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
  #define BOTTOM  18114 //复位完成后的HALL值
  #define BASEHALL 18000  // 机械最底端底端HALL值 
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
  #define BOTTOM  18114 //复位完成后的HALL值
  #define BASEHALL 18000  // 机械最底端底端HALL值 
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

extern u8 SysState;             //当前系统状态，分为NORMAL和RESET
extern u8 SysCmd;
extern u8 M1Cmd;                //电机1命令
extern u8 M2Cmd;                //电机2命令
extern u16 ErrCode;
extern u16 FinalPosition;       //延迟减速后的最终位置
extern u8 HealthMode;
extern u8 SaveFlag;             //当前高度存储标志位，当值为1时，进行存储
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