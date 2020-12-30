#ifndef TAP_H
#define TAP_H

#include "stm8s.h"

extern u8 Lsm6dIntState;
typedef struct
{
  u16 CheckTriggerTimerCnt;             //用于TAP触发信号的滤波
  u16 TapControlTimerCnt;
  
  u16 SingleTriggerTimerCnt;
  u16 DoubelTriggerTimerCnt;
  u8 TapTriggerState;                   //记录TAP状态   0：表示无触发，1：表示单敲击开状态，2：表示单敲击关状态，3：表示双击开状态，4：表示双击关状态
    
  //用于单击检测滤波,0:表示未检测到状态  1：表示规定时间内第一个检测到切换信号，2：表示规定时间内第2次检测到切换信号
  u8 SingleTapFilterCnt;    
  //用于双击检测滤波,0:表示未检测到状态  1：表示规定时间内第一个检测到切换信号，2：表示规定时间内第2次检测到切换信号
  u8 DoubleTapFilterCnt;         
  u8 TapFilterFlag;                     //进入TAP滤波状态 0：表示未进入 ，1：表示进入滤波状态
  
  u8 TapControlEN;                      //Tap控制使能位  0：未使能TAP控制  1：使能TAP控制
  
  u8 TapControlFlag;                    //Tap控制运行标志位  0：未运行  1：正在运行  （只有进入使能TAP控制状态后，这个标志位才有效）
  
  u8 TapBuzzerState;
  u16 TapBuzzerTimerCnt;
  
  u8  TapCheckDelayFlag;                //用于两次敲击间隔检测的标志位  0：表示未进入检测状态  1：进入两次敲击间隔检测
  u16 TapCheckDelayTimerCnt;            //用于计数两次敲击触发的间隔时间
  
  u8  TapCnt;
  
  u8  TapCheckOrderFlag;                //用于表示当前敲击检测的标志位   1：检测到敲击操作，且正在进行敲击检测   0：未检测到敲击操作
  u16 TapCheckOrderTimerCnt;            //敲击检测过程中，用于定时连续多长时间未检测到敲击，则完成当前状态的敲击检测操作，进入敲击控制状态
  
  u16 TapCtlThreshold; 
}TAP_PARAMETER;

extern TAP_PARAMETER Tap_Parameter;

#define TAP_PARAMETER_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
extern u16 Record_Temp[5];
extern u16 Savebuff_Down[5];
extern u16 Savebuff_Up[5];
extern u16 Min_Up_Record;
extern u16 Max_Down_Record;
extern u8 Down_Position;
extern u8 Up_Position;
extern void Save_Position_Tap();
extern u8 TapControlFlag;
extern void Tap_Control();

#endif 