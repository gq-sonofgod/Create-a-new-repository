#ifndef ERRDETECTION
#define ERRDETECTION
#include "stm8s.h"

#define  MAXCURRENT   380// 285//380 //最大电流，此处为ADC值




//系统错误定义
#define  Err_TimeEnd           0x0001   //运行时间到(连续运行2分钟)
#define  Err_Overheating       0x0002   //电机过热
#define  Err_M1Overcurrent     0x0003   //电机1过流
#define  Err_M2Overcurrent     0x0004   //电机2过流

#define  Err_M1Position        0x0005   //电机1位置故障
#define  Err_M2Position        0x0006   //电机2位置故障

#define  Err_M1OneHall         0x0007   //电机1单个HALL故障
#define  Err_M2OneHall         0x0008   //电机2单个HALL故障

#define  Err_M1TwoHall         0x0014   //电机1两个HALL全故障
#define  Err_M2TwoHall         0x0015   //电机2两个HALL全故障

#define  Err_M1AllWire         0x0016   //电机1HALL线和电源线全部故障

#define  Err_M2AllWire         0x0017   //电机2HALL线和电源线全部故障
  
#define  Err_LSM6DSL           0x001E   //三轴传感器故障
#define  Err_Unbalance         0x0009   //桌面不平衡
#define  Err_RESET             0xffff   //复位状态时错误代码

#define  Err_Fall              0xfffe   //结构下滑


extern u8 M1HallN1ErrCnt;       //电机1霍尔信号1错误计数值
extern u8 M1HallN2ErrCnt;       //电机1霍尔信号2错误计数值
extern u8 M2HallN1ErrCnt;       //电机2霍尔信号1错误计数值
extern u8 M2HallN2ErrCnt;       //电机2霍尔信号2错误计数值

extern u8 M1ErrFlag;
extern u8 M2ErrFlag;
extern u8 T18minFlag;

extern u8 M1OverCurFlag;
extern u8 M2OverCurFlag;
extern u8 OverCurFlag;

void FaultDetect(void);
void CurrentDetect(void);
void HallDetect(void);
void TemperatureDetect(void);
void AgingTestFaultDetect(void);
#endif