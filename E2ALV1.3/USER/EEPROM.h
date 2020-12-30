#ifndef EEPROM_H
#define EEPROM_H
#include "stm8s.h"

#define  ErrCodeAddr           0x4000                           //错误代码存储地址
#define  M1CurrentHallAddr     ErrCodeAddr+0x0002               //电机1当前HALL值存储地址
#define  M2CurrentHallAddr     M1CurrentHallAddr+0x0002         //电机2当前HALL值存储地址
#define  M3CurrentHallAddr     M2CurrentHallAddr+0x0002         //电机3当前HALL值存储地址
#define  M1LimitDownAddr       M3CurrentHallAddr+0x0002         //电机1运行下端点位置的HALL值存储地址
#define  M2LimitDownAddr       M1LimitDownAddr+0x0002           //电机2运行下端点位置的HALL值存储地址
#define  M3LimitDownAddr       M2LimitDownAddr+0x0002           //电机3运行下端点位置的HALL值存储地址
#define  M1Record1Addr         M3LimitDownAddr+0x0002           //电机1存储点1的HALL值存储地址
#define  M2Record1Addr         M1Record1Addr+0x0002             //电机2存储点1的HALL值存储地址
#define  M3Record1Addr         M2Record1Addr+0x0002             //电机3存储点1的HALL值存储地址
#define  M1Record2Addr         M3Record1Addr+0x0002             //电机1存储点2的HALL值存储地址
#define  M2Record2Addr         M1Record2Addr+0x0002             //电机2存储点2的HALL值存储地址
#define  M3Record2Addr         M2Record2Addr+0x0002             //电机3存储点2的HALL值存储地址
#define  M1Record3Addr         M3Record2Addr+0x0002             //电机1存储点3的HALL值存储地址
#define  M2Record3Addr         M1Record3Addr+0x0002             //电机2存储点3的HALL值存储地址
#define  M3Record3Addr         M2Record3Addr+0x0002             //电机3存储点3的HALL值存储地址
#define  SaveIndexAddr         M3Record3Addr+0x0002             //存储点索引标志的存储地址
#define  InitFlagAddr          SaveIndexAddr+0x0001             //初始化标志位存储地址
#define  TimeNowAddr           InitFlagAddr+0x0001
#define  SoftStateAddr         TimeNowAddr+0x0001               //软件状态标志位(测试状态和出厂状态)
#define  UnitAddr              SoftStateAddr+0x0001             //单位标志位

#define  INITIALIZED   0xAA

#define  M1  0x40
#define  M2  0x41
#define  M3  0x42
#define  M4  0x43

//#define  NULL      0
#define  BEGIN     1
#define  CONFIRM   2
#define  END       3

extern u8 Buffer[128];
extern u8 SaveIndex;
extern u8 Balance_ON;
void EEPROM_Init(void);
void EEPROM_Write(void);
void SysDataRead(void);
void PositionSave(void);
void LowPowerSave(void);
void DeleteSavedHeight(void);
void SysDataReadTest(void);









#endif