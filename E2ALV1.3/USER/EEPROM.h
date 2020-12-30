#ifndef EEPROM_H
#define EEPROM_H
#include "stm8s.h"

#define  ErrCodeAddr           0x4000                           //�������洢��ַ
#define  M1CurrentHallAddr     ErrCodeAddr+0x0002               //���1��ǰHALLֵ�洢��ַ
#define  M2CurrentHallAddr     M1CurrentHallAddr+0x0002         //���2��ǰHALLֵ�洢��ַ
#define  M3CurrentHallAddr     M2CurrentHallAddr+0x0002         //���3��ǰHALLֵ�洢��ַ
#define  M1LimitDownAddr       M3CurrentHallAddr+0x0002         //���1�����¶˵�λ�õ�HALLֵ�洢��ַ
#define  M2LimitDownAddr       M1LimitDownAddr+0x0002           //���2�����¶˵�λ�õ�HALLֵ�洢��ַ
#define  M3LimitDownAddr       M2LimitDownAddr+0x0002           //���3�����¶˵�λ�õ�HALLֵ�洢��ַ
#define  M1Record1Addr         M3LimitDownAddr+0x0002           //���1�洢��1��HALLֵ�洢��ַ
#define  M2Record1Addr         M1Record1Addr+0x0002             //���2�洢��1��HALLֵ�洢��ַ
#define  M3Record1Addr         M2Record1Addr+0x0002             //���3�洢��1��HALLֵ�洢��ַ
#define  M1Record2Addr         M3Record1Addr+0x0002             //���1�洢��2��HALLֵ�洢��ַ
#define  M2Record2Addr         M1Record2Addr+0x0002             //���2�洢��2��HALLֵ�洢��ַ
#define  M3Record2Addr         M2Record2Addr+0x0002             //���3�洢��2��HALLֵ�洢��ַ
#define  M1Record3Addr         M3Record2Addr+0x0002             //���1�洢��3��HALLֵ�洢��ַ
#define  M2Record3Addr         M1Record3Addr+0x0002             //���2�洢��3��HALLֵ�洢��ַ
#define  M3Record3Addr         M2Record3Addr+0x0002             //���3�洢��3��HALLֵ�洢��ַ
#define  SaveIndexAddr         M3Record3Addr+0x0002             //�洢��������־�Ĵ洢��ַ
#define  InitFlagAddr          SaveIndexAddr+0x0001             //��ʼ����־λ�洢��ַ
#define  TimeNowAddr           InitFlagAddr+0x0001
#define  SoftStateAddr         TimeNowAddr+0x0001               //���״̬��־λ(����״̬�ͳ���״̬)
#define  UnitAddr              SoftStateAddr+0x0001             //��λ��־λ

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