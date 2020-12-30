#ifndef ERRDETECTION
#define ERRDETECTION
#include "stm8s.h"

#define  MAXCURRENT   380// 285//380 //���������˴�ΪADCֵ




//ϵͳ������
#define  Err_TimeEnd           0x0001   //����ʱ�䵽(��������2����)
#define  Err_Overheating       0x0002   //�������
#define  Err_M1Overcurrent     0x0003   //���1����
#define  Err_M2Overcurrent     0x0004   //���2����

#define  Err_M1Position        0x0005   //���1λ�ù���
#define  Err_M2Position        0x0006   //���2λ�ù���

#define  Err_M1OneHall         0x0007   //���1����HALL����
#define  Err_M2OneHall         0x0008   //���2����HALL����

#define  Err_M1TwoHall         0x0014   //���1����HALLȫ����
#define  Err_M2TwoHall         0x0015   //���2����HALLȫ����

#define  Err_M1AllWire         0x0016   //���1HALL�ߺ͵�Դ��ȫ������

#define  Err_M2AllWire         0x0017   //���2HALL�ߺ͵�Դ��ȫ������
  
#define  Err_LSM6DSL           0x001E   //���ᴫ��������
#define  Err_Unbalance         0x0009   //���治ƽ��
#define  Err_RESET             0xffff   //��λ״̬ʱ�������

#define  Err_Fall              0xfffe   //�ṹ�»�


extern u8 M1HallN1ErrCnt;       //���1�����ź�1�������ֵ
extern u8 M1HallN2ErrCnt;       //���1�����ź�2�������ֵ
extern u8 M2HallN1ErrCnt;       //���2�����ź�1�������ֵ
extern u8 M2HallN2ErrCnt;       //���2�����ź�2�������ֵ

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