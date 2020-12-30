#ifndef KEY_H
#define KEY_H
#include "stm8s.h"


#define KEY1_Max   101
#define KEY2_Max   251
#define KEY2_Min   101
#define KEY3_Max   395
#define KEY3_Min   251
#define KEY4_Max   573
#define KEY4_Min   395
#define KEY5_Max   778
#define KEY5_Min   573
#define KEY6_Max   948
#define KEY6_Min   778


#define  KEY1  0x0001 //UP¼ü
#define  KEY2  0x0002 //DOWN¼ü
#define  KEY3  0x0004 //1¼ü
#define  KEY4  0x0008 //2¼ü
#define  KEY5  0x0010 //3¼ü
#define  KEY6  0x0020 //M¼ü
#define  KEY7  0x0040 //A¼ü
#define  KEY8  0x0060 //M+A×éºÏ¼ü
#define  KEY9  0x0030 //M+3×éºÏ¼ü
#define  KEY10 0x0024 //M+1×éºÏ¼ü
#define  KEY11 0x0028 //M+2×éºÏ¼ü
#define  KEY12 0x0080 //APPÅÜµ½Ö¸¶¨Î»ÖÃÖ¸Áî
#define  KEY13 0x0100 //4¼ü
#define  KEY14 0x0003 //UP+DOWN×éºÏ¼ü
#define  KEY15 0x0120 //M+4×éºÏ¼ü
#define  KEY_DEVINFO  0x0082 //APP»ñÈ¡Éè±¸ĞÅÏ¢ĞéÄâ¼ü
#define  KEY_NULL     0x0000
#define  KEY_Stop_M   0x1010
#define  KEY_ANTI     0X0084


#define STATE_NULL     0
#define STATE_WOBBLE   1
#define STATE_PRESS    2
#define STATE_LONG     3
#define STATE_HOLD     4

#define KEYPRESS   0x8000
#define KEYLONG    0x4000

#define KEY_UP        (KEY1|KEYPRESS)
#define KEY_DOWN      (KEY2|KEYPRESS)
#define KEY_M1        (KEY3|KEYPRESS)
#define KEY_M2        (KEY4|KEYPRESS)
#define KEY_M3        (KEY5|KEYPRESS)
#define KEY_M4        (KEY13|KEYPRESS)
#define KEY_SPECPOS   (KEY12|KEYPRESS)
#define KEY_MEM       (KEY6|KEYPRESS)
#define KEY_A         (KEY7|KEYPRESS)
#define KEY_A_LONG    (KEY7|KEYLONG)
#define KEY_VER       (KEY8|KEYLONG)
#define KEY_VER2      (KEY15|KEYLONG)
#define KEY_RST       (KEY9|KEYLONG)
#define KEY_UNIT      (KEY10|KEYLONG)
#define KEY_BASE      (KEY11|KEYLONG)
#define KEY_SENS      (KEY14|KEYPRESS)
#define KEY_LDOWN     (KEY2|KEYLONG)
#define KEY_MENU      (KEY6|KEYLONG)


#define Menu1_Max      3
#define Menu2_Max      3//2//ä¸€çº§èœå•ä¸ªæ•...°
#define Menu3_Max      2

extern u8 Menu1Flag;//ä¸€çº§èœå•æ ‡å¿—ä½
extern u8 Menu2Flag;//äºŒçº§èœå•æ ‡å¿—ä½
extern u8 Menu2Num;
extern u8 Menu1Num;
extern u8 Menu3Num;
extern u8 Menu2_0;

extern u8 KEY_Stop_M_Flag;
extern u8 Position;
extern u8 SavePosition;
extern u8 SaveState;
extern u16 MemCnt;
extern u8 RSTFlag;
extern u8 Release;
extern u8 InvalidFlag;
extern u16 LastKey;
extern u16 InvalidCnt;
extern u16 Key;
extern u16 Keyolder;
extern u8 InitFlag;
extern u8 BuzzerTest;
extern u8 AgingTest;
extern u8 KeyDisable;
extern u8 CombinationKey;
extern u8 VerSionDisplay;
extern u16 T2SecondCnt;
extern u8 KeyAValid;
extern u8 KeyMValid; 
extern u16 Com3KeyValue;
extern u16 Com1KeyValue;
extern u8 RunCntFlag;
extern u8 BlockFlag;
extern u8 Rst_EN;
extern u8 Check_EN;
extern u8 Balance_EN;
extern u16 KeyUpCnt;
extern u16 KeyDownCnt;
extern u16 VER_Cnt;
extern u8 sensKeyFlag;

u16 KeyScan(u16 value);
void KeyRespond(u16 value);

//u16 KeyScan(void);
//void KeyRespond(void);
void KeyRespondTest(u16 value);
void KeyNullProcess();

extern void Balance_Data_Refresh();


extern u8 DisplayRemind;
extern  u8 RST_Sat;
#endif