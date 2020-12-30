#ifndef LED_H
#define LED_H
#include "stm8s.h"


#define  SCL_H  GPIO_WriteHigh(GPIOG, GPIO_PIN_0)
#define  SCL_L  GPIO_WriteLow(GPIOG, GPIO_PIN_0)

#define  SDA_H  GPIO_WriteHigh(GPIOE, GPIO_PIN_3)
#define  SDA_L  GPIO_WriteLow(GPIOE, GPIO_PIN_3)

#define Char_R    0x77
#define Char_S    0x6D


#define Char_     0x40
#define Char_Dot  0x80
#define Char_O    0x3F
#define Char_N    0x37

#define Char_Pt   0x09
#define Char_n    0x54



#define Char_A    0x77
#define Char_b    0x7c
#define Char_C    0x39
#define Char_d    0x5e
#define Char_E    0x79
#define Char_F    0x71
#define Char_P    0x73
#define Char_U    0x3E
#define Char_T    0x31
#define Char_y    0x6E
#define Char_H    0x76
#define Char_L    0x38


//各种显示模式
#define  HeightMode  0x30       
#define  SaveMode    0x31
#define  RemindMode  0x32       //提醒模式
#define  ErrorMode   0x33
#define  TestMode    0x34

#define  MenuMode    0x35
#define  FLash_HeightMode 0x36

void LED_Init(void);
void TM1650_Set(u8 addr, u8 data);
void LED_Display(void);
void DispNum(float num);
void LED_DisplayTest(void);
void Disp_Temprature(double temp);
void KeyBoardSoftVersionDisplay(void);

extern u32 LEDRest;
extern u8 Dis_Char[3];
extern u8 DispFlag;
extern u8 DisplayMode;
extern const u8 Data_Char[10];
extern u8 Y_Off_EN_Fleg;
extern u8 Adjust_State;
extern u16 Flash_Cnt;

#endif