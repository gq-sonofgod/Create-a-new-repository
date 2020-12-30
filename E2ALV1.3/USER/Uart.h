#ifndef UART_H
#define UART_H
#include "stm8s.h"



#define FrameHeader  0x9B
#define FrameEnd     0x9D


extern u8 SendBuffer[64]; //串口数据发送缓冲区
extern u8 Com3ReceiveBuffer[64]; //串口数据接收缓冲区
extern u8 Com1ReceiveBuffer[64];
extern u8 Com3ReceiveCount;  //接收的有效数据计数值
extern u8 Com1ReceiveCount;
extern u8 Com3UartState; //串口状态
extern u8 Com1UartState;
extern u8 Com3RX_Data;  //Uart3接收到的数据
extern u8 Com1RX_Data;

#define  LinkNull 0x00          //串口无连接
#define  LinkKey  0x01          //串口连接按键板
#define  LinkApp  0x02          //串口连接APP

extern u8 com1Link;
extern u8 com3Link;

void Uart_Config(void);
void PackageSendData(u8* data, u8* len);
void PackageSendData1(u8* data, u8* len);
u8 UnpackReceivedData(u8* data, u8* len);
void Uart3SendData(u8* data, u8 len);
void Uart1SendData(u8* data, u8 len);

#endif