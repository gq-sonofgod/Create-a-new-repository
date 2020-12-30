#ifndef UART_H
#define UART_H
#include "stm8s.h"



#define FrameHeader  0x9B
#define FrameEnd     0x9D


extern u8 SendBuffer[64]; //�������ݷ��ͻ�����
extern u8 Com3ReceiveBuffer[64]; //�������ݽ��ջ�����
extern u8 Com1ReceiveBuffer[64];
extern u8 Com3ReceiveCount;  //���յ���Ч���ݼ���ֵ
extern u8 Com1ReceiveCount;
extern u8 Com3UartState; //����״̬
extern u8 Com1UartState;
extern u8 Com3RX_Data;  //Uart3���յ�������
extern u8 Com1RX_Data;

#define  LinkNull 0x00          //����������
#define  LinkKey  0x01          //�������Ӱ�����
#define  LinkApp  0x02          //��������APP

extern u8 com1Link;
extern u8 com3Link;

void Uart_Config(void);
void PackageSendData(u8* data, u8* len);
void PackageSendData1(u8* data, u8* len);
u8 UnpackReceivedData(u8* data, u8* len);
void Uart3SendData(u8* data, u8 len);
void Uart1SendData(u8* data, u8 len);

#endif