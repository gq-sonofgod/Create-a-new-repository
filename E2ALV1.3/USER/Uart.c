#include "Uart.h"
#include "Main.h"
#include "string.h"
#include "CRC16.h"
#include "Delay.h"

u8 SendBuffer[64]; //串口数据发送缓冲区
u8 Com3ReceiveBuffer[64];
u8 Com1ReceiveBuffer[64]; //串口数据接收缓冲区
u8 Com3ReceiveCount = 0;  //接收的有效数据计数值
u8 Com1ReceiveCount = 0;
u8 Com3UartState = IDLE; //串口状态
u8 Com1UartState = IDLE;
u8 Com3RX_Data=0;  //Uart3接收到的数据
u8 Com1RX_Data=0;

u8 com1Link = LinkNull;
u8 com3Link = LinkNull;

/***********************************************************************************************************
* 函数名称: Uart_Config()
* 输入参数: 无
* 返回值  : 无
* 功    能: Uart3设置，波特率9600,8位数据位，1位停止位,开启接收中断
************************************************************************************************************/
void Uart_Config(void)
{
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_OD_LOW_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);
  UART3_DeInit();
  UART3_Init((uint32_t)9600,UART3_WORDLENGTH_8D,UART3_STOPBITS_1,UART3_PARITY_NO,UART3_MODE_TXRX_ENABLE);
  UART3_ITConfig(UART3_IT_RXNE, ENABLE);
  UART3_Cmd(ENABLE);
  
  GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUT_OD_LOW_FAST);
  GPIO_Init(GPIOA, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
  UART1_DeInit();
  UART1_Init((u32)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, \
  UART1_PARITY_NO , UART1_SYNCMODE_CLOCK_DISABLE , UART1_MODE_TXRX_ENABLE);
  UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
  UART1_Cmd(ENABLE);
}

/***********************************************************************************************************
* 函数名称: PackageSendData()
* 输入参数: data,要发送的数据的首地址； *len,发送数据的总长度
* 返回值  : 无
* 功    能: 根据通信协议，封装要发送的数据
************************************************************************************************************/
void PackageSendData(u8* data, u8* len)
{
  u8 temp[64],temp1[64],i,j = 0;
  u16 crc;
  
  memset(&temp[0], 0, sizeof(temp));
  memset(&temp1[0], 0, sizeof(temp1));
  memcpy(&temp[2], data, *len); //复制传输数据
  temp[0] = FrameHeader; //加入帧头
  temp[1] = *len+3; //加入长度值
  crc = CRC16MODBUS(&temp[1], *len+1); //计算CRC16校验值
  memcpy(&temp[*len+2], &crc, 2); //复制校验值
  for(i=3; i < *len+3+j; )
  {
    if((temp[i]==FrameHeader)||(temp[i]==FrameEnd))//如果传输的数据中有和帧头或帧尾一样的数据则插入转义符
    {
      memset(&temp1[0], 0, sizeof(temp1));
      memcpy(&temp1[0], &temp[i], *len+3+j-i);
      temp[i] = 0x5C; //加入转义符 ‘\'
      memcpy(&temp[i+1], &temp1[0], *len+3+j-i);
      i = i+2;
      j++;
    }
    else
      i++;
  }
  temp[*len+4+j] = FrameEnd;
  memcpy(data, &temp[0], *len+5+j);
  *len = *len+5+j;
}

/***********************************************************************************************************
* 函数名称: UnpackReceivedData()
* 输入参数: data,要解包的数据的首地址，*len 要解包的数据的总长度(不包括帧头和帧尾)
* 返回值  : 1， 数据有效， 0，数据无效。
* 功    能: 解包接收到的数据并判断数据有效性
************************************************************************************************************/
u8 UnpackReceivedData(u8* data, u8* len)
{
  u8 temp[64];
  u16 crc,crctemp;
  
  memset(&temp[0], 0, 64);
  memcpy(&crctemp,(data+*len-2),2);     //复制数据中的CRC校验值
  crc = CRC16MODBUS(data, *len-2);      //计算数据的CRC16校验值
  if(crctemp == crc) //校验值正确
    return 1;
  
  return 0;
}

/***********************************************************************************************************
* 函数名称: PackageSendData1()
* 输入参数: data,要发送的数据的首地址； *len,发送数据的总长度
* 返回值  : 无
* 功    能: 根据通信协议，封装要发送的数据
************************************************************************************************************/
void PackageSendData1(u8* data, u8* len)
{
  u8 temp[64];
  u16 crc;
  
  memset(&temp[0], 0, sizeof(temp));
  memcpy(&temp[2], data, *len);                 //复制传输数据
  temp[0] = FrameHeader;                        //加入帧头
  temp[1] = *len+3;                             //加入长度值
  crc = CRC16MODBUS(&temp[1], *len+1);          //计算CRC16校验值
  memcpy(&temp[*len+2], &crc, 2);               //复制校验值  
  temp[*len+4] = FrameEnd;
  memcpy(data, &temp[0], *len+5);
  *len = *len+5;
}

/***********************************************************************************************************
* 函数名称: UartSendData()
* 输入参数: data,要发送的数据的首地址；len，要发送的数据的总长度
* 返回值  : 无
* 功    能: 串口发送数据函数
************************************************************************************************************/
void Uart3SendData(u8* data, u8 len)
{
  u8 i;
  
  for(i = 0; i < len; i++)
  {
    while((UART3->SR & 0x80)==0x00);    //注意此处必须用寄存器操作读取UART3_SR, 不要使用库函数
    UART3_SendData8(*(data+i));
  }
}

void Uart1SendData(u8* data, u8 len)
{
  u8 i;

  for(i = 0; i < len; i++)
  {
    while((UART1->SR & 0x80)==0x00); //注意此处必须用寄存器操作读取UART3_SR, 不要使用库函数
    UART1_SendData8(*(data+i));
  }
}