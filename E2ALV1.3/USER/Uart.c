#include "Uart.h"
#include "Main.h"
#include "string.h"
#include "CRC16.h"
#include "Delay.h"

u8 SendBuffer[64]; //�������ݷ��ͻ�����
u8 Com3ReceiveBuffer[64];
u8 Com1ReceiveBuffer[64]; //�������ݽ��ջ�����
u8 Com3ReceiveCount = 0;  //���յ���Ч���ݼ���ֵ
u8 Com1ReceiveCount = 0;
u8 Com3UartState = IDLE; //����״̬
u8 Com1UartState = IDLE;
u8 Com3RX_Data=0;  //Uart3���յ�������
u8 Com1RX_Data=0;

u8 com1Link = LinkNull;
u8 com3Link = LinkNull;

/***********************************************************************************************************
* ��������: Uart_Config()
* �������: ��
* ����ֵ  : ��
* ��    ��: Uart3���ã�������9600,8λ����λ��1λֹͣλ,���������ж�
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
* ��������: PackageSendData()
* �������: data,Ҫ���͵����ݵ��׵�ַ�� *len,�������ݵ��ܳ���
* ����ֵ  : ��
* ��    ��: ����ͨ��Э�飬��װҪ���͵�����
************************************************************************************************************/
void PackageSendData(u8* data, u8* len)
{
  u8 temp[64],temp1[64],i,j = 0;
  u16 crc;
  
  memset(&temp[0], 0, sizeof(temp));
  memset(&temp1[0], 0, sizeof(temp1));
  memcpy(&temp[2], data, *len); //���ƴ�������
  temp[0] = FrameHeader; //����֡ͷ
  temp[1] = *len+3; //���볤��ֵ
  crc = CRC16MODBUS(&temp[1], *len+1); //����CRC16У��ֵ
  memcpy(&temp[*len+2], &crc, 2); //����У��ֵ
  for(i=3; i < *len+3+j; )
  {
    if((temp[i]==FrameHeader)||(temp[i]==FrameEnd))//���������������к�֡ͷ��֡βһ�������������ת���
    {
      memset(&temp1[0], 0, sizeof(temp1));
      memcpy(&temp1[0], &temp[i], *len+3+j-i);
      temp[i] = 0x5C; //����ת��� ��\'
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
* ��������: UnpackReceivedData()
* �������: data,Ҫ��������ݵ��׵�ַ��*len Ҫ��������ݵ��ܳ���(������֡ͷ��֡β)
* ����ֵ  : 1�� ������Ч�� 0��������Ч��
* ��    ��: ������յ������ݲ��ж�������Ч��
************************************************************************************************************/
u8 UnpackReceivedData(u8* data, u8* len)
{
  u8 temp[64];
  u16 crc,crctemp;
  
  memset(&temp[0], 0, 64);
  memcpy(&crctemp,(data+*len-2),2);     //���������е�CRCУ��ֵ
  crc = CRC16MODBUS(data, *len-2);      //�������ݵ�CRC16У��ֵ
  if(crctemp == crc) //У��ֵ��ȷ
    return 1;
  
  return 0;
}

/***********************************************************************************************************
* ��������: PackageSendData1()
* �������: data,Ҫ���͵����ݵ��׵�ַ�� *len,�������ݵ��ܳ���
* ����ֵ  : ��
* ��    ��: ����ͨ��Э�飬��װҪ���͵�����
************************************************************************************************************/
void PackageSendData1(u8* data, u8* len)
{
  u8 temp[64];
  u16 crc;
  
  memset(&temp[0], 0, sizeof(temp));
  memcpy(&temp[2], data, *len);                 //���ƴ�������
  temp[0] = FrameHeader;                        //����֡ͷ
  temp[1] = *len+3;                             //���볤��ֵ
  crc = CRC16MODBUS(&temp[1], *len+1);          //����CRC16У��ֵ
  memcpy(&temp[*len+2], &crc, 2);               //����У��ֵ  
  temp[*len+4] = FrameEnd;
  memcpy(data, &temp[0], *len+5);
  *len = *len+5;
}

/***********************************************************************************************************
* ��������: UartSendData()
* �������: data,Ҫ���͵����ݵ��׵�ַ��len��Ҫ���͵����ݵ��ܳ���
* ����ֵ  : ��
* ��    ��: ���ڷ������ݺ���
************************************************************************************************************/
void Uart3SendData(u8* data, u8 len)
{
  u8 i;
  
  for(i = 0; i < len; i++)
  {
    while((UART3->SR & 0x80)==0x00);    //ע��˴������üĴ���������ȡUART3_SR, ��Ҫʹ�ÿ⺯��
    UART3_SendData8(*(data+i));
  }
}

void Uart1SendData(u8* data, u8 len)
{
  u8 i;

  for(i = 0; i < len; i++)
  {
    while((UART1->SR & 0x80)==0x00); //ע��˴������üĴ���������ȡUART3_SR, ��Ҫʹ�ÿ⺯��
    UART1_SendData8(*(data+i));
  }
}