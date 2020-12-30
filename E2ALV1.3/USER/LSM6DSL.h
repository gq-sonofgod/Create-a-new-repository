#ifndef LSM6DSL_H
#define LSM6DSL_H
#include "stm8s.h"


#define SCL1_H  GPIO_WriteHigh(GPIOG, GPIO_PIN_0)
#define SCL1_L  GPIO_WriteLow(GPIOG, GPIO_PIN_0)


#define SDA1_H  GPIO_WriteHigh(GPIOC, GPIO_PIN_5)
#define SDA1_L  GPIO_WriteLow(GPIOC, GPIO_PIN_5)


#define ReadSDA1  GPIO_ReadInputPin(GPIOC, GPIO_PIN_5)

void IIC2_Init(void);
void IIC2_Start(void);
void IIC2_Stop(void);
u8 IIC2_WaiteAck(void);
void IIC2_SendAck(void);
void IIC2_SendNoAck(void);
void IIC2_WriteByte(u8 data);
u8 IIC2_ReadByte(u8 ack);

void LSM6DSL_Init(void);

#endif