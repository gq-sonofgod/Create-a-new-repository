#include "EXTI.h"
#include "Motor.h"
#include "Main.h"
#include "FaultDetection.h"

/***********************************************************************************************************
* 函数名称: EXTI_Init()
* 输入参数: 无
* 返回值  : 无
* 功    能: 外部中断引脚初始化PE2,PE1; PC7，PC6; PD2，PD3
************************************************************************************************************/
void EXTI_Init(void)
{
  GPIO_Init(GPIOE, GPIO_PIN_2|GPIO_PIN_1, GPIO_MODE_IN_FL_IT); //M1
  GPIO_Init(GPIOC, GPIO_PIN_7|GPIO_PIN_6, GPIO_MODE_IN_FL_IT); //M2
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOE, EXTI_SENSITIVITY_RISE_FALL);//变化沿触发
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_RISE_FALL);//变化沿触发
  //GPIO_Init(GPIOG,GPIO_PIN_1,GPIO_MODE_IN_FL_NO_IT); //独立按键
  //GPIO_Init(GPIOE,GPIO_PIN_0,GPIO_MODE_IN_FL_NO_IT); //独立按键
  //GPIO_Init(GPIOC,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
}

//M1的霍尔信号触发的外部中断
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
  u8 NewState;
  
  NewState = ((GPIO_ReadInputPin(GPIOE,GPIO_PIN_2)>>2))|GPIO_ReadInputPin(GPIOE,GPIO_PIN_1); //获取当前的HALL电平状态
  if(M1State.HallState==0x02)
  {
    if(NewState == 0x03)
    {
      M1State.HallNow++;
      M1HallN2ErrCnt = 0;
      M1HallN1ErrCnt++;
    }
    else if(NewState == 0x00)
    {
      M1State.HallNow--;
      M1HallN1ErrCnt = 0;
      M1HallN2ErrCnt++;
    }
    M1State.HallState = NewState;
  }
  else if(M1State.HallState==0x03)
  {
    if(NewState == 0x01)
    {
      M1State.HallNow++;
      M1HallN1ErrCnt = 0;
      M1HallN2ErrCnt++;
    }
    else if(NewState == 0x02)
    {
      M1State.HallNow--;
      M1HallN2ErrCnt = 0;
      M1HallN1ErrCnt++;
    }
    M1State.HallState = NewState;
  }
  else if(M1State.HallState==0x01)
  {
    if(NewState == 0x00)
    {
      M1State.HallNow++;
      M1HallN2ErrCnt = 0;
      M1HallN1ErrCnt++;
    }
    else if(NewState == 0x03)
    {
      M1State.HallNow--;
      M1HallN1ErrCnt = 0;
      M1HallN2ErrCnt++;
    }
    M1State.HallState = NewState;
  }
  else if(M1State.HallState==0x00)
  {
    if(NewState == 0x02)
    {
      M1State.HallNow++;
      M1HallN1ErrCnt = 0;
      M1HallN2ErrCnt++;
    }
    else if(NewState == 0x01)
    {
      M1State.HallNow--;
      M1HallN2ErrCnt = 0;
      M1HallN1ErrCnt++;
    }
    M1State.HallState = NewState;
  }
}


//M2的霍尔信号触发的外部中断
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  u8 NewState;
  
  NewState = ((GPIO_ReadInputPin(GPIOC,GPIO_PIN_7)>>7))|((GPIO_ReadInputPin(GPIOC,GPIO_PIN_6)>>5)); //获取当前的HALL电平状态
  if(M2State.HallState==0x02)
  {
    if(NewState == 0x03)
    {
      M2State.HallNow++;
      M2HallN2ErrCnt = 0;
      M2HallN1ErrCnt++;
    }
    else if(NewState == 0x00)
    {
      M2State.HallNow--;
      M2HallN1ErrCnt = 0;
      M2HallN2ErrCnt++;
    }
    M2State.HallState = NewState;
  }
  else if(M2State.HallState==0x03)
  {
    if(NewState == 0x01)
    {
      M2State.HallNow++;
      M2HallN1ErrCnt = 0;
      M2HallN2ErrCnt++;
    }
    else if(NewState == 0x02)
    {
      M2State.HallNow--;
      M2HallN2ErrCnt = 0;
      M2HallN1ErrCnt++;
    }
    M2State.HallState = NewState;
  }
  else if(M2State.HallState==0x01)
  {
    if(NewState == 0x00)
    {
      M2State.HallNow++;
      M2HallN2ErrCnt = 0;
      M2HallN1ErrCnt++;
    }
    else if(NewState == 0x03)
    {
      M2State.HallNow--;
      M2HallN1ErrCnt = 0;
      M2HallN2ErrCnt++;
    }
    M2State.HallState = NewState;
  }
  else if(M2State.HallState==0x00)
  {
    if(NewState == 0x02)
    {
      M2State.HallNow++;
      M2HallN1ErrCnt = 0;
      M2HallN2ErrCnt++;
    }
    else if(NewState == 0x01)
    {
      M2State.HallNow--;
      M2HallN2ErrCnt = 0;
      M2HallN1ErrCnt++;
    }
    M2State.HallState = NewState;
  }
}



/***********************************************************************************************************
* 函数名称: GetLevel()
* 输入参数: 无
* 返回值  : 无
* 功    能: 获取三个电机的HALL信号电平状态
************************************************************************************************************/
void GetLevel(void)
{
  M1State.HallState = ((GPIO_ReadInputPin(GPIOE,GPIO_PIN_2)>>2))|GPIO_ReadInputPin(GPIOE,GPIO_PIN_1);
  M2State.HallState = ((GPIO_ReadInputPin(GPIOC,GPIO_PIN_7)>>7))|((GPIO_ReadInputPin(GPIOC,GPIO_PIN_6)>>5));   
}
