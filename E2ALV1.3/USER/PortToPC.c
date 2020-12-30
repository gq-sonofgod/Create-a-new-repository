#include "PortToPC.h"
#include "Main.h"
#include "EEPROM.h"
#include "Uart.h"
#include "stm8s_it.h"
#include "string.h"
#include "HealthMode.h"

const u8 SpeedArray[11] = {60,58,56,55,53,51,50,49,47,46,44};
const u8 SpeedArray2[11] ={38,37,36,35,34,33,32,31,30,29,28};

//接收处理来自上位机软件的命令
void RespondCmdOfPC(void)
{
  u16 temp,temp1;
  u8 ANSWER[7] = {0x9B, 0x05, 0x20, 0x01, 0xC1, 0xB9, 0x9D};
  u8 Data[25] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  u8 len,i;
  
  if((Com1DeviceState == SLAVER)&&(Com1UartState == FINISHED))
  { 
     if(UnpackReceivedData(&Com1ReceiveBuffer[0], &Com1ReceiveCount))
     {
      if(Com1ReceiveBuffer[1] == 0x04)
      {
        switch(Com1ReceiveBuffer[2])
        {
          case SetParamete:
            if(Com1ReceiveCount == 15)
            {       
              memcpy(&temp, &Com1ReceiveBuffer[3], 2);
              memcpy(&temp1, &Com1ReceiveBuffer[5], 2);
              MaxHeight = ((float)temp1)/10.0f;              
              BaseHeight = ((float)temp)/10.0f;
              memcpy(&temp, &Com1ReceiveBuffer[7], 2);
              memcpy(&temp1, &Com1ReceiveBuffer[9], 2);
              MinColumnHeight = ((float)temp)/10.0f;
              MaxColumnHeight = ((float)temp1)/10.0f; 
              DiffHall = (u16)((MaxColumnHeight-MinColumnHeight+0.2)*RATE);
              Speed = Com1ReceiveBuffer[11];
              Speed = SpeedArray[38-Speed];
              Sensitivity = Com1ReceiveBuffer[12];
              memset(&Buffer[0], 0, sizeof(Buffer));
              Buffer[33] = INITIALIZED;
              Buffer[35] = RELEASE;
              Buffer[36] = Unit;
              Buffer[34] = TIMEVAL;
              memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));
              memcpy(&Buffer[41],&DiffHall, 2);
              memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));
              Buffer[47] = Speed;
              Buffer[48] = Sensitivity;
              memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));
              memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));
              EEPROM_Write();
              Uart1SendData(&ANSWER[0], 7);
              ((void (*)(void))0x8000)();
            }
            break;
            
          case GetParamete:
            if(Com1ReceiveCount == 5)
            {
              Data[0] = GetParamete;              
              temp = MINHEIGHT*10;
              temp1 = MAXHEIGHT*10;
              memcpy(&Data[1], &temp, 2);
              memcpy(&Data[3], &temp1, 2);                                           
              Data[6] = SENS;
              temp = (u16)(BaseHeight*10.0f);
              temp1 = (u16)(MaxHeight*10.0f);
              memcpy(&Data[7], &temp, 2);
              memcpy(&Data[9], &temp1, 2);
              
              temp = (u16)(MinColumnHeight*10.0f);
              temp1 = (u16)(MaxColumnHeight*10.0f);
              memcpy(&Data[11], &temp, 2);
              memcpy(&Data[13], &temp1, 2);
              for(i=0; i < 11; i++)
              {
                if(SpeedArray[i]==Speed)
                  Data[15] = SpeedArray2[i];
                if(SpeedArray[i]==SPEED)
                  Data[5] = SpeedArray2[i];
              }
              Data[16] = Sensitivity;
              len = 17;
              PackageSendData1(&Data[0], &len);
              Uart1SendData(&Data[0],len); 
            }
            break;     
        default:
          break;
        }
      }
    }
    else
    {
       Com1UartState = WAIT;
    }
    Com1UartState = WAIT;
    Com1ReceiveCount = 0;
    memset(&Com1ReceiveBuffer[0], 0, sizeof(Com1ReceiveBuffer));
  }
  
  if((Com3DeviceState==SLAVER)&&(Com3UartState == FINISHED))
  {    
     if(UnpackReceivedData(&Com3ReceiveBuffer[0], &Com3ReceiveCount))
     {
      if(Com3ReceiveBuffer[1] == 0x04)
      {
        switch(Com3ReceiveBuffer[2])
        {
          case SetParamete:
            if(Com3ReceiveCount == 15)
            {       
              memcpy(&temp, &Com3ReceiveBuffer[3], 2);
              memcpy(&temp1, &Com3ReceiveBuffer[5], 2);
              MaxHeight = ((float)temp1)/10.0f;              
              BaseHeight = ((float)temp)/10.0f;
              memcpy(&temp, &Com3ReceiveBuffer[7], 2);
              memcpy(&temp1, &Com3ReceiveBuffer[9], 2);
              MinColumnHeight = ((float)temp)/10.0f;
              MaxColumnHeight = ((float)temp1)/10.0f; 
              DiffHall = (u16)((MaxColumnHeight-MinColumnHeight+0.2)*RATE);
              Speed = Com3ReceiveBuffer[11];
              Speed = SpeedArray[38-Speed];
              Sensitivity = Com3ReceiveBuffer[12];
              memset(&Buffer[0], 0, sizeof(Buffer));
              Buffer[33] = INITIALIZED;
              Buffer[35] = RELEASE;
              Buffer[36] = Unit;
              Buffer[34] = TIMEVAL;
              memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));
              memcpy(&Buffer[41],&DiffHall, 2);
              memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));
              Buffer[47] = Speed;
              Buffer[48] = Sensitivity;
              memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));
              memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));
              EEPROM_Write();
              Uart3SendData(&ANSWER[0], 7);
              ((void (*)(void))0x8000)();
            }
            break;
            
          case GetParamete:
            if(Com3ReceiveCount == 5)
            {
              Data[0] = GetParamete;              
              temp = MINHEIGHT*10;
              temp1 = MAXHEIGHT*10;
              memcpy(&Data[1], &temp, 2);
              memcpy(&Data[3], &temp1, 2);                                           
              Data[6] = SENS;
              temp = (u16)(BaseHeight*10.0f);
              temp1 = (u16)(MaxHeight*10.0f);
              memcpy(&Data[7], &temp, 2);
              memcpy(&Data[9], &temp1, 2);
              
              temp = (u16)(MinColumnHeight*10.0f);
              temp1 = (u16)(MaxColumnHeight*10.0f);
              memcpy(&Data[11], &temp, 2);
              memcpy(&Data[13], &temp1, 2);
              for(i=0; i < 11; i++)
              {
                if(SpeedArray[i]==Speed)
                  Data[15] = SpeedArray2[i];
                if(SpeedArray[i]==SPEED)
                  Data[5] = SpeedArray2[i];
              }
              Data[16] = Sensitivity;
              len = 17;
              PackageSendData1(&Data[0], &len);
              Uart3SendData(&Data[0],len); 
            }
            break;     
        default:
          break;
        }
      }
    }
    else
    {
      Com3UartState = WAIT;
    }
    Com3UartState = WAIT;
    Com3ReceiveCount = 0;
    memset(&Com3ReceiveBuffer[0], 0, sizeof(Com3ReceiveBuffer));
  }
}


