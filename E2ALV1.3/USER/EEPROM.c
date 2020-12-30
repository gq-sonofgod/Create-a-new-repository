#include "EEPROM.h"
#include "string.h"
#include "Main.h"
#include "Motor.h"
#include "LED.h"
#include "Key.h"
#include "ADC.h"
#include "stm8s_it.h"
#include "Delay.h"
#include "HealthMode.h"
#include "FaultDetection.h"
#include "Balance.h"
#include "Tap.h"
u8 Buffer[128];
u8 SaveIndex = 0;
u8 Balance_ON=0;
/***********************************************************************************************************
* ��������: EEPROM_Init()
* �������: ��
* ����ֵ  : ��
* ��    ��: ������Ƭ���ڲ�EEPROM����
************************************************************************************************************/
void EEPROM_Init(void)
{
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD); //���ñ��ʱ��Ϊ��׼���ʱ�� 
}

/***********************************************************************************************************
* ��������: EEPROM_Write()
* �������: ��
* ����ֵ  : ��
* ��    ��: �ڲ�EEPROM����д�����д���Ϊ��һ��
************************************************************************************************************/
void EEPROM_Write(void)
{
  Delay_ms(2);  
  FLASH_Unlock(FLASH_MEMTYPE_DATA); //�����ڲ�EEPROM
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
  {}
  FLASH_ProgramBlock(0,FLASH_MEMTYPE_DATA,FLASH_PROGRAMMODE_STANDARD,&Buffer[0]); //д��0��
  while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
  {}
  FLASH_Lock(FLASH_MEMTYPE_DATA);
  Delay_ms(2);
}

/***********************************************************************************************************
* ��������: EEPROM_Erase()
* �������: ��
* ����ֵ  : ��
* ��    ��: �����ڲ�EEPROM������
************************************************************************************************************/
void EEPROM_Erase(void)
{
  FLASH_Unlock(FLASH_MEMTYPE_DATA); //�����ڲ�EEPROM
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
  {}
  FLASH_EraseBlock(0, FLASH_MEMTYPE_DATA); //������0��
  while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
  {}
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}


/***********************************************************************************************************
* ��������: SysDataRead()
* �������: ��
* ����ֵ  : ��
* ��    ��: ��ϵͳû�о�����ʼ������г�ʼ���������ȡ��ȡ����
************************************************************************************************************/
void SysDataRead(void)
{
  u8 temp,i;
  
  memset(&Buffer[0], 0, sizeof(Buffer));        //��ʼ��Buffer�Ĵ���
  temp = FLASH_ReadByte(InitFlagAddr);          //��ȡϵͳ��ʼ����־λ
  
  //temp = INITIALIZED;
  
  if(temp != INITIALIZED)                       //û�г�ʼ���Ƚ��г�ʼ��
  {   
    Buffer[33] = INITIALIZED;   //INITIALIZED = 0xAA
    ErrCode = 0xffff;
    Dis_Char[0] = Char_R;
    Dis_Char[1] = Char_S;
    Dis_Char[2] = Char_T;
    SysState = RESET;
    memcpy(&Buffer[0],&ErrCode, sizeof(ErrCode));               //�洢���ϴ�����뻺������Buffer[0]��Buffer[1]��
    M1State.LimitDown = BASEHALL;
    M1State.HallNow = BASEHALL;
    M1State.HallLast = BASEHALL;
    memcpy(&Buffer[8],&M1State.LimitDown, sizeof(M1State.LimitDown));   //u16 LimitDown;   //������е�HALL����ֵ
                                                                        //Buffer[8],Buffer[9]�洢M1������е�HALL����ֵ
    M2State.LimitDown = BASEHALL;
    M2State.HallNow = BASEHALL;
    M2State.HallLast = BASEHALL;
    memcpy(&Buffer[10],&M2State.LimitDown, sizeof(M2State.LimitDown));  //Buffer[10],Buffer[11]�洢M2������е�HALL����ֵ
    TimeNow = TIMEVAL;                  //TIMEVAL = 45 
    TIMEcnt = ONE_MINUTE* TimeNow;      //ONE_MINUTE = 60000
    memcpy(&Buffer[34],&TimeNow, sizeof(TimeNow));
    SoftState = RELEASE;
    Buffer[35] = SoftState;             //#define  TEST     0  #define  RELEASE  1

        
    Buffer[36] = Unit;          //Buffer[36]��¼�߶ȵ�λ��UNIT = 0(CM),UNIT = 1(INCH)
    
    //#define MINHEIGHT 60.5f  #define MAXHEIGHT 126.0f  #define BASEHEIGHT 60.5f
    memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));       //Buffer[37],Buffer[38],Buffer[39],Buffer[40]��¼����  BASEHEIGHT
    memcpy(&Buffer[41],&DiffHall, 2);        //Buffer[41],Buffer[42]��¼����  DiffHall��DiffHall = HALLֵ���ֵ - HALLֵ��Сֵ��
    memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));         //Buffer[43],Buffer[44],Buffer[45],Buffer[46]��¼#define MAXHEIGHT 126.0f
    Buffer[47] = SPEED;         //Buffer[47]��¼SPEED
    Buffer[48] = SENS;          //Buffer[49]��¼SENS
    //MinColumnHeight = BASEHEIGHT
    memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));//Buffer[49],Buffer[50],Buffer[51],Buffer[52]������¼ BASEHEIGHT ֵ       
    memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));//Buffer[53],Buffer[54],Buffer[55],Buffer[56]������¼ MAXHEIGHT ֵ
    
    //memcpy(&Buffer[59],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[59]��Buffer[60]�洢���������Ӧ�������������HALL��ֵ
    
    memcpy(&Buffer[61],&Balance_Data.TwoMotorRunFlag,sizeof(Balance_Data.TwoMotorRunFlag));
    
    
    //Balance_Data.TwoMotorRunFlag = 0;
      
    EEPROM_Write();
    
    DisableHPSlopeFilter();     //��һ���ϵ磬�����ƫ�����ɼ�
    
    //Delay_ms(200);  
    
    //Buffer[61] = FLASH_ReadByte(ErrCodeAddr + 61);
    //memcpy(&Balance_Data.TwoMotorRunFlag,&Buffer[61],sizeof(Balance_Data.TwoMotorRunFlag));
    //Delay_ms(200);
  }
  else //�Ѿ���ʼ�����ȡ�洢������
  {
    //for(i = 0; i < 61; i++)
    for(i = 0; i <= 76; i++)
    {
      Buffer[i] = FLASH_ReadByte(ErrCodeAddr + i);
    }
    SoftState = Buffer[35];
    Unit = Buffer[36];
    Sensitivity = Buffer[48];
    Speed = Buffer[47];
    TapControlFlag= Buffer[70];
    Tap_Parameter.TapCtlThreshold= Buffer[71];
    Tap_Parameter.TapCtlThreshold=Tap_Parameter.TapCtlThreshold<<8;
    Tap_Parameter.TapCtlThreshold=Tap_Parameter.TapCtlThreshold+Buffer[72];
    Balance_ON=Buffer[76];
    //BaseHeight = Buffer[37];
    memcpy(&BaseHeight, &Buffer[37], sizeof(BaseHeight));//���е�����͸߶� 60.5cm
    memcpy(&MaxHeight, &Buffer[43], sizeof(MaxHeight)); //���е�����߸߶� 126cm
    
    //���е�����͸߶� 60.5cm
    memcpy(&MinColumnHeight, &Buffer[49], sizeof(MinColumnHeight));
    //���е�����߸߶� 126cm
    memcpy(&MaxColumnHeight, &Buffer[53], sizeof(MaxColumnHeight));
    
    //���е�����߸߶Ⱥ���͸߶ȵĲ�ֵ ��HALLֵ��ʾ
    memcpy(&DiffHall, &Buffer[41], 2);     
    
    //��ȡ������Ϣ
    memcpy(&ErrCode,&Buffer[0],sizeof(ErrCode));                

    //��ȡM1�����ǰHALLֵ
    memcpy(&M1State.HallNow,&Buffer[2],sizeof(M1State.HallNow));
    //��ȡM2�����ǰHALLֵ
    memcpy(&M2State.HallNow,&Buffer[4],sizeof(M2State.HallNow));
    
    //��ȡM1������е�HALL����ֵ
    memcpy(&M1State.LimitDown,&Buffer[8],sizeof(M1State.LimitDown));
    //ͨ��M1������е�HALL�����ƺͲ�ֵ�����M1������е�HALL����ֵ
    M1State.LimitUp = M1State.LimitDown + DiffHall;
    
    //��ȡM2������е�HALL����ֵ
    memcpy(&M2State.LimitDown,&Buffer[10],sizeof(M2State.LimitDown));
    //ͨ��M2������е�HALL�����ƺͲ�ֵ�����M2������е�HALL����ֵ
    M2State.LimitUp = M2State.LimitDown + DiffHall;
   
    //��ȡM1,M2����ĵ�1����¼ֵ���ֱ����Record[0]��
    memcpy(&M1State.Record[0],&Buffer[14],sizeof(M1State.Record[0]));   
    memcpy(&M2State.Record[0],&Buffer[16],sizeof(M2State.Record[0]));

    //��ȡM1,M2����ĵ�2����¼ֵ���ֱ����Record[1]��
    memcpy(&M1State.Record[1],&Buffer[20],sizeof(M1State.Record[1]));
    memcpy(&M2State.Record[1],&Buffer[22],sizeof(M2State.Record[1]));

    //��ȡM1,M2����ĵ�3����¼ֵ���ֱ����Record[2]��
    memcpy(&M1State.Record[2],&Buffer[26],sizeof(M1State.Record[2]));
    memcpy(&M2State.Record[2],&Buffer[28],sizeof(M2State.Record[2]));
    
    //��ȡM1,M2����ĵ�4����¼ֵ���ֱ����Record[5]��
    memcpy(&M1State.Record[5],&Buffer[57],sizeof(M1State.Record[5]));
    memcpy(&M2State.Record[5],&Buffer[59],sizeof(M2State.Record[5]));
    
    //Buffer[61]�洢ƽ�����ʱ��������˶������־λ
    Balance_Data.TwoMotorRunFlag = Buffer[61];          
    
    //Buffer[62],Buffer[63]ƽ������󣬴洢�����������HALLֵ
    memcpy(&Balance_Data.TwoMotorOffsetHall,&Buffer[62],sizeof(Balance_Data.TwoMotorOffsetHall));       
    
    //Buffer[64]��Buffer[65]��¼����ʱ�ģ�������ƫ����
    memcpy(&AccDataBag.Y_Offset,&Buffer[64],sizeof(AccDataBag.Y_Offset));  
    
    //��¼������ƫ�����Ƿ��Ѵ洢��־λ��0��δ��ȡ��1���ѻ�ȡ��
    memcpy(&AccDataBag.Y_OffsetFlag,&Buffer[66],sizeof(AccDataBag.Y_OffsetFlag));
    //memcpy(&Balance_Data.TwoMotorRunFlag,&Buffer[61],sizeof(Balance_Data.TwoMotorRunFlag));
    
    memcpy(&AccDataBag.Y_Offset_Spec,&Buffer[73],sizeof(AccDataBag.Y_Offset_Spec));
          
            
    memcpy(&AccDataBag.Y_OffsetFlag_Spec,&Buffer[75],sizeof(AccDataBag.Y_OffsetFlag_Spec));       //�ѻ�ȡ������ƫ�����ı�־λ����EEPROM�� 
    
    
    
    
    if(ErrCode == Err_LSM6DSL)
    {
       ErrCode = 0;
    }
    
    
    if(Balance_ON == 1)
    {
      SysState =RESET;
      ErrCode = 0xffff;
      DisplayMode = ErrorMode;
      Balance_ON=0; 
      Buffer[76] = Balance_ON;
      EEPROM_Write();
    }
                 
    if((ErrCode == 0xffff))
    { 
      
      SysState =RESET;
      //ErrCode = 0xffff;
      DisplayMode = ErrorMode;
      
    }
    else if(ErrCode != 0)
    {
      SysState = ERROR;
      DisplayMode = ErrorMode;
      M1State.HallNow  = BASEHALL;
      M1State.HallLast = BASEHALL;
      M2State.HallNow  = BASEHALL;
      M2State.HallLast = BASEHALL;
     
    }
    else
    {     
      SysState = NORMAL;
      DisplayMode = HeightMode;
    } 
    SaveIndex = Buffer[32]; //��ǰ �趨�߶ȵı�־λ��M1,M2,M3��
    TimeNow = Buffer[34];   //�趨 ��ʱʱ��
    MinValue = TimeNow;
    TIMEcnt = ONE_MINUTE* TimeNow;
    
    if((M1State.HallNow == 0)||(M2State.HallNow == 0))
    {
      ErrCode = 0xffff;
      Dis_Char[0] = Char_R;
      Dis_Char[1] = Char_S;
      Dis_Char[2] = Char_T;
      SysState = RESET;
      DisplayMode = ErrorMode;
      M1State.HallNow = BASEHALL;
      M1State.HallLast = BASEHALL;
      M2State.HallNow = BASEHALL;
      M2State.HallLast = BASEHALL;
    }    
    
        if (Balance_Data.TwoMotorOffsetHall >= 0)                                       //M1�����M2�����
        {
          M1State.RelativeLimitUp = M1State.LimitUp;
          M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
          M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
          M2State.RelativeLimitDown = M2State.LimitDown;

        }
        else  //Balance_Data.TwoMotorOffsetHall < 0           //M1�����M2�����
        {
          M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //����M1���λ���趨ֵ
          M1State.RelativeLimitDown = M1State.LimitDown;
            
          M2State.RelativeLimitUp = M2State.LimitUp;
          M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //����M2���λ���趨ֵ
         }
  }
  
  if(Lsm6dslErrFlg==1)
  {
    if(((Sensitivity ==3)||(Sensitivity ==2))&&(LSM6DSLFlag==1))
    {
      Lsm6dslErrFlg = 0;
      ErrCode=Err_LSM6DSL;
    }
  }
  
}


/***********************************************************************************************************
* ��������: PositionSave()
* �������: ��
* ����ֵ  : ��
* ��    ��: �洢��ǰλ��
************************************************************************************************************/
void PositionSave(void)
{  
  if(MemCnt < 3000)//����ʱ�䲻��3��  MemCnt��ʼ��Ϊ0      ////M�����º�ʼ3���ʱ
  {
    if(SaveState == CONFIRM)//�洢λ���Ѿ�ȷ��          //��key.c�У��Ȱ���M�����ٰ���1/2/3����SaveState = CONFIRM
    {
      Dis_Char[2] = Data_Char[SavePosition-M1+1];       //��ʾһ�� ��-��
      if(SavePosition < M4)
      {
        M1State.Record[SavePosition-M1] = M1State.HallNow;
        M2State.Record[SavePosition-M1] = M2State.HallNow;
        SaveIndex |= (1<<(SavePosition-M1));
        Buffer[32] = SaveIndex;         //��EEPROM��д���־λ
        memcpy(&Buffer[14+6*(SavePosition-M1)],&M1State.Record[SavePosition-M1],2);     //д�뻺����M1��λ��Buffer[14],Buffer[15],��¼���ǵ�ǰhallֵ
        memcpy(&Buffer[16+6*(SavePosition-M1)],&M2State.Record[SavePosition-M1],2);
      }
      else if(SavePosition == M4)
      {
        M1State.Record[5] = M1State.HallNow;
        M2State.Record[5] = M2State.HallNow;
        SaveIndex |= (1<<(SavePosition-M1));
        Buffer[32] = SaveIndex;
        memcpy(&Buffer[57],&M1State.Record[5],2);
        memcpy(&Buffer[59],&M2State.Record[5],2);
      }
      EEPROM_Write();
      MemCnt = 2600;
      SaveState = END;
    }
  }
  else  //M�����³���3s���򲻱��浱ǰֵ
  {
    if (GetBalaceState() == 0)      //�ǵ�ƽ��ģʽ
    {
        SaveState = NULL;
        MemCnt = 0;
        if(HealthMode != 1)
            //DisplayMode = HeightMode;
        if(HealthMode == 4)
        {
            HealthMode = 2;
            T2sCnt = 1000;
        }
    }
  }
}


/***********************************************************************************************************
* ��������: LowPowerSave()
* �������: ��
* ����ֵ  : ��
* ��    ��: �͵�ѹ�洢
************************************************************************************************************/
void LowPowerSave(void)
{
  if((SysCmd == CmdNull)&&
     (SaveFlag == 1)&&            //SaveFlag��ʼ��Ϊ1
     ((SysState == NORMAL)||(ErrCode==Err_Overheating)||(ErrCode==Err_TimeEnd)))
  {
    SaveFlag = 2;
    memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //�洢M1 HALL��ǰֵ
    memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //�洢M2 HALL��ǰֵ
    
    //��ƽ��㶯ģʽ�£��洢���ȼ�߶Ȳ�ֵ
    Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //�õ���������HALLƫ��ֵ    
    memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]��Buffer[63]�洢���������Ӧ�������������HALL��ֵ
        
    
    EEPROM_Write();
    Delay_ms(10);
  }
 //if ()
  
}

/***********************************************************************************************************
* ��������: DeleteSavedHeight()
* �������: ��
* ����ֵ  : ��
* ��    ��: ɾ���洢�ĵ�ǰ�߶�����
************************************************************************************************************/
void DeleteSavedHeight(void)
{
  if(SaveFlag !=0 )
  {
    //�洢M1�����ǰHALLֵ��M1State.HallNow��
    Buffer[2] = 0;
    Buffer[3] = 0; 
    
    //�洢M2�����ǰHALLֵ��M2State.HallNow��
    Buffer[4] = 0;
    Buffer[5] = 0;
    EEPROM_Write();
    Delay_ms(7);
    SaveFlag = 0;
  }
}


/***********************************************************************************************************
* ��������: SysDataReadTest()
* �������: ��
* ����ֵ  : ��
* ��    ��: ��ϵͳû�о�����ʼ������г�ʼ���������ȡ��ȡ����
************************************************************************************************************/
/*void SysDataReadTest(void)
{
   u8 i;
  
  for(i = 0; i < 44; i++)
  {
    Buffer[i] = FLASH_ReadByte(ErrCodeAddr+i);
  }  
  AgingTest = FLASH_ReadByte(ErrCodeAddr+39); //��ȡϵͳ��ʼ����־λ
  if(AgingTest != 1) //û�г�ʼ���Ƚ��г�ʼ��
  { 
    memset(&Buffer[0], 0, sizeof(Buffer));
    AgingTest = 0;
    Buffer[33] = INITIALIZED;
    ErrCode = 0;  
    SysState = NORMAL;
    DisplayMode = HeightMode;
    memcpy(&Buffer[0],&ErrCode, sizeof(ErrCode));
    M1State.LimitDown = BOTTOM;
    M1State.LimitUp = M1State.LimitDown + DIF_HALL;
    M1State.HallNow = BOTTOM;
    M1State.HallLast = BOTTOM;
    memcpy(&Buffer[8],&M1State.LimitDown, sizeof(M1State.LimitDown));
    M2State.LimitDown = BOTTOM;
    M2State.LimitUp = M2State.LimitDown + DIF_HALL; 
    M2State.HallNow = BOTTOM;
    M2State.HallLast = BOTTOM;
    memcpy(&Buffer[10],&M2State.LimitDown, sizeof(M2State.LimitDown));
    Buffer[35] = TEST;
    SoftState = TEST;
    EEPROM_Write();
  }
  else
  {
    DisplayMode = TestMode;
    Dis_Char[0] = Char_T;
    Dis_Char[1] = Char_E;
    Dis_Char[2] = Char_S;
    AgingTest = 2;
    AgingTime = 0;
    memcpy(&AgingAllTime, &Buffer[40], 4);
    AgingTurnFlag = 1;
    AgingTurnTime = 0;
    M1TestPWMDuty = 200;
    M2TestPWMDuty = 200;
  }
  else //�Ѿ���ʼ�����ȡ�洢������
  {
    for(i = 0; i < 36; i++)
    {
      Buffer[i] = FLASH_ReadByte(ErrCodeAddr+i);
    }
    SoftState = Buffer[35];
    memcpy(&ErrCode,&Buffer[0],sizeof(ErrCode));
    memcpy(&M1State.HallNow,&Buffer[2],sizeof(M1State.HallNow));
    memcpy(&M2State.HallNow,&Buffer[4],sizeof(M2State.HallNow));
    memcpy(&M1State.LimitDown,&Buffer[8],sizeof(M1State.LimitDown));
    M1State.LimitUp = M1State.LimitDown + DIF_HALL;
    M1State.HallNow = 
    memcpy(&M2State.LimitDown,&Buffer[10],sizeof(M2State.LimitDown));
    M2State.LimitUp = M2State.LimitDown + DIF_HALL;
   
    memcpy(&M1State.Record[0],&Buffer[14],sizeof(M1State.Record[0]));
    memcpy(&M2State.Record[0],&Buffer[16],sizeof(M2State.Record[0]));

    memcpy(&M1State.Record[1],&Buffer[20],sizeof(M1State.Record[1]));
    memcpy(&M2State.Record[1],&Buffer[22],sizeof(M2State.Record[1]));

    memcpy(&M1State.Record[2],&Buffer[26],sizeof(M1State.Record[2]));
    memcpy(&M2State.Record[2],&Buffer[28],sizeof(M2State.Record[2]));
    if(ErrCode == 0xffff)
    {     
      SysState = RESET;
      DisplayMode = ErrorMode;
    }
    else if(ErrCode != 0)
    {
      SysState = ERROR;
      DisplayMode = ErrorMode;
      M1State.HallNow = BASEHALL;
      M1State.HallLast = BASEHALL;
      M2State.HallNow = BASEHALL;
      M2State.HallLast = BASEHALL;
    }
    else
    {     
      SysState = NORMAL;
      DisplayMode = HeightMode;
    } 
    SaveIndex = Buffer[32];
    TimeNow = Buffer[34];
    MinValue = TimeNow;
    TIMEcnt = ONE_MINUTE* TimeNow;
  }
}*/




