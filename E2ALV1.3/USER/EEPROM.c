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
* 函数名称: EEPROM_Init()
* 输入参数: 无
* 返回值  : 无
* 功    能: 解锁单片机内部EEPROM操作
************************************************************************************************************/
void EEPROM_Init(void)
{
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD); //设置编程时间为标准编程时间 
}

/***********************************************************************************************************
* 函数名称: EEPROM_Write()
* 输入参数: 无
* 返回值  : 无
* 功    能: 内部EEPROM整块写入程序，写入块为第一块
************************************************************************************************************/
void EEPROM_Write(void)
{
  Delay_ms(2);  
  FLASH_Unlock(FLASH_MEMTYPE_DATA); //解锁内部EEPROM
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
  {}
  FLASH_ProgramBlock(0,FLASH_MEMTYPE_DATA,FLASH_PROGRAMMODE_STANDARD,&Buffer[0]); //写第0块
  while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
  {}
  FLASH_Lock(FLASH_MEMTYPE_DATA);
  Delay_ms(2);
}

/***********************************************************************************************************
* 函数名称: EEPROM_Erase()
* 输入参数: 无
* 返回值  : 无
* 功    能: 擦除内部EEPROM的数据
************************************************************************************************************/
void EEPROM_Erase(void)
{
  FLASH_Unlock(FLASH_MEMTYPE_DATA); //解锁内部EEPROM
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
  {}
  FLASH_EraseBlock(0, FLASH_MEMTYPE_DATA); //擦除第0块
  while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
  {}
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}


/***********************************************************************************************************
* 函数名称: SysDataRead()
* 输入参数: 无
* 返回值  : 无
* 功    能: 若系统没有经过初始化则进行初始化，否则读取存取数据
************************************************************************************************************/
void SysDataRead(void)
{
  u8 temp,i;
  
  memset(&Buffer[0], 0, sizeof(Buffer));        //初始化Buffer寄存器
  temp = FLASH_ReadByte(InitFlagAddr);          //读取系统初始化标志位
  
  //temp = INITIALIZED;
  
  if(temp != INITIALIZED)                       //没有初始化先进行初始化
  {   
    Buffer[33] = INITIALIZED;   //INITIALIZED = 0xAA
    ErrCode = 0xffff;
    Dis_Char[0] = Char_R;
    Dis_Char[1] = Char_S;
    Dis_Char[2] = Char_T;
    SysState = RESET;
    memcpy(&Buffer[0],&ErrCode, sizeof(ErrCode));               //存储故障代码进入缓存数组Buffer[0]和Buffer[1]中
    M1State.LimitDown = BASEHALL;
    M1State.HallNow = BASEHALL;
    M1State.HallLast = BASEHALL;
    memcpy(&Buffer[8],&M1State.LimitDown, sizeof(M1State.LimitDown));   //u16 LimitDown;   //电机运行的HALL下限值
                                                                        //Buffer[8],Buffer[9]存储M1电机运行的HALL下限值
    M2State.LimitDown = BASEHALL;
    M2State.HallNow = BASEHALL;
    M2State.HallLast = BASEHALL;
    memcpy(&Buffer[10],&M2State.LimitDown, sizeof(M2State.LimitDown));  //Buffer[10],Buffer[11]存储M2电机运行的HALL下限值
    TimeNow = TIMEVAL;                  //TIMEVAL = 45 
    TIMEcnt = ONE_MINUTE* TimeNow;      //ONE_MINUTE = 60000
    memcpy(&Buffer[34],&TimeNow, sizeof(TimeNow));
    SoftState = RELEASE;
    Buffer[35] = SoftState;             //#define  TEST     0  #define  RELEASE  1

        
    Buffer[36] = Unit;          //Buffer[36]记录高度单位，UNIT = 0(CM),UNIT = 1(INCH)
    
    //#define MINHEIGHT 60.5f  #define MAXHEIGHT 126.0f  #define BASEHEIGHT 60.5f
    memcpy(&Buffer[37], &BaseHeight, sizeof(BaseHeight));       //Buffer[37],Buffer[38],Buffer[39],Buffer[40]记录参数  BASEHEIGHT
    memcpy(&Buffer[41],&DiffHall, 2);        //Buffer[41],Buffer[42]记录参数  DiffHall（DiffHall = HALL值最大值 - HALL值最小值）
    memcpy(&Buffer[43], &MaxHeight, sizeof(MaxHeight));         //Buffer[43],Buffer[44],Buffer[45],Buffer[46]记录#define MAXHEIGHT 126.0f
    Buffer[47] = SPEED;         //Buffer[47]记录SPEED
    Buffer[48] = SENS;          //Buffer[49]记录SENS
    //MinColumnHeight = BASEHEIGHT
    memcpy(&Buffer[49], &MinColumnHeight, sizeof(MinColumnHeight));//Buffer[49],Buffer[50],Buffer[51],Buffer[52]参数记录 BASEHEIGHT 值       
    memcpy(&Buffer[53], &MaxColumnHeight, sizeof(MaxColumnHeight));//Buffer[53],Buffer[54],Buffer[55],Buffer[56]参数记录 MAXHEIGHT 值
    
    //memcpy(&Buffer[59],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[59]，Buffer[60]存储因地形自适应而产生的两电机HALL差值
    
    memcpy(&Buffer[61],&Balance_Data.TwoMotorRunFlag,sizeof(Balance_Data.TwoMotorRunFlag));
    
    
    //Balance_Data.TwoMotorRunFlag = 0;
      
    EEPROM_Write();
    
    DisableHPSlopeFilter();     //第一次上电，需进行偏移量采集
    
    //Delay_ms(200);  
    
    //Buffer[61] = FLASH_ReadByte(ErrCodeAddr + 61);
    //memcpy(&Balance_Data.TwoMotorRunFlag,&Buffer[61],sizeof(Balance_Data.TwoMotorRunFlag));
    //Delay_ms(200);
  }
  else //已经初始化则读取存储的数据
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
    memcpy(&BaseHeight, &Buffer[37], sizeof(BaseHeight));//运行到的最低高度 60.5cm
    memcpy(&MaxHeight, &Buffer[43], sizeof(MaxHeight)); //运行到的最高高度 126cm
    
    //运行到的最低高度 60.5cm
    memcpy(&MinColumnHeight, &Buffer[49], sizeof(MinColumnHeight));
    //运行到的最高高度 126cm
    memcpy(&MaxColumnHeight, &Buffer[53], sizeof(MaxColumnHeight));
    
    //运行到的最高高度和最低高度的差值 用HALL值表示
    memcpy(&DiffHall, &Buffer[41], 2);     
    
    //获取故障信息
    memcpy(&ErrCode,&Buffer[0],sizeof(ErrCode));                

    //获取M1电机当前HALL值
    memcpy(&M1State.HallNow,&Buffer[2],sizeof(M1State.HallNow));
    //获取M2电机当前HALL值
    memcpy(&M2State.HallNow,&Buffer[4],sizeof(M2State.HallNow));
    
    //获取M1电机运行的HALL下限值
    memcpy(&M1State.LimitDown,&Buffer[8],sizeof(M1State.LimitDown));
    //通过M1电机运行的HALL下限制和差值，获得M1电机运行的HALL上限值
    M1State.LimitUp = M1State.LimitDown + DiffHall;
    
    //获取M2电机运行的HALL下限值
    memcpy(&M2State.LimitDown,&Buffer[10],sizeof(M2State.LimitDown));
    //通过M2电机运行的HALL下限制和差值，获得M2电机运行的HALL上限值
    M2State.LimitUp = M2State.LimitDown + DiffHall;
   
    //获取M1,M2电机的第1个记录值，分别存入Record[0]中
    memcpy(&M1State.Record[0],&Buffer[14],sizeof(M1State.Record[0]));   
    memcpy(&M2State.Record[0],&Buffer[16],sizeof(M2State.Record[0]));

    //获取M1,M2电机的第2个记录值，分别存入Record[1]中
    memcpy(&M1State.Record[1],&Buffer[20],sizeof(M1State.Record[1]));
    memcpy(&M2State.Record[1],&Buffer[22],sizeof(M2State.Record[1]));

    //获取M1,M2电机的第3个记录值，分别存入Record[2]中
    memcpy(&M1State.Record[2],&Buffer[26],sizeof(M1State.Record[2]));
    memcpy(&M2State.Record[2],&Buffer[28],sizeof(M2State.Record[2]));
    
    //获取M1,M2电机的第4个记录值，分别存入Record[5]中
    memcpy(&M1State.Record[5],&Buffer[57],sizeof(M1State.Record[5]));
    memcpy(&M2State.Record[5],&Buffer[59],sizeof(M2State.Record[5]));
    
    //Buffer[61]存储平衡调整时，两电机运动方向标志位
    Balance_Data.TwoMotorRunFlag = Buffer[61];          
    
    //Buffer[62],Buffer[63]平衡调整后，存储两个电机相差的HALL值
    memcpy(&Balance_Data.TwoMotorOffsetHall,&Buffer[62],sizeof(Balance_Data.TwoMotorOffsetHall));       
    
    //Buffer[64]，Buffer[65]记录出厂时的，陀螺仪偏移量
    memcpy(&AccDataBag.Y_Offset,&Buffer[64],sizeof(AccDataBag.Y_Offset));  
    
    //记录陀螺仪偏移量是否已存储标志位（0，未获取，1，已获取）
    memcpy(&AccDataBag.Y_OffsetFlag,&Buffer[66],sizeof(AccDataBag.Y_OffsetFlag));
    //memcpy(&Balance_Data.TwoMotorRunFlag,&Buffer[61],sizeof(Balance_Data.TwoMotorRunFlag));
    
    memcpy(&AccDataBag.Y_Offset_Spec,&Buffer[73],sizeof(AccDataBag.Y_Offset_Spec));
          
            
    memcpy(&AccDataBag.Y_OffsetFlag_Spec,&Buffer[75],sizeof(AccDataBag.Y_OffsetFlag_Spec));       //已获取陀螺仪偏移量的标志位存入EEPROM中 
    
    
    
    
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
    SaveIndex = Buffer[32]; //当前 设定高度的标志位（M1,M2,M3）
    TimeNow = Buffer[34];   //设定 定时时间
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
    
        if (Balance_Data.TwoMotorOffsetHall >= 0)                                       //M1电机比M2电机高
        {
          M1State.RelativeLimitUp = M1State.LimitUp;
          M1State.RelativeLimitDown = M1State.LimitDown + Balance_Data.TwoMotorOffsetHall;
          
          M2State.RelativeLimitUp = M2State.LimitUp - Balance_Data.TwoMotorOffsetHall;
          M2State.RelativeLimitDown = M2State.LimitDown;

        }
        else  //Balance_Data.TwoMotorOffsetHall < 0           //M1电机比M2电机低
        {
          M1State.RelativeLimitUp = M1State.LimitUp + Balance_Data.TwoMotorOffsetHall;        //减少M1最高位置设定值
          M1State.RelativeLimitDown = M1State.LimitDown;
            
          M2State.RelativeLimitUp = M2State.LimitUp;
          M2State.RelativeLimitDown = M2State.LimitDown - Balance_Data.TwoMotorOffsetHall;    //升高M2最低位置设定值
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
* 函数名称: PositionSave()
* 输入参数: 无
* 返回值  : 无
* 功    能: 存储当前位置
************************************************************************************************************/
void PositionSave(void)
{  
  if(MemCnt < 3000)//计数时间不足3秒  MemCnt初始化为0      ////M键按下后开始3秒计时
  {
    if(SaveState == CONFIRM)//存储位置已经确认          //在key.c中，先按下M键，再按下1/2/3键后，SaveState = CONFIRM
    {
      Dis_Char[2] = Data_Char[SavePosition-M1+1];       //显示一个 “-”
      if(SavePosition < M4)
      {
        M1State.Record[SavePosition-M1] = M1State.HallNow;
        M2State.Record[SavePosition-M1] = M2State.HallNow;
        SaveIndex |= (1<<(SavePosition-M1));
        Buffer[32] = SaveIndex;         //在EEPROM中写入标志位
        memcpy(&Buffer[14+6*(SavePosition-M1)],&M1State.Record[SavePosition-M1],2);     //写入缓存中M1的位置Buffer[14],Buffer[15],记录的是当前hall值
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
  else  //M键按下超过3s，则不保存当前值
  {
    if (GetBalaceState() == 0)      //非调平衡模式
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
* 函数名称: LowPowerSave()
* 输入参数: 无
* 返回值  : 无
* 功    能: 低电压存储
************************************************************************************************************/
void LowPowerSave(void)
{
  if((SysCmd == CmdNull)&&
     (SaveFlag == 1)&&            //SaveFlag初始化为1
     ((SysState == NORMAL)||(ErrCode==Err_Overheating)||(ErrCode==Err_TimeEnd)))
  {
    SaveFlag = 2;
    memcpy(&Buffer[2],&M1State.HallNow,sizeof(M1State.HallNow));        //存储M1 HALL当前值
    memcpy(&Buffer[4],&M2State.HallNow,sizeof(M2State.HallNow));        //存储M2 HALL当前值
    
    //自平衡点动模式下，存储两腿间高度差值
    Balance_Data.TwoMotorOffsetHall = M1State.HallNow - M2State.HallNow;            //得到两电机间的HALL偏差值    
    memcpy(&Buffer[62],&Balance_Data.TwoMotorOffsetHall,sizeof(Balance_Data.TwoMotorOffsetHall));     //Buffer[62]，Buffer[63]存储因地形自适应而产生的两电机HALL差值
        
    
    EEPROM_Write();
    Delay_ms(10);
  }
 //if ()
  
}

/***********************************************************************************************************
* 函数名称: DeleteSavedHeight()
* 输入参数: 无
* 返回值  : 无
* 功    能: 删除存储的当前高度数据
************************************************************************************************************/
void DeleteSavedHeight(void)
{
  if(SaveFlag !=0 )
  {
    //存储M1电机当前HALL值（M1State.HallNow）
    Buffer[2] = 0;
    Buffer[3] = 0; 
    
    //存储M2电机当前HALL值（M2State.HallNow）
    Buffer[4] = 0;
    Buffer[5] = 0;
    EEPROM_Write();
    Delay_ms(7);
    SaveFlag = 0;
  }
}


/***********************************************************************************************************
* 函数名称: SysDataReadTest()
* 输入参数: 无
* 返回值  : 无
* 功    能: 若系统没有经过初始化则进行初始化，否则读取存取数据
************************************************************************************************************/
/*void SysDataReadTest(void)
{
   u8 i;
  
  for(i = 0; i < 44; i++)
  {
    Buffer[i] = FLASH_ReadByte(ErrCodeAddr+i);
  }  
  AgingTest = FLASH_ReadByte(ErrCodeAddr+39); //读取系统初始化标志位
  if(AgingTest != 1) //没有初始化先进行初始化
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
  else //已经初始化则读取存储的数据
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




