#ifndef _DISPLAY_H
#define _DISPLAY_H
#include "stm32f4xx.h"
#include "adc.h"
#include "key.h"
#include "nrf24l01.h"
#include "RC.h"

#define PID_Log_ROW 12
#define PID_Log_COL 0

typedef struct
{
	u8 strRow;//起始行
	u16 DirStartCol;//目录起始列
	u16 DataStartCol;//数据起始列
}ShowPosition_TypeDef;


//************************************/
void DisplayLogMessage(u32 u32Color, const char* format,...);

/************************************/

//Page 1 显示的内容函数
void Show_NRFData_Update(NRF_TypeDef n);
void Show_Throttle_Update(Input_TypeDef Input);
void Show_RC_STATUS_Update(RC_STSTUS_TypeDef sta);
void Show_RC_SENSOR_Update(RC_Sensor_TypeDef s);
void Show_RC_GPS_Update(RC_GPS_TypeDef p);
void Show_RC_Power_Update(RC_POWER_TypeDef p);

void Show_RC_PID_Update(void);
	
void Show_Update(void);
void Show_PID_Log(u8* p);
void Show_Page1_Log(u8* p);

extern u8 Page_Now;
extern u8 Page_Next;

#endif

