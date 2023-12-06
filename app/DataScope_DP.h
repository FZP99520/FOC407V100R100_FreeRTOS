#ifndef _DATA_PRTOCOL_H
#define _DATA_PRTOCOL_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"
#include "stm32f4xx.h"


//#define DataScope_SendData(pu8Data,u8len)   USART1_SendData(pu8Data,u8len)
#define DataScope_SendData(pu8Data,u8len)   //USART1_SendData(pu8Data,u8len)


extern TaskHandle_t hDataScope_Task;

typedef enum
{
    E_DATA_SCOPE_CHANNEL_MIN = 0,
    E_DATA_SCOPE_CHANNEL_1 = 1,
    E_DATA_SCOPE_CHANNEL_2,
    E_DATA_SCOPE_CHANNEL_3,
    E_DATA_SCOPE_CHANNEL_4,
    E_DATA_SCOPE_CHANNEL_5,
    E_DATA_SCOPE_CHANNEL_6,
    E_DATA_SCOPE_CHANNEL_7,
    E_DATA_SCOPE_CHANNEL_8,
    E_DATA_SCOPE_CHANNEL_9,
    E_DATA_SCOPE_CHANNEL_10,
    E_DATA_SCOPE_CHANNEL_MAX = E_DATA_SCOPE_CHANNEL_10
}DateScopeChannel_e;

extern unsigned char DataScope_OutPut_Buffer[];

void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
void DataScope_Send(void);
void DataScope_Task(void *pvParameters);
void DataScope_SendDuty(float f32ta,float f32tb,float f32tc);
void DataScope_SendPwmInfo(u16 u16PhaseA,u16 u16PhaseB,u16 u16PhaseC);
void DataScope_SendAngleInfo(float f32Data1, float f32Data2,float f32Data3, float f32Data4,  float f32Data5, float f32Data6);

 
#endif 



