#ifndef _INA226_H_
#define _INA226_H_


#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

typedef struct
{
    u16 u16VBusVal;
    u16 u16VShuntVal;
    u16 u16CurVal;
    u16 u16PowerVal;
    float f32Vbus;
    float f32VShunt;
    float f32Cur;
    float f32Power;
}ST_INA_Info_t;

extern TaskHandle_t hINA226_Task;
extern ST_INA_Info_t stInaInfo;
void INA226_Init(void);
void INA226_HandleTask(void *pvParameters);


#endif

