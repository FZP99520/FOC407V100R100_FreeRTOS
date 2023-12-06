#ifndef _DAC_H_
#define _DAC_H_


#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"



void DAC1_Init(void);
void DAC1_SetValue(u16 u16DacValue);
void DAC1_SetVoltage(u16 u16VoltageX1000);






#endif

