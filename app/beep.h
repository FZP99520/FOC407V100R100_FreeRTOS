#ifndef _BEEP_H
#define _BEEP_H

#include "stm32f4xx.h"

#define BeepOn   GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define BeepOff  GPIO_SetBits(GPIOC,GPIO_Pin_13)


void Beep_Init(void);
void Beep_ms(u16 nms);
void Beep_times(u8 n);
void Beep(u8 t,u16 time_work,u16 time_stop);

#endif
