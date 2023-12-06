#ifndef _exti_H
#define _exti_H
#include "stm32f4xx.h"
void EXTI0_Init(void);
void EXTI1_Init(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);


#endif

