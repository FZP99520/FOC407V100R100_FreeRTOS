#ifndef _systick_H
#define _systick_H

#include "stm32f4xx.h"

//void Delay_init(u8 SYSCLK);  


void SysTick_Init(u8 u8Sysclk);
void SysTick_delay_us(u32 u32delay_us);
void SysTick_delay_ms(u32 u32delay_ms);
void SysTick_os_delay_ms(u32 u32delay_ms);

void Delay_ms(u32 x);
void Delay_us(u32 x);


//#define Delay_ms(x) SysTick_os_delay_ms(x)
//#define Delay_us(x) SysTick_delay_us(x)

#endif
