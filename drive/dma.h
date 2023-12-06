#ifndef _DMA_H_
#define _DMA_H_

#include "stm32f4xx.h"
#include "usart.h"
#include "adc.h"


void DMA_USART1_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr);
void DMA_USART2_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr);
void DMA_USART3_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr);
void DMA_ADC1_Init(u16 *pu16AdcConvBuff);



#endif

