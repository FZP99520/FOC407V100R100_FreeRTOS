#ifndef _FSMC_H
#define _FSMC_H

#include "stm32f4xx.h"


#define 	FSMC_CMD     0x60000000		// FSMC 写寄存器地址
#define 	FSMC_DATA    0x60020000		// FSMC 写数据地址
void FSMC_Init(void);

#endif
