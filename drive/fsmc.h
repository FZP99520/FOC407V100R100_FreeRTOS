#ifndef _FSMC_H
#define _FSMC_H

#include "stm32f4xx.h"


#define 	FSMC_CMD     0x60000000		// FSMC д�Ĵ�����ַ
#define 	FSMC_DATA    0x60020000		// FSMC д���ݵ�ַ
void FSMC_Init(void);

#endif
