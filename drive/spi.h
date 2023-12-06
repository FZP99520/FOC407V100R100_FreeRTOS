#ifndef _spi_H
#define _spi_H
#include "stm32f4xx.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"


#define SPI1_READ_WRITE_DATA_SIZE (1) //unit: byte
#define SPI2_READ_WRITE_DATA_SIZE (2) //unit: byte

typedef enum
{
    E_SPI_OK,
    E_SPI_FAIL,
    E_SPI_TIMEOUT,
    E_SPI_INVALID_PARAM
}EN_SPI_RESULT;

u8 SPI1_Init(void);  //SPI初始化
#if SPI1_READ_WRITE_DATA_SIZE==1
u8 SPI1_ReadWriteByte(u8 dat);		 //SPI2读写一个字节
#else
u16 SPI1_ReadWriteByte(u16 dat);
#endif

void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);	//设置SPI2的速度



u8 SPI2_Init(u16 u16SPI_BaudRatePrescaler);
EN_SPI_RESULT SPI2_ReadWriteByte(u16 u16DataIn, u16 *pu16DataOut);
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);



#endif
