#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"


typedef enum
{
    E_FLASH_TYPE_W25Q80 = 0XEF13,
    E_FLASH_TYPE_W25Q16 = 0XEF14,
    E_FLASH_TYPE_W25Q32 = 0XEF15,
    E_FLASH_TYPE_W25Q64 = 0XEF16
}EN_W25QXX_Type_e;

typedef enum
{
    E_FLASH_RW_TYPE_READ,
    E_FLASH_RW_TYPE_WRITE
}EN_FLASH_RW_TYPE_e;
    

typedef enum
{
    E_FLASH_RW_DATA_OWNER_NULL,
    E_FLASH_RW_DATA_OWNER_PID1,
    E_FLASH_RW_DATA_OWNER_PID2,
    E_FLASH_RW_DATA_OWNER_PID3
}EN_FLASH_RW_OWNER_e;

typedef struct
{
    EN_FLASH_RW_TYPE_e eRW_Type;
    EN_FLASH_RW_OWNER_e eRW_Owner;
    u8 *pu8Data;
    u16 u16DataSize;
}ST_FLASH_RW_t;

//W25Xϵ��/Qϵ��оƬ�б�	   
//W25Q80 ID  0XEF13
//W25Q16 ID  0XEF14
//W25Q32 ID  0XEF15
//W25Q32 ID  0XEF16	
#define W25Q80  0XEF13
#define W25Q16  0XEF14
#define W25Q32  0XEF15
#define W25Q64  0XEF16

////////////////////////////////////////////////////////////////////////////
 
extern TaskHandle_t hFlash_RW_Task;
extern QueueHandle_t hFlash_RW_Queue;


void FLASH_RW_Task(void *pvParameters);

void FLASH_SPI_Init(void);
u16  FLASH_SPI_ReadID(void);  	    //��ȡFLASH ID
u8	 FLASH_SPI_ReadSR(void);        //��ȡ״̬�Ĵ��� 
void FLASH_SPI_Write_SR(u8 sr);  	//д״̬�Ĵ���
void FLASH_SPI_Write_Enable(void);  //дʹ�� 
void FLASH_SPI_Write_Disable(void);	//д����
void FLASH_SPI_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void FLASH_SPI_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //��ȡflash
void FLASH_SPI_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//д��flash
void FLASH_SPI_Erase_Chip(void);    	  //��Ƭ����
void FLASH_SPI_Erase_Sector(u32 Dst_Addr);//��������
void FLASH_SPI_Wait_Busy(void);           //�ȴ�����
void FLASH_SPI_PowerDown(void);           //�������ģʽ
void FLASH_SPI_WAKEUP(void);			  //����
#endif
















