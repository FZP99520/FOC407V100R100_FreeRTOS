#ifndef _I2C_H_
#define _I2C_H_
#include "stm32f4xx.h"
#include "systick.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"


typedef enum
{
    E_I2C_FAIL,
    E_I2C_OK,
    E_I2C_ERR_NACK,
    E_I2C_ERR_INAVLID_DATA
}EN_I2C_Ret_e;

typedef enum
{
    E_I2C_TYPE_WRITE,
    E_I2C_TYPE_READ,
    E_I2C_TYPE_SCAN
}EN_I2C_RW_TYPE_e;

typedef struct
{
    u8 u8SlaveAddr;
    u8 *pu8RegAddr;
    u8 u8RegAddrLen;
    u8 *pu8Data;
    u8 u8DateLen;
    EN_I2C_RW_TYPE_e eRW_Type;
    EN_I2C_Ret_e eRet;
}ST_I2C_Params_t;

extern TaskHandle_t   hI2C1_Task;
extern QueueHandle_t  hI2C1_Queue;
extern MutexHandle_t  hI2C1_Mutex;


void I2C1_DataHandle_Task(void * pvParameters);

void I2C1_SW_Init(void);//初始化IIC的IO口
EN_I2C_Ret_e I2C1_SendData(ST_I2C_Params_t *pstI2cParams);
EN_I2C_Ret_e I2C1_ReadData(ST_I2C_Params_t *pstI2cParams);
EN_I2C_Ret_e i2c1_write(u8 u8SlaveAddr,u8 u8RegAddr,u8 *pu8Data,u8 u8DataLen);
EN_I2C_Ret_e i2c1_read(u8 u8SlaveAddr,u8 u8RegAddr,u8 *pu8Data,u8 u8DataLen);
void I2C1_SendData_ByTask(ST_I2C_Params_t *pstI2cParams);
void I2C1_ReadData_ByTask(ST_I2C_Params_t *pstI2cParams);
void I2C1_ScanDevices_ByTask(void);

#endif

