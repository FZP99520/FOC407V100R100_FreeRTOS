#ifndef _USART_H_
#define _USART_H_

#include "stm32f4xx.h"
#include "key.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

#define USART1_ENABLE   (1)
#define USART2_ENABLE   (0)
#define USART3_ENABLE   (0)

#define USART1_BAUDRATE (115200)
#define USART2_BAUDRATE (115200)
#define USART3_BAUDRATE (115200)

#ifndef BIT
#define BIT(x) 1<<x
#endif

#define USART1_BUFF_SIZE    64
#define USART2_BUFF_SIZE    64
#define USART3_BUFF_SIZE    64

#define USART1_BUFF_RX_FIFO_NUM   3
#define USART2_BUFF_RX_FIFO_NUM   3
#define USART3_BUFF_RX_FIFO_NUM   3

typedef struct
{
    u8* pu8DataAddr;
    u8  u8len;
}ST_USART_DATA;

typedef struct
{
    u8  bIsBusy;
    u8  bTx_Ok;
    u16 u16Tx_Size;
    u32 u32Tx_Cnt;
    u8  bRx_Ok;
    u16 u16Rx_Size;
    u32 u32Rx_Cnt;
}ST_UsartStatus_t;

#if (USART1_ENABLE == 1)
extern TaskHandle_t hUSART1_TX_Task;
extern TaskHandle_t hUSART1_RX_Task;
extern QueueHandle_t hUSART1_TX_Queue;
extern QueueHandle_t hUSART1_RX_Queue;
extern SemaphoreHandle_t hUSART1_TxSemBinary;
extern ST_UsartStatus_t stUSART1_STA;
#endif

#if (USART2_ENABLE == 1)
extern TaskHandle_t hUSART2_TX_Task;
extern TaskHandle_t hUSART2_RX_Task;
extern QueueHandle_t hUSART2_TX_Queue;
extern QueueHandle_t hUSART2_RX_Queue;
extern SemaphoreHandle_t hUSART2_TxSemBinary;
extern ST_UsartStatus_t stUSART2_STA;
#endif

#if (USART3_ENABLE == 1)
extern TaskHandle_t hUSART3_TX_Task;
extern TaskHandle_t hUSART3_RX_Task;
extern QueueHandle_t hUSART3_TX_Queue;
extern QueueHandle_t hUSART3_RX_Queue;
extern SemaphoreHandle_t hUSART3_TxSemBinary;
extern ST_UsartStatus_t stUSART3_STA;
#endif


#if (USART1_ENABLE == 1)
void USART1_Init(u32 baudrate);
u8 USART1_SendData(u8 *data,u8 len);
void USART1_Send_Data_Task(void *pvParameters);
void USART1_Receive_Data_Task(void *pvParameters);
#else
#define USART1_Init(...)
#define USART1_SendData(...)   (FALSE)
#define USART1_Send_Data_Task(...)
#define USART1_Receive_Data_Task(...)
#endif

#if (USART2_ENABLE == 1)
void USART2_Init(u32 baudrate);
u8 USART2_SendData(u8 *data,u8 len);
void USART2_Send_Data_Task(void *pvParameters);
void USART2_Receive_Data_Task(void *pvParameters);
#else
#define USART2_Init(...)
#define USART2_SendData(...)    (FALSE)
#define USART2_Send_Data_Task(...)
#define USART2_Receive_Data_Task(...)
#endif

#if (USART3_ENABLE == 1)
void USART3_Init(u32 baudrate);
u8 USART3_SendData(u8 *data,u8 len);
void USART3_Send_Data_Task(void *pvParameters);
void USART3_Receive_Data_Task(void *pvParameters);
#else
#define USART3_Init(...)
#define USART3_SendData(...)    (FALSE)
#define USART3_Send_Data_Task(...)
#define USART3_Receive_Data_Task(...)
#endif


#endif
