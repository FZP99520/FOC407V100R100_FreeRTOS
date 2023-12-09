#include "usart.h"
#include "dma.h"
#include "systick.h"
#include "stdio.h"
#include "string.h"
#include "gps.h"
#include "stdarg.h"
#include "gps.h"
#include "log.h"
#include "ANO_DT.h"

#include <stdlib.h>
#include <string.h>


#if (USART1_ENABLE == 1)
TaskHandle_t hUSART1_TX_Task = NULL;
TaskHandle_t hUSART1_RX_Task = NULL;
QueueHandle_t hUSART1_TX_Queue;
QueueHandle_t hUSART1_RX_Queue;
SemaphoreHandle_t hUSART1_TxSemBinary = NULL;
u8 USART1_TX_Buff[USART1_BUFF_SIZE];
u8 USART1_RX_Buff[USART1_BUFF_SIZE];
u8 USART1_RX_Buff_FIFO[USART1_BUFF_RX_FIFO_NUM][USART1_BUFF_SIZE];
ST_UsartStatus_t stUSART1_STA;
#endif

#if (USART2_ENABLE == 1)
TaskHandle_t hUSART2_TX_Task = NULL;
TaskHandle_t hUSART2_RX_Task = NULL;
QueueHandle_t hUSART2_TX_Queue = NULL;
QueueHandle_t hUSART2_RX_Queue = NULL;
SemaphoreHandle_t hUSART2_TxSemBinary = NULL;
u8 USART2_TX_Buff[USART2_BUFF_SIZE];
u8 USART2_RX_Buff[USART2_BUFF_SIZE];
u8 USART2_RX_Buff_FIFO[USART2_BUFF_RX_FIFO_NUM][USART2_BUFF_SIZE];
ST_UsartStatus_t stUSART2_STA;
#endif

#if (USART3_ENABLE == 1)
TaskHandle_t hUSART3_TX_Task = NULL;
TaskHandle_t hUSART3_RX_Task = NULL;
QueueHandle_t hUSART3_TX_Queue = NULL;
QueueHandle_t hUSART3_RX_Queue = NULL;
SemaphoreHandle_t hUSART3_TxSemBinary = NULL;
u8 USART3_TX_Buff[USART3_BUFF_SIZE];
u8 USART3_RX_Buff[USART3_BUFF_SIZE];
u8 USART3_RX_Buff_FIFO[USART3_BUFF_RX_FIFO_NUM][USART3_BUFF_SIZE];
ST_UsartStatus_t stUSART3_STA;
#endif


#if (USART1_ENABLE == 1)
void USART1_Send_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART1_TX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            xReturn = xSemaphoreTake(hUSART1_TxSemBinary, portMAX_DELAY);
            if(xReturn == pdPASS)
            {
                if(stUsartData.u8len <= USART1_BUFF_SIZE)
                {
                    memcpy(USART1_TX_Buff, stUsartData.pu8DataAddr, stUsartData.u8len);
                    DMA_SetCurrDataCounter(DMA2_Stream7, stUsartData.u8len);
                    DMA_Cmd(DMA2_Stream7, ENABLE);
                }
            }
        }
                
    }
}

void USART1_Receive_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART1_RX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            if(xReturn == pdPASS)
            {
#if (ANO_DT_MODE_IS_MASTER == 0)
                ST_ANO_DT_RECEIVE_DATA stAnoDt_ReceiveData;
                stAnoDt_ReceiveData.pu8Buff = stUsartData.pu8DataAddr;
                stAnoDt_ReceiveData.u8len   = stUsartData.u8len;
                FR_OS_QueueSend(hAnoDtReceive_Queue, &stAnoDt_ReceiveData, 0);
#endif
            }
        }
    }

}

/*********************USART1*******************************/
u8 USART1_SendData(u8 *pu8data, u8 u8len)
{
    u16 u16Cnt = 0;
    ST_USART_DATA stUsartData;
    memset(&stUsartData,0,sizeof(ST_USART_DATA));

    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART1_TX_Queue != NULL))
    {
        stUsartData.u8len = u8len;
        stUsartData.pu8DataAddr = pu8data;
        return FR_OS_QueueSend(hUSART1_TX_Queue, &stUsartData, 0);
    }
    else
    {
        while(1)//wait for transmission complete
        {
            if(!stUSART1_STA.bIsBusy) break;
            SysTick_delay_us(1);
            u16Cnt++;
            if(u16Cnt >1000) return FALSE;
        }
        stUSART1_STA.bIsBusy = TRUE;
        memcpy(USART1_TX_Buff, pu8data, u8len);//Copy Data to buff
        DMA_SetCurrDataCounter(DMA2_Stream7, u8len);
        DMA_Cmd(DMA2_Stream7,ENABLE);
        return TRUE;
    }
}

void USART1_SendData_Callback(void)
{
    stUSART1_STA.u32Tx_Cnt++;
    stUSART1_STA.bTx_Ok = TRUE;
    stUSART1_STA.bIsBusy = FALSE;
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART1_TxSemBinary != NULL))
    {
        xSemaphoreGiveFromISR(hUSART1_TxSemBinary, NULL);
    }
}

void USART1_ReceiveData_Callback(void)
{
    static u8 u8RxFifoIndex = 0;
    u16 u16DataSize = 0;
    ST_USART_DATA stUsartData;

    u16DataSize = USART1_BUFF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5);

    memcpy(USART1_RX_Buff_FIFO[u8RxFifoIndex], USART1_RX_Buff, u16DataSize);

    stUSART1_STA.u16Rx_Size = u16DataSize;
    stUSART1_STA.u32Rx_Cnt++;//flag receive finished

    stUsartData.u8len = stUSART1_STA.u16Rx_Size;
    stUsartData.pu8DataAddr = USART1_RX_Buff_FIFO[u8RxFifoIndex];
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART1_RX_Queue != NULL))
    {
        FR_OS_QueueSendFromISR(hUSART1_RX_Queue, &stUsartData);
    }
    u8RxFifoIndex++;
    if(u8RxFifoIndex == USART1_BUFF_RX_FIFO_NUM)
    {
        u8RxFifoIndex = 0;
    }
}

void USART1_Init(u32 u32Baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);   
    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF7 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8 */
    USART_OverSampling8Cmd(USART1, ENABLE); 

    USART_InitStructure.USART_BaudRate = u32Baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//接收非空中断 使用DMA接收要失能RXNE
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //发送缓存器空中断
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);  //发送完成中断
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //空闲中断
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_IDLE);
    USART_ClearITPendingBit(USART1, USART_IT_IDLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART1,DISABLE);
    DMA_USART1_Init(USART1_TX_Buff, USART1_RX_Buff);
}

void USART1_IRQHandler(void)//USART1 IRQ Handler,use dma
{
    if(USART_GetITStatus(USART1,USART_IT_TC)!= RESET) // =TX OK interrupt
    {
        USART_ClearFlag(USART1,USART_FLAG_TC);
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);
        USART1_SendData_Callback();
    }

    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //RX OK interrupt
    {
        USART1->SR;
        USART1->DR;
        USART_ClearFlag(USART1, USART_FLAG_IDLE);
        DMA_Cmd(DMA2_Stream5, DISABLE);
        DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
        USART1_ReceiveData_Callback();
        DMA_SetCurrDataCounter(DMA2_Stream5, USART1_BUFF_SIZE);
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
}
#endif

#if (USART2_ENABLE == 1)
void USART2_Send_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART2_TX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            xReturn = xSemaphoreTake(hUSART2_TxSemBinary, portMAX_DELAY);
            if(xReturn == pdPASS)
            {
                if(stUsartData.u8len <= USART2_BUFF_SIZE)
                {
                    memcpy(USART2_TX_Buff, stUsartData.pu8DataAddr, stUsartData.u8len);
                    DMA_SetCurrDataCounter(DMA1_Stream6, stUsartData.u8len);
                    DMA_Cmd(DMA1_Stream6, ENABLE);
                }
            }
        }
                
    }
}

void USART2_Receive_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART2_RX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            if(xReturn == pdPASS)
            {
#if (ANO_DT_MODE_IS_MASTER == 0)
                ST_ANO_DT_RECEIVE_QUEUE stAnoDt_ReceiveInfo;
                stAnoDt_ReceiveInfo.eHandleType = E_ANO_DT_HANDLE_TYPE_SLAVER_RECEIVE;
                stAnoDt_ReceiveInfo.pu8Buff = stUsartData.pu8DataAddr;
                stAnoDt_ReceiveInfo.u8len   = stUsartData.u8len;
                FR_OS_QueueSend(hAnoDtSlaverReceive_Queue, &stAnoDt_ReceiveInfo, 0);
#endif
            }
        }
    }

}

u8 USART2_SendData(u8 *pu8data,u8 u8len)
{
    u16 u16Cnt = 0;
    ST_USART_DATA stUsartData;
    memset(&stUsartData,0,sizeof(ST_USART_DATA));

    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART2_TX_Queue != NULL))
    {
        stUsartData.u8len = u8len;
        stUsartData.pu8DataAddr = pu8data;
        return FR_OS_QueueSend(hUSART2_TX_Queue, &stUsartData, 0);
    }
    else
    {
        while(1)//wait for transmission complete
        {
            if(!stUSART2_STA.bIsBusy) break;
            SysTick_delay_us(1);
            u16Cnt++;
            if(u16Cnt >1000) return FALSE;
        }
        stUSART2_STA.bIsBusy = TRUE;
        memcpy(USART2_TX_Buff, pu8data, u8len);//Copy Data to buff
        DMA_SetCurrDataCounter(DMA1_Stream6, u8len);
        DMA_Cmd(DMA1_Stream6,ENABLE);
        return TRUE;
    }
}

void USART2_SendData_Callback(void)
{
    stUSART2_STA.u32Tx_Cnt++;
    stUSART2_STA.bTx_Ok = TRUE;
    stUSART2_STA.bIsBusy = FALSE;
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART2_TxSemBinary != NULL))
    {
        xSemaphoreGiveFromISR(hUSART2_TxSemBinary, NULL);
    }
}

void USART2_ReceiveData_Callback(void)
{
    static u8 u8RxFifoIndex = 0;
    u16 u16DataSize = 0;
    ST_USART_DATA stUsartData;

    u16DataSize = USART2_BUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);

    memcpy(USART2_RX_Buff_FIFO[u8RxFifoIndex], USART2_RX_Buff, u16DataSize);

    stUSART2_STA.u16Rx_Size = u16DataSize;
    stUSART2_STA.u32Rx_Cnt++;//flag receive finished

    stUsartData.u8len = stUSART2_STA.u16Rx_Size;
    stUsartData.pu8DataAddr = USART2_RX_Buff_FIFO[u8RxFifoIndex];
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART2_RX_Queue != NULL))
    {
        FR_OS_QueueSendFromISR(hUSART2_RX_Queue, &stUsartData);
    }
    u8RxFifoIndex++;
    if(u8RxFifoIndex == USART2_BUFF_RX_FIFO_NUM)
    {
        u8RxFifoIndex = 0;
    }
}

void USART2_Init(u32 baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);   
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF7 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8 */
    //USART_OverSampling8Cmd(USART2, ENABLE); 
     
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
     
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);
    USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
    USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
     
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
         
    USART_ClearFlag(USART2,USART_FLAG_TC);
    USART_ClearFlag(USART2,USART_FLAG_RXNE);    
    USART_ClearFlag(USART2,USART_FLAG_IDLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART2,ENABLE);
    DMA_USART2_Init(USART2_TX_Buff, USART2_RX_Buff);
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2,USART_IT_TC)!= RESET) //TX OK interrupt
    {
        USART_ClearFlag(USART2,USART_FLAG_TC);
        USART_ITConfig(USART2,USART_IT_TC,DISABLE);
        USART2_SendData_Callback();
    }

    if(USART_GetITStatus(USART2,USART_IT_IDLE) != RESET) //RX OK interrupt
    {
        USART2->SR;
        USART2->DR;
        USART_ClearFlag(USART2, USART_FLAG_IDLE);
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        USART2_ReceiveData_Callback();
        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_BUFF_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}
#endif

#if (USART3_ENABLE == 1)
/***************USART3************************/
void USART3_Send_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART3_TX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            xReturn = xSemaphoreTake(hUSART3_TxSemBinary, portMAX_DELAY);
            if(xReturn == pdPASS)
            {
                if(stUsartData.u8len <= USART3_BUFF_SIZE)
                {
                    memcpy(USART3_TX_Buff, stUsartData.pu8DataAddr, stUsartData.u8len);
                    DMA_SetCurrDataCounter(DMA1_Stream3, stUsartData.u8len);
                    DMA_Cmd(DMA1_Stream3, ENABLE);
                }
            }
        }
                
    }
}

void USART3_Receive_Data_Task(void *pvParameters)
{
    BaseType_t xReturn = pdFALSE;
    ST_USART_DATA stUsartData;

    memset(&stUsartData,0,sizeof(ST_USART_DATA));
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hUSART3_RX_Queue, &stUsartData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            if(xReturn == pdPASS)
            {
#if 1
                ST_ANO_DT_RECEIVE_DATA stAnoDt_ReceiveData;
                stAnoDt_ReceiveData.pu8Buff = stUsartData.pu8DataAddr;
                stAnoDt_ReceiveData.u8len   = stUsartData.u8len;
                FR_OS_QueueSend(hAnoDtReceive_Queue, &stAnoDt_ReceiveData, 0);
#endif
            }
        }
    }

}

u8 USART3_SendData(u8 *pu8data,u8 u8len)
{
    u16 u16Cnt = 0;
    ST_USART_DATA stUsartData;
    memset(&stUsartData,0,sizeof(ST_USART_DATA));

    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART3_TX_Queue != NULL))
    {
        stUsartData.u8len = u8len;
        stUsartData.pu8DataAddr = pu8data;
        return FR_OS_QueueSend(hUSART3_TX_Queue, &stUsartData, 0);
    }
    else
    {
        while(1)//wait for transmission complete
        {
            if(!stUSART3_STA.bIsBusy) break;
            SysTick_delay_us(1);
            u16Cnt++;
            if(u16Cnt >1000) return FALSE;
        }
        stUSART3_STA.bIsBusy = TRUE;
        memcpy(USART3_TX_Buff, pu8data, u8len);//Copy Data to buff
        DMA_SetCurrDataCounter(DMA1_Stream3, u8len);
        DMA_Cmd(DMA1_Stream3, ENABLE);
        return TRUE;
    }
}

void USART3_SendData_Callback(void)
{
    stUSART3_STA.u32Tx_Cnt++;
    stUSART3_STA.bTx_Ok = TRUE;
    stUSART3_STA.bIsBusy = FALSE;
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART3_TxSemBinary != NULL))
    {
        xSemaphoreGiveFromISR(hUSART3_TxSemBinary, NULL);
    }
}

void USART3_ReceiveData_Callback(void)
{
    static u8 u8RxFifoIndex = 0;
    u16 u16DataSize = 0;
    ST_USART_DATA stUsartData;

    u16DataSize = USART3_BUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);

    memcpy(USART3_RX_Buff_FIFO[u8RxFifoIndex], USART3_RX_Buff, u16DataSize);

    stUSART3_STA.u16Rx_Size = u16DataSize;
    stUSART3_STA.u32Rx_Cnt++;//flag receive finished

    stUsartData.u8len = stUSART3_STA.u16Rx_Size;
    stUsartData.pu8DataAddr = USART3_RX_Buff_FIFO[u8RxFifoIndex];
    if((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (hUSART3_RX_Queue != NULL))
    {
        FR_OS_QueueSendFromISR(hUSART3_RX_Queue, &stUsartData);
    }
    u8RxFifoIndex++;
    if(u8RxFifoIndex == USART3_BUFF_RX_FIFO_NUM)
    {
        u8RxFifoIndex = 0;
    }
}

void USART3_Init(u32 u32baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);   
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF7 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8 */
    // USART_OverSampling8Cmd(USART3, ENABLE); 
    USART_InitStructure.USART_BaudRate = u32baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3,USART_IT_TC,DISABLE);//发送完成中断
    USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);//接收非空中断
    USART_ITConfig(USART3,USART_IT_TXE,DISABLE);//发送缓存器空中断
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE); //空闲中断
    USART_ClearFlag(USART3,USART_FLAG_TC);
    USART_ClearITPendingBit(USART3,USART_IT_TC);
 
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
    USART_Cmd(USART3,ENABLE);

    DMA_USART3_Init(USART3_TX_Buff, USART3_RX_Buff);
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_TC)!= RESET) //TX OK interrupt
    {
        USART_ClearFlag(USART3, USART_FLAG_TC);
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
        USART3_SendData_Callback();
    }

    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) //RX OK interrupt
    {
        USART3->SR;
        USART3->DR;
        USART_ClearFlag(USART3, USART_FLAG_IDLE);
        DMA_Cmd(DMA1_Stream1, DISABLE);
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        USART3_ReceiveData_Callback();
        DMA_SetCurrDataCounter(DMA1_Stream1, USART3_BUFF_SIZE);
        DMA_Cmd(DMA1_Stream1, ENABLE);
    }
}

#endif

#if 0
/***************USART6*******************/
u8 USART6_SendData(u8 *data,u8 len)//Use DMA to Send Data
{
	while(USART6_STA.busy){}//wait for transmission complete
	USART6_STA.busy = 1;
	memcpy(USART6_TX_Buff,data,len);//Copy Data to buff
	USART_ClearFlag(USART6,USART_IT_TC);
	DMA_SetCurrDataCounter(DMA2_Stream6,len);//
 	DMA_Cmd(DMA2_Stream6,ENABLE);//
	return 1;
}
void USART6_Init(u32 baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);   
    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF8 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8 */
    //USART_OverSampling8Cmd(USART2, ENABLE); 
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);

    USART_ITConfig(USART6,USART_IT_TC,DISABLE);//发送完成中断
    USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);//接收非空中断
    USART_ITConfig(USART6,USART_IT_TXE,DISABLE);//发送缓存器空中断
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE); //空闲中断

    USART_ClearFlag(USART6,USART_FLAG_IDLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3; //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
    USART_Cmd(USART6,ENABLE);
}
void USART6_IRQHandler(void)
{
	u8 receive;
	static u8 temp=0;
	if(USART_GetITStatus(USART6,USART_IT_TC)!= RESET)  //发送完成中断
 {
	 USART_ClearFlag(USART6,USART_FLAG_TC);
	 USART6_STA.busy=0;//释放传输通道
	 USART_ITConfig(USART6,USART_IT_TC,DISABLE);  //发送完成中断
 }
 if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET)
 {
	  USART6->SR;
	  USART6->DR;
	 
	  DMA_Cmd(DMA2_Stream1,DISABLE);
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);
		USART6_STA.rx_len = USART6_Buff_Len - DMA_GetCurrDataCounter(DMA2_Stream1);
		//memcpy(gps_buff,USART6_RX_Buff,USART6_STA.rx_len);//copy data to gps buff
	 	USART6_STA.rx_ok=1;
		DMA_SetCurrDataCounter(DMA2_Stream1,USART6_Buff_Len);
		DMA_Cmd(DMA2_Stream1,ENABLE);
 }

}
#endif

