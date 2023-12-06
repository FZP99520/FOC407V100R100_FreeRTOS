#ifndef _FR_OS_H
#define _FR_OS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


#define TRUE      (1)
#define FALSE     (0)
#define FR_OK     0x01
#define FR_ERR    0x02
#define FR_FAILED 0x03

typedef uint32_t FR_RESULT;
typedef SemaphoreHandle_t MutexHandle_t;
typedef SemaphoreHandle_t SemBinaryHandle_t;
typedef EventGroupHandle_t EventHandle_t;

typedef struct
{
    TaskFunction_t TaskCode;
    char * pcName;
    configSTACK_DEPTH_TYPE usStackDepth;
    void * pvParameters;
    UBaseType_t uxPriority;
    //TaskHandle_t * pxCreatedTask;
}TaskCreateParams_t;


typedef struct
{
    EventBits_t EventBitWaitFor;
    BaseType_t bClearOnExit;
    BaseType_t bWaitForAllBits;
    TickType_t xTicksToWait;
}EventWaitParams_t;

#ifndef LIMIT
#define LIMIT(x,xMax,xMin) x>=xMax?xMax:x<=xMin?xMin:x
#endif

FR_RESULT FR_OS_TaskCreate(TaskHandle_t * pTaskHandle, TaskCreateParams_t stTaskCreateParams);
FR_RESULT FR_OS_EventWait( EventHandle_t hEventHandle, EventWaitParams_t EventWaitParams , EventBits_t * EventBitOut);

#define BIT(n) 1<<n

#define FR_OS_MutexCreate()              xSemaphoreCreateMutex()
#define FR_OS_MUTEX_LOCK(hMutexHandle)   xSemaphoreTake(hMutexHandle, portMAX_DELAY)
#define FR_OS_MUTEX_UNLOCK(hMutexHandle) xSemaphoreGive(hMutexHandle)

#define FR_OS_SemBinaryCreate()                 xSemaphoreCreateBinary()
#define FR_OS_SemBinaryTake(hSemBinaryHandle, nTicks)   xSemaphoreTake(hSemBinaryHandle, nTicks)
#define FR_OS_SemBinaryGive(hSemBinaryHandle)   xSemaphoreGive(hSemBinaryHandle)

#define FR_OS_EventCreate()              xEventGroupCreate()
#define FR_OS_EventSetBits(hEventHandle,EventBitToSet)  xEventGroupSetBits(hEventHandle, EventBitToSet)

#define FR_OS_QueueCreate(uxQueueLength,uxItemSize)        xQueueCreate(uxQueueLength, uxItemSize)
#define FR_OS_QueueSend(xQueue,pvBuffer,xTicksToWait)      xQueueSend(xQueue, pvBuffer, xTicksToWait)
#define FR_OS_QueueSendFromISR(xQueue,pvBuffer)           xQueueSendFromISR(xQueue, pvBuffer, NULL)

#define FR_OS_QueueReceive(xQueue,pvBuffer,xTicksToWait)   xQueueReceive(xQueue, pvBuffer, xTicksToWait)


#define FR_OS_DelayMs(nms)               SysTick_os_delay_ms(nms)

#define memset(dest, val, n)  memset(dest, val, n)
#define memcpy(dst, src, n)   memcpy(dst, src, n)

#endif

