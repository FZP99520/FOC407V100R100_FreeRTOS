#include "FR_OS.h"

FR_RESULT FR_OS_TaskCreate(TaskHandle_t * pTaskHandle, TaskCreateParams_t stTaskCreateParams)
{
    FR_RESULT eRet = FR_OK;
    BaseType_t xReturn = pdTRUE;
    xReturn = xTaskCreate((TaskFunction_t   )stTaskCreateParams.TaskCode,
                          (const char *     )stTaskCreateParams.pcName,
                          (uint16_t         )stTaskCreateParams.usStackDepth,
                          (void *           )stTaskCreateParams.pvParameters,
                          (UBaseType_t      )stTaskCreateParams.uxPriority,
                          (TaskHandle_t *   )pTaskHandle);
    return xReturn?FR_OK:FR_FAILED;
}

FR_RESULT FR_OS_EventWait( EventHandle_t hEventHandle, EventWaitParams_t EventWaitParams , EventBits_t * EventBitOut)
{
   *EventBitOut = xEventGroupWaitBits((EventHandle_t      )hEventHandle,
                                      (EventBits_t        )EventWaitParams.EventBitWaitFor,
                                      (BaseType_t         )EventWaitParams.bClearOnExit,
                                      (BaseType_t         )EventWaitParams.bWaitForAllBits,
                                      (TickType_t         )EventWaitParams.xTicksToWait);
   return pdPASS;
}




