#include "main.h"
#include "adc.h"
#include "dac.h"
#include "mpu9250.h"
#include "ili9488.h"
#include "key.h"
#include "pwm.h"
#include "display.h"
#include "log.h"
#include "input_ctrl.h"
#include "systick.h"
#include "DataScope_DP.h"
#include "foc.h"
#include "as5048a.h"
#include "ina226.h"
#include "oled.h"
#include "flash.h"
#include "drv8323.h"

#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

#include <stdlib.h>

#define LED_PWM_PERIOD  1000

#define TestBuffSize 10
u8 pu8TestBuff[TestBuffSize]={0,1,2,3,4,5,6,7,8,9};

/****************Dynamic Task*************/

TaskHandle_t hStartTask;
TaskHandle_t hLed1Task;
TaskHandle_t hLed2Task;
TaskHandle_t hKeyTask;
TaskHandle_t hLcdTask;
TaskHandle_t hBeepTask;

void Start_Task(void *pvParameters);
void Led1_Task(void *pvParameters);
void Led2_Task(void *pvParameters);
void Key_Task(void *pvParameters);
void LCD_Task(void *pvParameters);
void BEEP_Task(void *pvParameters);
/*****************************************/

/*******************Queue******************/
QueueHandle_t Test_QueueHandle;
#define TestQueueLen  2
#define TestQueueSize sizeof(EN_KEY_FLAG)


MutexHandle_t hLed2Mutex;
EventGroupHandle_t hKeyEvent;


int main()
{
    unsigned int i=0;
    u8 delay=0;
    SystemClock_Init();//设置时钟
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//中断分组
/****************/
#if (USART1_ENABLE == 1)
        USART1_Init(USART1_BAUDRATE);
#endif
#if (USART2_ENABLE == 1)
        USART2_Init(USART2_BAUDRATE);
#endif
#if (USART3_ENABLE == 1)
        USART3_Init(USART3_BAUDRATE);
#endif

    DEBUG_PRINT("Power On\n");
    DEBUG_PRINT("USART: %d\n",USART1_BAUDRATE);
    Led_Init();
    //Beep_Init();
    //KEY_Init();
    I2C1_SW_Init();

    FOC_DriverInit();

    //FOC_Motor_Cal();
    SysTick_delay_ms(300);
    //DAC1_Init();
    TIM5_PWM_Init(LED_PWM_PERIOD);

    OLED_Init();
    FLASH_SPI_Init(); //for adc or dac
    SPI1_Init();
    #if 0
    USART1_Init(115200);
    DMA_USART1_Init();
    USART2_Init(115200);
    DMA_USART2_Init();
    USART3_Init(115200);
    DMA_USART3_Init();
    USART6_Init(115200);
    DMA_USART6_Init();
    USART_CLR_ALL_BUFF();
    #endif
    
  Page_Next=1;
//	TIM2_Int_Init();//主要事件处理 5ms中断
    //TIM3_Int_Init();//T=5ms Only Use TIM3 
//	TIM4_Int_Init();

    SysTick_Init(168);
    TaskCreateParams_t stTaskCreateParams;
    memset(&stTaskCreateParams,0,sizeof(TaskCreateParams_t));
    
    stTaskCreateParams.TaskCode = Start_Task;
    stTaskCreateParams.pcName = "start_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 1;
    FR_OS_TaskCreate(&hStartTask, stTaskCreateParams);

    vTaskStartScheduler();
    while(pdTRUE);
}

#define CONFIG_DEBUG_STATIC_TASK 0
#if CONFIG_DEBUG_STATIC_TASK
#define Task1_Task_PRIO   3
#define TASK1_STK_SIZE   128
StackType_t Task1TaskStack[TASK1_STK_SIZE];
StaticTask_t Task1_TaskTCB;
TaskHandle_t Task1_Handler;
#endif

void Start_Task(void * pvParameters)
{
    taskENTER_CRITICAL();
    /*Create Queue...*/
    Test_QueueHandle = xQueueCreate(TestQueueLen, TestQueueSize);
#if (USART1_ENABLE == 1)
    hUSART1_TX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART1_RX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART1_TxSemBinary = xSemaphoreCreateBinary();
    xSemaphoreGive(hUSART1_TxSemBinary);
#endif

#if (USART2_ENABLE == 1)
    hUSART2_TX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART2_RX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART2_TxSemBinary = xSemaphoreCreateBinary();
    xSemaphoreGive(hUSART2_TxSemBinary);
#endif

#if (USART3_ENABLE == 1)
    hUSART3_TX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART3_RX_Queue    = FR_OS_QueueCreate(3, sizeof(ST_USART_DATA));
    hUSART3_TxSemBinary = xSemaphoreCreateBinary();
    xSemaphoreGive(hUSART3_TxSemBinary);
#endif

    hI2C1_Queue      = FR_OS_QueueCreate(3, sizeof(ST_I2C_Params_t));
    hOledDisp_Queue  = FR_OS_QueueCreate(3, sizeof(ST_OLED_Disp_t));
    hFlash_RW_Queue  = FR_OS_QueueCreate(3, sizeof(ST_FLASH_RW_t));
    /*Create Semaphore */
    hI2C1_Mutex      = FR_OS_MutexCreate();
    //hAdcConv_Queue = FR_OS_QueueCreate(1, sizeof(ST_ADC_Conv_t));
    hUSART1_TxSemBinary = xSemaphoreCreateBinary();
    xSemaphoreGive(hUSART1_TxSemBinary);
    //KeySemBin_Handle = xSemaphoreCreateBinary();
    //Led2SemMutex_Handle = xSemaphoreCreateMutex();

    /*Create Event*/
    hKeyEvent = FR_OS_EventCreate();
    hFocEvent = FR_OS_EventCreate();

    /*Create Task*/
    TaskCreateParams_t stTaskCreateParams;
    memset(&stTaskCreateParams,0,sizeof(TaskCreateParams_t));

    stTaskCreateParams.TaskCode = Led1_Task;
    stTaskCreateParams.pcName = "led1_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hLed1Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = Led2_Task;
    stTaskCreateParams.pcName = "led2_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 2;
    FR_OS_TaskCreate(&hLed2Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = Key_Task;
    stTaskCreateParams.pcName = "key_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    //FR_OS_TaskCreate(&hKeyTask, stTaskCreateParams);

    stTaskCreateParams.TaskCode = LCD_Task;
    stTaskCreateParams.pcName = "lcd_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    //FR_OS_TaskCreate(&hLcdTask, stTaskCreateParams);
    
    stTaskCreateParams.TaskCode = BEEP_Task;
    stTaskCreateParams.pcName = "beep_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    //FR_OS_TaskCreate(&hBeepTask, stTaskCreateParams);

#if (USART1_ENABLE == 1)
    stTaskCreateParams.TaskCode = USART1_Send_Data_Task;
    stTaskCreateParams.pcName = "usart1_send_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART1_TX_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = USART1_Receive_Data_Task;
    stTaskCreateParams.pcName = "usart1_receive_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART1_RX_Task, stTaskCreateParams);
#endif

#if (USART2_ENABLE == 1)
    stTaskCreateParams.TaskCode = USART2_Send_Data_Task;
    stTaskCreateParams.pcName = "usart2_send_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART2_TX_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = USART2_Receive_Data_Task;
    stTaskCreateParams.pcName = "usart2_receive_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART2_RX_Task, stTaskCreateParams);
#endif
    
#if (USART3_ENABLE == 1)
    stTaskCreateParams.TaskCode = USART3_Send_Data_Task;
    stTaskCreateParams.pcName = "usart3_send_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART3_TX_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = USART3_Receive_Data_Task;
    stTaskCreateParams.pcName = "usart3_receive_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hUSART3_RX_Task, stTaskCreateParams);
#endif


    stTaskCreateParams.TaskCode = I2C1_DataHandle_Task;
    stTaskCreateParams.pcName = "i2c1_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hI2C1_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = DataScope_Task;
    stTaskCreateParams.pcName = "DataScope_task";
    stTaskCreateParams.usStackDepth = 256;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    //FR_OS_TaskCreate(&hDataScope_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = FOC_Control_Start_Task;
    stTaskCreateParams.pcName = "FOC_Control_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hFocControlStart_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = INA226_HandleTask;
    stTaskCreateParams.pcName = "ina226_handle_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hINA226_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = OLED_Display_Task;
    stTaskCreateParams.pcName = "oled_disp_task";
    stTaskCreateParams.usStackDepth = 512;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hOledDisp_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = FLASH_RW_Task;
    stTaskCreateParams.pcName = "flash_rw_task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    //FR_OS_TaskCreate(&hFlash_RW_Task, stTaskCreateParams);

    stTaskCreateParams.TaskCode = ANO_DT_Start_Task;
    stTaskCreateParams.pcName = "ANO_DT_Start_Task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hAnoDtStartTask, stTaskCreateParams);

    #if CONFIG_DEBUG_STATIC_TASK 
    Task1_Handler = xTaskCreateStatic((TaskFunction_t  )Led1_Task,
                                      (char *          )"led1_task",
                                      (uint32_t        )TASK1_STK_SIZE,       
                                      (void *          )NULL,
                                      (UBaseType_t     )Task1_Task_PRIO,
                                      (StackType_t *   )Task1TaskStack,
                                      (StaticTask_t *  )&Task1_TaskTCB );
    #endif
    vTaskDelete(hStartTask);
    taskEXIT_CRITICAL();
}

void Led1_Task(void * pvParameters)
{
    BaseType_t xReturn = pdTRUE;
    #define LED_PWM_SET_COMPARE(x) TIM_SetCompare2(TIM5,x)
    float f32theta = 0;
    float f32SinVal = 0;
    float f32CosVal = 0;
    float f32FreqHz = 1;
    float f32Tcnt = 0;
    float f32Ts = 20;//unit: ms

    while(pdTRUE)
    {
#if 1
        arm_sin_cos_f32(360*f32FreqHz*f32Tcnt, &f32SinVal, &f32CosVal);
        LED_PWM_SET_COMPARE((f32SinVal+1.0f)/2*LED_PWM_PERIOD);
        f32Tcnt +=f32Ts/1000;
        FR_OS_DelayMs(f32Ts);
        //test for git

#else
        float f32Angle;
        float f32Omega;
        float f32Alpha;
        u16 u16PwmDuty = 0;
        AS5048A_GetAngleInfo(&f32Angle, &f32Omega, &f32Alpha);
        u16PwmDuty = f32Omega/360.0f*LED_PWM_PERIOD;
        LED_PWM_SET_COMPARE(u16PwmDuty);

        FR_OS_DelayMs(20);
#endif
    }
}

void Led2_Task(void * pvParameters)
{
    BaseType_t xRet = pdTRUE;
    u32 u32Cnt = 0;
    EventBits_t EventBitsOut = 0;
    EventWaitParams_t stEventWaitParams;
    u32 NotificationValue;
    memset(&stEventWaitParams,0,sizeof(EventWaitParams_t));

    stEventWaitParams.bClearOnExit = pdTRUE;
    stEventWaitParams.bWaitForAllBits = pdTRUE;
    stEventWaitParams.xTicksToWait = portMAX_DELAY;
    stEventWaitParams.EventBitWaitFor = E_KEY_EVENT_LEFT_UPPER |
                                        E_KEY_EVENT_LEFT_LEFT;
    while(pdTRUE)
    {
        //xRet = xSemaphoreTake(Led2SemMutex_Handle,0);
        //FR_OS_MUTEX_LOCK(Led2SemMutex_Handle);
        //xEventGroupWaitBits(hKeyEvent,E_KEY_EVENT_LEFT_UPPER,pdTRUE,pdFALSE,portMAX_DELAY);
        //FR_OS_EventWait(hKeyEvent,stEventWaitParams, &EventBitsOut);
        //if(EventBitsOut&E_KEY_EVENT_LEFT_UPPER)
        //xRet = xTaskNotifyWait(ULONG_MAX, 0xFFFFFFFF, &NotificationValue, portMAX_DELAY);
        //if(NotificationValue ==2)
        {
            //NotificationValue = 0;
            LED2_ON;
            FR_OS_DelayMs(500);
            LED2_OFF;
            FR_OS_DelayMs(500);

#if 0
            char pStr[]="Apollo 123.456,789,mp,2101";
            char *p;
            int a=100;
            int *pint = &a;
            
            p = strtok(pStr," ,.");
            while(p != NULL)
            {
                DEBUG_INFO("%s\n", p);
                p = strtok(NULL, " ,.");
            }
            
            /*DEBUG_VAR_CHAR(p);
            DEBUG_VAR_INT(a);
            DEBUG_VAR_HEX(a);
            DEBUG_VAR_PIONT(pint);
            DEBUG_INFO("Hello\n");*/
#endif
        }
        //FR_OS_MUTEX_UNLOCK(Led2SemMutex_Handle);
        //FR_OS_DelayMs(20);
    }
}

void LCD_Task(void * pvParameters)
{
    u32 u32Cnt = 0;
    LCD_Clear();
    BaseType_t xRet = pdFAIL;

    ST_ADC_Conv_t stAdcConv;
    memset(&stAdcConv, 0, sizeof(ST_ADC_Conv_t));

    ST_I2C_Params_t stI2cParams;
    memset(&stI2cParams, 0, sizeof(ST_I2C_Params_t));
    
    while(pdTRUE)
    {
        /*u8 u8RegAddr = 0x19;
        u8 pu8Data[4] = {0x04,0x01,0x02,0x03};
        stI2cParams.u8SlaveAddr = 0xD0;
        stI2cParams.pu8RegAddr = &u8RegAddr;
        stI2cParams.u8RegAddrLen = 1;
        stI2cParams.pu8Data = pu8Data;
        stI2cParams.u8DateLen = 4;
        I2C1_SendData_ByTask(&stI2cParams);
        //FR_OS_DelayMs(200);
        //I2C1_ReadData_ByTask(&stI2cParams);
        FR_OS_DelayMs(200);*/

        /*u8 u8SlaveAddr = 0;
        for(u8SlaveAddr=0; u8SlaveAddr< 0xFF; u8SlaveAddr+=2)
        {
            if(I2C1_Check_Device_Online(u8SlaveAddr)==TRUE)
            {
                debug("Found i2c device 0x%02x.\n", u8SlaveAddr);
            }
            FR_OS_DelayMs(20);
        }*/
        FR_OS_DelayMs(500);
        //INA226_Init();

    }
}

void Key_Task(void *pvParameters)
{
    BaseType_t xReturn = pdPASS;
    EN_KEY_FLAG eKeyFlag;
    TickType_t TickCount = 0;
    ST_KeyStatus_t stKeyStatus;
    memset(&stKeyStatus,0,sizeof(ST_KeyStatus_t));

    while(pdTRUE)
    {
        KEY_Scan(&stKeyStatus);
        if(stKeyStatus.u32KeyFlag & E_KEY_LEFT_UPPER)
        {
            KEY_StatusClearBit(&stKeyStatus,E_KEY_LEFT_UPPER);
            eKeyFlag = E_KEY_LEFT_UPPER;
            //xReturn = xQueueSend(Test_QueueHandle, &eKeyFlag, 0);
            //xReturn = xSemaphoreGive(KeySemBin_Handle);
            //xReturn = xSemaphoreTake(Led2SemMutex_Handle, pdTRUE);
            //FR_OS_MUTEX_LOCK(Led2SemMutex_Handle);
            //xEventGroupSetBits(hKeyEvent,E_KEY_EVENT_LEFT_UPPER);
            //FR_OS_EventSetBits(hKeyEvent, E_KEY_EVENT_LEFT_UPPER);
            //xReturn = xTaskNotify(hLed2Task, 0x02, eSetValueWithOverwrite);
            if(xReturn == pdPASS)
            {
                //LED2_ON;
                //FR_OS_DelayMs(100);
                //LED2_OFF;
            }
        }
        if(stKeyStatus.u32KeyFlag & E_KEY_LEFT_LEFT)
        {
            KEY_StatusClearBit(&stKeyStatus,E_KEY_LEFT_LEFT);
            eKeyFlag = E_KEY_LEFT_LEFT;
            //xReturn = xQueueSend(Test_QueueHandle, &eKeyFlag, 0);
            //xReturn = xSemaphoreGive(KeySemBin_Handle);
            //FR_OS_MUTEX_UNLOCK(Led2SemMutex_Handle);
            //FR_OS_EventSetBits(hKeyEvent, E_KEY_EVENT_LEFT_LEFT);
            if(xReturn == pdPASS)
            {
                //LED2_ON;
                //FR_OS_DelayMs(100);
                //LED2_OFF;
            }
        }
        if(stKeyStatus.u32KeyFlag & E_KEY_LEFT_DOWN)
        {
            KEY_StatusClearBit(&stKeyStatus,E_KEY_LEFT_DOWN);
            eKeyFlag = E_KEY_LEFT_LEFT;
            //xReturn = xQueueSend(Test_QueueHandle, &eKeyFlag, 0);
            //xReturn = xSemaphoreGive(KeySemBin_Handle);
            //FR_OS_MUTEX_UNLOCK(Led2SemMutex_Handle);
            FR_OS_EventSetBits(hKeyEvent, E_KEY_EVENT_LEFT_DOWN);
            if(xReturn == pdPASS)
            {
                //LED2_ON;
                //FR_OS_DelayMs(100);
                //LED2_OFF;
            }
        }
        FR_OS_DelayMs(20);
    }
}

void BEEP_Task(void *pvParameters)
{
    BaseType_t xReturn = pdTRUE;
    EN_KEY_FLAG eKeyFlag;
    EventBits_t EventBitsOut = 0;
    EventWaitParams_t stEventWaitParams;
    memset(&stEventWaitParams,0,sizeof(EventWaitParams_t));

    stEventWaitParams.bClearOnExit = pdTRUE;
    stEventWaitParams.bWaitForAllBits = pdFALSE;
    stEventWaitParams.xTicksToWait = portMAX_DELAY;
    stEventWaitParams.EventBitWaitFor = E_KEY_EVENT_LEFT_UPPER |
                                        E_KEY_EVENT_LEFT_LEFT  |
                                        E_KEY_EVENT_LEFT_DOWN;
    while(pdTRUE)
    {
        //xReturn = xQueueReceive(Test_QueueHandle,&eKeyFlag,portMAX_DELAY);
        FR_OS_EventWait(hKeyEvent, stEventWaitParams, &EventBitsOut);
        if(EventBitsOut&E_KEY_EVENT_LEFT_DOWN)
        {
            BeepOn;
            FR_OS_DelayMs(100);
            BeepOff;
            FR_OS_DelayMs(100);
        }
    }
}




