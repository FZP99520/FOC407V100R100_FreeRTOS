#include "ANO_DT.h"
#include "usart.h"

#include "foc.h"

//#include "imu.h"
//#include "mpu6050.h"
//#include "ak8975.h"
//#include "ms5611.h"
//#include "rc.h"
//#include "ctrl.h"
//#include "time.h"
//#include "usbd_user_hid.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define ANO_DT_LEN_INDEX              (3)
#define ANO_DT_DATA_LEN_TO_REMOVE     (4)
#define ANO_DT_BUFF_MAX               (32)//max 32 bytes
#define ANO_DT_FRAME_HEAD_M2S         (0xAAAF)
#define ANO_DT_FRAME_HEAD_S2M         (0xAAAA)

TaskHandle_t hAnoDtStartTask = NULL;
TimerHandle_t hAnoDt_SendData_Timer = NULL;
TaskHandle_t  hAnoDtSendTask = NULL;
TaskHandle_t  hAnoDtReceiveTask = NULL;
QueueHandle_t hAnoDtSend_Queue = NULL;
QueueHandle_t hAnoDtReceive_Queue = NULL;
#if (ANO_DT_SLAVER_MODE_PASSIVE)
SemBinaryHandle_t hAnoDtSendSemBinary = NULL;
#endif

u8 bAnoDtRxErrCode = FALSE;

ST_ANO_DT_FRAME_INFO stAnoDtRecFrameInfo;

ST_ANO_DT_DATA_M2S_RCDATA stAnoDtDataM2SRcdata;

ST_ANO_DT_DATA_S2M_VER    stAnoDtDataS2MVer;
ST_ANO_DT_DATA_S2M_STATUS stAnoDtDataS2MStatus;
ST_ANO_DT_DATA_S2M_SENSER stAnoDtDataS2MSenser;
ST_ANO_DT_DATA_S2M_RCDATA stAnoDtDataS2MRcdata;
ST_ANO_DT_DATA_S2M_POWER  stAnoDtDataS2MPower;

#if (ANO_DT_SUPPORT_FOC == 1)
ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL stAnoDtDataM2SFocInputCtrl;
ST_ANO_DT_DATA_S2M_FOC_STATUS     stAnoDtDataS2MFocStatus;
ST_ANO_DT_DATA_S2M_FOC_INT_INFO1  stAnoDtDataS2MFocIntInfo1;
ST_ANO_DT_DATA_S2M_FOC_INT_INFO2  stAnoDtDataS2MFocIntInfo2;
ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL stAnoDtDataS2MFocInputCtrl;
#endif
ST_ANO_DT_DATA_S2M_MSG    stAnoDtDataS2MMsg;
ST_ANO_DT_DATA_S2M_CHECK  stAnoDtDataS2MCheck;

ST_ANO_DT_FRAME_LIST stAnoDtFrameList;

#if (ANO_DT_MODE_IS_MASTER)
ST_ANO_DT_FRAME_LIST_ITEM astAnoDtFrameListItem[] =
{
    {E_ANO_DT_FUNC_M2S_TYPE_RCDATA, TRUE, TRUE, &stAnoDtDataM2SRcdata, sizeof(ST_ANO_DT_DATA_M2S_RCDATA), NULL, NULL},
#if (ANO_DT_SUPPORT_FOC)
    {E_ANO_DT_FUNC_M2S_TYPE_FOC_INPUT_CTRL, TRUE, TRUE, &stAnoDtDataM2SFocInputCtrl, sizeof(ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL), NULL, NULL}
#endif
};

#else // else (ANO_DT_MODE_IS_MASTER)

ST_ANO_DT_FRAME_LIST_ITEM astAnoDtFrameListItem[] =
{
    {E_ANO_DT_FUNC_S2M_TYPE_STATUS, FALSE, TRUE, &stAnoDtDataS2MStatus, sizeof(ST_ANO_DT_DATA_S2M_STATUS), NULL, NULL},
    {E_ANO_DT_FUNC_S2M_TYPE_POWER, TRUE, TRUE,  &stAnoDtDataS2MPower, sizeof(ST_ANO_DT_DATA_S2M_POWER), NULL, NULL},
    {E_ANO_DT_FUNC_S2M_TYPE_RCDATA, FALSE, TRUE, &stAnoDtDataS2MRcdata, sizeof(ST_ANO_DT_DATA_S2M_RCDATA), NULL, NULL},
#if (ANO_DT_SUPPORT_FOC == 1)
    {E_ANO_DT_FUNC_S2M_TYPE_FOC_STATUS, TRUE, TRUE, &stAnoDtDataS2MFocStatus, sizeof(ST_ANO_DT_DATA_S2M_FOC_STATUS), NULL, NULL}
#endif
};
#endif

#if (ANO_DT_MODE_IS_MASTER)
static void _ANO_DT_Master_Send_Timer_Callback(TimerHandle_t hTimer);
#else
static void _ANO_DT_Slaver_Send_Timer_Callback(TimerHandle_t hTimer);
#endif

void ANO_DT_FRAME_List_Init(ST_ANO_DT_FRAME_LIST *pstList);
void ANO_DT_FRAME_List_InsertEnd(ST_ANO_DT_FRAME_LIST *pstList, ST_ANO_DT_FRAME_LIST_ITEM *pstListItem);

u8 ANO_DT_Send_Data(u8 *pu8Buff, u8 u8len)
{
    u8 u8Ret = FALSE;

    if(u8len <= ANO_DT_BUFF_MAX)
    {
        u8Ret = USART1_SendData(pu8Buff, u8len);
    }
    return u8Ret;
}

void ANO_DT_Start_Task(void * pvParameters)
{
    u8 u8Temp = 0;
    ST_ANO_DT_FRAME_LIST_ITEM *pstAnoDtFrameListItem = NULL;
    TaskCreateParams_t stTaskCreateParams;
    memset(&stTaskCreateParams, 0, sizeof(TaskCreateParams_t));

    taskENTER_CRITICAL();
#if (ANO_DT_MODE_IS_MASTER) 
    stTaskCreateParams.TaskCode = ANO_DT_Master_Send_Task;
    stTaskCreateParams.pcName = "ANO_DT_Master_Send_Task";
#else
    stTaskCreateParams.TaskCode = ANO_DT_Slaver_Send_Task;
    stTaskCreateParams.pcName = "ANO_DT_Slaver_Send_Task";
#endif
    stTaskCreateParams.usStackDepth = 256;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hAnoDtSendTask, stTaskCreateParams);

#if (ANO_DT_MODE_IS_MASTER) 
    stTaskCreateParams.TaskCode = ANO_DT_Master_Receive_Task;
    stTaskCreateParams.pcName = "ANO_DT_Master_Receive_Task";
#else
    stTaskCreateParams.TaskCode = ANO_DT_Slaver_Receive_Task;
    stTaskCreateParams.pcName = "ANO_DT_Slaver_Receive_Task";
#endif
    stTaskCreateParams.usStackDepth = 256;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hAnoDtReceiveTask, stTaskCreateParams);
    //create task end

    //create queue start
    hAnoDtSend_Queue = FR_OS_QueueCreate(3, sizeof(ST_ANO_DT_SEND_DATA));
    hAnoDtReceive_Queue = FR_OS_QueueCreate(3, sizeof(ST_ANO_DT_RECEIVE_DATA));
    //hAnoDtSend_Event = FR_OS_EventCreate();

#if (ANO_DT_SLAVER_MODE_PASSIVE)
    hAnoDtSendSemBinary = FR_OS_SemBinaryCreate();
#endif
    
    hAnoDt_SendData_Timer = xTimerCreate((const char *)   "ANO_DT",
#if (ANO_DT_SLAVER_MODE_PASSIVE == 1) && (!ANO_DT_MODE_IS_MASTER)
                                        (TickType_t)             1,
                                        (UBaseType_t)      pdFALSE, 
#else
                                        (TickType_t)        ANO_DT_FRAME_PERIOD,
                                        (UBaseType_t)       pdTRUE, 
#endif
                                        (void *)                 1,
#if (ANO_DT_MODE_IS_MASTER)
                                        (TimerCallbackFunction_t)_ANO_DT_Master_Send_Timer_Callback);
#else
                                        (TimerCallbackFunction_t)_ANO_DT_Slaver_Send_Timer_Callback);
#endif

#if (!ANO_DT_SLAVER_MODE_PASSIVE)
    if(hAnoDt_SendData_Timer != NULL)
    {
        xTimerStart(hAnoDt_SendData_Timer, 0);
    }
#endif

    ANO_DT_FRAME_List_Init(&stAnoDtFrameList);
    for(u8Temp=0; u8Temp<sizeof(astAnoDtFrameListItem)/sizeof(ST_ANO_DT_FRAME_LIST_ITEM); u8Temp++)
    {
        ANO_DT_FRAME_List_InsertEnd(&stAnoDtFrameList, &astAnoDtFrameListItem[u8Temp]);
    }

    vTaskDelete(hAnoDtStartTask);
    taskEXIT_CRITICAL();
}

void ANO_DT_FRAME_List_Init(ST_ANO_DT_FRAME_LIST *pstList)
{
    memset(&pstList->stListItemStart, 0, sizeof(ST_ANO_DT_FRAME_LIST_ITEM));
    pstList->u8ListItemNum = 0;
    pstList->pstHead  = &(pstList->stListItemStart);
    pstList->pstIndex = &(pstList->stListItemStart);
    pstList->pstEnd   = &(pstList->stListItemStart);
}

void ANO_DT_FRAME_List_InsertEnd(ST_ANO_DT_FRAME_LIST *pstList, ST_ANO_DT_FRAME_LIST_ITEM *pstListItem)
{
    if(pstList == NULL) return;
    if(pstListItem == NULL) return;

    pstListItem->pvContainer = pstList;
    if(pstList->u8ListItemNum == 0)
    {
        memcpy(&pstList->stListItemStart, pstListItem, sizeof(ST_ANO_DT_FRAME_LIST_ITEM));
        pstList->stListItemStart.pstNext = NULL;
    }
    else
    {
        pstList->pstEnd->pstNext = pstListItem;
        pstList->pstEnd = pstListItem;
        pstList->pstEnd->pstNext = NULL;
    }
    pstList->u8ListItemNum++;
    if(pstListItem->bAutoRun && pstListItem->bActive)
    {
       pstList->pstIndex =  pstListItem;
    }
}


#if (ANO_DT_MODE_IS_MASTER)

static u8 _ANO_DT_Master_Send_Command(u8 *pu8DataBuff, u8 u8Cmd)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    EN_ANO_DT_M2S_COMMAND_TYPE eCmd = (EN_ANO_DT_M2S_COMMAND_TYPE)u8Cmd;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAF;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_M2S_TYPE_COMMAND;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = (u8)eCmd;
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Master_Send_Ack(u8 *pu8DataBuff, u8 u8Ack)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    EN_ANO_DT_M2S_ACK_TYPE eAck = (EN_ANO_DT_M2S_ACK_TYPE)u8Ack;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAF;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_M2S_TYPE_ACK;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = (u8)eAck;
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Master_Send_Rcdata(u8 *pu8DataBuff, u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 u8Cnt = 0;
    u8 u8temp = 0;
    u8 u8Sum = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAF;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_M2S_TYPE_RCDATA;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = BYTE1(thr);
    pu8DataBuff[u8Cnt++] = BYTE0(thr);
    pu8DataBuff[u8Cnt++] = BYTE1(yaw);
    pu8DataBuff[u8Cnt++] = BYTE0(yaw);
    pu8DataBuff[u8Cnt++] = BYTE1(rol);
    pu8DataBuff[u8Cnt++] = BYTE0(rol);
    pu8DataBuff[u8Cnt++] = BYTE1(pit);
    pu8DataBuff[u8Cnt++] = BYTE0(pit);
    pu8DataBuff[u8Cnt++] = BYTE1(aux1);
    pu8DataBuff[u8Cnt++] = BYTE0(aux1);
    pu8DataBuff[u8Cnt++] = BYTE1(aux2);
    pu8DataBuff[u8Cnt++] = BYTE0(aux2);
    pu8DataBuff[u8Cnt++] = BYTE1(aux3);
    pu8DataBuff[u8Cnt++] = BYTE0(aux3);
    pu8DataBuff[u8Cnt++] = BYTE1(aux4);
    pu8DataBuff[u8Cnt++] = BYTE0(aux4);
    pu8DataBuff[u8Cnt++] = BYTE1(aux5);
    pu8DataBuff[u8Cnt++] = BYTE0(aux5);
    pu8DataBuff[u8Cnt++] = BYTE1(aux6);
    pu8DataBuff[u8Cnt++] = BYTE0(aux6);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Master_Send_PID(u8 *pu8DataBuff, u8 group,
                                    s16 s16PID1_P, s16 s16PID1_I, s16 s16PID1_D,
                                    s16 s16PID2_P, s16 s16PID2_I, s16 s16PID2_D,
                                    s16 s16PID3_P, s16 s16PID3_I, s16 s16PID3_D)
{
    u8 u8Cnt=0;
    u8 u8Sum = 0;
    u8 u8temp = 0;
    vs16 vs16temp;

    if((group<E_ANO_DT_FUNC_S2M_TYPE_PID1) || (group>E_ANO_DT_FUNC_S2M_TYPE_PID6))
    {
        return FALSE;
    }

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAF;
    pu8DataBuff[u8Cnt++] = (EN_ANO_DT_FUNC_M2S_TYPE)group;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16PID1_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID1_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID1_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

#if (ANO_DT_SUPPORT_FOC)
static u8 _ANO_DT_Master_Send_FOC_InputCtrl(u8 *pu8DataBuff, s16 s16Angle, s16 s16Omega, s16 s16I_d, s16 s16I_q, u8 u8lock_mode)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    vs16 vs16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAF;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_M2S_TYPE_FOC_INPUT_CTRL;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16Angle;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Omega;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_d;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_q;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[u8Cnt++] = u8lock_mode;

    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}
#endif
static u8 _ANO_DT_Master_Send_Handler(u8 *pu8DataBuff, EN_FUNC_S2M_TYPE eFuncType, const void *pParma, u8 u8size)
{
    u8 u8Ret = FALSE;
    switch(eFuncType)
    {
        case E_ANO_DT_FUNC_M2S_TYPE_COMMAND:
        {
            EN_ANO_DT_M2S_COMMAND_TYPE eCmd = *(EN_ANO_DT_M2S_COMMAND_TYPE *)pParma;
            if(u8size != sizeof(EN_ANO_DT_M2S_COMMAND_TYPE)) return FALSE;
            u8Ret = _ANO_DT_Master_Send_Command(pu8DataBuff, eCmd);
            break;
        }
        case E_ANO_DT_FUNC_M2S_TYPE_ACK:
        {
            EN_ANO_DT_M2S_ACK_TYPE eAckType = *(EN_ANO_DT_M2S_ACK_TYPE *)pParma;
            if(u8size != sizeof(EN_ANO_DT_M2S_ACK_TYPE)) return FALSE;
            u8Ret = _ANO_DT_Master_Send_Command(pu8DataBuff, eAckType);
            break;
        }
        case E_ANO_DT_FUNC_M2S_TYPE_RCDATA:
        {
            ST_ANO_DT_DATA_M2S_RCDATA *pstRcdata = (ST_ANO_DT_DATA_M2S_RCDATA *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_M2S_RCDATA)) return FALSE;
            u8Ret = _ANO_DT_Master_Send_Rcdata(pu8DataBuff,
                                                pstRcdata->s16THR,
                                                pstRcdata->s16YAW,
                                                pstRcdata->s16ROL,
                                                pstRcdata->s16PIT,
                                                pstRcdata->s16AUX1,
                                                pstRcdata->s16AUX2,
                                                pstRcdata->s16AUX3,
                                                pstRcdata->s16AUX4,
                                                pstRcdata->s16AUX5,
                                                pstRcdata->s16AUX6);
            break;
        }
#if (ANO_DT_SUPPORT_FOC)
        case E_ANO_DT_FUNC_M2S_TYPE_FOC_INPUT_CTRL:
        {
            ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL *pstAnoDt_DataM2SFocInputCtrl = (ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL)) return FALSE;
            u8Ret = _ANO_DT_Master_Send_FOC_InputCtrl(pu8DataBuff,
                                                pstAnoDt_DataM2SFocInputCtrl->s16Angle,
                                                pstAnoDt_DataM2SFocInputCtrl->s16Omega,
                                                pstAnoDt_DataM2SFocInputCtrl->s16I_d,
                                                pstAnoDt_DataM2SFocInputCtrl->s16I_q,
                                                pstAnoDt_DataM2SFocInputCtrl->u8lock_mode);
            break;
        }
#endif
        default:
        {
            break;
        }
    }

    return u8Ret;
}

static void _ANO_DT_Master_Send_Timer_Callback(TimerHandle_t hTimer)
{
    static ST_ANO_DT_SEND_DATA *pstAnoDtSendData = NULL;
    ST_ANO_DT_FRAME_LIST_ITEM *pstListItem = NULL;

    if(pstAnoDtSendData == NULL)
    {
        pstAnoDtSendData = pvPortMalloc(sizeof(ST_ANO_DT_SEND_DATA));
        if(pstAnoDtSendData == NULL) return;
    }

    if(stAnoDtFrameList.pstIndex != NULL)
    {
        if((stAnoDtFrameList.pstIndex->bAutoRun == TRUE) && (stAnoDtFrameList.pstIndex->bActive == TRUE))
        {
            pstAnoDtSendData->u8FuncType = stAnoDtFrameList.pstIndex->eFuncType;
            pstAnoDtSendData->u8DataSize = stAnoDtFrameList.pstIndex->u8ParmaSize;
            if(stAnoDtFrameList.pstIndex->pParma != NULL)
            {
                memcpy(pstAnoDtSendData->pu8DataBuff, stAnoDtFrameList.pstIndex->pParma, stAnoDtFrameList.pstIndex->u8ParmaSize);
            }
            if(uxQueueMessagesWaiting(hAnoDtSend_Queue) == 0)
            {
                FR_OS_QueueSend(hAnoDtSend_Queue, pstAnoDtSendData, 0);
            }
        }

        while(TRUE)
        {
            pstListItem = stAnoDtFrameList.pstIndex->pstNext;
            if(pstListItem == NULL)//means to the end
            {
                stAnoDtFrameList.pstIndex = stAnoDtFrameList.pstHead;
                break;
            }
            else
            {
                stAnoDtFrameList.pstIndex = pstListItem;
                if(pstListItem->bAutoRun && pstListItem->bActive)
                {
                    break;
                }
            }
        }
    }    
}


void ANO_DT_Master_Send_Task(void * pvParameters)
{
    u8 u8Index = 0;
    u8 *pu8SendBuff = NULL;
    ST_ANO_DT_SEND_DATA *pstAnoDtSendData = NULL;
    BaseType_t xReturn = pdFALSE;


    pu8SendBuff = pvPortMalloc(ANO_DT_BUFF_MAX);
    pstAnoDtSendData = pvPortMalloc(sizeof(ST_ANO_DT_SEND_DATA));

    if(pu8SendBuff == NULL) while(1);
    if(pstAnoDtSendData == NULL) while(1);
 
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hAnoDtSend_Queue, pstAnoDtSendData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            _ANO_DT_Master_Send_Handler(pu8SendBuff,pstAnoDtSendData->u8FuncType,
                                                    pstAnoDtSendData->pu8DataBuff,
                                                    pstAnoDtSendData->u8DataSize);
        }
    }
}

static u8 _ANO_DT_Master_Receive_Version(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_VER *pstAnoDtDataS2MVer)
{
    pstAnoDtDataS2MVer->u8HardwareType = pu8DataBuff[0];
    pstAnoDtDataS2MVer->u16HardwareVER = (u16)pu8DataBuff[1]<<8 | pu8DataBuff[2];
    pstAnoDtDataS2MVer->u16SoftwareVER = (u16)pu8DataBuff[3]<<8 | pu8DataBuff[4];
    pstAnoDtDataS2MVer->u16ProtocolVER = (u16)pu8DataBuff[5]<<8 | pu8DataBuff[6];
    pstAnoDtDataS2MVer->u16BootloaderVER = (u16)pu8DataBuff[7]<<8 | pu8DataBuff[8];
}

static u8 _ANO_DT_Master_Receive_Status(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_STATUS *pstAnoDtDataS2MStatus)
{
    pstAnoDtDataS2MStatus->s16ROL = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MStatus->s16PIT = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MStatus->s16YAW = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MStatus->s32ALT_USE = (s32)pu8DataBuff[6]<<24|pu8DataBuff[7]<<16|pu8DataBuff[8]<<8|pu8DataBuff[9];
    pstAnoDtDataS2MStatus->u8FLY_MODEL = (u8)pu8DataBuff[10];
    pstAnoDtDataS2MStatus->u8ARMED = (u8)pu8DataBuff[11];
    return TRUE;
}

static u8 _ANO_DT_Master_Receive_Senser(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_SENSER *pstAnoDtDataS2MSenser)
{
    pstAnoDtDataS2MSenser->s16ACC_X  = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MSenser->s16ACC_Y  = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MSenser->s16ACC_Z  = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MSenser->s16GYRO_X = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MSenser->s16GYRO_Y = (s16)pu8DataBuff[8]<<8 | pu8DataBuff[9];
    pstAnoDtDataS2MSenser->s16GYRO_Z = (s16)pu8DataBuff[10]<<8 | pu8DataBuff[11];
    pstAnoDtDataS2MSenser->s16MAG_X  = (s16)pu8DataBuff[12]<<8 | pu8DataBuff[13];
    pstAnoDtDataS2MSenser->s16MAG_Y  = (s16)pu8DataBuff[14]<<8 | pu8DataBuff[15];
    pstAnoDtDataS2MSenser->s16MAG_Z  = (s16)pu8DataBuff[16]<<8 | pu8DataBuff[17];
    return TRUE;
}

static u8 _ANO_DT_Master_Receive_Power(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_POWER *pstAnoDtDataS2MPower)
{
    pstAnoDtDataS2MPower->u16Votage  = (u16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MPower->u16Current = (u16)pu8DataBuff[2]<<8 | pu8DataBuff[3];

    return TRUE;
}
#if (ANO_DT_SUPPORT_FOC)
static u8 _ANO_DT_Master_Receive_FOC_Status(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_FOC_STATUS *pstAnoDtDataS2MFOCStatus)
{
    pstAnoDtDataS2MFOCStatus->s16Angle_M = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MFOCStatus->s16Omega_M = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MFOCStatus->s16Alpha_M = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MFOCStatus->s16Angle_Sum = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MFOCStatus->u8lock_mode = (u8)pu8DataBuff[8];
    pstAnoDtDataS2MFOCStatus->u8IsActive = (u8)pu8DataBuff[9];
    pstAnoDtDataS2MFOCStatus->u8status =   (u8)pu8DataBuff[10];
    return TRUE;
}

static u8 _ANO_DT_Master_Receive_FOC_IntInfo1(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_FOC_INT_INFO1 *pstAnoDtDataS2MFOCIntInfo1)
{
    pstAnoDtDataS2MFOCIntInfo1->s16U_d      = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MFOCIntInfo1->s16U_q      = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MFOCIntInfo1->s16theta    = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MFOCIntInfo1->s16U_alpha  = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MFOCIntInfo1->s16U_beta   = (s16)pu8DataBuff[8]<<8 | pu8DataBuff[9];
    pstAnoDtDataS2MFOCIntInfo1->u8N         = (u8)pu8DataBuff[10];
    pstAnoDtDataS2MFOCIntInfo1->u8Sector    = (u8)pu8DataBuff[11];
    pstAnoDtDataS2MFOCIntInfo1->u16T_a      = (u16)pu8DataBuff[12]<<8 | pu8DataBuff[13];
    pstAnoDtDataS2MFOCIntInfo1->u16T_b      = (u16)pu8DataBuff[14]<<8 | pu8DataBuff[15];
    pstAnoDtDataS2MFOCIntInfo1->u16T_c      = (u16)pu8DataBuff[16]<<8 | pu8DataBuff[17];
    return TRUE;
}

static u8 _ANO_DT_Master_Receive_FOC_IntInfo2(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_FOC_INT_INFO2 *pstAnoDtDataS2MFOCIntInfo2)
{
    pstAnoDtDataS2MFOCIntInfo2->s16I_a      = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MFOCIntInfo2->s16I_b      = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MFOCIntInfo2->s16I_c      = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MFOCIntInfo2->s16I_d      = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MFOCIntInfo2->s16I_q      = (s16)pu8DataBuff[8]<<8 | pu8DataBuff[9];
    pstAnoDtDataS2MFOCIntInfo2->s16I_alpha  = (s16)pu8DataBuff[10]<<8| pu8DataBuff[11];
    pstAnoDtDataS2MFOCIntInfo2->s16I_beta   = (s16)pu8DataBuff[12]<<8 | pu8DataBuff[13];
    pstAnoDtDataS2MFOCIntInfo2->s16Sin_theta    = (s16)pu8DataBuff[14]<<8 | pu8DataBuff[15];
    pstAnoDtDataS2MFOCIntInfo2->s16Cos_theta    = (s16)pu8DataBuff[16]<<8 | pu8DataBuff[17];
    pstAnoDtDataS2MFOCIntInfo2->u16PWMA    = (u16)pu8DataBuff[18]<<8 | pu8DataBuff[19];
    pstAnoDtDataS2MFOCIntInfo2->u16PWMB    = (u16)pu8DataBuff[20]<<8 | pu8DataBuff[21];
    pstAnoDtDataS2MFOCIntInfo2->u16PWMC    = (u16)pu8DataBuff[22]<<8 | pu8DataBuff[23];
    return TRUE;
}

static u8 _ANO_DT_Master_Receive_FOC_Input(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL *pstAnoDtDataS2MFOCInput)
{
    pstAnoDtDataS2MFOCInput->s16Angle      = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MFOCInput->s16Omega      = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MFOCInput->s16I_d        = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MFOCInput->s16I_q        = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MFOCInput->u8lock_mode   = (u8)pu8DataBuff[8];
    return TRUE;
}


#endif

static u8 _ANO_DT_Master_Receive_Handler(u8 *pu8Buff, u8 u8len)
{
    u8 u8Ret = FALSE;
    u8 u8temp = 0;
    u8 u8CaculateSum = 0;
    u8 u8GetSum = 0;
    u8 *pu8DataBuff = NULL;
    u8 u8DataLen = 0;
    u16 u16Head = 0;
    EN_FUNC_S2M_TYPE eFuncType = E_ANO_DT_FUNC_S2M_TYPE_VER;

    u16Head = ((u16)pu8Buff[0]<<8)|pu8Buff[1];
    stAnoDtRecFrameInfo.u16Head     = u16Head;
    if(u16Head != ANO_DT_FRAME_HEAD_S2M)
    {
        stAnoDtRecFrameInfo.u8ErrCnt++;
        return FALSE;
    }

    for(u8temp=0; u8temp<(u8len-1); u8temp++)
    {
        u8CaculateSum += pu8Buff[u8temp];
    }
    u8GetSum = pu8Buff[u8len-1];
    stAnoDtRecFrameInfo.u8sum       = u8GetSum;
    if(u8CaculateSum != u8GetSum) 
    {
        //check sum failed
        stAnoDtRecFrameInfo.u8ErrCnt |= 1<<7;
        return FALSE;
    }

    eFuncType = pu8Buff[2];
    u8DataLen = pu8Buff[3];
    pu8DataBuff = &pu8Buff[4];
    
    stAnoDtRecFrameInfo.u8FuncType  = (EN_ANO_DT_FUNC_M2S_TYPE)eFuncType;
    stAnoDtRecFrameInfo.u8datalen   = u8DataLen;
    stAnoDtRecFrameInfo.pu8Buff     = pu8DataBuff; //get data pointer
    
    switch(eFuncType)
    {
        case E_ANO_DT_FUNC_S2M_TYPE_VER:
        {
            u8Ret = _ANO_DT_Master_Receive_Version(pu8DataBuff, &stAnoDtDataS2MVer);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_STATUS:
        {
            u8Ret = _ANO_DT_Master_Receive_Status(pu8DataBuff, &stAnoDtDataS2MStatus);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_SENSER:
        {
            u8Ret = _ANO_DT_Master_Receive_Senser(pu8DataBuff, &stAnoDtDataS2MSenser);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_POWER:
        {
            u8Ret = _ANO_DT_Master_Receive_Power(pu8DataBuff, &stAnoDtDataS2MPower);
            break;
        }
#if (ANO_DT_SUPPORT_FOC)
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_STATUS:
        {
            u8Ret = _ANO_DT_Master_Receive_FOC_Status(pu8DataBuff, &stAnoDtDataS2MFocStatus);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO1:
        {
            u8Ret = _ANO_DT_Master_Receive_FOC_IntInfo1(pu8DataBuff, &stAnoDtDataS2MFocIntInfo1);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO2:
        {
            u8Ret = _ANO_DT_Master_Receive_FOC_IntInfo2(pu8DataBuff, &stAnoDtDataS2MFocIntInfo2);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INPUT_CTRL:
        {
            u8Ret = _ANO_DT_Master_Receive_FOC_Input(pu8DataBuff, &stAnoDtDataS2MFocInputCtrl);
            break;
        }
#endif
        default:
        {
            u8Ret = FALSE;
            break;
        }
    }

    return u8Ret;
    
}

void ANO_DT_Master_Receive_Task(void * pvParameters)
{
    u8 *pu8RecBuff = NULL;
    BaseType_t xReturn = pdFALSE;
    ST_ANO_DT_RECEIVE_DATA stAnoDt_ReceiveData;

    memset(&stAnoDt_ReceiveData, 0, sizeof(ST_ANO_DT_RECEIVE_DATA));
    pu8RecBuff = pvPortMalloc(ANO_DT_BUFF_MAX);
    if(pu8RecBuff == NULL) while(1);
 
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hAnoDtReceive_Queue, &stAnoDt_ReceiveData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            memcpy(pu8RecBuff, stAnoDt_ReceiveData.pu8Buff, stAnoDt_ReceiveData.u8len);
            if(_ANO_DT_Master_Receive_Handler(pu8RecBuff, stAnoDt_ReceiveData.u8len) != TRUE)
            {
                bAnoDtRxErrCode = TRUE;
            }
        }
    }
}


#else //(ANO_DT_MODE_IS_MASTER == 0)
static u8 _ANO_DT_Slaver_Send_Version(u8 *pu8DataBuff, u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
    u8 u8Cnt=0;
    u8 u8Sum = 0;
    u8 u8temp = 0;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_VER;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = hardware_type;
    pu8DataBuff[u8Cnt++] = BYTE1(hardware_ver);
    pu8DataBuff[u8Cnt++] = BYTE0(hardware_ver);
    pu8DataBuff[u8Cnt++] = BYTE1(software_ver);
    pu8DataBuff[u8Cnt++] = BYTE0(software_ver);
    pu8DataBuff[u8Cnt++] = BYTE1(protocol_ver);
    pu8DataBuff[u8Cnt++] = BYTE0(protocol_ver);
    pu8DataBuff[u8Cnt++] = BYTE1(bootloader_ver);
    pu8DataBuff[u8Cnt++] = BYTE0(bootloader_ver);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++]=u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_Status(u8 *pu8DataBuff, s16 s16ROL, s16 s16PIT, s16 s16YAW, s32 u32alt, u8 u8fly_model, u8 u8armed)
{
    u8 u8Cnt=0;
    u8 u8Sum = 0;
    u8 u8temp = 0;
    vs16 vs16temp = 0;
    vs32 vs32temp = u32alt;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_STATUS;
    pu8DataBuff[u8Cnt++] = 0;

    vs16temp = s16ROL;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PIT;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16YAW;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE3(vs32temp);
    pu8DataBuff[u8Cnt++] = BYTE2(vs32temp);
    pu8DataBuff[u8Cnt++] = BYTE1(vs32temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs32temp);
    pu8DataBuff[u8Cnt++] = u8fly_model;
    pu8DataBuff[u8Cnt++] = u8armed;
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++]=u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_Senser(u8 *pu8DataBuff,s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 u8Cnt = 0;
    u8 u8temp = 0;
    u8 u8Sum = 0;
    vs16 vs16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_SENSER;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = a_x;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = a_y;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = a_z;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = g_x;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = g_y;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = g_z;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = m_x;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = m_y;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = m_z;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_RCData(u8 *pu8DataBuff,u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 u8Cnt = 0;
    u8 u8temp = 0;
    u8 u8Sum = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_RCDATA;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = BYTE1(thr);
    pu8DataBuff[u8Cnt++] = BYTE0(thr);
    pu8DataBuff[u8Cnt++] = BYTE1(yaw);
    pu8DataBuff[u8Cnt++] = BYTE0(yaw);
    pu8DataBuff[u8Cnt++] = BYTE1(rol);
    pu8DataBuff[u8Cnt++] = BYTE0(rol);
    pu8DataBuff[u8Cnt++] = BYTE1(pit);
    pu8DataBuff[u8Cnt++] = BYTE0(pit);
    pu8DataBuff[u8Cnt++] = BYTE1(aux1);
    pu8DataBuff[u8Cnt++] = BYTE0(aux1);
    pu8DataBuff[u8Cnt++] = BYTE1(aux2);
    pu8DataBuff[u8Cnt++] = BYTE0(aux2);
    pu8DataBuff[u8Cnt++] = BYTE1(aux3);
    pu8DataBuff[u8Cnt++] = BYTE0(aux3);
    pu8DataBuff[u8Cnt++] = BYTE1(aux4);
    pu8DataBuff[u8Cnt++] = BYTE0(aux4);
    pu8DataBuff[u8Cnt++] = BYTE1(aux5);
    pu8DataBuff[u8Cnt++] = BYTE0(aux5);
    pu8DataBuff[u8Cnt++] = BYTE1(aux6);
    pu8DataBuff[u8Cnt++] = BYTE0(aux6);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_Power(u8 *pu8DataBuff, u16 votage, u16 current)
{
    u8 u8Cnt=0;
    u8 u8temp = 0;
    u16 temp;
    u8 u8Sum = 0;
    
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_POWER;
    pu8DataBuff[u8Cnt++] = 0;
    temp = votage;
    pu8DataBuff[u8Cnt++] = BYTE1(temp);
    pu8DataBuff[u8Cnt++] = BYTE0(temp);
    temp = current;
    pu8DataBuff[u8Cnt++] = BYTE1(temp);
    pu8DataBuff[u8Cnt++] = BYTE0(temp);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_MotoPWM(u8 *pu8DataBuff, u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
    u8 u8Cnt=0;
    u8 u8temp = 0;
    u8 u8Sum = 0;
    
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_MOTO;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = BYTE1(m_1);
    pu8DataBuff[u8Cnt++] = BYTE0(m_1);
    pu8DataBuff[u8Cnt++] = BYTE1(m_2);
    pu8DataBuff[u8Cnt++] = BYTE0(m_2);
    pu8DataBuff[u8Cnt++] = BYTE1(m_3);
    pu8DataBuff[u8Cnt++] = BYTE0(m_3);
    pu8DataBuff[u8Cnt++] = BYTE1(m_4);
    pu8DataBuff[u8Cnt++] = BYTE0(m_4);
    pu8DataBuff[u8Cnt++] = BYTE1(m_5);
    pu8DataBuff[u8Cnt++] = BYTE0(m_5);
    pu8DataBuff[u8Cnt++] = BYTE1(m_6);
    pu8DataBuff[u8Cnt++] = BYTE0(m_6);
    pu8DataBuff[u8Cnt++] = BYTE1(m_7);
    pu8DataBuff[u8Cnt++] = BYTE0(m_7);
    pu8DataBuff[u8Cnt++] = BYTE1(m_8);
    pu8DataBuff[u8Cnt++] = BYTE0(m_8);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;
    
    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_PID(u8 *pu8DataBuff, u8 group,
                                    s16 s16PID1_P, s16 s16PID1_I, s16 s16PID1_D,
                                    s16 s16PID2_P, s16 s16PID2_I, s16 s16PID2_D,
                                    s16 s16PID3_P, s16 s16PID3_I, s16 s16PID3_D)
{
    u8 u8Cnt=0;
    u8 u8Sum = 0;
    u8 u8temp = 0;
    vs16 vs16temp;

    if((group<E_ANO_DT_FUNC_S2M_TYPE_PID1) || (group>E_ANO_DT_FUNC_S2M_TYPE_PID6))
    {
        return FALSE;
    }

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = (EN_FUNC_S2M_TYPE)group;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16PID1_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID1_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID1_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID2_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_P;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_I;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16PID3_D;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}
#if (ANO_DT_SUPPORT_FOC == 1)
static u8 _ANO_DT_Slaver_Send_FOC_Status(u8 *pu8DataBuff, s16 s16Angle, s16 s16Omega, s16 s16Alpha,
                                                    s16 s16Angle_Sum, u8 u8lock_mode, u8 u8IsActive, u8 u8status)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    vs16 vs16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_FOC_STATUS;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16Angle;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Omega;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Alpha;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Angle_Sum;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[u8Cnt++] = u8lock_mode;
    pu8DataBuff[u8Cnt++] = u8IsActive;
    pu8DataBuff[u8Cnt++] = u8status;
    
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_FOC_IntInfo1(u8 *pu8DataBuff, s16 s16U_d, s16 s16U_q, s16 s16theta,
                                                    s16 s16U_alpha, s16 s16U_beta, u8 u8N, u8 Sector,
                                                    u16 u16T_a, u16 u16T_b, u16 u16T_c)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    vs16 vs16temp = 0;
    vu16 vu16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO1;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16U_d;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16U_q;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16theta;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16U_alpha;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16U_beta;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[u8Cnt++] = u8N;
    pu8DataBuff[u8Cnt++] = Sector;
    vu16temp = u16T_a;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    vu16temp = u16T_b;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    vu16temp = u16T_c;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_FOC_IntInfo2(u8 *pu8DataBuff, s16 s16I_a, s16 s16I_b, s16 s16I_c,
                                                    s16 s16I_d, s16 s16I_q, s16 s16I_alpha, s16 s16I_beta, 
                                                    s16 s16Sin, s16 s16Cos, u16 u16PWMA, u16 u16PWMB, u16 u16PWMC)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    vs16 vs16temp = 0;
    vu16 vu16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO2;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16I_a;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_b;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_c;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_d;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_q;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_alpha;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_beta;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Sin;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Cos;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vu16temp = u16PWMA;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    vu16temp = u16PWMB;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    vu16temp = u16PWMC;
    pu8DataBuff[u8Cnt++] = BYTE1(vu16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vu16temp);
    
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_FOC_InputCtrl(u8 *pu8DataBuff, s16 s16Angle, s16 s16Omega, s16 s16I_d, s16 s16I_q, u8 u8lock_mode)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    vs16 vs16temp = 0;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_FOC_INPUT_CTRL;
    pu8DataBuff[u8Cnt++] = 0;
    vs16temp = s16Angle;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16Omega;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_d;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    vs16temp = s16I_q;
    pu8DataBuff[u8Cnt++] = BYTE1(vs16temp);
    pu8DataBuff[u8Cnt++] = BYTE0(vs16temp);
    pu8DataBuff[u8Cnt++] = u8lock_mode;

    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

#endif
static u8 _ANO_DT_Slaver_Send_Msg(u8 *pu8DataBuff, u8 u8MsgID, u8 u8MsgData)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    EN_ANO_DT_S2M_MSG_ID eMsgID = (EN_ANO_DT_S2M_MSG_ID)u8MsgID;
    EN_ANO_DT_S2M_MSG_DATA eMsgData = (EN_ANO_DT_S2M_MSG_DATA)u8MsgData;

    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_MSG;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = (u8)eMsgID;
    pu8DataBuff[u8Cnt++] = (u8)eMsgData;
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_Check(u8 *pu8DataBuff, u8 head, u8 check_u8Sum)
{
    u8 u8Sum = 0;
    u8 u8temp = 0;
    u8 u8Cnt = 0;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = E_ANO_DT_FUNC_S2M_TYPE_CHECK;
    pu8DataBuff[u8Cnt++] = 0;
    pu8DataBuff[u8Cnt++] = head;
    pu8DataBuff[u8Cnt++] = check_u8Sum;
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<6; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_UserData(u8 *pu8DataBuff, u8 *buff,u8 len,u8 channel)
{
    u8 u8Cnt = 0;
    u8 u8temp = 0;
    u8 u8Sum = 0;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = 0xAA;
    pu8DataBuff[u8Cnt++] = channel + E_ANO_DT_FUNC_S2M_TYPE_USER_DATA1;
    pu8DataBuff[u8Cnt++] = 0;
    for(u8temp=0; u8temp<len; u8temp++)
    {
        pu8DataBuff[u8Cnt++] = buff[u8temp];
    }
    pu8DataBuff[ANO_DT_LEN_INDEX] = u8Cnt - ANO_DT_DATA_LEN_TO_REMOVE;

    for(u8temp=0; u8temp<u8Cnt; u8temp++)
    {
        u8Sum += pu8DataBuff[u8temp];
    }
    pu8DataBuff[u8Cnt++] = u8Sum;

    return ANO_DT_Send_Data(pu8DataBuff, u8Cnt);
}

static u8 _ANO_DT_Slaver_Send_Handler(u8 *pu8DataBuff, EN_FUNC_S2M_TYPE eFuncType, const void *pParma, u8 u8size)
{
    u8 u8Ret = FALSE;
    switch(eFuncType)
    {
        case E_ANO_DT_FUNC_S2M_TYPE_VER:
        {
            ST_ANO_DT_DATA_S2M_VER *pstAnoDt_DataS2MVer = (ST_ANO_DT_DATA_S2M_VER *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_VER)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Version(pu8DataBuff,
                                                pstAnoDt_DataS2MVer->u8HardwareType,
                                                pstAnoDt_DataS2MVer->u16HardwareVER,
                                                pstAnoDt_DataS2MVer->u16SoftwareVER,
                                                pstAnoDt_DataS2MVer->u16ProtocolVER,
                                                pstAnoDt_DataS2MVer->u16BootloaderVER);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_STATUS:
        {
            ST_ANO_DT_DATA_S2M_STATUS *pstAnoDt_DataS2MStatus = (ST_ANO_DT_DATA_S2M_STATUS *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_STATUS)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Status(pu8DataBuff,
                                                pstAnoDt_DataS2MStatus->s16ROL,
                                                pstAnoDt_DataS2MStatus->s16PIT,
                                                pstAnoDt_DataS2MStatus->s16YAW,
                                                pstAnoDt_DataS2MStatus->s32ALT_USE,
                                                pstAnoDt_DataS2MStatus->u8FLY_MODEL,
                                                pstAnoDt_DataS2MStatus->u8ARMED);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_SENSER:
        {
            ST_ANO_DT_DATA_S2M_SENSER *pstAnoDt_DataS2MSensor = (ST_ANO_DT_DATA_S2M_SENSER *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_SENSER)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Senser(pu8DataBuff,
                                                pstAnoDt_DataS2MSensor->s16ACC_X,
                                                pstAnoDt_DataS2MSensor->s16ACC_Y,
                                                pstAnoDt_DataS2MSensor->s16ACC_Z,
                                                pstAnoDt_DataS2MSensor->s16GYRO_X,
                                                pstAnoDt_DataS2MSensor->s16GYRO_Y,
                                                pstAnoDt_DataS2MSensor->s16GYRO_Z,
                                                pstAnoDt_DataS2MSensor->s16MAG_X,
                                                pstAnoDt_DataS2MSensor->s16MAG_Y,
                                                pstAnoDt_DataS2MSensor->s16MAG_Z);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_RCDATA:
        {
            ST_ANO_DT_DATA_S2M_RCDATA *pstAnoDt_DataS2MRcdata = (ST_ANO_DT_DATA_S2M_RCDATA *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_RCDATA)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_RCData(pu8DataBuff,
                                                pstAnoDt_DataS2MRcdata->s16THR,
                                                pstAnoDt_DataS2MRcdata->s16YAW,
                                                pstAnoDt_DataS2MRcdata->s16ROL,
                                                pstAnoDt_DataS2MRcdata->s16PIT,
                                                pstAnoDt_DataS2MRcdata->s16AUX1,
                                                pstAnoDt_DataS2MRcdata->s16AUX2,
                                                pstAnoDt_DataS2MRcdata->s16AUX3,
                                                pstAnoDt_DataS2MRcdata->s16AUX4,
                                                pstAnoDt_DataS2MRcdata->s16AUX5,
                                                pstAnoDt_DataS2MRcdata->s16AUX6);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_POWER:
        {
            ST_ANO_DT_DATA_S2M_POWER *pstAnoDt_DataS2MPower = (ST_ANO_DT_DATA_S2M_POWER *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_POWER)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Power(pu8DataBuff,
                                                pstAnoDt_DataS2MPower->u16Votage,
                                                pstAnoDt_DataS2MPower->u16Current);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_MOTO:
        {
            ST_ANO_DT_DATA_S2M_MOTO *pstAnoDt_DataS2MMoto = (ST_ANO_DT_DATA_S2M_MOTO *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_MOTO)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_MotoPWM(pu8DataBuff,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO1,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO2,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO3,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO4,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO5,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO6,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO7,
                                                pstAnoDt_DataS2MMoto->u16PWM_MOTO8);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_PID1:
        case E_ANO_DT_FUNC_S2M_TYPE_PID2:
        case E_ANO_DT_FUNC_S2M_TYPE_PID3:
        case E_ANO_DT_FUNC_S2M_TYPE_PID4:
        case E_ANO_DT_FUNC_S2M_TYPE_PID5:
        case E_ANO_DT_FUNC_S2M_TYPE_PID6:
        {
            ST_ANO_DT_DATA_S2M_PID *pstAnoDt_DataS2MPid = (ST_ANO_DT_DATA_S2M_PID *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_PID)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_PID(pu8DataBuff, eFuncType,
                                                pstAnoDt_DataS2MPid->s16PID1_P,
                                                pstAnoDt_DataS2MPid->s16PID1_I,
                                                pstAnoDt_DataS2MPid->s16PID1_D,
                                                pstAnoDt_DataS2MPid->s16PID2_P,
                                                pstAnoDt_DataS2MPid->s16PID2_I,
                                                pstAnoDt_DataS2MPid->s16PID2_D,
                                                pstAnoDt_DataS2MPid->s16PID3_P,
                                                pstAnoDt_DataS2MPid->s16PID3_I,
                                                pstAnoDt_DataS2MPid->s16PID3_D);
            break;
        }
#if (ANO_DT_SUPPORT_FOC == 1)
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_STATUS:
        {
            ST_ANO_DT_DATA_S2M_FOC_STATUS *pstAnoDt_DataS2MFocStatus = (ST_ANO_DT_DATA_S2M_FOC_STATUS *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_FOC_STATUS)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_FOC_Status(pu8DataBuff,
                                                pstAnoDt_DataS2MFocStatus->s16Angle_M,
                                                pstAnoDt_DataS2MFocStatus->s16Omega_M,
                                                pstAnoDt_DataS2MFocStatus->s16Alpha_M,
                                                pstAnoDt_DataS2MFocStatus->s16Angle_Sum,
                                                pstAnoDt_DataS2MFocStatus->u8lock_mode,
                                                pstAnoDt_DataS2MFocStatus->u8IsActive,
                                                pstAnoDt_DataS2MFocStatus->u8status);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO1:
        {
            ST_ANO_DT_DATA_S2M_FOC_INT_INFO1 *pstAnoDt_DataS2MFocIntInfo1 = (ST_ANO_DT_DATA_S2M_FOC_INT_INFO1 *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_FOC_INT_INFO1)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_FOC_IntInfo1(pu8DataBuff,
                                                pstAnoDt_DataS2MFocIntInfo1->s16U_d,
                                                pstAnoDt_DataS2MFocIntInfo1->s16U_q,
                                                pstAnoDt_DataS2MFocIntInfo1->s16theta,
                                                pstAnoDt_DataS2MFocIntInfo1->s16U_alpha,
                                                pstAnoDt_DataS2MFocIntInfo1->s16U_beta,
                                                pstAnoDt_DataS2MFocIntInfo1->u8N,
                                                pstAnoDt_DataS2MFocIntInfo1->u8Sector,
                                                pstAnoDt_DataS2MFocIntInfo1->u16T_a,
                                                pstAnoDt_DataS2MFocIntInfo1->u16T_b,
                                                pstAnoDt_DataS2MFocIntInfo1->u16T_c);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INT_INFO2:
        {
            ST_ANO_DT_DATA_S2M_FOC_INT_INFO2 *pstAnoDt_DataS2MFocIntInfo2 = (ST_ANO_DT_DATA_S2M_FOC_INT_INFO2 *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_FOC_INT_INFO2)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_FOC_IntInfo2(pu8DataBuff,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_a,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_b,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_c,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_d,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_q,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_alpha,
                                                pstAnoDt_DataS2MFocIntInfo2->s16I_beta,
                                                pstAnoDt_DataS2MFocIntInfo2->s16Sin_theta,
                                                pstAnoDt_DataS2MFocIntInfo2->s16Cos_theta,
                                                pstAnoDt_DataS2MFocIntInfo2->u16PWMA,
                                                pstAnoDt_DataS2MFocIntInfo2->u16PWMB,
                                                pstAnoDt_DataS2MFocIntInfo2->u16PWMC);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_FOC_INPUT_CTRL:
        {
            ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL *pstAnoDt_DataS2MFocInputCtrl = (ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_FOC_InputCtrl(pu8DataBuff,
                                                pstAnoDt_DataS2MFocInputCtrl->s16Angle,
                                                pstAnoDt_DataS2MFocInputCtrl->s16Omega,
                                                pstAnoDt_DataS2MFocInputCtrl->s16I_d,
                                                pstAnoDt_DataS2MFocInputCtrl->s16I_q,
                                                pstAnoDt_DataS2MFocInputCtrl->u8lock_mode);
            break;
        }
#endif
        case E_ANO_DT_FUNC_S2M_TYPE_MSG:
        {
            ST_ANO_DT_DATA_S2M_MSG *pstAnoDt_DataS2MMsg = (ST_ANO_DT_DATA_S2M_MSG *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_MSG)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Msg(pu8DataBuff,
                                                pstAnoDt_DataS2MMsg->u8MSG_ID,
                                                pstAnoDt_DataS2MMsg->u8MSG_DATA);
            break;
        }
        case E_ANO_DT_FUNC_S2M_TYPE_CHECK:
        {
            ST_ANO_DT_DATA_S2M_CHECK *pstAnoDt_DataS2MCheck = (ST_ANO_DT_DATA_S2M_CHECK *)pParma;
            if(u8size != sizeof(ST_ANO_DT_DATA_S2M_CHECK)) return FALSE;
            u8Ret = _ANO_DT_Slaver_Send_Check(pu8DataBuff,
                                                pstAnoDt_DataS2MCheck->u8FREAM_HEAD,
                                                pstAnoDt_DataS2MCheck->u8CHECK_SUM);
            break;
        }
        default:
            break;
    }

    return u8Ret;
}

static void _ANO_DT_Slaver_Send_Timer_Callback(TimerHandle_t hTimer)
{
    static ST_ANO_DT_SEND_DATA *pstAnoDtSendData = NULL;
    ST_ANO_DT_FRAME_LIST_ITEM *pstListItem = NULL;

    if(pstAnoDtSendData == NULL)
    {
        pstAnoDtSendData = pvPortMalloc(sizeof(ST_ANO_DT_SEND_DATA));
        if(pstAnoDtSendData == NULL) return;
    }

    if(stAnoDtFrameList.pstIndex != NULL)
    {
        if((stAnoDtFrameList.pstIndex->bAutoRun == TRUE) && (stAnoDtFrameList.pstIndex->bActive == TRUE))
        {
            pstAnoDtSendData->u8FuncType = stAnoDtFrameList.pstIndex->eFuncType;
            pstAnoDtSendData->u8DataSize = stAnoDtFrameList.pstIndex->u8ParmaSize;
            if(stAnoDtFrameList.pstIndex->pParma != NULL)
            {
                memcpy(pstAnoDtSendData->pu8DataBuff, stAnoDtFrameList.pstIndex->pParma, stAnoDtFrameList.pstIndex->u8ParmaSize);
            }

            if(uxQueueMessagesWaiting(hAnoDtSend_Queue) == 0)
            {
                FR_OS_QueueSend(hAnoDtSend_Queue, pstAnoDtSendData, 0);
            }
            else
            {
                //wait for next time call in and to send data
                return;
            }
        }

        while(TRUE)
        {
            pstListItem = stAnoDtFrameList.pstIndex->pstNext;
            if(pstListItem == NULL)//means to the end
            {
                stAnoDtFrameList.pstIndex = stAnoDtFrameList.pstHead;
                if(stAnoDtFrameList.pstIndex->bActive && stAnoDtFrameList.pstIndex->bAutoRun)
                {
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                stAnoDtFrameList.pstIndex = pstListItem;
                if(pstListItem->bAutoRun && pstListItem->bActive)
                {
                    break;
                }
            }
        }
    }    
}

u8 ANO_DT_Slaver_Set_Send_Msg(u8 u8MsgID, u8 u8MsgData)
{
    ST_ANO_DT_DATA_S2M_MSG stAnoDtS2MMsg;
    ST_ANO_DT_SEND_DATA stAnoDtSendData;
    memset(&stAnoDtS2MMsg, 0, sizeof(ST_ANO_DT_DATA_S2M_MSG));
    memset(&stAnoDtSendData, 0, sizeof(ST_ANO_DT_SEND_DATA));

    stAnoDtSendData.u8FuncType = E_ANO_DT_FUNC_S2M_TYPE_MSG;
    stAnoDtSendData.u8DataSize = sizeof(ST_ANO_DT_DATA_S2M_MSG);

    stAnoDtS2MMsg.u8MSG_ID = u8MsgID;
    stAnoDtS2MMsg.u8MSG_DATA = u8MsgData;

    memcpy(stAnoDtSendData.pu8DataBuff, &stAnoDtS2MMsg, sizeof(ST_ANO_DT_DATA_S2M_MSG));

    return FR_OS_QueueSend(hAnoDtSend_Queue, &stAnoDtSendData, 0);
}

u8 ANO_DT_Slaver_Set_Send_Check(u8 u8head, u8 u8check_Sum)
{
    ST_ANO_DT_DATA_S2M_CHECK stAnoDtS2MCheck;
    ST_ANO_DT_SEND_DATA stAnoDtSendData;
    memset(&stAnoDtS2MCheck, 0, sizeof(ST_ANO_DT_DATA_S2M_MSG));
    memset(&stAnoDtSendData, 0, sizeof(ST_ANO_DT_SEND_DATA));

    stAnoDtSendData.u8FuncType = E_ANO_DT_FUNC_S2M_TYPE_MSG;
    stAnoDtSendData.u8DataSize = sizeof(ST_ANO_DT_DATA_S2M_MSG);

    stAnoDtS2MCheck.u8FREAM_HEAD = u8head;
    stAnoDtS2MCheck.u8CHECK_SUM = u8check_Sum;

    memcpy(stAnoDtSendData.pu8DataBuff, &stAnoDtS2MCheck, sizeof(ST_ANO_DT_DATA_S2M_CHECK));

    return FR_OS_QueueSend(hAnoDtSend_Queue, &stAnoDtSendData, 0);
}

u8 ANO_DT_Slaver_Send_Enable(u8 bEn)
{
    static u8 _bPreEnable = TRUE;
    eTaskState eTaskSta;
    if(_bPreEnable != bEn)
    {
        //vTaskGetInfo(hAnoDtSlaverSendTask, pxTaskStatus, pdFALSE, eInvalid);
        taskENTER_CRITICAL();
        eTaskSta = eTaskGetState(hAnoDtSendTask);
        if(bEn)
        {
            if(eTaskSta != eRunning)
            {
                vTaskResume(hAnoDtSendTask);
            }
            if(xTimerIsTimerActive(hAnoDt_SendData_Timer) == FALSE)
            {
                xTimerStart(hAnoDt_SendData_Timer, 0);
            }
        }
        else
        {
            if(eTaskSta != eSuspended)
            {
                vTaskSuspend(hAnoDtSendTask);
            }
            if(xTimerIsTimerActive(hAnoDt_SendData_Timer) != FALSE)
            {
                xTimerStop(hAnoDt_SendData_Timer, 0);
            }
        }
        _bPreEnable = bEn;
        taskEXIT_CRITICAL();
    }

    return TRUE;
}

void ANO_DT_Slaver_Send_Task(void * pvParameters)
{
    u8 *pu8SendBuff = NULL;
    //u8 u8RunInfoListNum = sizeof(astAnoDtS2MFrameSetting)/sizeof(ST_ANO_DT_FRAME_SETTING);
    BaseType_t xReturn = pdFALSE;
    ST_ANO_DT_SEND_DATA *pstAnoDtSendData = NULL;

    pu8SendBuff = pvPortMalloc(ANO_DT_BUFF_MAX);
    pstAnoDtSendData = pvPortMalloc(sizeof(ST_ANO_DT_SEND_DATA));
    if(pu8SendBuff == NULL) while(1);
    if(pstAnoDtSendData == NULL) while(1);

    memset(pu8SendBuff, 0, sizeof(ANO_DT_BUFF_MAX));
    memset(pstAnoDtSendData, 0, sizeof(ST_ANO_DT_SEND_DATA));

    while(pdTRUE)
    {
#if (ANO_DT_SLAVER_MODE_PASSIVE)
        xReturn = FR_OS_SemBinaryTake(hAnoDtSendSemBinary, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
#endif
            xReturn = FR_OS_QueueReceive(hAnoDtSend_Queue, pstAnoDtSendData, portMAX_DELAY);
            if(xReturn == pdTRUE)
            {
                _ANO_DT_Slaver_Send_Handler(pu8SendBuff,pstAnoDtSendData->u8FuncType,
                                                    pstAnoDtSendData->pu8DataBuff,
                                                    pstAnoDtSendData->u8DataSize);
            }
#if (ANO_DT_SLAVER_MODE_PASSIVE)
        }
#endif
    }
}


static u8 _ANO_DT_Slaver_Receive_Command(u8 *pu8Buff, u8 u8data_size)
{
    u8 u8Cmd = 0;

    u8Cmd = pu8Buff[0];

    switch(u8Cmd)
    {
        case E_ANO_DT_M2S_TYPE_CMD_ACCEL:
        {
            //do something here
            ANO_DT_Slaver_Set_Send_Msg(E_ANO_DT_S2M_TYPE_MSG_ID_ACCEL, E_ANO_DT_S2M_TYPE_MSG_DATA_CALI_PASS);
            break;
        }
        default:
        {
            break;
        }
    }

    return TRUE;
}

static u8 _ANO_DT_Slaver_Receive_Ack(u8 *pu8Buff, u8 u8data_size)
{
    u8 u8Ack = 0;

    u8Ack = pu8Buff[0];
    switch(u8Ack)
    {
        case E_ANO_DT_M2S_TYPE_ACK_READ_PID:
        {
            //do something here
            break;
        }
        case E_ANO_DT_M2S_TYPE_ACK_READ_FLY_MODE:
        {
            //do something here
            break;
        }
        default:
        {
            break;
        }
    }

    return TRUE;
}

static u8 _ANO_DT_Slaver_Receive_Rcdata(u8 *pu8Buff, u8 u8data_size, ST_ANO_DT_DATA_M2S_RCDATA *pstDataOut)
{
    pstDataOut->s16THR  = (s16)pu8Buff[0]<<8 | pu8Buff[1];
    pstDataOut->s16YAW  = (s16)pu8Buff[2]<<8 | pu8Buff[3];
    pstDataOut->s16ROL  = (s16)pu8Buff[4]<<8 | pu8Buff[5];
    pstDataOut->s16PIT  = (s16)pu8Buff[6]<<8 | pu8Buff[7];
    pstDataOut->s16AUX1 = (s16)pu8Buff[8]<<8 | pu8Buff[9];
    pstDataOut->s16AUX2 = (s16)pu8Buff[10]<<8 | pu8Buff[11];
    pstDataOut->s16AUX3 = (s16)pu8Buff[12]<<8 | pu8Buff[13];
    pstDataOut->s16AUX4 = (s16)pu8Buff[14]<<8 | pu8Buff[15];
    pstDataOut->s16AUX5 = (s16)pu8Buff[16]<<8 | pu8Buff[17];
    pstDataOut->s16AUX6 = (s16)pu8Buff[18]<<8 | pu8Buff[19];
    return TRUE;
}

#if (ANO_DT_SUPPORT_FOC)
static u8 _ANO_DT_Slaver_Receive_FOC_Input(u8 *pu8DataBuff, ST_ANO_DT_DATA_S2M_FOC_INPUT_CTRL *pstAnoDtDataS2MFOCInput)
{
    pstAnoDtDataS2MFOCInput->s16Angle      = (s16)pu8DataBuff[0]<<8 | pu8DataBuff[1];
    pstAnoDtDataS2MFOCInput->s16Omega      = (s16)pu8DataBuff[2]<<8 | pu8DataBuff[3];
    pstAnoDtDataS2MFOCInput->s16I_d        = (s16)pu8DataBuff[4]<<8 | pu8DataBuff[5];
    pstAnoDtDataS2MFOCInput->s16I_q        = (s16)pu8DataBuff[6]<<8 | pu8DataBuff[7];
    pstAnoDtDataS2MFOCInput->u8lock_mode   = (u8)pu8DataBuff[8];
    return TRUE;
}

#endif
static u8 _ANO_DT_Slaver_Receive_Handler(u8 *pu8Buff, u8 u8len)
{
    u8 u8Ret = FALSE;
    u8 u8temp = 0;
    u8 u8CaculateSum = 0;
    u8 u8GetSum = 0;
    u8 *pu8DataBuff = NULL;
    u8 u8DataLen = 0;
    u16 u16Head = 0;
    EN_ANO_DT_FUNC_M2S_TYPE eFuncType = E_ANO_DT_FUNC_M2S_TYPE_NULL;

    u16Head = ((u16)pu8Buff[0]<<8)|pu8Buff[1];
    stAnoDtRecFrameInfo.u16Head     = u16Head;
    if(u16Head != ANO_DT_FRAME_HEAD_M2S)
    {
        stAnoDtRecFrameInfo.u8ErrCnt++;
        return FALSE;
    }

    for(u8temp=0; u8temp<(u8len-1); u8temp++)
    {
        u8CaculateSum += pu8Buff[u8temp];
    }
    u8GetSum = pu8Buff[u8len-1];
    stAnoDtRecFrameInfo.u8sum       = u8GetSum;
    if(u8CaculateSum != u8GetSum) 
    {
        //check sum failed
        stAnoDtRecFrameInfo.u8ErrCnt |= 1<<7;
        return FALSE;
    }

    eFuncType = pu8Buff[2];
    u8DataLen = pu8Buff[3];
    pu8DataBuff = &pu8Buff[4];
    
    stAnoDtRecFrameInfo.u8FuncType  = (EN_ANO_DT_FUNC_M2S_TYPE)eFuncType;
    stAnoDtRecFrameInfo.u8datalen   = u8DataLen;
    stAnoDtRecFrameInfo.pu8Buff     = pu8DataBuff; //get data pointer

    switch(eFuncType)
    {
        case E_ANO_DT_FUNC_M2S_TYPE_COMMAND:
        {
            u8Ret = _ANO_DT_Slaver_Receive_Command(pu8DataBuff, u8DataLen);
            break;
        }
        case E_ANO_DT_FUNC_M2S_TYPE_ACK:
        {
            u8Ret = _ANO_DT_Slaver_Receive_Ack(pu8DataBuff, u8DataLen);
            break;
        }
        case E_ANO_DT_FUNC_M2S_TYPE_RCDATA:
        {
            ST_ANO_DT_DATA_M2S_RCDATA *pstAnoDt_RCDATA;
            pstAnoDt_RCDATA = pvPortMalloc(sizeof(ST_ANO_DT_DATA_M2S_RCDATA));
            if(pstAnoDt_RCDATA != NULL)
            {
                memset(pstAnoDt_RCDATA, 0, sizeof(ST_ANO_DT_DATA_M2S_RCDATA));
                u8Ret = _ANO_DT_Slaver_Receive_Rcdata(pu8DataBuff, u8DataLen, pstAnoDt_RCDATA);
                //here to use data
                vPortFree(pstAnoDt_RCDATA);
                pstAnoDt_RCDATA = NULL;
            }
            break;
        }
#if (ANO_DT_SUPPORT_FOC)
        case E_ANO_DT_FUNC_M2S_TYPE_FOC_INPUT_CTRL:
        {
            ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL *pstAnoDt_FOC_Input;
            pstAnoDt_FOC_Input = pvPortMalloc(sizeof(ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL));
            if(pstAnoDt_FOC_Input != NULL)
            {
                memset(pstAnoDt_FOC_Input, 0, sizeof(ST_ANO_DT_DATA_M2S_FOC_INPUT_CTRL));
                u8Ret = _ANO_DT_Slaver_Receive_FOC_Input(pu8DataBuff, pstAnoDt_FOC_Input);
                //here to use data
                ST_FOC_Ctrl_Params_t stFocCtrlParams;
                memset(&stFocCtrlParams, 0x0, sizeof(ST_FOC_Ctrl_Params_t));
                stFocCtrlParams.f32InputAngle = (float)pstAnoDt_FOC_Input->s16Angle/100.0f;
                stFocCtrlParams.f32InputOmega = (float)pstAnoDt_FOC_Input->s16Omega/10.0f;
                stFocCtrlParams.f32InputId    = (float)pstAnoDt_FOC_Input->s16I_d/100.0f;
                stFocCtrlParams.f32InputIq    = (float)pstAnoDt_FOC_Input->s16I_q/100.0f;
                stFocCtrlParams.u8lock_mode   = pstAnoDt_FOC_Input->u8lock_mode;
                FOC_Control_Set_InputCtrl(&stFocCtrlParams);
                //end
                if(pstAnoDt_FOC_Input != NULL)
                {
                    vPortFree(pstAnoDt_FOC_Input);
                    pstAnoDt_FOC_Input = NULL;
                }
            }
            break;
        }
#endif
        default:
        {
            u8Ret = FALSE;
            break;
        }
    }

    return u8Ret;
    
}

void ANO_DT_Slaver_Receive_Task(void * pvParameters)
{
    u8 *pu8RecBuff = NULL;
    BaseType_t xReturn = pdFALSE;
    ST_ANO_DT_RECEIVE_DATA stAnoDt_ReceiveData;

    memset(&stAnoDt_ReceiveData, 0, sizeof(ST_ANO_DT_RECEIVE_DATA));
    pu8RecBuff = pvPortMalloc(ANO_DT_BUFF_MAX);
    if(pu8RecBuff == NULL) while(1);
 
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hAnoDtReceive_Queue, &stAnoDt_ReceiveData, portMAX_DELAY);
        if(xReturn == pdTRUE)
        {
            memcpy(pu8RecBuff, stAnoDt_ReceiveData.pu8Buff, stAnoDt_ReceiveData.u8len);
            if(_ANO_DT_Slaver_Receive_Handler(pu8RecBuff, stAnoDt_ReceiveData.u8len) != TRUE)
            {
                bAnoDtRxErrCode = TRUE;
            }

#if (ANO_DT_SLAVER_MODE_PASSIVE)
            FR_OS_SemBinaryGive(hAnoDtSendSemBinary);
            xTimerStart(hAnoDt_SendData_Timer, 0);
#endif
        }
    }
}



#endif

