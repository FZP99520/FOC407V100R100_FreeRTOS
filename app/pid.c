#include "pid.h"
#include "systick.h"

ST_PidInfo_t g_stPID_Data = {0};


//位置式PID计算函数
//PID ：指向将被计算的PID结构体
//target：目标值
//measure：测量值
//Integ:  误差积分最大值
//OutputMax：输出最大值
#define DT  0.001f

void PID_Position_Cal(ST_PidInfo_t *pstPidInfo,float f32target,float f32measure,float f32IntegMax,float f32OutputMax)
{
    float f32Kp = pstPidInfo->stPidParams.f32Kp;
    float f32Ki = pstPidInfo->stPidParams.f32Ki;
    float f32Kd = pstPidInfo->stPidParams.f32Kd;
    pstPidInfo->f32Error = f32target - f32measure; //计算误差
    pstPidInfo->f32Integ += pstPidInfo->f32Error*DT; //误差积分
    pstPidInfo->f32Integ = LIMIT(pstPidInfo->f32Integ,f32IntegMax,-f32IntegMax);   //积分限幅
    pstPidInfo->f32Deriv = pstPidInfo->f32Error - pstPidInfo->f32PreError;//计算微分
    pstPidInfo->f32Output = f32Kp*pstPidInfo->f32Error + f32Ki*pstPidInfo->f32Integ + f32Kd*pstPidInfo->f32Deriv;//输出计算
    pstPidInfo->f32Output = LIMIT(pstPidInfo->f32Output,f32OutputMax,-f32OutputMax);//输出限幅
    pstPidInfo->f32PreError = pstPidInfo->f32Error;//保存上一次误差
}
void PID_Reset(ST_PidInfo_t *pstPidInfo)
{
    pstPidInfo->f32Error=0;
    pstPidInfo->f32PreError=0;
    pstPidInfo->f32Integ=0;
    pstPidInfo->f32Output=0;
    pstPidInfo->f32Deriv=0;
}
void PID_Reset_All(void)
{
	/*PID_Reset(&Roll_angle_PID);
	PID_Reset(&Pitch_angle_PID);
	PID_Reset(&Yaw_angle_PID);
	PID_Reset(&High_dis_PID);
	PID_Reset(&Roll_rate_PID);
	PID_Reset(&Pitch_rate_PID);
	PID_Reset(&Yaw_rate_PID);
	PID_Reset(&High_v_PID);*/
}
void PID_Reset_Integ(ST_PidInfo_t *pstPidInfo)
{
	pstPidInfo->f32Integ=0;
}

#if 0

#define PID_PARAMS_KP_OFFSET    (0)
#define PID_PARAMS_KI_OFFSET    (1)
#define PID_PARAMS_KD_OFFSET    (2)
#define PID_PARAMS_NUM          (3)


FR_RESULT PID_Params_Init_From_Flash(u8 u8PageIndex)
{
    FR_RESULT eRet = FR_FAILED;
    u8 u8temp = 0;
    u16 u16length = 0;
    u32 u32ReadAddr = 0;
    u16 *pu16Buff = NULL;
    EN_PID_INDEX eIndex = E_PID_INDEX_MAX;

    memset(&g_stPID_Data,0,sizeof(ST_PID_DATA));

    pu16Buff = malloc(sizeof(s16)*PID_PARAMS_NUM*E_PID_INDEX_MAX);
    if(pu16Buff == NULL)
    {
        eRet = API_ERROR;
        goto fail_point;
    }
    if(u8PageIndex == PID_PAGE_INDEX1)
    {
        u32ReadAddr = FLASH_ADDR_PID_DATA1; //use pid1 setting
    }
    else if(u8PageIndex == PID_PAGE_INDEX2)
    {
        u32ReadAddr = FLASH_ADDR_PID_DATA2; //use pid2 setting
    }
    else if(u8PageIndex == PID_PAGE_INDEX3)
    {
        u32ReadAddr = FLASH_ADDR_PID_DATA3; //use pid3 setting
    }
    else
    {
        eRet = API_INVALID_PARAM;
        goto fail_point;
    }

    u16length = FlashRead(u32ReadAddr, pu16Buff,PID_PARAMS_NUM*E_PID_INDEX_MAX);
    if(u16length != PID_PARAMS_NUM*E_PID_INDEX_MAX)
    {
        eRet = FR_ERR;
    }
    //Check data valid or not
    for(u16length = 0; u16length < PID_PARAMS_NUM*E_PID_INDEX_MAX; u16length++)
    {
        if(pu16Buff[u16length] == 0xFFFF) 
        {
            eRet = FR_FAILED;
            goto fail_point;
        }
    }
    for(u8temp = 0; u8temp < E_PID_INDEX_MAX; u8temp++)
    {
        eIndex = (EN_PID_INDEX)u8temp;
        g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Kp = 0.001f*((s16)pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KP_OFFSET]);
        g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Ki = 0.001f*((s16)pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KI_OFFSET]);
        g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Kd = 0.001f*((s16)pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KD_OFFSET]);
    }
    g_stPID_Data.bPID_InitDone = TRUE;

    eRet = API_OK;

 fail_point:
    if(pu16Buff != NULL)
    {
        free(pu16Buff);
        pu16Buff = NULL;
    }
    return eRet;

}

FR_RESULT PID_Params_Save_To_Flash(u8 u8PageIndex)
{
    FR_RESULT eRet = FR_FAILED;
    u32 u32WriteAddr = 0;
    u8 u8temp = 0;
    u16 u16length = 0;
    u32 u32ReadAddr = 0;
    u16 *pu16Buff = NULL;
    EN_PID_INDEX eIndex = E_PID_INDEX_MAX;

    if(u8PageIndex == PID_PAGE_INDEX1)
    {
        u32WriteAddr = FLASH_ADDR_PID_DATA1; //use pid1 setting
    }
    else if(u8PageIndex == PID_PAGE_INDEX2)
    {
        u32WriteAddr = FLASH_ADDR_PID_DATA2; //use pid2 setting
    }
    else if(u8PageIndex == PID_PAGE_INDEX3)
    {
        u32WriteAddr = FLASH_ADDR_PID_DATA3; //use pid3 setting
    }
    else
    {
        eRet = API_INVALID_PARAM;
        goto fail_point;
    }
    //Erase page first
    if(FlashErase(u32WriteAddr) != API_OK)
    {
        eRet = API_FAIL;
        goto fail_point;
    }

    pu16Buff = malloc(sizeof(u16)*PID_PARAMS_NUM*E_PID_INDEX_MAX);
    if(pu16Buff == NULL)
    {
        eRet = API_ERROR;
        goto fail_point;
    }
    for(u8temp = 0; u8temp < E_PID_INDEX_MAX; u8temp++)
    {
        eIndex = (EN_PID_INDEX)u8temp;
        pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KP_OFFSET] = (u16)(g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Kp * 1000);
        pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KI_OFFSET] = (u16)(g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Ki * 1000);
        pu16Buff[PID_PARAMS_NUM*u8temp + PID_PARAMS_KD_OFFSET] = (u16)(g_stPID_Data.astPID_Info[eIndex].stPID_Params.f32Kd * 1000);
    }
    u16length = PID_PARAMS_NUM*E_PID_INDEX_MAX;
    if(FlashWrite(u32WriteAddr,pu16Buff,u16length) != u16length)
    {
        eRet = API_FAIL;
        goto fail_point;
    }
    else
    {
        eRet = API_OK;
    }

fail_point:
    if(pu16Buff != NULL)
    {
        free(pu16Buff);
        pu16Buff = NULL;
    }
    return eRet;
}
#endif

