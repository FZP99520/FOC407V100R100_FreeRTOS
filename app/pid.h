#ifndef _PID_H_
#define _PID_H_
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"


typedef struct
{
    float f32Kp;
    float f32Ki;
    float f32Kd;
}ST_PidParams_t;

typedef struct
{
    ST_PidParams_t stPidParams;
    float f32Error;
    float f32PreError;
    float f32Integ;
    float f32Deriv;
    float f32Output;
    float f32IntegLimit;
    float f32OutputLimit;
}ST_PidInfo_t;


FR_RESULT PID_Params_DeInit(void);
FR_RESULT PID_Params_Init_From_Flash(u8 u8PageIndex);
FR_RESULT PID_Params_Save_To_Flash(u8 u8PageIndex);

void PID_Position_Cal(ST_PidInfo_t *pstPID,float f32target,float f32measure,float f32IntegMax,float f32OutputMax);
void PID_Reset(ST_PidInfo_t *pstPID);
void PID_Reset_All(void);
void PID_Reset_Integ(ST_PidInfo_t *pstPID);
void PID_Cal_Update(void);
#endif

