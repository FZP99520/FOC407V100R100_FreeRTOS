#include "foc.h"
#include "pid.h"
#include "log.h"
#include "as5048a.h"
#include "drv8323.h"
#include "DataScope_DP.h"
#include "filter.h"
#include "ANO_DT.h"


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define HIGH TRUE
#define LOW  FALSE

#define CTRL_PERIOD_T    0.001f
#define ELECT_OMEGA_MAX  60/(CTRL_PERIOD_T)*0.1f

#define FOC_DRIVER_GPIO_ENABLE   GPIO_Pin_8
#define FOC_DRIVER_GPIO_RESET    GPIO_Pin_9
#define FOC_DRIVER_GPIO_SLEEP    GPIO_Pin_10
#define FOC_DRIVER_GPIO_FAULT    GPIO_Pin_11
#define FOC_DRIVER_GPIO_PORT     GPIOC


#define FOC_DriverGpioSet_Enable(bEnable)   DRV8323_Set_GPIO_ENABLE(bEnable)
#define FOC_DriverGpioSet_Cal(bEnable)      DRV8323_Set_GPIO_CAL(bEnable)
#define FOC_DriverGpioGet_Fault(pu8Fault)   DRV8323_Get_GPIO_FAULT(pu8Fault)
#define FOC_DriverSetSvpwm(u16PhaseA, u16PhaseB, u16PhaseC) TIM1_SetPWM_Duty(u16PhaseA, u16PhaseB, u16PhaseC)
#define TO_180_DEG(x) \
    do{\
        if(x>=180.0f) x-=360.0f;\
        else if(x<-180.0f) x+=360.0f;\
        else;\
    }while(0);

void iPark(float *pf32U_alpha,float *pf32U_beta,float f32U_d,float f32U_q,float f32Sine,float f32Cosine);
void Park(float *pf32I_d,float *pf32I_q,float f32I_alpha,float f32I_beta,float f32Sine,float f32Cosine);
void Clarke(float *pf32I_alpha,float *pf32I_beta,float f32I_a,float f32I_b, float f32I_c);

float _af32AcquireCurrent[E_FOC_ACQUIRE_CURRENT_INDEX_MAX] = {0};
static u8 _bIsFocActive = FALSE;
EN_FOC_STATUS_e _eFocStatus = E_FOC_STATUS_NULL;
EN_FOC_LOCK_MODE_e _eFocLockMode = E_FOC_LOCK_ANGLE;

ST_FOC_Ctrl_Params_t stFocInputCtrl;

float f32InputAngle = 0;
float f32InputOmega = 60;
float f32InputId = 0.0f;
float f32InputIq = 0.5f;
float f32MeasureAngle = 0;
float f32MeasureOmega = 0;
float f32MeasureAlpha = 0;
float f32MeasureId = 0;
float f32MeasureIq = 0;
float f32TargetOmega = 0;
float f32TargetAngle = 0;
float f32TargetId = 0.0f;
float f32TargetIq = 0.5f;
float f32OmegaLimit = 0;
float f32Ud_Comp = 0;
float f32Uq_Comp = 0;

TaskHandle_t hFocControlStart_Task = NULL;
TaskHandle_t hFocControlCali_Task = NULL;
TaskHandle_t hFocControlFaultDet_Task = NULL;
TaskHandle_t hFocControlDataScope_Task = NULL;
TaskHandle_t hFocControlRunning_Task = NULL;
TaskHandle_t hFocControlStatus_Task = NULL;
TaskHandle_t hFocControlFilter_Task = NULL;

EventGroupHandle_t hFocEvent;
ST_FOC_Angle_Info_t stFocAngleInfo;


ST_FOC_t stFoc = 
{
  .fiPark  = iPark,
  .fPark   = Park,
  .fClarke = Clarke
};

//PID Control Parameters start-------
ST_PidInfo_t stPidParams_AnglePos = 
{
    .stPidParams.f32Kp = 0.20f,
    .stPidParams.f32Ki = 80.0f,
    .stPidParams.f32Kd = 0,
    .f32IntegLimit = 20.0f,
    .f32OutputLimit = 360 //here to limit omega
};

ST_PidInfo_t stPidParams_AngleOmega = 
{
    .stPidParams.f32Kp = 0.00082f,
    .stPidParams.f32Ki = 0.0100f,
    .stPidParams.f32Kd = 0.0,
    .f32IntegLimit = 20.0f,
    .f32OutputLimit = FOC_IQ_ID_MAX
};

ST_PidInfo_t stPidParams_Ud = 
{
    .stPidParams.f32Kp = 180.0f,
    .stPidParams.f32Ki = 60.0f,
    .stPidParams.f32Kd = 0,
    .f32IntegLimit = 0.6f,
    .f32OutputLimit = FOC_UD_UQ_MAX
};
    
ST_PidInfo_t stPidParams_Uq = 
{
    .stPidParams.f32Kp = 180.0f,//120
    .stPidParams.f32Ki = 60.0f,//20
    .stPidParams.f32Kd = 0,
    .f32IntegLimit = 0.60f,
    .f32OutputLimit = FOC_UD_UQ_MAX
};
//PID Control Parameters end-------

void FOC_Self_Test(void)
{
    static float f32theta = 0;
    static float f32SinVal = 0;
    static float f32CosVal = 0;
    static float f32FreqHz = 0.5f;
    static float f32Tcnt = 0;

    u16 u16PWM_A = 0;
    u16 u16PWM_B = 0;
    u16 u16PWM_C = 0;

    FOC_DEBUG_GPIO_OUT(HIGH);

    stFoc.f32U_d = 3.0f;
    stFoc.f32U_q = 0.0f;

    f32Tcnt +=HANDLE_DT;
    stFocAngleInfo.f32AngleNow_M = 360.0f*f32FreqHz*f32Tcnt;
    stFocAngleInfo.f32Omega_M = (stFocAngleInfo.f32AngleNow_M - stFocAngleInfo.f32AnglePre_M)/HANDLE_DT;
    stFocAngleInfo.f32AnglePre_M = stFocAngleInfo.f32AngleNow_M;
    stFocAngleInfo.f32AngleNow_E = stFocAngleInfo.f32AngleNow_M*MOTOR_POLE_PAIR_NUM;
    stFocAngleInfo.f32Omega_E = stFocAngleInfo.f32Omega_M*MOTOR_POLE_PAIR_NUM;
    
    stFoc.f32theta += f32InputOmega*HANDLE_DT;
    if(stFoc.f32theta >= 360.0f)
    {
        stFoc.f32theta -=360.0f;
    }
    else if(stFoc.f32theta <= -360.0f)
    {
        stFoc.f32theta +=360.0f;
    }
    else;

    arm_sin_cos_f32(stFoc.f32theta, &stFoc.f32Sine, &stFoc.f32Cosine);
    stFoc.fiPark(&stFoc.f32U_alpha,&stFoc.f32U_beta,stFoc.f32U_d,stFoc.f32U_q,stFoc.f32Sine,stFoc.f32Cosine);
    FOC_SVPMW_Cal(&stFoc);

    u16PWM_A = stFoc.f32T_a;
    u16PWM_B = stFoc.f32T_b;
    u16PWM_C = stFoc.f32T_c;
    TIM1_SetPWM_Duty(u16PWM_A,u16PWM_B,u16PWM_C);
    FOC_DEBUG_GPIO_OUT(LOW);
}

void FOC_Set_Zero_Position(void)
{
    float f32MeaAngle = 0;
    float f32MeaOmega = 0;
    float f32MeaAlpha = 0;
    stFoc.f32U_d = 2.0f;
    stFoc.f32U_q = 0.0f;
    stFoc.f32theta = 0;
    arm_sin_cos_f32(stFoc.f32theta, &stFoc.f32Sine, &stFoc.f32Cosine);
    stFoc.fiPark(&stFoc.f32U_alpha,&stFoc.f32U_beta,stFoc.f32U_d,stFoc.f32U_q,stFoc.f32Sine,stFoc.f32Cosine);
    FOC_SVPMW_Cal(&stFoc);

    stFoc.u16PWM_PhaseA = stFoc.f32T_a;
    stFoc.u16PWM_PhaseB = stFoc.f32T_b;
    stFoc.u16PWM_PhaseC = stFoc.f32T_c;
    FOC_DriverSetSvpwm(stFoc.u16PWM_PhaseA,stFoc.u16PWM_PhaseB,stFoc.u16PWM_PhaseC);
    SysTick_delay_ms(100);
}

void FOC_PWM_Update_IRQ_Handle(void)
{

    ST_DRV_CUR_SENSE_INFO stDrvCurSenseInfo;
    if(!_bIsFocActive)
    {
        
        return;
    }
#if 0//for debug
    FOC_Self_Test();
#else
    u8 u8Ret = FALSE;
    float f32OmegaE;
    float f32MeaAlpha = 0;
    float f32EleAngle = 0;
    FOC_DEBUG_GPIO_OUT(HIGH);
//Step1:Get measurement data from sensor
    u8Ret = AS5048A_GetAngleInfo(&f32MeasureAngle, &f32MeasureOmega, &f32MeasureAlpha);

    if(!u8Ret)
    {
        f32MeasureAngle = stFocAngleInfo.f32AnglePre_M + stFocAngleInfo.f32Omega_M*FOC_HANDLE_DT;
    }
    stFocAngleInfo.f32AngleNow_M = f32MeasureAngle;
    stFocAngleInfo.f32Omega_M = f32MeasureOmega;
    stFocAngleInfo.f32AngleDelta = (stFocAngleInfo.f32AngleNow_M - stFocAngleInfo.f32AnglePre_M);
    DEGREE_TO_180(stFocAngleInfo.f32AngleDelta);
    stFocAngleInfo.f32AnglePre_M = stFocAngleInfo.f32AngleNow_M;
    stFocAngleInfo.f32AngleNow_E = stFocAngleInfo.f32AngleNow_M*MOTOR_POLE_PAIR_NUM;
    stFocAngleInfo.f32Omega_E = stFocAngleInfo.f32Omega_M*MOTOR_POLE_PAIR_NUM;
    f32EleAngle = stFocAngleInfo.f32AngleNow_E;
    DRV8323_GetCurSenseInfo(&stFoc.f32I_a, &stFoc.f32I_b, &stFoc.f32I_c);

    switch(_eFocStatus)
    {
        case E_FOC_STATUS_TO_ZERO:
        {
            DEGREE_TO_180(f32MeasureAngle);
            f32TargetAngle = 0;
            f32OmegaLimit = 90;
            stPidParams_AnglePos.stPidParams.f32Kp = 8.0f;
            stPidParams_AnglePos.stPidParams.f32Ki = 8.0f;
            stPidParams_AnglePos.f32IntegLimit = 0.8f;
            break;
        }
        case E_FOC_STATUS_READY:
        {
            stFocAngleInfo.f32AngleSum = stFocAngleInfo.f32AngleSum + stFocAngleInfo.f32AngleDelta;
            f32MeasureAngle = stFocAngleInfo.f32AngleSum;
            //f32TargetAngle = f32TargetAngle;
            f32OmegaLimit = 180;
            stPidParams_AnglePos.stPidParams.f32Kp = 10.0f;
            stPidParams_AnglePos.stPidParams.f32Ki = 12.0f;
            stPidParams_AnglePos.f32IntegLimit = 0.5f;
            break;
        }
        case E_FOC_STATUS_RUNNING:
        {
            stFocAngleInfo.f32AngleSum = stFocAngleInfo.f32AngleSum + stFocAngleInfo.f32AngleDelta;
            f32MeasureAngle = stFocAngleInfo.f32AngleSum;
            f32OmegaLimit = FOC_OMEGA_MAX;
            stPidParams_AnglePos.stPidParams.f32Kp = 0.20f;
            stPidParams_AnglePos.stPidParams.f32Ki = 20.0f;
            stPidParams_AnglePos.f32IntegLimit = 20.0f;
            break;
        }
        default:
            return;
            
    }

    //Consider lock mode
    switch(_eFocLockMode)
    {
        case E_FOC_LOCK_ANGLE:
        {
            float f32OmegaKa = 0.003f;
            f32TargetAngle += 0.001f*(f32InputAngle - f32TargetAngle);
            stPidParams_AnglePos.f32OutputLimit = f32OmegaLimit;
            PID_Position_Cal(&stPidParams_AnglePos, f32TargetAngle, f32MeasureAngle, \
                                stPidParams_AnglePos.f32IntegLimit, stPidParams_AnglePos.f32OutputLimit);
            
            f32TargetOmega += f32OmegaKa*(stPidParams_AnglePos.f32Output - f32TargetOmega);
            //f32TargetOmega = stPidParams_AnglePos.f32Output;
            PID_Position_Cal(&stPidParams_AngleOmega, f32TargetOmega, f32MeasureOmega, \
                                stPidParams_AngleOmega.f32IntegLimit, stPidParams_AngleOmega.f32OutputLimit);
            f32TargetIq = stPidParams_AngleOmega.f32Output;
            break;
        }
        case E_FOC_LOCK_OMEGA:
        {
            float f32OmegaKa = 0.005f;
            f32TargetOmega += f32OmegaKa*(f32InputOmega - f32TargetOmega);
            stPidParams_AngleOmega.f32OutputLimit = f32OmegaLimit;
            PID_Position_Cal(&stPidParams_AngleOmega, f32TargetOmega, f32MeasureOmega, \
                                stPidParams_AngleOmega.f32IntegLimit, stPidParams_AngleOmega.f32OutputLimit);
            //Here is not enable filter!!!!
            //f32TargetIq = f32TargetIq + f32OmegaKa*(stPidParams_AngleOmega.f32Output - f32TargetIq);
            f32TargetIq = stPidParams_AngleOmega.f32Output;
            break;
        }
        case E_FOC_LOCK_ID_IQ:
        {
            float f32k = 0.01f;
            //DEGREE_TO_180(f32MeasureAngle);
            //f32TargetId = 0;
            //f32TargetIq = -f32k*f32MeasureAngle;
            f32TargetId = f32InputId;
            f32TargetIq = f32InputIq;
            break;
        }
        case E_FOC_LOCK_TEST_MODE:
        {
            break;
        }
        default:
            return;
    }

//Step2:Caculate Ialpha and Ibeta by clarke transform
    stFoc.fClarke(&stFoc.f32I_alpha, &stFoc.f32I_beta, stFoc.f32I_a, stFoc.f32I_b, stFoc.f32I_c);

//Step3:Caculate Id and Iq by Park transform.
    stFoc.f32theta = f32EleAngle;
    arm_sin_cos_f32(stFoc.f32theta, &stFoc.f32Sine, &stFoc.f32Cosine);
    stFoc.fPark(&stFoc.f32I_d, &stFoc.f32I_q, stFoc.f32I_alpha, stFoc.f32I_beta, stFoc.f32Sine, stFoc.f32Cosine);

//Step4:Caculate PID error with Id/Iq === Id*/Iq*
    float f32Id_Iq_Ka = 0.001f;
#if 0
    //for debug start
    f32TargetId = f32InputId;
    f32TargetIq = f32InputIq;
    static float f32angle = 0;
    //f32angle += 0.3; 
    //stFoc.f32theta = f32angle;
    arm_sin_cos_f32(stFoc.f32theta, &stFoc.f32Sine, &stFoc.f32Cosine);
    stFoc.fPark(&stFoc.f32I_d, &stFoc.f32I_q, stFoc.f32I_alpha, stFoc.f32I_beta, stFoc.f32Sine, stFoc.f32Cosine);
    //for debug end
#endif
    f32MeasureId += f32Id_Iq_Ka*(stFoc.f32I_d - f32MeasureId);
    f32MeasureIq += f32Id_Iq_Ka*(stFoc.f32I_q - f32MeasureIq);
    PID_Position_Cal(&stPidParams_Ud, f32TargetId, f32MeasureId, stPidParams_Ud.f32IntegLimit, stPidParams_Ud.f32OutputLimit);
    PID_Position_Cal(&stPidParams_Uq, f32TargetIq, f32MeasureIq, stPidParams_Uq.f32IntegLimit, stPidParams_Uq.f32OutputLimit);

//Step5:Caculate Ud and Uq
#define CONFIG_FOC_CONSIDER_COMPENSATE  (TRUE)
#if CONFIG_FOC_CONSIDER_COMPENSATE
    //#define FOC_MOTOR_J  42g.cm^2
    #define FOC_MOTOR_FLUX  0.03418f
    #define FOC_MOTOR_LS    1.53e-3
    #define FOC_MOTOR_R     4.3f //7.5f
    f32OmegaE = f32MeasureOmega*MOTOR_POLE_PAIR_NUM;
    f32Ud_Comp = -0.01744444f*(f32OmegaE*FOC_MOTOR_LS*f32MeasureIq);
    f32Uq_Comp =  0.01744444f*f32OmegaE*(FOC_MOTOR_FLUX + FOC_MOTOR_LS*f32MeasureId);
    f32Ud_Comp = LIMIT(f32Ud_Comp, FOC_UD_UQ_MAX, -FOC_UD_UQ_MAX);
    f32Uq_Comp = LIMIT(f32Uq_Comp, FOC_UD_UQ_MAX, -FOC_UD_UQ_MAX);
    stFoc.f32U_d = stPidParams_Ud.f32Output + f32Ud_Comp;
    stFoc.f32U_q = stPidParams_Uq.f32Output + f32Uq_Comp;
    stFoc.f32U_d = LIMIT(stFoc.f32U_d, FOC_UD_UQ_MAX, -FOC_UD_UQ_MAX);
    stFoc.f32U_q = LIMIT(stFoc.f32U_q, FOC_UD_UQ_MAX, -FOC_UD_UQ_MAX);

    FOC_Ud_Uq_Limit(stFoc.f32U_d, stFoc.f32U_q, &stFoc.f32U_d, &stFoc.f32U_q);
#else
    stFoc.f32U_d = stPidParams_Ud.f32Output;
    stFoc.f32U_q = stPidParams_Uq.f32Output;
    FOC_Ud_Uq_Limit(stFoc.f32U_d, stFoc.f32U_q, &stFoc.f32U_d, &stFoc.f32U_q);
#endif

//Step6:Caculate Ualpha and Ubeta by iPark
    stFoc.fiPark(&stFoc.f32U_alpha,&stFoc.f32U_beta,stFoc.f32U_d,stFoc.f32U_q,stFoc.f32Sine,stFoc.f32Cosine);

//Step7:Caculate PWM DutyCycle
    FOC_SVPMW_Cal(&stFoc);

//Step8:Set PWM Output
    stFoc.u16PWM_PhaseA = stFoc.f32T_a;
    stFoc.u16PWM_PhaseB = stFoc.f32T_b;
    stFoc.u16PWM_PhaseC = stFoc.f32T_c;
    FOC_DriverSetSvpwm(stFoc.u16PWM_PhaseA,stFoc.u16PWM_PhaseB,stFoc.u16PWM_PhaseC);

    FOC_DEBUG_GPIO_OUT(LOW);
#endif
}

void FOC_Control_Start_Task(void *pvParameters)
{
    EventBits_t EventBitsOut = 0;
    EventWaitParams_t stEventWaitParams;
    u32 NotificationValue;
    memset(&stEventWaitParams,0,sizeof(EventWaitParams_t));

    stEventWaitParams.bClearOnExit = pdTRUE;
    stEventWaitParams.bWaitForAllBits = pdFALSE;
    stEventWaitParams.xTicksToWait = portMAX_DELAY;
    stEventWaitParams.EventBitWaitFor = E_FOC_EVENT_CALI_DONE | E_FOC_EVENT_FAULT_DETECTED;

    TaskCreateParams_t stTaskCreateParams;
    memset(&stTaskCreateParams,0,sizeof(TaskCreateParams_t));

//Create sub task to cali
    taskENTER_CRITICAL();
    stTaskCreateParams.TaskCode = FOC_Control_Cali_Task;
    stTaskCreateParams.pcName = "FOC_Control_Cali_Task";
    stTaskCreateParams.usStackDepth = 256;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hFocControlCali_Task, stTaskCreateParams);
    stTaskCreateParams.TaskCode = FOC_Control_FaultDet_Task;
    stTaskCreateParams.pcName = "FOC_Control_FaultDet_Task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hFocControlFaultDet_Task, stTaskCreateParams);
    stTaskCreateParams.TaskCode = FOC_Control_DataScope_Task;
    stTaskCreateParams.pcName = "FOC_Control_DataScpoe_Task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hFocControlDataScope_Task, stTaskCreateParams);
    stTaskCreateParams.TaskCode = FOC_Control_Running_Task;
    stTaskCreateParams.pcName = "FOC_Control_Running_Task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hFocControlRunning_Task, stTaskCreateParams);
    stTaskCreateParams.TaskCode = FOC_Control_Status_Task;
    stTaskCreateParams.pcName = "FOC_Control_Status_Task";
    stTaskCreateParams.usStackDepth = 128;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 4;
    FR_OS_TaskCreate(&hFocControlStatus_Task, stTaskCreateParams);
    stTaskCreateParams.TaskCode = FOC_Control_Filter_Task;
    stTaskCreateParams.pcName = "FOC_Control_Filter_Task";
    stTaskCreateParams.usStackDepth = 256;
    stTaskCreateParams.pvParameters = NULL;
    stTaskCreateParams.uxPriority = 3;
    FR_OS_TaskCreate(&hFocControlFilter_Task, stTaskCreateParams);
    taskEXIT_CRITICAL();

    FR_OS_EventSetBits(hFocEvent, E_FOC_EVENT_CALI_START);

    while(pdTRUE)
    {
        FR_OS_EventWait(hFocEvent, stEventWaitParams, &EventBitsOut);
        if(EventBitsOut&E_FOC_EVENT_CALI_DONE)
        {
            _bIsFocActive = TRUE;
            FOC_DriverGpioSet_Enable(TRUE);
            _eFocStatus = E_FOC_STATUS_TO_ZERO;
            FR_OS_EventSetBits(hFocEvent, E_FOC_EVENT_RUNNING);
            vTaskSuspend(hFocControlStart_Task);
        }
    }
}

void FOC_Control_Cali_Task(void *pvParameters)
{
    u8 u8Ret = FALSE;
    u16 u16temp = 0;
    u16 u16Times = 50;
    float f32offset[3] = {0};
    float f32Ia, f32Ib, f32Ic;
    EventBits_t EventBitsOut = 0;
    EventWaitParams_t stEventWaitParams;
    u32 NotificationValue;
  
    memset(&stEventWaitParams,0,sizeof(EventWaitParams_t));

    stEventWaitParams.bClearOnExit = pdTRUE;
    stEventWaitParams.bWaitForAllBits = pdFALSE;
    stEventWaitParams.xTicksToWait = portMAX_DELAY;
    stEventWaitParams.EventBitWaitFor = E_FOC_EVENT_CALI_START;
    
    while(pdTRUE)
    {
        FR_OS_EventWait(hFocEvent, stEventWaitParams, &EventBitsOut);
        if(EventBitsOut&E_FOC_EVENT_CALI_START)
        {
            //step 1,cali adc channel
            FOC_DriverGpioSet_Cal(TRUE);
            FR_OS_DelayMs(100);
            DEBUG_INFO("Start Current sense cali.\n");
            for(u16temp = 0; u16temp<u16Times; u16temp++)
            {
                DRV8323_GetCurSenseForCali(&f32Ia, &f32Ib, &f32Ic);
                f32offset[0] += f32Ia;
                f32offset[1] += f32Ib;
                f32offset[2] += f32Ic;
                FR_OS_DelayMs(5);
            }
            f32offset[0] /= u16Times;
            f32offset[1] /= u16Times;
            f32offset[2] /= u16Times;
            DRV8323_SetCurSenseOffset(f32offset[0], f32offset[1], f32offset[2]);
            FOC_DriverGpioSet_Cal(FALSE);
            DEBUG_INFO("Current sense cali done.\n");
            DEBUG_INFO("OffsetA = %2.3f, OffsetB = %2.3f, OffsetC = %2.3f\n", f32offset[0], f32offset[1], f32offset[2]);

            //step 2,cali angle sensor
            u8Ret = AS5048A_Init_ZeroPosition(0x258B);
            //FR_OS_DelayMs(200); // make sure sensor to be stable
            //FOC_Set_Zero_Position();

            //step 3,final set cal done event bit
            if(u8Ret)
            {
                FR_OS_EventSetBits(hFocEvent, E_FOC_EVENT_CALI_DONE);
            }
        }
    }
}

void FOC_Control_FaultDet_Task(void *pvParameters)
{
    u8 bFault = FALSE;
    u8 bFocDisable = FALSE;
    while(pdTRUE)
    {
        FOC_DriverGpioGet_Fault(&bFault);
        if(bFault)
        {
            //DEBUG_INFO("FOC Fault Detected!!!\n");
            FOC_DriverGpioSet_Enable(FALSE);
            bFocDisable = TRUE;
        }
        else
        {
            //DEBUG_INFO("FOC status is ok~\n");
            if(bFocDisable)
            {
                FOC_DriverGpioSet_Enable(TRUE);
            }
        }
        FR_OS_DelayMs(500);
    }
}

void FOC_Control_DataScope_Task(void *pvParameters)
{
    //ST_DRV_CUR_SENSE_INFO stDrvCurSenseInfo;
    //memset(&stDrvCurSenseInfo, 0, sizeof(ST_DRV_CUR_SENSE_INFO));
    u8 u8ErrFlag;
    u16 u16Angle = 0;
    while(pdTRUE)
    {
        //DataScope_SendDuty(stFoc.f32I_a, stFoc.f32I_b, stFoc.f32I_c);
        //DataScope_SendDuty(stFocAngleInfo.f32AngleNow_M, stFocAngleInfo.f32AngleNow_E, stFoc.f32I_c);
        //FR_OS_DelayMs(5);
        /*
        DataScope_SendAngleInfo(f32TargetAngle, \
                                f32MeasureAngle, \
                                f32TargetOmega, \
                                stFocAngleInfo.f32Omega_M);
                                */
        /*DataScope_SendAngleInfo(f32InputIq, \
                                f32TargetIq, \
                                f32MeasureIq, \
                                f32InputId, \
                                f32TargetId, \
                                f32MeasureId);*/
        FR_OS_DelayMs(2000);
    }
}

void FOC_Control_Running_Task(void *pvParameters)
{
    float f32Ts = 0.002f;
    EventBits_t EventBitsOut = 0;
    EventWaitParams_t stEventWaitParams;
    u32 NotificationValue;
    float f32Omega = 360;
    float f32FreqHz = 0.1f;
    float f32Tcnt = 0.01;
    float f32theta = 0;
    float f32SinVal, f32CosVal;
    u8 u8dir = 1;

    memset(&stEventWaitParams,0,sizeof(EventWaitParams_t));

    stEventWaitParams.bClearOnExit = pdTRUE;
    stEventWaitParams.bWaitForAllBits = pdFALSE;
    stEventWaitParams.xTicksToWait = portMAX_DELAY;
    stEventWaitParams.EventBitWaitFor = E_FOC_EVENT_RUNNING;
    while(pdTRUE)
    {
        //FR_OS_EventWait(hFocEvent, stEventWaitParams, &EventBitsOut);
        #if 0
        if(_eFocStatus == E_FOC_STATUS_RUNNING)
        {
            #if 0
            f32theta = 360.0f*f32FreqHz*f32Tcnt;
            arm_sin_cos_f32(f32theta, &f32SinVal, &f32CosVal);
            #endif
            f32TargetAngle += f32Ts*f32Omega;
        }
        FR_OS_DelayMs(f32Ts*1000);
        #endif
        if(_eFocStatus == E_FOC_STATUS_RUNNING)
        {
            _eFocLockMode = (EN_FOC_LOCK_MODE_e)stFocInputCtrl.u8lock_mode;
            if(_eFocLockMode == E_FOC_LOCK_ANGLE)
            {
                f32InputAngle = stFocInputCtrl.f32InputAngle;
            }
            else if(_eFocLockMode == E_FOC_LOCK_OMEGA)
            {
                f32InputOmega = stFocInputCtrl.f32InputOmega;
            }
            else
            {
            }
        }
        FR_OS_DelayMs(10);
        
    }
}

void FOC_Control_Status_Task(void *pvParameters)
{
    float f32Angle;
    float f32Omega;
    float f32Alpha;
    u16 u16Cnt = 0;
    while(pdTRUE)
    {
        AS5048A_GetAngleInfo(&f32Angle, &f32Omega, &f32Alpha);
        if(_eFocStatus == E_FOC_STATUS_TO_ZERO)
        {
            if(fabs(f32Angle)<2.0f)
            {
                u16Cnt++;
            }
            if(u16Cnt > 50) //20ms*50=1s
            {
                _eFocStatus = E_FOC_STATUS_READY;
                u16Cnt = 0;
            }
        }
        else if(_eFocStatus == E_FOC_STATUS_READY) //selt test
        {
            _eFocLockMode = E_FOC_LOCK_ANGLE;
            f32InputAngle = 360;
            FR_OS_DelayMs(3000);
            f32InputAngle = 0;
            FR_OS_DelayMs(3000);
            _eFocStatus = E_FOC_STATUS_RUNNING;
        }
        FR_OS_DelayMs(20);
    }
}

void FOC_Control_Set_InputCtrl(ST_FOC_Ctrl_Params_t *pstFocCtrlParmas)
{
    //memset(&stFocInputCtrl, 0, sizeof(ST_FOC_Ctrl_Params_t));
    memcpy(&stFocInputCtrl, pstFocCtrlParmas, sizeof(ST_FOC_Ctrl_Params_t));
}

void FOC_Control_Filter_Task(void *pvParameters)
{
    ST_BiquadFilter_t stFilterAngle;
    ST_BiquadFilter_t stFilterOmega;
    ST_BiquadFilter_t stFilterIa;
    ST_BiquadFilter_t stFilterIb;
    ST_BiquadFilter_t stFilterIc;
    float f32Angle, f32Omega, f32Alpha;
    float f32AngleOut, f32OmegaOut;
    float f32Ia, f32Ib, f32Ic;

    extern ST_DRV_CUR_SENSE_INFO _stDrvCurSenseInfo;

    memset(&stFilterAngle, 0x00, sizeof(ST_BiquadFilter_t));
    memset(&stFilterOmega, 0x00, sizeof(ST_BiquadFilter_t));
    memset(&stFilterIa, 0x00, sizeof(ST_BiquadFilter_t));
    memset(&stFilterIb, 0x00, sizeof(ST_BiquadFilter_t));
    memset(&stFilterIc, 0x00, sizeof(ST_BiquadFilter_t));

    stFilterIa.f32Gain = 0.0526537763793;
    stFilterIa.f32a1 = 1.0f;
    stFilterIa.f32a2 = 1.0f;
    stFilterIa.f32b1 = 0.8946924472413718;

    stFilterIb.f32Gain = 0.0526537763793;
    stFilterIb.f32a1 = 1.0f;
    stFilterIb.f32a2 = 1.0f;
    stFilterIb.f32b1 = 0.8946924472413718;

    stFilterIc.f32Gain = 0.0526537763793;
    stFilterIc.f32a1 = 1.0f;
    stFilterIc.f32a2 = 1.0f;
    stFilterIc.f32b1 = 0.8946924472413718;

    while(pdTRUE)
    {
        //AS5048A_GetAngleInfo(&f32Angle, &f32Omega, &f32Alpha);
        //f32AngleOut = Biquad_Filter(&stFilterAngle, f32Angle);
        //f32OmegaOut = Biquad_Filter(&stFilterOmega, f32Omega);

        f32Ia = Biquad_Filter(&stFilterIa, _stDrvCurSenseInfo.f32AdcVoltCHA);
        f32Ib = Biquad_Filter(&stFilterIb, _stDrvCurSenseInfo.f32AdcVoltCHB);
        f32Ic = Biquad_Filter(&stFilterIc, _stDrvCurSenseInfo.f32AdcVoltCHC);
        /*DataScope_SendAngleInfo(f32Ia, \
                                _stDrvCurSenseInfo.f32AdcVoltCHA, \
                                f32Ib, \
                                _stDrvCurSenseInfo.f32AdcVoltCHB, \
                                f32Ic, \
                                _stDrvCurSenseInfo.f32AdcVoltCHC);*/
        FR_OS_DelayMs(2);
        
    }
}

void FOC_DriverInit(void)
{
    u16 u16temp = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  FOC_DEBUG_GPIO_NUM;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//¸´ÓÃÍÆÍìÊä³ö
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(FOC_DEBUG_GPIO_PORT, &GPIO_InitStructure);

    Adc_Init();
    AS5048A_Init();
    DRV8323_Init();
    SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//for AS5048A
    TIM1_PWM_Init();
    DEBUG_TRACE("OK\n");
    return;
}

void iPark(float *pf32U_alpha,float *pf32U_beta,float f32U_d,float f32U_q,float f32Sine,float f32Cosine)
{
    *pf32U_alpha = f32U_d*f32Cosine - f32U_q*f32Sine;
    *pf32U_beta =  f32U_d*f32Sine   + f32U_q*f32Cosine;
}

void Park(float *pf32I_d,float *pf32I_q,float f32I_alpha,float f32I_beta,float f32Sine,float f32Cosine)
{
    *pf32I_d = f32I_alpha * f32Cosine + f32I_beta * f32Sine;
    *pf32I_q = -f32I_alpha * f32Sine + f32I_beta * f32Cosine;
}

void Clarke(float *pf32I_alpha,float *pf32I_beta,float f32I_a,float f32I_b, float f32I_c)
{
    //Ia+Ib+Ic=0
    //I_beta = Ib*COS(Pi/6)-Ic*COS(Pi/6)
    *pf32I_alpha = f32I_a*2/3-(f32I_b+f32I_c)/3.0f;
    *pf32I_beta = (f32I_b-f32I_c)*0.5773502691896258f; //here to be *2/3
}

void FOC_Ud_Uq_Limit(float f32Ud, float f32Uq, float *pf32Ud, float *pf32Uq)
{
    float f32temp0 = 0;
    float f32Norm = 0;

    f32temp0 = f32Ud*f32Ud + f32Uq*f32Uq;
    arm_sqrt_f32(f32temp0, &f32Norm);
    if(f32Norm > FOC_UD_UQ_MAX)
    {
        *pf32Ud = f32Ud*FOC_UD_UQ_MAX/f32Norm;
        *pf32Uq = f32Uq*FOC_UD_UQ_MAX/f32Norm;
    }
    else
    {
        *pf32Ud = f32Ud;
        *pf32Uq = f32Uq;
    }
}

void FOC_SVPMW_Cal(ST_FOC_t *pstFoc)
{
    //float f32Ts = 1.0f;
    float f32Ts = SVPWM_PERIOD;
    float u1 =  pstFoc->f32U_beta;
    float u2 =  0.866025f*pstFoc->f32U_alpha - pstFoc->f32U_beta/2.0f;
    float u3 = -0.866025f*pstFoc->f32U_alpha - pstFoc->f32U_beta/2.0f;

#if 1
    pstFoc->u1 = u1;
    pstFoc->u2 = u2;
    pstFoc->u3 = u3;
#endif
    float k_svpwm = 1.732f*f32Ts/FOC_Udc;
    //float k_svpwm = 1.0f;

    float taon = 0;
    float tbon = 0;
    float tcon = 0;

    u8 u8N = (u1>0.0f) + ((u2>0.0f)<<1) + ((u3>0.0f)<<2);
    stFoc.u8N = u8N;
    EN_FocSector_e eSector = 0;
    switch(u8N)
    {
        case 1:eSector = E_FOC_SECTORT_2;break;
        case 2:eSector = E_FOC_SECTORT_6;break;
        case 3:eSector = E_FOC_SECTORT_1;break;
        case 4:eSector = E_FOC_SECTORT_4;break;
        case 5:eSector = E_FOC_SECTORT_3;break;
        case 6:eSector = E_FOC_SECTORT_5;break;
        default:return;
    }

    pstFoc->u8Sector = eSector;
    /******************************************************************/
#if 1
    float T1 = 0;
    float T2 = 0;
    switch(u8N)
    {
        case 1:{T1 = -k_svpwm*u2;T2 = -k_svpwm*u3;break;}
        case 2:{T1 = -k_svpwm*u3;T2 = -k_svpwm*u1;break;}
        case 3:{T1 =  k_svpwm*u2;T2 =  k_svpwm*u1;break;}
        case 4:{T1 = -k_svpwm*u1;T2 = -k_svpwm*u2;break;}
        case 5:{T1 =  k_svpwm*u1;T2 =  k_svpwm*u3;break;}
        case 6:{T1 =  k_svpwm*u3;T2 =  k_svpwm*u2;break;}
    }
    if((T1+T2)>f32Ts)
    {
        float f32sum = T1 + T2;
        T1 = f32Ts*T1/f32sum;
        T2 = f32Ts*T2/f32sum;
    }
    taon = (f32Ts-T1-T2)*0.5f;
    tbon = taon + T1;
    tcon = tbon + T2;

    switch(u8N)
    {
        case 1:
        {
            pstFoc->f32T_a = tbon;
            pstFoc->f32T_b = taon;
            pstFoc->f32T_c = tcon;
            break;
        }
        case 2:
        {
            pstFoc->f32T_a = taon;
            pstFoc->f32T_b = tcon;
            pstFoc->f32T_c = tbon;
            break;
        }
        case 3:
        {
            pstFoc->f32T_a = taon;
            pstFoc->f32T_b = tbon;
            pstFoc->f32T_c = tcon;
            break;
        }
        case 4:
        {
            pstFoc->f32T_a = tcon;
            pstFoc->f32T_b = tbon;
            pstFoc->f32T_c = taon;
            break;
        }
        case 5:
        {
            pstFoc->f32T_a = tcon;
            pstFoc->f32T_b = taon;
            pstFoc->f32T_c = tbon;
            break;
        }
        case 6:
        {
            pstFoc->f32T_a = tbon;
            pstFoc->f32T_b = tcon;
            pstFoc->f32T_c = taon;
            break;
        }
    }
    #if 1
    pstFoc->f32T_a = SVPWM_PERIOD - pstFoc->f32T_a;
    pstFoc->f32T_b = SVPWM_PERIOD - pstFoc->f32T_b;
    pstFoc->f32T_c = SVPWM_PERIOD - pstFoc->f32T_c;
    #endif
    /*****************************************************************/
#else
    if(eSector == E_FOC_SECTORT_1)
    {
        float t4 = k_svpwm*u2;
        float t6 = k_svpwm*u1;
        float sum = t4+t6;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t4 = k*t4;
            t6 = k*t6;
        }
        //float t7 = 0.5f*(f32Ts - t4 - t6);
        taon = (f32Ts - t4 - t6)*0.5f;
        tbon = taon + t4;
        tcon = tbon + t6;
        pstFoc->f32T_a = taon;
        pstFoc->f32T_b = tbon;
        pstFoc->f32T_c = tcon;
    }
    else if(eSector == E_FOC_SECTORT_2)
    {
        float t6 = -k_svpwm*u3;
        float t2 = -k_svpwm*u2;
        float sum = t2+t6;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t6 = k*t6;
            t2 = k*t2;
        }
        float t7 = 0.5f*(f32Ts - t2 - t6);
        taon = (f32Ts - t2 - t6)*0.5f;
        tbon = taon + t2;
        tcon = tbon + t6;
        pstFoc->f32T_a = tbon;
        pstFoc->f32T_b = taon;
        pstFoc->f32T_c = tcon;
    }
    else if(eSector == E_FOC_SECTORT_3)
    {
        float t2 = k_svpwm*u1;
        float t3 = k_svpwm*u3;
        float sum = t2+t3;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t2 = k*t2;
            t3 = k*t3;
        }
        float t7 = 0.5f*(f32Ts - t2 - t3);
        taon = (f32Ts - t2 - t3)*0.5f;
        tbon = taon + t2;
        tcon = tbon + t3;
        pstFoc->f32T_a = tcon;
        pstFoc->f32T_b = taon;
        pstFoc->f32T_c = tbon;
    }
    else if(eSector == E_FOC_SECTORT_4)
    {
        float t3 = -k_svpwm*u2;
        float t1 = -k_svpwm*u1;
        float sum = t1+t3;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t3 = k*t3;
            t1 = k*t1;
        }
        float t7 = 0.5f*(f32Ts - t1 - t3);
        taon = (f32Ts - t1 - t3)*0.5f;
        tbon = taon + t1;
        tcon = tbon + t3;
        pstFoc->f32T_a = tcon;
        pstFoc->f32T_b = tbon;
        pstFoc->f32T_c = taon;
    }
    else if(eSector == E_FOC_SECTORT_5)
    {
        float t1 = k_svpwm*u3;
        float t5 = k_svpwm*u2;
        float sum = t1+t5;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t1 = k*t1;
            t5 = k*t5;
        }
        float t7 = 0.5f*(f32Ts - t1 - t5);
        taon = (f32Ts - t1 - t5)*0.5f;
        tbon = taon + t1;
        tcon = tbon + t5;
        pstFoc->f32T_a = tbon;
        pstFoc->f32T_b = tcon;
        pstFoc->f32T_c = taon;
    }
    else if(eSector == E_FOC_SECTORT_6)
    {
        float t5 = -k_svpwm*u1;
        float t4 = -k_svpwm*u3;
        float sum = t4+t5;
        if(sum > f32Ts)
        {
            float k = f32Ts/sum;
            t5 = k*t5;
            t4 = k*t4;
        }
        float t7 = 0.5f*(f32Ts - t4 - t5);
        taon = (f32Ts - t4 - t5)*0.5f;
        tbon = taon + t4;
        tcon = tbon + t5;
        pstFoc->f32T_a = taon;
        pstFoc->f32T_b = tcon;
        pstFoc->f32T_c = tbon;
    }
    else;
#endif
    pstFoc->taon = taon;
    pstFoc->tbon = tbon;
    pstFoc->tcon = tcon;
}

