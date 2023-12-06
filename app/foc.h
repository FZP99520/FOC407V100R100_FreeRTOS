#ifndef _FOC_H
#define _FOC_H

#include "stm32f4xx.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"
#include "pwm.h"

#define FOC_DEBUG_GPIO_NUM       GPIO_Pin_6
#define FOC_DEBUG_GPIO_PORT      GPIOE

#define FOC_DEBUG_GPIO_ENABLE    TRUE
#if FOC_DEBUG_GPIO_ENABLE
    #define FOC_DEBUG_GPIO_IN()      GPIO_ReadOutputDataBit(FOC_DEBUG_GPIO_PORT, FOC_DEBUG_GPIO_NUM)
    #define FOC_DEBUG_GPIO_OUT(x)    x?GPIO_SetBits(FOC_DEBUG_GPIO_PORT, FOC_DEBUG_GPIO_NUM):\
                                        GPIO_ResetBits(FOC_DEBUG_GPIO_PORT, FOC_DEBUG_GPIO_NUM)
    #define FOC_DEBUG_GPIO_TOGGLE()  FOC_DEBUG_GPIO_IN()?FOC_DEBUG_GPIO_OUT(FALSE):FOC_DEBUG_GPIO_OUT(TRUE)
#else
    #define FOC_DEBUG_GPIO_IN(...)
    #define FOC_DEBUG_GPIO_OUT(...)
#endif

#define DEGREE_TO_360(x) x<0.0f?(x+360.0f):x
#define DEGREE_TO_180(x)            \
            do{                     \
                if(x<-180.0f)       \
                    {x += 360.0f;}  \
                else if(x>180.0f)   \
                    {x-= 360.0f;}   \
            }while(0);

#define MOTOR_POLE_PAIR_NUM     11
#define FOC_HANDLE_DT           0.0001f
#define FOC_Udc                 12.0f
#define FOC_UD_UQ_MAX           FOC_Udc*2.0f/3.0f*1.732f/2.0f
#define SVPWM_PERIOD            TIM1_PWM_PERIOD
#define FOC_IQ_ID_MAX           1.4f   //D and Q axes max current
#define HANDLE_DT               0.001f
#define FOC_OMEGA_MAX           5400 //360*15, unit:degree/sec
#define FOC_MOTOR_PHASE_R       4.3f //4.3R           

typedef void (*IO_fIPART_TYPE)   (float*,float*,float,float,float,float);
typedef void (*IO_fPART_TYPE)         (float*,float*,float,float,float,float);
typedef void (*IO_fCLARKE_TYPE)  (float*,float*,float,float,float); 

typedef struct
{
    float f32AngleNow_M;
    float f32AnglePre_M;
    float f32AngleOffset_M;
    float f32AngleNow_E;
    float f32AngleSum;
    float f32AngleDelta;
    float f32Omega_M;
    float f32Omega_E;
    float f32Alpha;
}ST_FOC_Angle_Info_t;

typedef struct
{
    float f32U_d;
    float f32U_q;
    float f32theta;
    float f32U_alpha;
    float f32U_beta;
    u8    u8N;
    u8    u8Sector;
    float f32T_a;
    float f32T_b;
    float f32T_c;
    float f32I_a;
    float f32I_b;
    float f32I_c;
    float f32I_alpha;
    float f32I_beta;
    float f32I_d;
    float f32I_q;
    float f32Sine;
    float f32Cosine;
    IO_fIPART_TYPE  fiPark;
    IO_fPART_TYPE   fPark;
    IO_fCLARKE_TYPE fClarke;
    float taon;
    float tbon;
    float tcon;
    float u1;
    float u2;
    float u3;
    u16   u16PWM_PhaseA;
    u16   u16PWM_PhaseB;
    u16   u16PWM_PhaseC;
}ST_FOC_t;

typedef struct
{
    float f32IqMax;
}ST_FOC_Ctrl_Params_t;


typedef enum
{
    E_FOC_SECTORT_1 = 1,
    E_FOC_SECTORT_2 = 2,
    E_FOC_SECTORT_3 = 3,
    E_FOC_SECTORT_4 = 4,
    E_FOC_SECTORT_5 = 5,
    E_FOC_SECTORT_6 = 6
}EN_FocSector_e;

typedef enum
{
    E_FOC_ACQUIRE_CURRENT_INDEX_0 = 0,
    E_FOC_ACQUIRE_CURRENT_INDEX_1 = 1,
    E_FOC_ACQUIRE_CURRENT_INDEX_2 = 2,
    E_FOC_ACQUIRE_CURRENT_INDEX_MAX
}EN_FOC_ACQUIRE_CURRENT_INDEX;

typedef enum
{
    E_FOC_STATUS_NULL,
    E_FOC_STATUS_TO_ZERO,
    E_FOC_STATUS_READY,
    E_FOC_STATUS_RUNNING
}EN_FOC_STATUS_e;

typedef enum
{

    E_FOC_LOCK_ANGLE,
    E_FOC_LOCK_OMEGA,
    E_FOC_LOCK_ID_IQ,
    E_FOC_LOCK_TEST_MODE
}EN_FOC_LOCK_MODE_e;

typedef enum
{
    E_FOC_EVENT_CALI_START      = BIT(0),
    E_FOC_EVENT_CALI_DONE       = BIT(1),
    E_FOC_EVENT_FAULT_DETECTED  = BIT(2),
    E_FOC_EVENT_RUNNING         = BIT(3)
    //E_FOC_START_EVENT_CUR_CAL_DONE = BIT(0)
}EN_FOC_EVENT;

extern ST_FOC_t stFoc;
extern TaskHandle_t hFocControlStart_Task;
extern TaskHandle_t hFocControlCali_Task;
extern EventGroupHandle_t hFocEvent;

void FOC_Set_Zero_Position(void);
void FOC_DriverInit(void);
void FOC_Ud_Uq_Limit(float f32Ud, float f32Uq, float *pf32Ud, float *pf32Uq);
void FOC_SVPMW_Cal(ST_FOC_t *pstFoc);
void FOC_PWM_Update_IRQ_Handle(void);
void FOC_Control_Start_Task(void *pvParameters);
void FOC_Control_Cali_Task(void *pvParameters);
void FOC_Control_FaultDet_Task(void *pvParameters);
void FOC_Control_DataScope_Task(void *pvParameters);
void FOC_Control_Running_Task(void *pvParameters);
void FOC_Control_Status_Task(void *pvParameters);
void FOC_Control_Filter_Task(void *pvParameters);

#endif

