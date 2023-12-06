#ifndef _PWM_H
#define _PWM_H
#include "stm32f4xx.h"

typedef struct
{
    float f32Ch1Duty;
    float f32Ch2Duty;
    float f32Ch3Duty;
    float f32Ch4Duty;
    float f32DutyMin;
    float f32DutyMax;
    u16   u16PwmPeriod;
}ST_PWM_DUTY_SETTING;

void TIM1_PWM_Init(void);
void TIM5_PWM_Init(u32 u32fclk);
void TIM8_PWM_Init(void);

void TIM1_SetPWM_Duty(u16 u16CH1,u16 u16CH2,u16 u16CH3);
void TIM8_SetPWM_Duty(ST_PWM_DUTY_SETTING stPwmDutySetting);

#define TIM1_PWM_PERIOD        SystemCoreClock/20000
#define TIM1_PWM_PERIOD_LIMIT  0.95f*(SystemCoreClock/20000)


#define TIM8_PWM_PERIOD        SystemCoreClock/20000
#define TIM8_PWM_PERIOD_LIMIT  0.95f*(SystemCoreClock/20000)



#endif

