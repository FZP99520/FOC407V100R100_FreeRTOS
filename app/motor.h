#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx.h"

#define EL1_H GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define EL1_L GPIO_ResetBits(GPIOE,GPIO_Pin_7)

#define EL2_H GPIO_SetBits(GPIOE,GPIO_Pin_10)
#define EL2_L GPIO_ResetBits(GPIOE,GPIO_Pin_10)

#define ZF1_H GPIO_SetBits(GPIOE,GPIO_Pin_8)
#define ZF1_L GPIO_ResetBits(GPIOE,GPIO_Pin_8)

#define ZF2_H GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define ZF2_L GPIO_ResetBits(GPIOE,GPIO_Pin_12)
typedef struct
{
	u8 lock;
	int pwm1;
	int pwm2;
	u8 zf1;
	u8 zf2;
	u8 stop1;//el1 É²³µ
	u8 stop2;//el2
}MOTOR;
extern MOTOR motor;

u8 Motor_Unlock(void);
void Motor_Brake(void);
void MOTOR_Init(void);

#endif
