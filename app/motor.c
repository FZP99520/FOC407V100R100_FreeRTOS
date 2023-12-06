#include "motor.h"
#include "pwm.h"
#include "systick.h"
MOTOR motor;
void MOTOR_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	 GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_8|GPIO_Pin_12; 
	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	 GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	 GPIO_Init(GPIOE,&GPIO_InitStruct); 
	
	 TIM1_PWM_Init();//open pwm channel
	 motor.lock=1;//Ëø¶¨
}
u8 Motor_Unlock(void)
{
	if(motor.lock==1)
	{
	 EL1_H;
	 EL2_H;
	 Delay_ms(50);
	 EL1_L;
	 EL2_L;
	 motor.lock=0;
		return 1;
	}
	else return 0;
}
void Motor_Brake()
{
	
}










