#include "beep.h"
#include "systick.h"
#include "log.h"

void Beep_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct; //结构体变量定义

    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;  //beep
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT; //推挽输出模式
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;

    GPIO_Init(GPIOC,&GPIO_InitStruct);  
    GPIO_SetBits(GPIOC,GPIO_Pin_13);

    DEBUG_TRACE("OK\n");
}
void beep_delay(u16 n)
{
	for(u16 i=0;i<n;i++) __nop();
}
void Beep_ms(u16 nms)
{
    GPIO_ResetBits(GPIOC,GPIO_Pin_13);
    Delay_ms(nms);
    GPIO_SetBits(GPIOC,GPIO_Pin_13);
}
void Beep_times(u8 n)
{
    u8 i;
    for(i=0;i<n;i++)
    {
     Beep_ms(100);
     Delay_ms(100);
    }
}
void Beep(u8 times,u16 time_work,u16 time_stop)
{
   u8 i;
   for(i=0;i<times;i++)
   {
     GPIO_ResetBits(GPIOC,GPIO_Pin_13);
     Delay_ms(time_work);
     GPIO_SetBits(GPIOC,GPIO_Pin_13);
    Delay_ms(time_stop);
   }
}
