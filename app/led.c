#include "led.h"
#include "systick.h"
#include "log.h"

void Led_Init()  //LED初始化函数
{
    GPIO_InitTypeDef  GPIO_InitStruct; //结构体变量定义
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    DEBUG_TRACE("IN\n");
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;  //LED1
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT; //推挽输出模式
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);  
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;  //LED2
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_2MHz;
    GPIO_Init(GPIOC,&GPIO_InitStruct);
    LED1_OFF;
    LED2_OFF;

    DEBUG_TRACE("OK\n");
}
void LED_Flash(u8 times)
{
    u8 i;
    for(i=0;i<times;i++)
    {
        LED1_ON;
        LED1_ON;
        Delay_ms(200);
        LED1_OFF;
        LED1_OFF;
        Delay_ms(200);
    }
}

