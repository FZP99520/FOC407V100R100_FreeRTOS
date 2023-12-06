#include "dac.h"
#include "log.h"

//DAC1 ==> PA4
//DAC2 ==> PA5
void DAC1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef  DAC_InitStructure;

    DEBUG_TRACE("IN\n");

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //使能ADC1时钟
 
    //先初始化ADC1通道1 IO口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//不带上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);

    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);

    DEBUG_TRACE("OK\n");
}

void DAC1_SetValue(u16 u16DacValue)
{
    DAC_SetChannel1Data(DAC_Align_12b_R,u16DacValue);
}

void DAC1_SetVoltage(u16 u16VoltageX1000)
{
    u16 u16val;
    float f32temp = 0;
    f32temp = u16VoltageX1000/1000.0f;
    f32temp  = f32temp*4095/3.3f;
    u16val = f32temp;
    DAC_SetChannel1Data(DAC_Align_12b_R, u16val);
}


