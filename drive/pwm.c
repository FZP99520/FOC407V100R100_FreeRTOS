#include "pwm.h"
#include "foc.h"
#include "log.h"

static u8 _bTIM1_PWM_InitDone = FALSE;

void TIM1_PWM_Init(void)
{
//�˲������ֶ��޸�IO������
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DEBUG_TRACE("IN\n");

    if(_bTIM1_PWM_InitDone)
    {
        DEBUG_ERROR("already init.\n");
        return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//TIM1ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��PORTAʱ�� 

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);//����GPIOE_Pin8ΪTIM1_Ch1N,   
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);//����GPIOE_Pin9ΪTIM1_Ch1,   
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);//����GPIOE_Pin10ΪTIM1_Ch2N,   
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);//����GPIOE_Pin11ΪTIM1_Ch2,
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);//����GPIOE_Pin10ΪTIM1_Ch3N,
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);//����GPIOE_Pin11ΪTIM1_Ch3,
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);//����GPIOE_Pin11ΪTIM1_Ch4,
          
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;//GPIO  
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;        //���ù���  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //�ٶ�100MHz  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����  
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;        //����  
    GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��P  

    TIM_TimeBaseStructure.TIM_Prescaler          = 0;  //��ʱ����Ƶ  
    TIM_TimeBaseStructure.TIM_CounterMode        = TIM_CounterMode_CenterAligned1; //���ϼ���ģʽ  
    TIM_TimeBaseStructure.TIM_Period             = TIM1_PWM_PERIOD - 1;   //�Զ���װ��ֵ  
    TIM_TimeBaseStructure.TIM_ClockDivision      = TIM_CKD_DIV1;   
    TIM_TimeBaseStructure.TIM_RepetitionCounter  = 0x1;//Ĭ�Ͼ�Ϊ0  
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1  
    //��ʼ��TIM1  PWMģʽ       
    //PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ��.   
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ  
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;//�ڿ���ʱ�����,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
      
    TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��  
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1��ʼ��  
    TIM_OCInitStructure.TIM_Pulse = 0; 	
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);//ͨ��2
    TIM_OCInitStructure.TIM_Pulse = 0; 	
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);//ͨ��3
    
    TIM_OCInitStructure.TIM_Pulse = TIM1_PWM_PERIOD_LIMIT;// for external adc trig
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);//ͨ��4

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�  
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ��   

#if 1
    TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;  //�Զ��������ʹ��
    TIM1_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;           //ʧ��ɲ������
    TIM1_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;      //ɲ������ܽż��Ը�
    TIM1_BDTRInitStructure.TIM_DeadTime        = 0x0;                        //����ʱ������ �ο�CSDN���㷽��,������3us
    TIM1_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;           //����ƽ�����������κ�λ
    TIM1_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Disable;       //����������ģʽ�·ǹ���״̬
    TIM1_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Disable;       //����������ģʽ�·ǹ���״̬
    TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);
#endif

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn; //��ʱ��1�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1  
    TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����  

    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);//For ADC trig acquire

    _bTIM1_PWM_InitDone = TRUE;

    DEBUG_TRACE("OK\n");

}  

void TIM1_SetPWM_Duty(u16 u16CH1,u16 u16CH2,u16 u16CH3)
{
    u16CH1 = u16CH1>TIM1_PWM_PERIOD_LIMIT?TIM1_PWM_PERIOD_LIMIT:u16CH1;
    u16CH2 = u16CH2>TIM1_PWM_PERIOD_LIMIT?TIM1_PWM_PERIOD_LIMIT:u16CH2;
    u16CH3 = u16CH3>TIM1_PWM_PERIOD_LIMIT?TIM1_PWM_PERIOD_LIMIT:u16CH3;

    TIM_SetCompare1(TIM1, u16CH1);
    TIM_SetCompare2(TIM1, u16CH2);
    TIM_SetCompare3(TIM1, u16CH3);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update))
    {
        FOC_PWM_Update_IRQ_Handle();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/**************************TIM8 PWM Setting*************************************/

void TIM8_SetPWM_Duty(ST_PWM_DUTY_SETTING stPwmDutySetting)
{
    float f32Ch1Duty = stPwmDutySetting.f32Ch1Duty;
    float f32Ch2Duty = stPwmDutySetting.f32Ch2Duty;
    float f32Ch3Duty = stPwmDutySetting.f32Ch3Duty;
    float f32Ch4Duty = stPwmDutySetting.f32Ch4Duty;
    u16 u16Ch1OnDuty,u16Ch2OnDuty,u16Ch3OnDuty,u16Ch4OnDuty;

    f32Ch1Duty = LIMIT(f32Ch1Duty, stPwmDutySetting.f32DutyMax, stPwmDutySetting.f32DutyMin);
    f32Ch2Duty = LIMIT(f32Ch2Duty, stPwmDutySetting.f32DutyMax, stPwmDutySetting.f32DutyMin);
    f32Ch3Duty = LIMIT(f32Ch3Duty, stPwmDutySetting.f32DutyMax, stPwmDutySetting.f32DutyMin);
    f32Ch4Duty = LIMIT(f32Ch4Duty, stPwmDutySetting.f32DutyMax, stPwmDutySetting.f32DutyMin);

    u16Ch1OnDuty = f32Ch1Duty*stPwmDutySetting.u16PwmPeriod;
    u16Ch2OnDuty = f32Ch2Duty*stPwmDutySetting.u16PwmPeriod;
    u16Ch3OnDuty = f32Ch3Duty*stPwmDutySetting.u16PwmPeriod;
    u16Ch4OnDuty = f32Ch4Duty*stPwmDutySetting.u16PwmPeriod;

    TIM_SetCompare1(TIM8, u16Ch1OnDuty);
    TIM_SetCompare2(TIM8, u16Ch2OnDuty);
    TIM_SetCompare3(TIM8, u16Ch3OnDuty);
    TIM_SetCompare4(TIM8, u16Ch4OnDuty); //channel4 is for adc acquire
}

void TIM8_PWM_Init(void)
{
//�˲������ֶ��޸�IO������
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//TIM1ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��PORTAʱ�� 

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//����GPIOC_Pin6ΪTIM8_Ch1,
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);//����GPIOC_Pin7ΪTIM8_Ch2, 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);//����GPIOC_Pin8ΪTIM8_Ch3,
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);//����GPIOC_Pin8ΪTIM8_Ch4,
          
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           //GPIO  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //�ٶ�100MHz  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //����  
    GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��P  
      
    TIM_TimeBaseStructure.TIM_Prescaler=0;  //��ʱ����Ƶ  
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned1; //���ϼ���ģʽ  
    TIM_TimeBaseStructure.TIM_Period= TIM8_PWM_PERIOD - 1;  //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x1;//
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//��ʼ����ʱ��1  
    //��ʼ��TIM1  PWMģʽ       
    //PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ��.   
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ  
  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ�����,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������  
      
    TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��  
    TIM_OC1Init(TIM8, &TIM_OCInitStructure); //Ch1��ʼ��  
      
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);//ͨ��2
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = 100;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�  
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM8,DISABLE);//ARPEʹ��

    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn; //��ʱ��8�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8  
    TIM_CtrlPWMOutputs(TIM8, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����   

    TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_OC4Ref);//For ADC trig acquire

}  

void TIM8_UP_TIM13_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM8, TIM_IT_Update))
    {
        //FOC_PWM_Update_IRQ_Handle();
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
}

void TIM5_PWM_Init(u32 u32Fclk)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    DEBUG_TRACE("IN\n");
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//TIM1ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��PORTAʱ�� 

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);//����GPIOA_Pin1ΪTIM2_Ch2,   
    //GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);//����GPIOE_Pin11ΪTIM1_Ch2,
          
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           //GPIO  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ�100MHz  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //����  
    GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��P  
      
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  //��ʱ����Ƶ  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ  
    TIM_TimeBaseStructure.TIM_Period = u32Fclk-1;   //�Զ���װ��ֵ  
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;   
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//Ĭ�Ͼ�Ϊ0  
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��2  
    //��ʼ��TIM1  PWMģʽ       
    //PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ��.   
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��  
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ  
  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ�����,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������  
      
    //TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��  
    //TIM_OC1Init(TIM5, &TIM_OCInitStructure); //Ch1��ʼ��  
      
    TIM_OCInitStructure.TIM_Pulse = 0; 	
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);//ͨ��2
      
    //TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�  
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ��   
    TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM1  
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����  

    
    DEBUG_TRACE("OK\n");
}  

