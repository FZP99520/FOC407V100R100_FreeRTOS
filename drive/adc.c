#include "adc.h"
#include "systick.h"
#include "RC.h"
#include "display.h"
#include "foc.h"
#include "log.h"
#include "drv8323.h"
#include "filter.h"

#define ADC_DEBUG_GPIO_ENABLE    TRUE
#if ADC_DEBUG_GPIO_ENABLE
    #define ADC_DEBUG_GPIO_NUM       GPIO_Pin_7
    #define ADC_DEBUG_GPIO_PORT      GPIOE
    #define ADC_DEBUG_GPIO_IN()      GPIO_ReadOutputDataBit(ADC_DEBUG_GPIO_PORT, ADC_DEBUG_GPIO_NUM)
    #define ADC_DEBUG_GPIO_OUT(x)    x?GPIO_SetBits(ADC_DEBUG_GPIO_PORT, ADC_DEBUG_GPIO_NUM):\
                                        GPIO_ResetBits(ADC_DEBUG_GPIO_PORT, ADC_DEBUG_GPIO_NUM)
    #define ADC_DEBUG_GPIO_TOGGLE()  ADC_DEBUG_GPIO_IN()?ADC_DEBUG_GPIO_OUT(FALSE):ADC_DEBUG_GPIO_OUT(TRUE)
#else
    #define ADC_DEBUG_GPIO_IN(...)     
    #define ADC_DEBUG_GPIO_OUT(...)
#endif


//#include "dma.h"

static u16 _au16JAdcValue[3] = {0};
static float _af32Voltage[3] = {0};

u16 pu16AdcConvertVal[ADC_CHANNEL_INUSE_NUM] = {0};
u8  _bADC1_OverRunFlag = FALSE;
ST_ADC_Conv_t stAdcConv = 
{
    .u8ValLen = 4,
    .pu16AdcConvVal = pu16AdcConvertVal
};


QueueHandle_t hAdcConv_Queue = NULL;//dma interrupt send queue

ST_MoveAverageFilter_t F_ADC_LEFT_X={10,0,0,{0}};
ST_MoveAverageFilter_t F_ADC_LEFT_Y={10,0,0,{0}};
ST_MoveAverageFilter_t F_ADC_RIGHT_X={10,0,0,{0}};
ST_MoveAverageFilter_t F_ADC_RIGHT_Y={10,0,0,{0}};
static float MoveAvarageFilter(ST_MoveAverageFilter_t * filter,float data);


void Adc_Init(void)
{
    GPIO_InitTypeDef      GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;
    NVIC_InitTypeDef      NVIC_InitStructure;

    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��
 
    //�ȳ�ʼ��ADC1ͨ��1 IO��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3;//PA1 ͨ��1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

#if ADC_DEBUG_GPIO_ENABLE
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  ADC_DEBUG_GPIO_NUM;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�����������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(ADC_DEBUG_GPIO_PORT, &GPIO_InitStructure);
#endif

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);                 //ADC1��λ
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);               //��λ����
 
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay =ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//DMAʧ��
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
 
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_TRGO;
    ADC_InitStructure.ADC_DataAlign= ADC_DataAlign_Right;//�Ҷ���   
    ADC_InitStructure.ADC_NbrOfConversion = ADC_CHANNEL_INUSE_NUM;
    ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��

    //ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_28Cycles);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_28Cycles);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_28Cycles);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_28Cycles);

    ADC_InjectedSequencerLengthConfig(ADC1, 3);
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_0,1, ADC_SampleTime_28Cycles);
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_2,2, ADC_SampleTime_28Cycles);
    ADC_InjectedChannelConfig(ADC1,ADC_Channel_3,3, ADC_SampleTime_28Cycles);

    ADC_SoftwareStartInjectedConv(ADC1);

    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
    //ADC_AutoInjectedConvCmd(ADC1, DISABLE);
    //ADC_ExternalTrigInjectedConvCmd();

    //ADC_ClearFlag(ADC1,ADC_FLAG_EOC);//
    //unnessesary to use EOC interrupt, because dma move data will clear the EOC FLAG
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //DMA_ADC1_Init((u32)pu16AdcConvertVal);
    //ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);//����ADת����

    //ADC_SoftwareStartConv(ADC1);

    DEBUG_TRACE("OK\n");

}

void ADC_IRQHandler(void)
{
    if(ADC_GetITStatus(ADC1,ADC_IT_OVR) != RESET)
    {
        DMA_ADC1_Init(pu16AdcConvertVal);
        ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
        ADC_SoftwareStartConv(ADC1);
        _bADC1_OverRunFlag = TRUE;
    }
#if 1
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
#endif
    if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        ADC_DEBUG_GPIO_OUT(TRUE);
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        _au16JAdcValue[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        _au16JAdcValue[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
        _au16JAdcValue[2] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);

        _af32Voltage[0] = _au16JAdcValue[0]*3.3f/4096;
        _af32Voltage[1] = _au16JAdcValue[1]*3.3f/4096;
        _af32Voltage[2] = _au16JAdcValue[2]*3.3f/4096;

        DRV8323_SetCurSenseInfo(_af32Voltage[0], _af32Voltage[1], _af32Voltage[2]);

        //At the same time, read angle sensor data by spi
        AS5048A_Update_Angle_Info();
        ADC_DEBUG_GPIO_OUT(FALSE);
        //ADC_DEBUG_GPIO_TOGGLE();
    }
}
 
//���ADCֵ
//ch: @ref ADC_channels
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
static u16 GET_ADC(u8 ch)  
{
    ADC_RegularChannelConfig(ADC1,ch,1, ADC_SampleTime_480Cycles );//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��
    ADC_SoftwareStartConv(ADC1);  //ʹ��ָ����ADC1�����ת����������        
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
    return ADC_GetConversionValue(ADC1);//�������һ��ADC1�������ת�����
}
#define Thro_MAX 1999u
#define Thro_MIN 1001u
void GetThro_Data(void)
{
    INPUT.ADC_LEFT_X=GET_ADC(4);
    INPUT.ADC_LEFT_Y=GET_ADC(5);
    INPUT.ADC_RIGHT_Y=GET_ADC(6);
    INPUT.ADC_RIGHT_X=GET_ADC(7);
    //�Զ�ȡ����ADCֵ������ƽ���˲�
    INPUT.ADC_LEFT_X=MoveAvarageFilter(&F_ADC_LEFT_X,INPUT.ADC_LEFT_X);
    INPUT.ADC_LEFT_Y=MoveAvarageFilter(&F_ADC_LEFT_Y,INPUT.ADC_LEFT_Y);
    INPUT.ADC_RIGHT_X=MoveAvarageFilter(&F_ADC_RIGHT_X,INPUT.ADC_RIGHT_X);
    INPUT.ADC_RIGHT_Y=MoveAvarageFilter(&F_ADC_RIGHT_Y,INPUT.ADC_RIGHT_Y);

    INPUT.Volt_LEFT_X=(float)(INPUT.ADC_LEFT_X)*3.3f/4096;
    INPUT.Volt_LEFT_Y=(float)(INPUT.ADC_LEFT_Y)*3.3f/4096;
    INPUT.Volt_RIGHT_X=(float)(INPUT.ADC_RIGHT_X)*3.3f/4096;
    INPUT.Volt_RIGHT_Y=(float)(INPUT.ADC_RIGHT_Y)*3.3f/4096;

    if(INPUT.Volt_Offset_sta==1)//У׼����ټ���������
    {
        INPUT.Thro_Yaw=(INPUT.Volt_LEFT_X-INPUT.Volt_LEFT_X_Offset)*300.0f + 1500u;
        //if(LEFT_Up_STA==0) INPUT.Thro_High=1;
        //else if(LEFT_Down_STA==0) INPUT.Thro_High=-1;
        //else INPUT.Thro_High=0;
        INPUT.Thro_High=(INPUT.Volt_LEFT_Y-INPUT.Volt_LEFT_Y_Offset)*300.0f + 1500u;
        INPUT.Thro_Pitch=(INPUT.Volt_RIGHT_Y-INPUT.Volt_RIGHT_Y_Offset)*300.0f + 1500u;
        INPUT.Thro_Roll=(INPUT.Volt_RIGHT_X-INPUT.Volt_RIGHT_X_Offset)*300.0f + 1500u;

        INPUT.Thro_High = INPUT.Thro_High>Thro_MAX ? Thro_MAX:INPUT.Thro_High;
        INPUT.Thro_High = INPUT.Thro_High<Thro_MIN ? Thro_MIN:INPUT.Thro_High;
        INPUT.Thro_Yaw = INPUT.Thro_Yaw>Thro_MAX ? Thro_MAX:INPUT.Thro_Yaw;
        INPUT.Thro_Yaw = INPUT.Thro_Yaw<Thro_MIN ? Thro_MIN:INPUT.Thro_Yaw;
        INPUT.Thro_Roll = INPUT.Thro_Roll>Thro_MAX ? Thro_MAX:INPUT.Thro_Roll;
        INPUT.Thro_Roll = INPUT.Thro_Roll<Thro_MIN ? Thro_MIN:INPUT.Thro_Roll;
        INPUT.Thro_Pitch = INPUT.Thro_Pitch>Thro_MAX ? Thro_MAX:INPUT.Thro_Pitch;
        INPUT.Thro_Pitch = INPUT.Thro_Pitch<Thro_MIN ? Thro_MIN:INPUT.Thro_Pitch;
        if(Page_Now==1)Show_Throttle_Update(INPUT);      //�������ſ�������
    }
}
u8 GetThro_Offset(void)
{
	static u16 cnt=0;
	static float vol[4]={0,0,0,0};
	GetThro_Data();
	cnt++;
	  if(cnt>=OffsetNum)
		{
			INPUT.Volt_LEFT_X_Offset =  vol[0]/cnt;
			//INPUT.Volt_LEFT_Y_Offset =  vol[1]/cnt;
			INPUT.Volt_LEFT_Y_Offset =  vol[1]/cnt;
			INPUT.Volt_RIGHT_X_Offset = vol[2]/cnt;
			INPUT.Volt_RIGHT_Y_Offset = vol[3]/cnt;
			vol[0] = 0;
		  vol[1] = 0;
		  vol[2] = 0;
		  vol[3] = 0;
			INPUT.Volt_Offset_sta=1;
			return 1;//У׼���
		}
		else
		{
		 vol[0] += INPUT.Volt_LEFT_X;
		 vol[1] += INPUT.Volt_LEFT_Y;
		 vol[2] += INPUT.Volt_RIGHT_X;
		 vol[3] += INPUT.Volt_RIGHT_Y;
			return 0;
		}
}
float MoveAvarageFilter(ST_MoveAverageFilter_t* filter,float data)
{
    if((filter->Num_Wind>Max_Size_Wind)||(0==filter->Num_Wind))
    {
        filter->Num_Wind=Max_Size_Wind;
    }
    if(filter->Index>=filter->Num_Wind)
    {
        filter->Index=0;
    }   
    filter->Sum-=filter->Wind[filter->Index];
    filter->Wind[filter->Index]=data;
    filter->Sum+=filter->Wind[filter->Index];
    filter->Index++;

    return filter->Sum/filter->Num_Wind;  
}

