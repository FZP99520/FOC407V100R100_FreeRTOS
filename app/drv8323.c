#include "drv8323.h"
#include "spi.h"
#include "log.h"
#include "filter.h"

#define DRV8323_GPIO_ENABLE_PIN   GPIO_Pin_0
#define DRV8323_GPIO_ENABLE_PORT  GPIOE

#define DRV8323_GPIO_CAL_PIN      GPIO_Pin_1
#define DRV8323_GPIO_CAL_PORT     GPIOE

#define DRV8323_GPIO_FAULT_PIN    GPIO_Pin_2
#define DRV8323_GPIO_FAULT_PORT   GPIOE

#define DRV8323_SPI_CS_PIN        GPIO_Pin_12
#define DRV8323_SPI_CS_PORT       GPIOB

#define DRV8323_SPI_CS_HIGH()     GPIO_SetBits(DRV8323_SPI_CS_PORT, DRV8323_SPI_CS_PIN)
#define DRV8323_SPI_CS_LOW()      GPIO_ResetBits(DRV8323_SPI_CS_PORT, DRV8323_SPI_CS_PIN)


#define DRV8323_CONFIG_PWM_MODE DRV8323_CONFIG_PWM_MODE_6X
#define DRV8323_SPI_READ_WRITE(u16DataIn, pu16DataOut)    SPI2_ReadWriteByte(u16DataIn, pu16DataOut)

#define DRV8323_CURRENT_SENSE_RESISTER          (0.1f)
#define DRV8323_CURRENT_SENSE_AMPLIFIER_GAIN    (5)

//Fs=10000KHz Fpass=50Hz
#define ADC_FILTER_I_GAIN   (0.029947695741426f)
#define ADC_FILTER_I_A1     (1.0f)
#define ADC_FILTER_I_A2     (1.0f)
#define ADC_FILTER_I_B1     (0.940104608517470f)

ST_DRV_CUR_SENSE_INFO _stDrvCurSenseInfo;
static u8 _bCurSenseInitDone = FALSE;


static ST_BiquadFilter_t _stFltIa =
{
    .f32Gain = ADC_FILTER_I_GAIN,
    .f32a1 = ADC_FILTER_I_A1,
    .f32a2 = ADC_FILTER_I_A2,
    .f32b1 = ADC_FILTER_I_B1
};

static ST_BiquadFilter_t _stFltIb =
{
    .f32Gain = ADC_FILTER_I_GAIN,
    .f32a1 = ADC_FILTER_I_A1,
    .f32a2 = ADC_FILTER_I_A2,
    .f32b1 = ADC_FILTER_I_B1
};

static ST_BiquadFilter_t _stFltIc = 
{ 
    .f32Gain = ADC_FILTER_I_GAIN,
    .f32a1 = ADC_FILTER_I_A1,
    .f32a2 = ADC_FILTER_I_A2,
    .f32b1 = ADC_FILTER_I_B1
};


u8 DRV8323_GetPwmMode(void)
{
    return DRV8323_CONFIG_PWM_MODE;
}

void DRV8323_Init(void)
{
    u16 u16temp = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  DRV8323_GPIO_ENABLE_PIN | DRV8323_GPIO_CAL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(DRV8323_GPIO_ENABLE_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DRV8323_GPIO_FAULT_PIN; //input gpio
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DRV8323_GPIO_FAULT_PORT, &GPIO_InitStructure);

    DRV8323_Set_GPIO_CAL(FALSE);
    DRV8323_Set_GPIO_ENABLE(TRUE);

#if CONFIG_DRV8323_SPI_SUPPORT
    DEBUG_TRACE("Support SPI Mode\n");
    //SPI_CS
    GPIO_InitStructure.GPIO_Pin =  DRV8323_SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DRV8323_SPI_CS_PORT, &GPIO_InitStructure);

    DRV8323_SPI_CS_HIGH();
    SPI2_Init(SPI_BaudRatePrescaler_256);

    //u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_WRITE|DRV8323_SPI_REG_GATE_DRIVER_HS);
    //u16temp = DRV8323_SPI_ReadWrite((DRV8323_SPI_WRITE|DRV8323_SPI_REG_DRIVER_CONTROL)+(0x0<<5));
    u16temp = DRV8323_SPI_ReadWrite((DRV8323_SPI_WRITE|DRV8323_SPI_REG_CSA_CONTROL)+(0x203));// 5V/V
    //SPI2_SetSpeed(SPI_BaudRatePrescaler_256);//for drv8323
    Delay_ms(10);
    DEBUG_TRACE("Read register value after setting.\n");
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_FAULT_STATUS_1);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_FAULT_STATUS_1), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_VGS_STATUS_2);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_VGS_STATUS_2), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_DRIVER_CONTROL);
    DEBUG_TRACE("read reg:%s:0x%x\n",VAR_TO_STR(DRV8323_SPI_REG_DRIVER_CONTROL), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_GATE_DRIVER_HS);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_GATE_DRIVER_HS), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_GATE_DRIVER_LS);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_GATE_DRIVER_LS), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_OCP_CONTROL);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_OCP_CONTROL), u16temp);
    Delay_ms(10);
    u16temp = DRV8323_SPI_ReadWrite(DRV8323_SPI_READ|DRV8323_SPI_REG_CSA_CONTROL);
    DEBUG_TRACE("read reg:%s:0x%04x\n",VAR_TO_STR(DRV8323_SPI_REG_CSA_CONTROL), u16temp);

    u8 bFault = FALSE;
    DRV8323_Get_GPIO_FAULT(&bFault);
    DEBUG_TRACE("bFault=%d\n",bFault);

#endif
    DEBUG_TRACE("OK\n");

    return;
}

u16 DRV8323_SPI_ReadWrite(u16 u16data)
{
    u16 u16data2read;
    DRV8323_SPI_CS_LOW();
    DRV8323_SPI_READ_WRITE(u16data, &u16data2read);
    DRV8323_SPI_CS_HIGH();
    return u16data2read;
}

void DRV8323_SetCurSenseOffset(float f32AdcVoltCHA, float f32AdcVoltCHB, float f32AdcVoltCHC)
{
    _stDrvCurSenseInfo.f32AdcVoltCHA_Offset = f32AdcVoltCHA;
    _stDrvCurSenseInfo.f32AdcVoltCHB_Offset = f32AdcVoltCHB;
    _stDrvCurSenseInfo.f32AdcVoltCHC_Offset = f32AdcVoltCHC;
    _bCurSenseInitDone = TRUE;
}

void DRV8323_GetCurSenseForCali(float *pf32VolCHA, float *pf32VolCHB, float *pf32VolCHC)
{
    *pf32VolCHA = _stDrvCurSenseInfo.f32AdcVoltCHA;
    *pf32VolCHB = _stDrvCurSenseInfo.f32AdcVoltCHB;
    *pf32VolCHC = _stDrvCurSenseInfo.f32AdcVoltCHC;
}

void DRV8323_SetCurSenseInfo(float f32AdcVoltCHA, float f32AdcVoltCHB, float f32AdcVoltCHC)
{
    //float f32Ka = 0.1f;
    if(_bCurSenseInitDone == TRUE)
    {
        _stDrvCurSenseInfo.f32AdcVoltCHA = Biquad_Filter(&_stFltIa, f32AdcVoltCHA);
        _stDrvCurSenseInfo.f32AdcVoltCHB = Biquad_Filter(&_stFltIb, f32AdcVoltCHB);
        _stDrvCurSenseInfo.f32AdcVoltCHC = Biquad_Filter(&_stFltIc, f32AdcVoltCHC);
        /*
        _stDrvCurSenseInfo.f32AdcVoltCHA = (1.0f-f32Ka)*_stDrvCurSenseInfo.f32AdcVoltCHA + f32Ka*f32AdcVoltCHA;
        _stDrvCurSenseInfo.f32AdcVoltCHB = (1.0f-f32Ka)*_stDrvCurSenseInfo.f32AdcVoltCHB + f32Ka*f32AdcVoltCHB;
        _stDrvCurSenseInfo.f32AdcVoltCHC = (1.0f-f32Ka)*_stDrvCurSenseInfo.f32AdcVoltCHC + f32Ka*f32AdcVoltCHC;
        */
    }
    else
    {
        _stDrvCurSenseInfo.f32AdcVoltCHA = f32AdcVoltCHA;
        _stDrvCurSenseInfo.f32AdcVoltCHB = f32AdcVoltCHB;
        _stDrvCurSenseInfo.f32AdcVoltCHC = f32AdcVoltCHC;
    }
}

u8 DRV8323_GetCurSenseInfo(float *pf32CurCHA, float *pf32CurCHB, float *pf32CurCHC)
{
    //电流流入中性点方向为正，否则为负
    _stDrvCurSenseInfo.f32I_A = (_stDrvCurSenseInfo.f32AdcVoltCHA - _stDrvCurSenseInfo.f32AdcVoltCHA_Offset)/DRV8323_CURRENT_SENSE_AMPLIFIER_GAIN/DRV8323_CURRENT_SENSE_RESISTER;
    _stDrvCurSenseInfo.f32I_B = (_stDrvCurSenseInfo.f32AdcVoltCHB - _stDrvCurSenseInfo.f32AdcVoltCHB_Offset)/DRV8323_CURRENT_SENSE_AMPLIFIER_GAIN/DRV8323_CURRENT_SENSE_RESISTER;
    _stDrvCurSenseInfo.f32I_C = (_stDrvCurSenseInfo.f32AdcVoltCHC - _stDrvCurSenseInfo.f32AdcVoltCHC_Offset)/DRV8323_CURRENT_SENSE_AMPLIFIER_GAIN/DRV8323_CURRENT_SENSE_RESISTER;
    *pf32CurCHA = _stDrvCurSenseInfo.f32I_A;
    *pf32CurCHB = _stDrvCurSenseInfo.f32I_B;
    *pf32CurCHC = _stDrvCurSenseInfo.f32I_C;
    return TRUE;
}

void DRV8323_Set_GPIO_ENABLE(u8 bSet)
{
    if(bSet)
    {
        GPIO_SetBits(DRV8323_GPIO_ENABLE_PORT, DRV8323_GPIO_ENABLE_PIN);
    }
    else
    {
        GPIO_ResetBits(DRV8323_GPIO_ENABLE_PORT, DRV8323_GPIO_ENABLE_PIN);
    }
}

void DRV8323_Set_GPIO_CAL(u8 bSet)
{
    if(bSet)
    {
        GPIO_SetBits(DRV8323_GPIO_CAL_PORT, DRV8323_GPIO_CAL_PIN);
    }
    else
    {
        GPIO_ResetBits(DRV8323_GPIO_CAL_PORT, DRV8323_GPIO_CAL_PIN);
    }
}

void DRV8323_Get_GPIO_FAULT(u8 *pbSet)
{
    u8 uGpioStatus = FALSE;
    //logic low means fault happend
    uGpioStatus = GPIO_ReadInputDataBit(DRV8323_GPIO_FAULT_PORT, DRV8323_GPIO_FAULT_PIN);
    *pbSet = !uGpioStatus;
    
}



