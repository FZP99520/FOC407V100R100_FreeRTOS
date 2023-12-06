#include "as5048a.h"
#include "foc.h"
#include "log.h"
#include "filter.h"
#include "spi.h"
#include "filter.h"

#if AS5048A_SUPPORT_SPI_MODE
#define AS5048A_SPI_CS_PIN      GPIO_Pin_7
#define AS5048A_SPI_CS_PORT     GPIOB

#define AS5048A_SPI_CS_HIGH()     GPIO_SetBits(AS5048A_SPI_CS_PORT, AS5048A_SPI_CS_PIN)
#define AS5048A_SPI_CS_LOW()      GPIO_ResetBits(AS5048A_SPI_CS_PORT, AS5048A_SPI_CS_PIN)

#define AS5048A_SPI_READ_WRITE(u16DataIn, pu16DataOut)  SPI2_ReadWriteByte(u16DataIn,pu16DataOut)

static u8 _AS5048A_SPI_ReadWrite(u16 u16DataIn, u16 *pu16DataOut);

ST_Angle_Sensor_Info_t _stAngleSensorInfo;

static ST_BiquadFilter_t _stFltOmega = { //Fs=10000Hz, Fpass=0.8Hz
    .f32Gain = 0.000251264267979f,
    .f32a1 = 1.0f,
    .f32a2 = 1.0f,
    .f32b1 = 0.99949747146404f
};



#endif


#define AS5048A_PWM_TYPE_FREQ 1000 //unit: Hz
#define AS5048A_CAPTURE_PERIOD_CNT_MAX  1.1f*SystemCoreClock/2/AS5048A_PWM_TYPE_FREQ
#define AS5048A_CAPTURE_PERIOD_CNT_MIN  0.9f*SystemCoreClock/2/AS5048A_PWM_TYPE_FREQ

static u8 _bIsZeroAngleInitDone = FALSE;
ST_CaptureInfo_t _stAS5048Capture;


u8 AS5048A_GetAngleInfo(float *pf32Angle, float *pf32Omega, float *pf32Alpha)
{
#if AS5048A_SUPPORT_SPI_MODE
    *pf32Angle = _stAngleSensorInfo.f32AngleNow;
    *pf32Omega = _stAngleSensorInfo.f32OmegaNow;
    *pf32Alpha = _stAngleSensorInfo.f32Alpha;
    return TRUE;
#else
    if(_stAS5048Capture.bIsUpdateDone == TRUE)
    {
        *pf32Angle = _stAS5048Capture.f32Angle;
        *pf32Omega = _stAS5048Capture.f32Omega;
        _stAS5048Capture.bIsUpdateDone = FALSE;
        return TRUE;
    }
    return FALSE;
#endif
}

void AS5048A_SetAngleOffset(float f32offset)
{
#if AS5048A_SUPPORT_SPI_MODE
    _stAngleSensorInfo.f32AngleOffset = f32offset;
#else
    _stAS5048Capture.f32AngleOffset = f32offset;
#endif
}

void AS5048A_GetAngleOffset(float *f32offset)
{
#if AS5048A_SUPPORT_SPI_MODE
    *f32offset = _stAngleSensorInfo.f32AngleOffset;
#else
    *f32offset = _stAS5048Capture.f32AngleOffset;
#endif
}

//Use TIM input capture
//PB6 TIM4_CH1

void AS5048A_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DEBUG_TRACE("IN\n");

#if AS5048A_SUPPORT_SPI_MODE

    GPIO_InitStructure.GPIO_Pin =  AS5048A_SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(AS5048A_SPI_CS_PORT, &GPIO_InitStructure);
    AS5048A_SPI_CS_HIGH();
    SPI2_Init(SPI_BaudRatePrescaler_8);
    //SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//5Mbps
#else
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;  //定时器分频  
    TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up; //向上计数模式  
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;   //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0 
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器1

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update|TIM_IT_CC1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //定时器4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
#endif

    DEBUG_TRACE("OK\n");

}

#if AS5048A_SUPPORT_SPI_MODE
static u8 _AS5048A_SPI_ReadWrite(u16 u16DataIn, u16 *pu16DataOut)
{
    u8 bRet = TRUE;
    __nop();
    __nop();
    __nop();
    __nop();
    AS5048A_SPI_CS_LOW();
    if(AS5048A_SPI_READ_WRITE(u16DataIn, pu16DataOut) != E_SPI_OK)
    {
        bRet = FALSE;
    }
    AS5048A_SPI_CS_HIGH();
    __nop();
    __nop();
    __nop();
    __nop();
    return bRet;
}
static u8 _parity_even(u16 v)
{
      if(v == 0) return 0;
      v ^= v >> 8;
      v ^= v >> 4;
      v ^= v >> 2;
      v ^= v >> 1;
      return v & 1;
}

u8 AS5048A_SPI_Read(u16 u16Addr, u16 *pu16DataOut)
{
    u8 u8ErrFlag = TRUE;
    u16 u16data = 0;
    u16 u16Data2Write = 0;
    u16 u16Data2Read = 0;

    u16Data2Write = AS5048A_SPI_READ;
    u16Data2Write = u16Data2Write | u16Addr;
    u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);//Add a parity bit on the the MSB
    if(_AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read) != TRUE)
    {
        goto FUNCTION_END;
    }

    u16Data2Write = AS5048A_SPI_READ;// PAR=0 R/W=R
    u16Data2Write = u16Data2Write | AS5048A_SPI_CMD_NOP;
    u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);//Add a parity bit on the the MSB
    if(_AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read) != TRUE)
    {
        goto FUNCTION_END;
    }
    if ((u16Data2Read & (1 << 14)) == 0)
    {
        u16data = (u16Data2Read & 0x3FFF);
        u8ErrFlag = (_parity_even(u16data) ^ (u16Data2Read >> 15));
    }
    else
    {
        u16Data2Write = AS5048A_SPI_READ;
        u16Data2Write = u16Data2Write | AS5048A_SPI_CMD_CLEAR_ERR_FLAG;
        u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);//Add a parity bit on the the MSB
        _AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read);
    }
    *pu16DataOut = u16data;

FUNCTION_END:
    return u8ErrFlag;

}

u8 AS5048A_SPI_Write(u16 u16Addr, u16 u16Data)
{
    u8 u8ErrorFlag = 0;
    u16 u16data = 0;
    u16 u16Data2Write = 0;
    u16 u16Data2Read = 0;

    u16Data2Write = AS5048A_SPI_WRITE;
    u16Data2Write |= u16Addr&0x3FFF;
    u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);
    if(_AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read) != TRUE)
    {
        goto FUNCTION_END;
    }

    u16Data2Write = AS5048A_SPI_WRITE;
    u16Data2Write |= u16Data&0x3FFF;
    u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);
    if(_AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read) != TRUE)
    {
        goto FUNCTION_END;
    }

    u16Data2Write = 0x0000;
    u16Data2Write |= AS5048A_SPI_READ;
    u16Data2Write |= AS5048A_SPI_CMD_NOP;
    u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);
    if(_AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read) != TRUE)
    {
        goto FUNCTION_END;
    }

    if ((u16Data2Read & (1 << 14)) == 0)
    {
        u16data = (u16Data2Read & 0x3FFF);
        u8ErrorFlag = (_parity_even(u16data) ^ (u16Data2Read >> 15));
    }
    else
    {
        u16Data2Write = 0x0000;
        u16Data2Write |= AS5048A_SPI_READ;
        u16Data2Write |= AS5048A_SPI_CMD_CLEAR_ERR_FLAG;
        u16Data2Write |= ((u16)_parity_even(u16Data2Write)<<15);
        _AS5048A_SPI_ReadWrite(u16Data2Write, &u16Data2Read);
    }

FUNCTION_END:
    return u8ErrorFlag;
}

void AS5048A_Update_Angle_Info(void)
{
    float f32Delta = 0;
    float f32Omega = 0;
    float f32Ka_Alpha = 0.00005f;
    static float _f32AlphaPre;
    static u8 _bInitDone = FALSE;

    if(!_bIsZeroAngleInitDone)
    {
        return;
    }

    if(_bInitDone == FALSE) //for stable power on, omega should be near to 0
    {
        _stAngleSensorInfo.bErrFlag = AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &_stAngleSensorInfo.u16AngleRegValue);
        _stAngleSensorInfo.f32AngleNow = (float)_stAngleSensorInfo.u16AngleRegValue*AS5048A_ANGLE_DATA_LSB;
        _stAngleSensorInfo.f32AnglePre = _stAngleSensorInfo.f32AngleNow;
        _bInitDone = TRUE;
        return;
    }
    _stAngleSensorInfo.bErrFlag = AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &_stAngleSensorInfo.u16AngleRegValue);
    if(_stAngleSensorInfo.bErrFlag)
    {
        _stAngleSensorInfo.u16ErrCnt++;
        return;
    }
    else
    {
        _stAngleSensorInfo.f32AngleNow = (float)_stAngleSensorInfo.u16AngleRegValue*AS5048A_ANGLE_DATA_LSB;
        f32Delta = _stAngleSensorInfo.f32AngleNow - _stAngleSensorInfo.f32AnglePre;
        DEGREE_TO_180(f32Delta);
        f32Omega = f32Delta/FOC_HANDLE_DT;
        //_stAngleSensorInfo.f32OmegaNow = Biquad_Filter(&_stFltOmega, f32Omega);
        _stAngleSensorInfo.f32OmegaNow += 0.001f*(f32Omega - _stAngleSensorInfo.f32OmegaNow);
        f32Delta = _stAngleSensorInfo.f32OmegaNow - _stAngleSensorInfo.f32OmegaPre;
        _stAngleSensorInfo.f32Alpha = f32Delta/FOC_HANDLE_DT;
        _stAngleSensorInfo.f32Alpha = (1.0f - f32Ka_Alpha)*_f32AlphaPre + f32Ka_Alpha*_stAngleSensorInfo.f32Alpha;
        //Save Preious Data
        _stAngleSensorInfo.f32AnglePre = _stAngleSensorInfo.f32AngleNow;
        _stAngleSensorInfo.f32OmegaPre = _stAngleSensorInfo.f32OmegaNow;
        _f32AlphaPre = _stAngleSensorInfo.f32Alpha;
    }
}

u8 AS5048A_If_Need_Init_ZeroPosition(void)
{
    u16 u16AngleZeroRegVal = 0;
    u16 u16AngleRegVal = 0;
    u16 u16AngleHigh = 0, u16AngleLow = 0;
    DEBUG_TRACE("Step0.0==> Read Zero Position Register:\n");
    FR_OS_DelayMs(300);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_H, &u16AngleHigh);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_L, &u16AngleLow);
    FR_OS_DelayMs(10);

    DEBUG_TRACE("Step0.1==>u16AngleHigh=0x%04x\n", u16AngleHigh);
    DEBUG_TRACE("Step0.2==>u16AngleLow=0x%04x\n", u16AngleLow);
    u16AngleZeroRegVal = ((u16AngleHigh&0xFF)<<6) | (u16AngleLow&0x3F);
    DEBUG_TRACE("Step0.3==>u16AngleZeroRegVal = 0x%04x\n", u16AngleZeroRegVal);

    if(u16AngleZeroRegVal != 0)
    {
        DEBUG_TRACE("Step0.4==>u16AngleZeroRegVal is not 0!\n");
        AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &u16AngleRegVal);
        DEBUG_TRACE("Step0.5==>read angle value: 0x%04x\n", u16AngleRegVal);
        _bIsZeroAngleInitDone = TRUE;
        return FALSE;
    }
    else
    {
        DEBUG_TRACE("Step0.4==>u16AngleZeroRegVal is 0!\n");
        _bIsZeroAngleInitDone = FALSE;
        return TRUE;
    }
}

u8 AS5048A_Init_ZeroPosition(u16 u16ZeroAngleVal)
{
    u16 u16temp = 0;
    u16 u16AngleRegVal = 0;
    u16 u16AngleHigh = 0, u16AngleLow = 0;
    u16 u16AngleHighRead = 0, u16AngleLowRead = 0;
    u16 u16Data2Write = 0;
    u16 u16Data2Read = 0;
    u16 u16AngleZeroRegVal = 0;

    DEBUG_TRACE("IN\n");

//Step1,Write 0 to zero register
    u16AngleHigh = 0;
    u16AngleLow = 0;
    DEBUG_TRACE("Step1.0==>set 0 to Zero Position Register\n");
    AS5048A_SPI_Write(AS5048A_SPI_CMD_OTP_ZERO_POS_H, u16AngleHigh);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Write(AS5048A_SPI_CMD_OTP_ZERO_POS_L, u16AngleLow);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_H, &u16AngleHighRead);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_L, &u16AngleLowRead);
    DEBUG_TRACE("Step1.1==>read back u16AngleHighRead=0x%04x, u16AngleLowRead=0x%04x\n", u16AngleHighRead, u16AngleLowRead);
    u16AngleZeroRegVal = ((u16AngleHighRead&0xFF)<<6) | (u16AngleLowRead&0x3F);
    DEBUG_TRACE("Step1.2==>u16AngleZeroRegVal = 0x%04x\n", u16AngleZeroRegVal);
    if(u16AngleZeroRegVal != 0)
    {
        DEBUG_TRACE("Step1.2==>[Error]u16AngleZeroRegVal is not 0, set fail!!!\n", u16AngleZeroRegVal);
        return FALSE;
    }

//Step2,read angle value
    AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &u16AngleRegVal);
    //u16AngleRegVal = 0xb63;
    //DEBUG_TRACE("Step2.0==>read angle value (0x%04x),and set to Zero Position Register\n", u16AngleRegVal);

//Step3,write angle value to zero register
    u16AngleHigh = u16ZeroAngleVal>>6;
    u16AngleLow  = u16ZeroAngleVal&0x3F;
    DEBUG_TRACE("Step3.0==> input u16ZeroAngleVal (0x%04x),and set to Zero Position Register\n", u16ZeroAngleVal);
    AS5048A_SPI_Write(AS5048A_SPI_CMD_OTP_ZERO_POS_H, u16AngleHigh);
    DEBUG_TRACE("Step3.1==>write to Zero Position Register u16AngleHigh: 0x%04x\n", u16AngleHigh);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Write(AS5048A_SPI_CMD_OTP_ZERO_POS_L, u16AngleLow);
    DEBUG_TRACE("Step3.2==>write to Zero Position Register u16AngleLow: 0x%04x\n", u16AngleLow);
    FR_OS_DelayMs(10);

    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_H, &u16AngleHighRead);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_L, &u16AngleLowRead);
    FR_OS_DelayMs(10);
    DEBUG_TRACE("Step3.3==>read back zero register u16AngleHighRead=0x%04x\n", u16AngleHighRead);
    DEBUG_TRACE("Step3.4==>read back zero register u16AngleLowRead=0x%04x\n", u16AngleLowRead);
    if((u16AngleHigh!=u16AngleHighRead)||(u16AngleLow!=u16AngleLowRead))
    {
        DEBUG_ERROR("FAIL0,read back register value is not eque to write.\n");
        return FALSE;
    }
    AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &u16AngleRegVal);
    FR_OS_DelayMs(10);
    DEBUG_TRACE("Step3.5==>after init zero position, read angle value :0x%04x\n", u16AngleRegVal);
    /*
    if((u16AngleRegVal>(0x3FFF-0x10))||(u16AngleRegVal<(0x10)))//after set zero, read data to be +-0x10
    {
        DEBUG_TRACE("Step3.6==>check data ok!!! In range[-0x10,+0x10]\n");
    }
    else
    {
        DEBUG_TRACE("FAIL 1,after init, u16AngleRegVal=0x%04x is not near to zero.\n", u16AngleRegVal);
        return FALSE;
    }
    */

#if 0 //program action only once
//Step4,enable programming
    AS5048A_SPI_Read(AS5048A_SPI_CMD_PROGRAMMING_CTRL, &u16temp);
    DEBUG_TRACE("Step4.0==>Read Program Control Register Value 0x%04x, and set program enable\n", u16temp);
    AS5048A_SPI_Write(AS5048A_SPI_CMD_PROGRAMMING_CTRL, 0x01);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_PROGRAMMING_CTRL, &u16temp);
    DEBUG_TRACE("Step4.1==>Read Program Control Register Value 0x%04x\n", u16temp);
    FR_OS_DelayMs(10);

//Step5,set burn it
    DEBUG_TRACE("Step5.0==>burn it\n");
    AS5048A_SPI_Write(AS5048A_SPI_CMD_PROGRAMMING_CTRL, 0x09);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_PROGRAMMING_CTRL, &u16temp);
    DEBUG_TRACE("Step5.1==>get program reg value:0x%04x\n", u16temp);
    FR_OS_DelayMs(10);

//Step6,read back zero register value to check 
    AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &u16AngleRegVal);
    FR_OS_DelayMs(10);
    DEBUG_TRACE("Step6.0==>after burn, read angle value :0x%04x\n", u16AngleRegVal);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_H, &u16AngleHighRead);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_OTP_ZERO_POS_L, &u16AngleLowRead);
    FR_OS_DelayMs(10);
    DEBUG_TRACE("Step6.1==>get u16AngleHighRead=0x%04x, u16AngleLowRead=0x%04x\n", u16AngleHighRead, u16AngleLowRead);

    if(u16AngleHigh != u16AngleHighRead)
    {
        DEBUG_ERROR("FAIL 1, u16AngleHigh=0x%04x, u16AngleHighRead=0x%04x.\n", u16AngleHigh, u16AngleHighRead);
        return FALSE;
    }
    if(u16AngleLow != u16AngleLowRead)
    {
        DEBUG_ERROR("FAIL 2, u16AngleLow=0x%04x, u16AngleLowRead=0x%04x.\n", u16AngleHigh, u16AngleLowRead);
        return FALSE;
    }

//Step7,verity data
    DEBUG_TRACE("Step7.0==>Enable verity!!!\n");
    AS5048A_SPI_Write(AS5048A_SPI_CMD_PROGRAMMING_CTRL, 0x49);
    FR_OS_DelayMs(10);
    AS5048A_SPI_Read(AS5048A_SPI_CMD_PROGRAMMING_CTRL, &u16temp);
    FR_OS_DelayMs(10);
    DEBUG_TRACE("Step7.1==>Read Program Control Register Value 0x%04x\n", u16temp);

//Step8,read angle data
    AS5048A_SPI_Read(AS5048A_SPI_CMD_READ_ANGLE, &u16AngleRegVal);
    DEBUG_TRACE("Step8.0==>get angle data u16AngleRegVal=0x%04x\n", u16AngleRegVal);
    FR_OS_DelayMs(10);
    
    if((u16AngleRegVal>(0x3FFF-0x10))||(u16AngleRegVal<(0x10)))//after set zero, read data to be +-0x10
    {
        DEBUG_TRACE("Step8.1==>check data ok!!! In range[-0x10,+0x10]\n");
    }
    else
    {
        DEBUG_TRACE("FAIL 3, after burn, u16AngleRegVal=0x%04x is not zero.\n", u16AngleRegVal);
        return FALSE;
    }
#endif
    _bIsZeroAngleInitDone = TRUE;

    DEBUG_TRACE("OK\n");
    return TRUE;
}

#endif

static u8 _AS5048A_CaculateAngleInfo(ST_CaptureInfo_t *pstCaptureInfo)
{
    u16 u16StartClock;
    u16 u16EndClock;
    u16 u16Duty;
    float f32PeriodCnt = pstCaptureInfo->u32PeriodCnt;
    float f32PluseWidth = pstCaptureInfo->u32HighCnt;
    static float _f32Angle_M_Pre = 0;
    static float _f32Omega_M_Pre = 0;
    float f32Ka_Omg = 0.008f;
    float f32Ka_Ang = 0.9f;

    if((f32PeriodCnt > AS5048A_CAPTURE_PERIOD_CNT_MAX) || (f32PeriodCnt < AS5048A_CAPTURE_PERIOD_CNT_MIN))
    {
        pstCaptureInfo->bIsUpdateDone = TRUE;
        return FALSE;
    }

    if(f32PeriodCnt > 0)
    {
        u16StartClock = f32PeriodCnt*16/4119.0f;
        u16EndClock = u16StartClock/2;
        f32PluseWidth = f32PluseWidth - u16StartClock;
        if(f32PluseWidth < 0.0f)
        {
            f32PluseWidth = 0;
        }
        f32PeriodCnt = f32PeriodCnt - u16StartClock - u16EndClock;
        pstCaptureInfo->f32Duty = (float)f32PluseWidth*100.0f/f32PeriodCnt;
        pstCaptureInfo->f32AngleMeas = pstCaptureInfo->f32Duty*3.60f;
        if((pstCaptureInfo->f32AngleMeas > 360.0f) || (pstCaptureInfo->f32AngleMeas < 0.0f))
        {
            pstCaptureInfo->f32AngleMeas = pstCaptureInfo->f32AngleMeasPre;
        }
        pstCaptureInfo->f32AngleDelta    = pstCaptureInfo->f32AngleMeas - pstCaptureInfo->f32AngleMeasPre;
        if(pstCaptureInfo->f32AngleDelta < -180.0f) //clock wise
        {
            pstCaptureInfo->f32AngleDelta += 360.0f;
        }
        else if(pstCaptureInfo->f32AngleDelta > 180.0f)
        {
            pstCaptureInfo->f32AngleDelta -= 360.0f;
        }
        pstCaptureInfo->f32Omega = (double)pstCaptureInfo->f32AngleDelta*(SystemCoreClock/2.0f)/pstCaptureInfo->u32PeriodCnt;
        pstCaptureInfo->f32Omega = (1.0f - f32Ka_Omg)*_f32Omega_M_Pre + f32Ka_Omg*pstCaptureInfo->f32Omega;

        //pstCaptureInfo->f32Angle += pstCaptureInfo->f32AngleDelta;
        //pstCaptureInfo->f32Angle = DEGREE_TO_360(pstCaptureInfo->f32Angle + pstCaptureInfo->f32AngleDelta - pstCaptureInfo->f32AngleOffset);

        pstCaptureInfo->f32Angle = DEGREE_TO_360(pstCaptureInfo->f32AngleMeas - pstCaptureInfo->f32AngleOffset);
        //pstCaptureInfo->f32Angle = (1.0f - f32Ka_Ang)*_f32Angle_M_Pre + f32Ka_Ang*pstCaptureInfo->f32Angle;
        //pstCaptureInfo->f32Angle = pstCaptureInfo->f32AngleMeas;
        _f32Omega_M_Pre = pstCaptureInfo->f32Omega;
        _f32Angle_M_Pre = pstCaptureInfo->f32Angle;
        pstCaptureInfo->f32AngleMeasPre = pstCaptureInfo->f32AngleMeas;
        pstCaptureInfo->bIsUpdateDone = TRUE;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void TIM4_IRQHandler(void)
{
    static u8 _bIsHighPeriod = FALSE;
    static u8 _u8HighUpdateCnt = 0;
    static u8 _u8PeriodUpdateCnt = 0;
    u16 u16CaptureVal = 0;

    if(TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
    {
        if(_bIsHighPeriod)
        {
            _u8HighUpdateCnt++;
            _u8PeriodUpdateCnt++;
        }
        else
        {
            _u8PeriodUpdateCnt++;
        }
    }

    if(TIM_GetITStatus(TIM4, TIM_IT_CC1)!=RESET)
    {
        if(_bIsHighPeriod)
        {
            u16CaptureVal = TIM_GetCapture1(TIM4);
            _bIsHighPeriod = FALSE;
            TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);
            _stAS5048Capture.u32HighCnt = _u8HighUpdateCnt*65536 + u16CaptureVal;
        }
        else
        {
            u16CaptureVal = TIM_GetCapture1(TIM4);
            _stAS5048Capture.u32PeriodCnt = _u8PeriodUpdateCnt*65536 + u16CaptureVal;
            _AS5048A_CaculateAngleInfo(&_stAS5048Capture);

            _bIsHighPeriod = TRUE;
            _u8HighUpdateCnt = 0;
            _u8PeriodUpdateCnt = 0;
            TIM_SetCounter(TIM4,0x0);
            TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);
        }
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update|TIM_IT_CC1);

}



