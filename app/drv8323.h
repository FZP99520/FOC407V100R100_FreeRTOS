#ifndef _DRV8323_H_
#define _DRV8323_H_

#include "stm32f4xx.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

#define VAR_TO_STR(args...) #args 


typedef struct
{
    float f32AdcVoltCHA;
    float f32AdcVoltCHB;
    float f32AdcVoltCHC;
    float f32AdcVoltCHA_Offset;
    float f32AdcVoltCHB_Offset;
    float f32AdcVoltCHC_Offset;
    float f32I_A;
    float f32I_B;
    float f32I_C;
}ST_DRV_CUR_SENSE_INFO;

#define CONFIG_DRV8323_SPI_SUPPORT  TRUE

#ifdef CONFIG_DRV8323_SPI_SUPPORT
#define DRV8323_SPI_WRITE 0x0000
#define DRV8323_SPI_READ  0x8000

#define DRV8323_SPI_REG_FAULT_STATUS_1      (0x00<<11)
#define DRV8323_SPI_REG_VGS_STATUS_2        (0x01<<11)
#define DRV8323_SPI_REG_DRIVER_CONTROL      (0x02<<11)
#define DRV8323_SPI_REG_GATE_DRIVER_HS      (0x03<<11)
#define DRV8323_SPI_REG_GATE_DRIVER_LS      (0x04<<11)
#define DRV8323_SPI_REG_OCP_CONTROL         (0x05<<11)
#define DRV8323_SPI_REG_CSA_CONTROL         (0x06<<11)
#define DRV8323_SPI_REG_RESERVE             (0x07<<11)

#endif

#define DRV8323_CONFIG_PWM_MODE_6X           (0x00)
#define DRV8323_CONFIG_PWM_MODE_3X           (0x01)
#define DRV8323_CONFIG_PWM_MODE_1X           (0x02)
#define DRV8323_CONFIG_PWM_MODE_INDEPENDENT  (0x03)

void DRV8323_SetCurSenseOffset(float f32AdcVoltCHA, float f32AdcVoltCHB, float f32AdcVoltCHC);
void DRV8323_SetCurSenseInfo(float f32AdcVoltCHA, float f32AdcVoltCHB, float f32AdcVoltCHC);
void DRV8323_GetCurSenseForCali(float *pf32VolCHA, float *pf32VolCHB, float *pf32VolCHC);
u8 DRV8323_GetCurSenseInfo(float *pf32CurCHA, float *pf32CurCHB, float *pf32CurCHC);
void DRV8323_Init(void);
void DRV8323_Set_GPIO_ENABLE(u8 bSet);
void DRV8323_Set_GPIO_CAL(u8 bSet);
void DRV8323_Get_GPIO_FAULT(u8* bSet);
u16 DRV8323_SPI_ReadWrite(u16 u16data);




#endif

