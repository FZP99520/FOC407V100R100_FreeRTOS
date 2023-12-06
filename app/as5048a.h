#ifndef _AS5048_H
#define _AS5048_H

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

#define AS5048A_SUPPORT_SPI_MODE TRUE

typedef struct
{
    u8    bIsUpdateDone;
    u32   u32HighCnt;
    u32   u32PeriodCnt;
    float f32Duty;
    float f32AngleMeas;
    float f32AngleMeasPre;
    float f32AngleDelta;
    float f32AngleOffset;
    float f32Angle;
    float f32Omega;
}ST_CaptureInfo_t;

#if AS5048A_SUPPORT_SPI_MODE
typedef struct
{
    u8 bErrFlag;
    u16 u16ErrCnt;
    u16 u16AngleRegValue;
    float f32AngleNow;
    float f32AnglePre;
    float f32AngleOffset;
    float f32OmegaNow;
    float f32OmegaPre;
    float f32Alpha;
}ST_Angle_Sensor_Info_t;

#endif


void AS5048A_Init(void);
u8 AS5048A_GetAngleInfo(float *pf32Angle, float *pf32Omega, float *pf32Alpha);
void AS5048A_SetAngleOffset(float f32offset);
void AS5048A_GetAngleOffset(float *f32offset);

#if AS5048A_SUPPORT_SPI_MODE

#define AS5048A_ANGLE_DATA_LSB 360.0f/16384 //14bit data

#define AS5048A_SPI_READ    0x1<<14
#define AS5048A_SPI_WRITE   0x0<<14

#define AS5048A_SPI_CMD_NOP                 0x0000
#define AS5048A_SPI_CMD_CLEAR_ERR_FLAG      0x0001
#define AS5048A_SPI_CMD_PROGRAMMING_CTRL    0x0003
#define AS5048A_SPI_CMD_OTP_ZERO_POS_H      0x0016
#define AS5048A_SPI_CMD_OTP_ZERO_POS_L      0x0017
#define AS5048A_SPI_CMD_READ_DIAG           0x3FFD //Diagnostics + Automatic Gain Control (AGC)
#define AS5048A_SPI_CMD_READ_MAGNITUDE      0x3FFE
#define AS5048A_SPI_CMD_READ_ANGLE          0x3FFF

u8 AS5048A_SPI_Write(u16 u16Addr, u16 u16Data);
u8 AS5048A_SPI_Read(u16 u16Addr, u16 *pu16DataOut);
void AS5048A_Update_Angle_Info(void);
u8 AS5048A_If_Need_Init_ZeroPosition(void);
u8 AS5048A_Init_ZeroPosition(u16 u16ZeroAngleVal);

#endif



#endif

