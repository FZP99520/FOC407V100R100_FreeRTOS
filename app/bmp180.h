#ifndef _BMP180_H_
#define _BMP180_H_

#include "stm32f10x.h"
typedef struct __BMP180
{
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;
    long UT;
    long UP;
    long X1;
    long X2;
    long X3;
    long B3;
    unsigned long B4;
    long B5;
    long B6;
    long B7;
    long p;
    long Temp;
    float altitude;
}BMP180;
#define BMP180_SlaveAddress 0xEE
#define OUT_XLSB 0xF8
#define OUT_LSB  0xF7
#define OUT_MSB  0xF6
#define CTRL_MEAS 0xF4
#define SOFT_RESET 0xE0
#define BMP180_ID 0xD0
//*********************//
#define oss 0
#define p0 101325 //1013.25 hPa
//*********************//
#define AC1_H 0xAA
#define AC1_L 0xAB
#define AC2_H 0xAC
#define AC2_L 0xAD
#define AC3_H 0xAE
#define AC3_L 0xAF
#define AC4_H 0xB0
#define AC4_L 0xB1
#define AC5_H 0xB2
#define AC5_L 0xB3
#define AC6_H 0xB4
#define AC6_L 0xB5
#define B1_H  0xB6
#define B1_L  0xB7
#define B2_H  0xB8
#define B2_L  0xB9
#define MB_H  0xBA
#define MB_L  0xBB
#define MC_H  0xBC
#define MC_L  0xBD
#define MD_H  0xBE
#define MD_L  0xBF
//funtion
void BMP180_Write_Byte(u8 addr,u8 data);
u8 BMP180_Read_Byte(u8 addr);
short BMP180_Read_TwoByte(u8 addr);
//////////////////////
void BMP180_Init(void);
u8 BMP180_Start_Conversion(void);
void BMP180_CaculateData(void);
u8 BMP180_Check(void);
void BMP_UncompemstatedToTrue(void);
long BMP180_Read_UT(void);
long BMP180_Read_UP(void);
void BMP180_ReadCalibrationData(void);

void BMP_CaculateData(void);

extern long temperature;
extern long pressure;
extern float altitude;
extern BMP180 bmp;
#endif
