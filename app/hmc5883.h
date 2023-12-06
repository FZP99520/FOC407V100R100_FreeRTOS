#ifndef _hmc5883_H_
#define _hmc5883_H_

#define MAG_QMC5883 
//#define MAG_HMC5883
#include "stm32f4xx.h"
#include "iic.h"
typedef struct
{
	u8 update_ok;
	u8 STA;
	s16 MAG_X;
	s16 MAG_Y;
	s16 MAG_Z;
	float Mag_x;
	float Mag_y;
	float Mag_z;
	float Total_MAG;
}MAG_Data;
extern MAG_Data mag_data;

void MAG_WriteData(u8 addr,u8 dat);
u8 MAG_ReadByte(u8 addr);
void MAG_Init(void);
u8 MAG_DataUpdate(void);
void MAG_GetData(u8 *buff);
#ifdef MAG_HMC5883
#define	 Address  0x3c
#define  Reg_Config_A 0x00  //ÅäÖÃ¼Ä´æÆ÷A
#define  Reg_Config_B 0x01  //ÅäÖÃ¼Ä´æÆ÷B
#define  Reg_Mode     0x02  //Ä£Ê½¼Ä´æÆ÷
#define  Reg_XData_H 0x03
#define  Reg_XData_L 0x04
#define  Reg_YData_H 0x05
#define  Reg_YData_L 0x06
#define  Reg_ZData_H 0x07
#define  Reg_ZData_L 0x08
#define  Reg_Status  0x09 //×´Ì¬¼Ä´æÆ÷
#endif
/**********************************/
#ifdef   MAG_QMC5883
#define	 Address  0x1a
#define  Reg_Config_A 0x00  //ÅäÖÃ¼Ä´æÆ÷A
#define  Reg_Config_B 0x01  //ÅäÖÃ¼Ä´æÆ÷B
#define  Reg_Mode     0x02  //Ä£Ê½¼Ä´æÆ÷
#define  Reg_XData_L 0x00
#define  Reg_XData_H 0x01
#define  Reg_YData_L 0x02
#define  Reg_YData_H 0x03
#define  Reg_ZData_L 0x04
#define  Reg_ZData_H 0x05
#define  Reg_Status  0x06  //×´Ì¬¼Ä´æÆ÷ ¶Á
#define  Reg_Temp_L  0x07  //temperature 100LSB/c
#define  Reg_Temp_H  0x08
#define  Reg_Config  0x09 //ÅäÖÃ¼Ä´æÆ÷
#define  Reg_Config2 0x0a //interrupt reg
#define  Reg_Period  0x0b //recommanded 0x01

#define MAG_DRY 0x01 
#define MAG_OVL 0x02 //³¬¹ıÁ¿³Ì
#define MAG_DOR 0x04 //Ìø¹ıÊı¾İÃ»ÓĞ¶ÁÈ¡
#endif
/*******************************/
#endif
