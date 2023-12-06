#ifndef intput_ctrl_H
#define intput_ctrl_H


#include "stm32f4xx.h"
typedef struct
{
	u8 Scan_OK;
	u8 LEFT_Left; //按键是否按下标志位
	u16 LEFT_Left_cnt;
	
	u8 LEFT_Right;
	u16 LEFT_Right_cnt;
	
	u8 LEFT_Up;
	u16 LEFT_Up_cnt;
	
	u8 LEFT_Down;
	u16 LEFT_Down_cnt;
	
	u8 RIGHT_Left;
	u16 RIGHT_Left_cnt;
	
	u8 RIGHT_Right;
	u16 RIGHT_Right_cnt;
	
	u8 RIGHT_Up;
	u16 RIGHT_Up_cnt;
	
	u8 RIGHT_Down;
	u16 RIGHT_Down_cnt;
	
	u8 LEFT_Upper;
	u16 LEFT_Upper_cnt;
	
	u8 RIGHT_Upper;
	u8 RIGHT_Upper_cnt;
	
	u16 ADC_LEFT_X;
	u16 ADC_LEFT_Y;
	u16 ADC_RIGHT_X;
	u16 ADC_RIGHT_Y;
	u16 ADC_VBat;
	
	u8 Volt_Offset_sta;
	float Volt_LEFT_X_Offset;//归中电压
	float Volt_LEFT_Y_Offset;
	float Volt_RIGHT_X_Offset;
	float Volt_RIGHT_Y_Offset;
	
	float Volt_LEFT_X;   //测量电压，将转化成油门
	float Volt_LEFT_Y;
	float Volt_RIGHT_X;
	float Volt_RIGHT_Y;
	float Volt_VBat;
	
	s16 Thro_High;
	s16 Thro_Pitch;
	s16 Thro_Roll;
	s16 Thro_Yaw;
	
}Input_TypeDef;
extern Input_TypeDef INPUT;
extern u8 PID_R_Index; //指向的PID数据位置
extern u8 PID_C_Index;
extern float* PID_Point;//指针指向被修改的某个PID参数
void ScanInput_Handle(void);
#endif
