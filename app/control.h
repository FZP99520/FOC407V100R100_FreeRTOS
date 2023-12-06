#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx.h"
typedef struct
{
	int   Balance_P;
	float Balance_D;
	int   Velocity_P;
	float Velocity_I;
	int   Turn_P;
	float Turn_D;
}PID;
typedef struct 
{
	u8 Flag_Forward;
	u8 Flag_Backward;
	u8 Flag_Left;
	u8 Flag_Right;
}MOVEMENT;
extern MOVEMENT move;
extern PID pid;

#endif
