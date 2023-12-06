#include "DataExc.h"
#include "adc.h"
#define a
//static u8 buf[32]; 
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
void RC_Send_CMD(u8* buf)
{
	
}

void CNT_AdjPID()
{
	
}

void ANL_CMD()
{
}
void ANL_FLY()
{
}
void ANL_AdjPID()
{
	
}