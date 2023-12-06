#include "display.h"
#include "ili9488.h"
#include "RC.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "stdarg.h"

#define PRINTF_ROW 13
#define PRINTF_COL 0

u8 Page_Now=0;
u8 Page_Next=0; 
u8 pu8StringBuff[64];
char LogBuff[4][48];
char strTemp[48];

ShowPosition_TypeDef SP_NRF={2,0*8,(5)*10};
ShowPosition_TypeDef SP_Sensor={7,0*8,(8)*10};
ShowPosition_TypeDef SP_Status={7,(10+6)*10,(10+6+6)*10};
ShowPosition_TypeDef SP_GPS={7,(10+6+6+6+2)*10,(10+6+6+6+2+6)*10};
ShowPosition_TypeDef SP_Power={14,(10+6)*10,(10+6+5)*10};
ShowPosition_TypeDef SP_Thro={2,11*10,(11+7+1)*10};



void DisplayLogMessage(u32 u32Color, const char* format,...)
{
    #define DISPLAY_LOG_MESSAGE_COL_START  0
    #define DISPLAY_LOG_MESSAGE_ROW_MAX    16
    u16 u16StrLen = 0;
    static u16 _u16RowCnt = 0;
    va_list args;
    va_start(args,format);
    u16StrLen = vsprintf(pu8StringBuff,format,args);
    va_end(args);
    LCD_SetColor(u32Color);
    if(_u16RowCnt >= DISPLAY_LOG_MESSAGE_ROW_MAX)
    {
        LCD_Clear();
        _u16RowCnt = 0;
    }
    LCD_DisplayString(DISPLAY_LOG_MESSAGE_COL_START,(_u16RowCnt++)*20,pu8StringBuff);

}
//**********************//
static void Show_DrawBar(u16 x,u16 y,u16 barlen,float temp,u32 postColor,u32 negaColor,u32 backColor);
/******************/
u16 row,col;
u8 s;
void Show_Update(void)
{
	if(Page_Now!=Page_Next) //刷新目录行
	{
		Page_Now = Page_Next;
		LCD_Clear();
		if(Page_Now ==1)
		{
			Show_Page1_Log("Welcome to controller system!");
			  LCD_SetColor(LCD_BLUE);
			  LCD_DrawLine(0,130,480,130);
			  LCD_DrawLine(0,131,480,131);
			  LCD_DrawLine(0,132,480,132);
			  LCD_DrawLine(149,130,149,360);
			  LCD_DrawLine(150,130,150,360);
			  LCD_DrawLine(151,130,151,360);
			  LCD_DrawLine(289,130,289,360);
			  LCD_DrawLine(290,130,290,360);
			  LCD_DrawLine(291,130,291,360);
			  LCD_DrawLine(150,269,290,269);
			  LCD_DrawLine(150,270,290,270);
			  LCD_DrawLine(150,271,290,271);
			  LCD_SetColor(LCD_GREEN);
			  LCD_DisplayString(190,0,"MONITOR");
			  row = SP_NRF.strRow;
	      col = SP_NRF.DirStartCol;
			  LCD_DisplayString(col,(row++)*20,"S:");
	      LCD_DisplayString(col,(row++)*20,"TX:");//RC Data
	      LCD_DisplayString(col,(row++)*20,"RX:");
	      LCD_DisplayString(col,(row++)*20,"MAX:");
			  row = SP_Sensor.strRow;
	      col = SP_Sensor.DirStartCol;
	      LCD_DisplayString(col,(row++)*20,"ACCEL_X:");//Sensor data
	      LCD_DisplayString(col,(row++)*20,"ACCEL_Y:");
	      LCD_DisplayString(col,(row++)*20,"ACCEL_Z:");
	      LCD_DisplayString(col,(row++)*20,"GYRO_X:");
	      LCD_DisplayString(col,(row++)*20,"GYRO_Y:");
				LCD_DisplayString(col,(row++)*20,"GYRO_Z:");
	      LCD_DisplayString(col,(row++)*20,"MAG_X:");
	      LCD_DisplayString(col,(row++)*20,"MAG_Y:");
	      LCD_DisplayString(col,(row++)*20,"MAG_Z:");
	      row = SP_Thro.strRow;
	      col = SP_Thro.DirStartCol;
	      LCD_DisplayString(col,(row++)*20,"T_High:");//Throttle
	      LCD_DisplayString(col,(row++)*20,"T_Yaw:");
	      LCD_DisplayString(col,(row++)*20,"T_Roll:");
	      LCD_DisplayString(col,(row++)*20,"T_Pitch:");
	      row = SP_Status.strRow;
	      col = SP_Status.DirStartCol;
	      LCD_DisplayString(col,(row++)*20,"Roll:");//Drone Status
	      LCD_DisplayString(col,(row++)*20,"Pitch:");
	      LCD_DisplayString(col,(row++)*20,"Yaw:");
	      LCD_DisplayString(col,(row++)*20,"Alt:");
				LCD_DisplayString(col,(row++)*20,"Spd:");
	      LCD_DisplayString(col,(row++)*20,"Mode:");
	      LCD_DisplayString(col,(row++)*20,"STA:");
				row = SP_Power.strRow;
				col = SP_Power.DirStartCol;
				LCD_DisplayString(col,(row++)*20,"VBAT:");
	      LCD_DisplayString(col,(row++)*20,"CBAT:");
				row=SP_GPS.strRow;
				col=SP_GPS.DirStartCol;
				LCD_DisplayString(col,(row++)*20,"STA:");
				LCD_DisplayString(col,(row++)*20,"SVNUM:");
				LCD_DisplayString(col,(row++)*20,"PLNUM:");
				LCD_DisplayString(col,(row++)*20,"NS:");
				LCD_DisplayString(col,(row++)*20,"EW:");
				LCD_DisplayString(col,(row++)*20,"ALT:");
				LCD_DisplayString(col,(row++)*20,"Speed:");
				LCD_DisplayString(col,(row++)*20,"Time:");
				
				LCD_SetColor(LCD_WHITE);
				row=2;
				col=370;
				LCD_DisplayString(col,(row++)*20,"ACCEL:");
				LCD_DisplayString(col,(row++)*20,"GYRO:");
				LCD_DisplayString(col,(row++)*20,"MAG:");
				LCD_DisplayString(col,(row++)*20,"BAR:");
				LCD_SetColor(LCD_RED);
				row=2;
				LCD_FillCircle(col+70,(row++)*20+10,7);
				LCD_FillCircle(col+70,(row++)*20+10,7);
				LCD_FillCircle(col+70,(row++)*20+10,7);
				LCD_FillCircle(col+70,(row++)*20+10,7);
				Show_RC_GPS_Update(RC_gps);
				Show_RC_STATUS_Update(RC_status);
				Show_RC_SENSOR_Update(RC_sensor);
				Show_RC_Power_Update(RC_power);
				Show_Throttle_Update(INPUT);
				
		}
		else if(Page_Now==2)
		{
			row=0;
			col=20;
			LCD_SetColor(LCD_WHITE);
			LCD_DisplayString(col,(row++)*20,"PID Adjust");
			LCD_DisplayString(400,0,"Read PID");
			
			row=3;
			col=20;
			LCD_DisplayString(col,(row++)*20,"Parameter");
			
			row=3;
			LCD_DisplayString(180,(row)*20,"P");
			LCD_DisplayString(270,(row)*20,"I");
			LCD_DisplayString(360,(row)*20,"D");
			
			row=4;
			col=10;
			LCD_SetColor(LCD_BLUE);
			LCD_DisplayString(col,(row++)*20,"Roll_Angle:");
			LCD_DisplayString(col,(row++)*20,"Pitch_Angle:");
			LCD_DisplayString(col,(row++)*20,"Yaw_Angle:");
			LCD_DisplayString(col,(row++)*20,"Height_Dis:");
			LCD_SetColor(LCD_RED);
			LCD_DisplayString(col,(row++)*20,"Roll_Rate:");
			LCD_DisplayString(col,(row++)*20,"Pitch_Rate:");
			LCD_DisplayString(col,(row++)*20,"Yaw_Rate:");
			LCD_DisplayString(col,(row++)*20,"Height_Vel:");

			Show_PID_Log("Waiting To Read Drone PID Data ......");
			Show_PID_Log("Long Press Right Upper KEY To Read.");
			Show_RC_PID_Update();
			
			
		}
	}
	/**********************/
		if(Page_Now == 1)
		{   
		}
		else if(Page_Now==2)
		{
			
		}
}
void Show_NRFData_Update(NRF_TypeDef n)
{
	  LCD_SetColor(LCD_WHITE);
	  row = SP_NRF.strRow;
		col = SP_NRF.DataStartCol;
	  Show_DrawBar(SP_NRF.DirStartCol+2*10+5,(row++)*20+3,60,n.Signal-0.0005f,LCD_GREEN,LCD_GREEN,LCD_RED);
	  sprintf(strTemp,"%4d",n.TX_Cnt);
	  LCD_DisplayString(col,(row++)*20,strTemp);
	  sprintf(strTemp,"%4d",n.RX_Cnt);
	  LCD_DisplayString(col,(row++)*20,strTemp);
	  sprintf(strTemp,"%4d",n.MAX_cnt);
	  LCD_DisplayString(col,(row++)*20,strTemp);
}
void Show_Throttle_Update(Input_TypeDef Input)
{
	  s16 temp;
	  row = SP_Thro.strRow;
		col = SP_Thro.DataStartCol;
	  if(Input.Thro_High>1500) LCD_SetColor(LCD_RED);
	  else LCD_SetColor(LCD_GREEN);
		sprintf(strTemp,"%4d",Input.Thro_High);
	  LCD_DisplayString(col,(row++)*20,strTemp);
    LCD_SetColor(LCD_WHITE);
	  sprintf(strTemp,"%4d",Input.Thro_Yaw);
	  LCD_DisplayString(col,(row++)*20,strTemp);
	  sprintf(strTemp,"%4d",Input.Thro_Roll);
	  LCD_DisplayString(col,(row++)*20,strTemp);
	  sprintf(strTemp,"%4d",Input.Thro_Pitch);
	  LCD_DisplayString(col,(row++)*20,strTemp);
	 row = SP_Thro.strRow;
	 col=SP_Thro.DataStartCol+4*10+5;
	 Show_DrawBar(col,(row++)*20+3,120,(Input.Thro_High-1500)/500.0f,LCD_RED,LCD_GREEN,LCD_BLUE);
	 Show_DrawBar(col,(row++)*20+3,120,(Input.Thro_Yaw-1500)/500.0f,LCD_RED,LCD_GREEN,LCD_BLUE);
	 Show_DrawBar(col,(row++)*20+3,120,(Input.Thro_Roll-1500)/500.0f,LCD_RED,LCD_GREEN,LCD_BLUE);
	 Show_DrawBar(col,(row++)*20+3,120,(Input.Thro_Pitch-1500)/500.0f,LCD_RED,LCD_GREEN,LCD_BLUE);

}
void Show_RC_STATUS_Update(RC_STSTUS_TypeDef sta)
{
		 row = SP_Status.strRow;
	   col=  SP_Status.DataStartCol;
	   LCD_SetColor(LCD_WHITE);
     sprintf(strTemp,"%+6.1f",sta.roll);
		 LCD_DisplayString(col,(row++)*20,strTemp);
	   sprintf(strTemp,"%+6.1f",sta.pitch);
		 LCD_DisplayString(col,(row++)*20,strTemp);
	   sprintf(strTemp,"%+6.1f",sta.yaw);
		 LCD_DisplayString(col,(row++)*20,strTemp);
	   sprintf(strTemp,"%+7.2fm",sta.alt);
		 LCD_DisplayString(col-20,(row++)*20,strTemp);
	   sprintf(strTemp,"%+7.2fm",sta.alt_speed);
		 LCD_DisplayString(col-20,(row++)*20,strTemp);
		 sprintf(strTemp,"%3d",sta.mode);
		 LCD_DisplayString(col,(row++)*20,strTemp);
	
		 if(sta.flysta ==FlyStaLock)
			  LCD_DisplayString(col,(row++)*20,"Lock  ");
		 else if(sta.flysta==FlyStaRdy)
			  LCD_DisplayString(col,(row++)*20,"Ready ");
		 else if(sta.flysta==FlyStaUnlock)
			  LCD_DisplayString(col,(row++)*20,"Unlock");
		 else if(sta.flysta==FlyStaFlying)
			  LCD_DisplayString(col,(row++)*20,"Flying");
		 else if(sta.flysta==FlyStaPIDAdj)
			  LCD_DisplayString(col,(row++)*20,"PIDAdj");
		 else if(sta.flysta==FlyStaAngleErr)
			  LCD_DisplayString(col,(row++)*20,"AngErr");
		 
}
void Show_RC_SENSOR_Update(RC_Sensor_TypeDef s)
{
	row = SP_Sensor.strRow;
	col = SP_Sensor.DataStartCol;
	LCD_SetColor(LCD_WHITE);
  sprintf(strTemp,"%+6d",s.Accel_X);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Accel_Y);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Accel_Z);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Gyro_X);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Gyro_Y);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Gyro_Z);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Mag_X);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Mag_Y);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%+6d",s.Mag_Z);
	LCD_DisplayString(col,(row++)*20,strTemp);
}
void Show_RC_GPS_Update(RC_GPS_TypeDef p)
{
	row = SP_GPS.strRow;
	col= SP_GPS.DataStartCol;
	if(p.gpssta==0)
		LCD_SetColor(LCD_RED);
	else
		LCD_SetColor(LCD_WHITE);
	sprintf(strTemp,"%4d",p.gpssta);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%4d",p.svnum);//可见卫星数量
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%4d",p.posslnum);//用于定位的卫星数量
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%10.6f%0c",p.latitude,p.nshemi);
	LCD_DisplayString(col-20,(row++)*20,strTemp);
	sprintf(strTemp,"%10.6f%c",p.longitude,p.ewhemi);
	LCD_DisplayString(col-20,(row++)*20,strTemp);
	sprintf(strTemp,"%5.1fm",p.altitude);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%6.2fKm/h",p.speed);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%02d:%02d:%02d",p.hh%24,p.mm,p.ss);
	LCD_DisplayString(col,(row++)*20,strTemp);
}
void Show_RC_Power_Update(RC_POWER_TypeDef p)
{
	row = SP_Power.strRow;
	col=  SP_Power.DataStartCol;
	if(p.voltage<11.5f)
		LCD_SetColor(LCD_RED);
	else
		LCD_SetColor(LCD_WHITE);
	sprintf(strTemp,"%5.2fV",p.voltage);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%5.2fA",p.current);
	LCD_DisplayString(col,(row++)*20,strTemp);
}
/**********************/
void Show_RC_PID_Update(void)
{
	row=4;
	col=150;
	LCD_SetColor(LCD_BLUE);
	sprintf(strTemp,"%7.3f",RC_Roll_angle_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_angle_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_angle_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_dis_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	row=4;
	col=240;
	sprintf(strTemp,"%7.3f",RC_Roll_angle_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_angle_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_angle_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_dis_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	row=4;
	col=330;
	sprintf(strTemp,"%7.3f",RC_Roll_angle_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_angle_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_angle_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_dis_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	/***********************/
	row=8;
	col=110+40;
	LCD_SetColor(LCD_RED);
	sprintf(strTemp,"%7.3f",RC_Roll_rate_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_rate_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_rate_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_v_PID.P);
	LCD_DisplayString(col,(row++)*20,strTemp);
	row=8;
	col=200+40;
	sprintf(strTemp,"%7.3f",RC_Roll_rate_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_rate_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_rate_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_v_PID.I);
	LCD_DisplayString(col,(row++)*20,strTemp);
	row=8;
	col=290+40;
	sprintf(strTemp,"%7.3f",RC_Roll_rate_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Pitch_rate_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_Yaw_rate_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
	sprintf(strTemp,"%7.3f",RC_High_v_PID.D);
	LCD_DisplayString(col,(row++)*20,strTemp);
}


//另外添加油门条形指示
//temp已经归一化
void Show_DrawBar(u16 x,u16 y,u16 barlen,float temp,u32 postColor,u32 negaColor,u32 backColor)
{
	  float len;
	  len = (temp*barlen);
	  if( len>=0 ) LCD_SetColor(postColor);
	  else {LCD_SetColor(negaColor);len=-len;}//设定颜色
		
		LCD_FillRect(x,y,len,12);
		LCD_SetColor(backColor);
		LCD_FillRect(x+len,y,barlen-len,12);
}
//日志输出函数
void Show_PID_Log(u8* p)
{
	static u16 cnt=0;
//  va_list args;
//	const char *arg1;
//	va_start(args,format);
//	arg1 = va_arg(args,const char *);
//  va_end(args);
	cnt++;
	LCD_SetColor(LCD_WHITE);
	if(cnt>4)
	{
	  memcpy(LogBuff[0],LogBuff[1],48);
		memcpy(LogBuff[1],LogBuff[2],48);
		memcpy(LogBuff[2],LogBuff[3],48);
		sprintf(LogBuff[3],"#%d:%-48s",cnt,p);
		LCD_DisplayString(PID_Log_COL,(PID_Log_ROW+0)*20,LogBuff[0]);
		LCD_DisplayString(PID_Log_COL,(PID_Log_ROW+1)*20,LogBuff[1]);
		LCD_DisplayString(PID_Log_COL,(PID_Log_ROW+2)*20,LogBuff[2]);
		LCD_DisplayString(PID_Log_COL,(PID_Log_ROW+3)*20,LogBuff[3]);
	}
	else
	{
		sprintf(LogBuff[cnt-1],"#%d:%-48s",cnt,p);
    LCD_DisplayString(PID_Log_COL,(PID_Log_ROW+cnt-1)*20,LogBuff[cnt-1]);
	}
}
void Show_Page1_Log(u8* p)
{
	static u8 cnt=0;
	cnt++;
	LCD_SetColor(LCD_WHITE);
	sprintf(strTemp,"#%d:%-48s",cnt,p);
  LCD_DisplayString(0,(1)*20,strTemp);
}