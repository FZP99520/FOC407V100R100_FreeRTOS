#include "RC.h"
#include "adc.h"
#include "beep.h"
#include "display.h"
#include "ili9488.h"
#include "stdio.h"
RC_Sensor_TypeDef RC_sensor;
RC_STSTUS_TypeDef RC_status;
RC_GPS_TypeDef RC_gps;
RC_CmdReqFlag_TypeDef RC_CmdReqFlag;
RC_POWER_TypeDef RC_power;

PID_TypeDef RC_Pitch_angle_PID;
PID_TypeDef RC_Roll_angle_PID;
PID_TypeDef RC_Yaw_angle_PID;

PID_TypeDef RC_Pitch_rate_PID;
PID_TypeDef RC_Roll_rate_PID;
PID_TypeDef RC_Yaw_rate_PID;

PID_TypeDef RC_High_dis_PID;
PID_TypeDef RC_High_v_PID;

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
static u8 data_to_send[32];
u8 sum_pid[3]; //PID校验


void RC_Send(u8* buff,u8 len)
{
	 NRF24L01_Send_Data(buff,len);
}
/****************/
void RC_Send_Handle(void)
{
	 if(RC_CmdReqFlag.Unlock==1)
	 {
		  RC_Send_CMD1(CMD_UNLOCK);
		  if(RC_status.flysta == FlyStaUnlock ||RC_status.flysta == FlyStaFlying )
			{
			  RC_CmdReqFlag.Unlock=0;
				BeepOn;
				Delay_ms(300);
				BeepOff;
				Show_Page1_Log("The Drone is UNLOCK! Ready to Fly!");
			}
	 }
	 else if(RC_CmdReqFlag.Lock==1)
	 {
		 RC_Send_CMD1(CMD_LOCK);
		 if(RC_status.flysta == FlyStaLock ||RC_status.flysta == FlyStaRdy )
		 {
			 RC_CmdReqFlag.Lock=0;
			 Show_Page1_Log("The Drone is LOCK!");
			 BeepOn;
			 Delay_ms(300);
			 BeepOff;
		 }
	 }
   else if(RC_CmdReqFlag.cmd_sta == 1)
	 {
		  RC_CmdReqFlag.cmd_sta=0;
		  if(RC_status.flysta != FlyStaFlying)  
			{
				 if(RC_CmdReqFlag.ACCEL==1)        
	       {
		       RC_CmdReqFlag.ACCEL=0;
		       RC_Send_CMD1(CMD_ACCEL); //传感器校准命令
	       }
	       else if(RC_CmdReqFlag.GYRO ==1)   
				 {
					 RC_CmdReqFlag.GYRO=0;
					 RC_Send_CMD1(CMD_GYRO);  
				 }
	       else if(RC_CmdReqFlag.MAG==1)    
				 {
					 RC_CmdReqFlag.MAG=0;
					 RC_Send_CMD1(CMD_MAG);  
				 }
	       else if(RC_CmdReqFlag.BAR == 1)   
				 {
					 RC_CmdReqFlag.BAR=0;
					 RC_Send_CMD1(CMD_BAR);   
				 }
			}
	 }
	 else if(RC_CmdReqFlag.Read_pid==1)
	 {
		 RC_CmdReqFlag.Read_pid=0;
		 RC_Send_CMD2(0x01);
	 }
	 else if(RC_CmdReqFlag.Send_pid1)
	 {
		 RC_CmdReqFlag.Send_pid1=0;
		 RC_Send_PID(1,RC_Roll_rate_PID,RC_Pitch_rate_PID,RC_Yaw_rate_PID);
	 }
	 else if(RC_CmdReqFlag.Send_pid2)
	 {
		 RC_CmdReqFlag.Send_pid2=0;
		 RC_Send_PID(2,RC_Roll_angle_PID,RC_Pitch_angle_PID,RC_Yaw_angle_PID);
	 }
	 else if(RC_CmdReqFlag.Send_pid3)
	 {
		 RC_CmdReqFlag.Send_pid3=0;
		 RC_Send_PID(3,RC_High_v_PID,RC_High_dis_PID,RC_High_dis_PID);
	 }
	 else 
	   RC_Send_CNT(INPUT);
	
}
void RC_Send_CMD1(u8 cmd)//发送命令
{
	u8 i;
	u8 sum=0;
	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAF;
	data_to_send[2] = 0x01;//功能字节
	data_to_send[3] = 1;//Length
	data_to_send[4] = cmd;
	for(i=0;i<5;i++)
	   sum+=data_to_send[i];
	data_to_send[5]=sum;
	
	RC_Send(data_to_send,6);
}
void RC_Send_CMD2(u8 cmd)//发送命令
{
	u8 i;
	u8 sum=0;
	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAF;
	data_to_send[2] = 0x02;//功能字节
	data_to_send[3] = 1;//Length
	data_to_send[4] = cmd;
	for(i=0;i<5;i++)
	   sum+=data_to_send[i];
	data_to_send[5]=sum;
	RC_Send(data_to_send,6);
}
void RC_Send_CNT(Input_TypeDef s)
{
	u8 sum = 0;
	u8 cnt=0;
	vs16 temp;
	data_to_send[cnt++]= 0xAA;
	data_to_send[cnt++]= 0xAF;
	data_to_send[cnt++]= 0x03;
	data_to_send[cnt++]= 20;
	temp = s.Thro_High;
	data_to_send[cnt++]=BYTE1(temp);
	data_to_send[cnt++]=BYTE0(temp);
	temp = s.Thro_Yaw;
	data_to_send[cnt++]=BYTE1(temp);
	data_to_send[cnt++]=BYTE0(temp);
	temp = s.Thro_Roll;
	data_to_send[cnt++]=BYTE1(temp);
	data_to_send[cnt++]=BYTE0(temp);
	temp = s.Thro_Pitch;
	data_to_send[cnt++]=BYTE1(temp);
	data_to_send[cnt++]=BYTE0(temp);
	for(u8 i=0;i<cnt;i++)
		  sum += data_to_send[i];
	data_to_send[cnt++]=sum;
	
	RC_Send(data_to_send,cnt);
}
void RC_Send_PID(u8 group,PID_TypeDef p1,PID_TypeDef p2,PID_TypeDef p3)
{
	u8 i;
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	_temp = p1.P * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1.I  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1.D  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2.P  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2.I  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2.D * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3.P  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3.I  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3.D * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
		
	for(i=0;i<_cnt;i++)
		 sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	sum_pid[group-1]=sum;
	RC_Send(data_to_send, _cnt);
}







/******************************************************/
static void RC_Anl_STATUS(u8* buff,RC_STSTUS_TypeDef* s);
static void RC_Anl_GPS(u8* buff,RC_GPS_TypeDef* g);
static void RC_Anl_POWER(u8* buff,RC_POWER_TypeDef* p);
static void RC_Anl_SENSOR(u8* buff,RC_Sensor_TypeDef* p);
static void RC_Anl_MSG(u8* buff);
static void RC_Anl_PID(u8* buff);
static void RC_Anl_CHECK(u8* buff);
/****************************************************/
void RC_Anl_BUFF(u8* buff,u8 len)
{
	u8 temp;
	u8 sum = 0;
	u8 i;
	for(i=0;i<(len-1);i++)
		sum += *(buff+i);
	if(!(sum==*(buff+len-1)))		return;		//判断sum
	if(!(*(buff)==0xAA && *(buff+1)==0xAA))		return;		//判断帧头
	
	if(buff[0] ==0xAA && buff[1]==0xAA)
	{
		temp = buff[2];
		switch(temp)
		{
			case 0x01:{RC_Anl_STATUS(buff,&RC_status);break;}
			case 0x04:{RC_Anl_GPS(buff,&RC_gps);   break;}
			case 0x02:{RC_Anl_SENSOR(buff,&RC_sensor);break;}
			case 0x05:{RC_Anl_POWER(buff,&RC_power);break;}
			case 0xEE:{RC_Anl_MSG(buff);break;}
			case 0x10:{RC_Anl_PID(buff);break;}
			case 0x11:{RC_Anl_PID(buff);break;}
			case 0x12:{RC_Anl_PID(buff);break;}
			case 0xEF:{RC_Anl_CHECK(buff);break;}
			default:break;
		}
	}
}
void RC_Anl_STATUS(u8* buff,RC_STSTUS_TypeDef* s)
{
		if(buff[2]==0x01)
		{
			s->roll   = ((vs16)(buff[4]<<8 | buff[5]))*0.01f;
			s->pitch  = ((vs16)(buff[6]<<8 | buff[7]))*0.01f;
			s->yaw    = ((vs16)(buff[8]<<8 | buff[9]))*0.01f;
			s->alt    = ((vs32)(buff[10]<<24 | buff[11]<<16 | buff[12]<<8 | buff[13]))*0.01f;
			s->alt_speed=((vs16)(buff[14]<<8 | buff[15]))*0.01f;
			s->mode   = buff[16];
			s->flysta = buff[17];
		}
		if(Page_Now==1) Show_RC_SENSOR_Update(RC_sensor);  //更新显示传感器数据
}
void RC_Anl_GPS(u8* buff,RC_GPS_TypeDef* g)//GPS 信息
{
	u32 temp;
	if(buff[0]==0xAA && buff[1]==0xAA)
	{
		if(buff[2]==0x04)
		{
			g->gpssta   = buff[4];
			g->svnum    = buff[5];
			g->posslnum = buff[6];
			temp = (vu32)(buff[7]<<24 | buff[8]<<16 | buff[9]<<8 |buff[10]);
			g->latitude = temp/1000000.0f;
			g->nshemi   = buff[11];
	    temp = (vu32)(buff[12]<<24 | buff[13]<<16 | buff[14]<<8 |buff[15]);
			g->longitude=temp/1000000.0f;
			g->ewhemi   = buff[16];
			g->altitude  = ((s16)(buff[17]<<8 | buff[18]))/10.0f;
			g->speed     = ((s16)(buff[19]<<8 | buff[20]))/100.0f;
			g->hh = buff[21];
			g->mm = buff[22];
			g->ss = buff[23];
		}
		 if(Page_Now==1)Show_RC_GPS_Update(RC_gps);
	}
}
void RC_Anl_POWER(u8* buff,RC_POWER_TypeDef* p)
{
		if(buff[2]==0x05) 
		{
			p->voltage   = 0.01f* ((u16)(buff[4]<<8 | buff[5]));
			p->current = 0.01f*((u16)(buff[6]<<8 | buff[7]));
			if(Page_Now==1)Show_RC_Power_Update(RC_power);
		}
}
void RC_Anl_SENSOR(u8* buff,RC_Sensor_TypeDef* p)
{
		if(buff[2]==0x02) //表明是传感器数据
		{
			p->Accel_X = (s16)((buff[4]<<8) + buff[5]);
			p->Accel_Y = (s16)((buff[6]<<8) + buff[7]);
			p->Accel_Z = (s16)((buff[8]<<8) + buff[9]);
			p->Gyro_X  = (s16)((buff[10]<<8) + buff[11]);
			p->Gyro_Y  = (s16)((buff[12]<<8) + buff[13]);
			p->Gyro_Z  = (s16)((buff[14]<<8) + buff[15]);
			p->Mag_X   = (s16)((buff[16]<<8) + buff[17]);
			p->Mag_Y   = (s16)((buff[18]<<8) + buff[19]);
			p->Mag_Z   = (s16)((buff[20]<<8) + buff[21]);
		}
		if(Page_Now==1)Show_RC_STATUS_Update(RC_status); //更新显示飞行器状态数据
}
void RC_Anl_MSG(u8* buff)
{
	u8 id;
	u8 result;
	if(buff[2]==0xEE)
	{
		id=buff[4];
		result = buff[5];
		if(result == CMD_SUCCEED)
		{
		if(id==CMD_ACCEL||id==CMD_GYRO||id==CMD_MAG||id==CMD_BAR||id==CMD_UNLOCK||id==CMD_LOCK)
		{
			RC_CmdReqFlag.cmd_sta=0;
			if(id==CMD_ACCEL) 
			{
				RC_CmdReqFlag.ACCEL=0;
				LCD_SetColor(LCD_GREEN);
				LCD_FillCircle(440,2*20+10,7);
				if(Page_Now==1)Show_Page1_Log("MPU6050 acceleration calibrated!");
			}
			else if(id==CMD_GYRO) 
			{
				RC_CmdReqFlag.GYRO=0;
				LCD_SetColor(LCD_GREEN);
				LCD_FillCircle(440,3*20+10,7);
				if(Page_Now==1)Show_Page1_Log("MPU6050 gyroscope calibrated!");
			}
			else if(id==CMD_MAG) 
			{
				RC_CmdReqFlag.MAG=0;
				LCD_SetColor(LCD_GREEN);
				LCD_FillCircle(440,4*20+10,7);
				if(Page_Now==1)Show_Page1_Log("MPU6050 QMC5883 calibrated!");
			}
			else if(id==CMD_BAR) 
			{
				RC_CmdReqFlag.BAR=0;
				LCD_SetColor(LCD_GREEN);
				LCD_FillCircle(440,5*20+10,7);
				if(Page_Now==1)Show_Page1_Log("MS5611 calibrated!");
			}
//			else if(id==CMD_UNLOCK) 
//			{
//				RC_CmdReqFlag.Unlock=0;
//			}
//			else if(id==CMD_LOCK) RC_CmdReqFlag.Lock=0;
//			else;
			Beep_times(1);
		}
	}
	}
}
void RC_Anl_PID(u8* buff)
{
	char temp[10];
	if(buff[2]==0x10) //表明数据是PID1
	{
		RC_Roll_rate_PID.P 	= 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
    RC_Roll_rate_PID.I 	= 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
    RC_Roll_rate_PID.D 	= 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
    RC_Pitch_rate_PID.P 	= 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
    RC_Pitch_rate_PID.I 	= 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
    RC_Pitch_rate_PID.D 	= 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
    RC_Yaw_rate_PID.P	  = 0.001f*( (vs16)(*(buff+16)<<8)|*(buff+17) );
    RC_Yaw_rate_PID.I 	  = 0.001f*( (vs16)(*(buff+18)<<8)|*(buff+19) );
    RC_Yaw_rate_PID.D 	  = 0.001f*( (vs16)(*(buff+20)<<8)|*(buff+21) );
		Show_RC_PID_Update();
	}
	else if(*(buff+2)==0X11)								//PID2
   {
			  RC_Roll_angle_PID.P  = 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
        RC_Roll_angle_PID.I  = 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
        RC_Roll_angle_PID.D  = 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
        RC_Pitch_angle_PID.P = 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
        RC_Pitch_angle_PID.I = 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
        RC_Pitch_angle_PID.D = 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
        RC_Yaw_angle_PID.P 	= 0.001f*( (vs16)(*(buff+16)<<8)|*(buff+17) );
        RC_Yaw_angle_PID.I 	= 0.001f*( (vs16)(*(buff+18)<<8)|*(buff+19) );
        RC_Yaw_angle_PID.D 	= 0.001f*( (vs16)(*(buff+20)<<8)|*(buff+21) );
		 Show_RC_PID_Update();
	}
	 else if(*(buff+2)==0X12)								//PID3
   {	
        RC_High_v_PID.P  = 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
        RC_High_v_PID.I  = 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
        RC_High_v_PID.D  = 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
        RC_High_dis_PID.P    = 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
        RC_High_dis_PID.I    = 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
        RC_High_dis_PID.D    = 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
		 Show_RC_PID_Update();
		 Show_PID_Log("Read Drone PID Data Finished !");
		 LCD_SetColor(LCD_YELLOW);
	   sprintf(temp,"%7.3f",*PID_Point);
	   LCD_DisplayString(150+(PID_C_Index-1)*90,(4+PID_R_Index-1)*20,temp);
	 }
	 else;
}
void RC_Anl_CHECK(u8* buff)
{
	if(buff[2]==0xEF)
	{
		if(buff[4]==0x10)//表明是PID1
		{
			if(sum_pid[0]==buff[5]) 
			{
				
				Show_PID_Log("PID1 Update Succeed!");
			}
			else Show_PID_Log("PID1 sum Error!");
		}
		if(buff[4]==0x11)//表明是PID2
		{
			if(sum_pid[1]==buff[5]) 
			{
				Show_PID_Log("PID2 Update Succeed!");
			}
			else Show_PID_Log("PID2 sum Error!");
		}
		if(buff[4]==0x12)//表明是PID3
		{
			if(sum_pid[2]==buff[5]) 
			{
				Show_PID_Log("PID3 Update Succeed!");
			}
			else Show_PID_Log("PID3 sum Error!");
		}
	}
}
