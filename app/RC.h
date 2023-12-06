#ifndef _RC_H
#define _RC_H

#include "stm32f4xx.h"
#include "nrf24l01.h"
#include "adc.h"
#include "input_ctrl.h"
typedef struct
{
	u8 cmd_sta;
	u8 ACCEL;
	u8 GYRO;
	u8 MAG;
	u8 BAR;
	u8 Lock;
	u8 Unlock;
	u8 Read_pid;
	u8 Send_pid1;
	u8 Send_pid2;
	u8 Send_pid3;
	u8 Send_pid4;
	u8 Send_pid5;
	u8 Send_pid6;
}RC_CmdReqFlag_TypeDef; //命令请求结构体
typedef struct
{
	s16 Accel_X;
	s16 Accel_Y;
	s16 Accel_Z;
	s16 Gyro_X;
	s16 Gyro_Y;
	s16 Gyro_Z;
	s16 Mag_X;
	s16 Mag_Y;
	s16 Mag_Z;
}RC_Sensor_TypeDef;
typedef enum   
{
	FlyStaLock = 0,//锁定
	FlyStaRdy=1,//数据已准备
	FlyStaUnlock =2,//解锁，解锁后检查起飞角度，正常后才可以进入正常飞行模式
	FlyStaFlying=3,//正常模式，可以起飞
	FlyStaPIDAdj=4,//PID调整状态
	FlyStaAngleErr=5,//角度错误
}RC_Fly_Status_t;
typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float alt; //m
	float alt_speed;//m/s
	u8 mode;
	RC_Fly_Status_t flysta;
}RC_STSTUS_TypeDef;
typedef struct
{
	u8 gpssta;   //定位状态
	u8 svnum;    //可见卫星数
	u8 posslnum; //用于定位卫星数
	float latitude;	//纬度
	u8 nshemi;    //南北
	float longitude;//经度
	u8 ewhemi;    //东西
	float altitude;
	float speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时	 
	float speed_angle;    //航向角，放大10倍
	u8 hh;
	u8 mm;
	u8 ss;
}RC_GPS_TypeDef;
typedef struct
{
	float voltage;
	float current;
}RC_POWER_TypeDef;

typedef struct
{
	float P;
	float I;
	float D;
	float Error;
	float PreError;
	double Integ;
	float Deriv;
	float Output;
}PID_TypeDef;

extern RC_Sensor_TypeDef RC_sensor;
extern RC_STSTUS_TypeDef RC_status;
extern RC_GPS_TypeDef RC_gps;
extern RC_POWER_TypeDef RC_power;
extern RC_CmdReqFlag_TypeDef RC_CmdReqFlag;

extern PID_TypeDef RC_Pitch_angle_PID;
extern PID_TypeDef RC_Roll_angle_PID;
extern PID_TypeDef RC_Yaw_angle_PID;

extern PID_TypeDef RC_Pitch_rate_PID;
extern PID_TypeDef RC_Roll_rate_PID;
extern PID_TypeDef RC_Yaw_rate_PID;

extern PID_TypeDef RC_High_dis_PID;
extern PID_TypeDef RC_High_v_PID;

void RC_Send(u8* buff,u8 len);
void RC_Send_CMD_Handle(void);
void RC_Send_CMD1(u8 cmd);
void RC_Send_CMD2(u8 cmd);
void RC_Send_CNT(Input_TypeDef s);
void RC_Send_PID(u8 group,PID_TypeDef p1,PID_TypeDef p2,PID_TypeDef p3);

void RC_Anl_BUFF(u8* buff,u8 len);
void RC_Send_Handle(void);


#define CMD_ACCEL  0x01
#define CMD_GYRO   0x02
#define CMD_MAG    0x03
#define CMD_BAR    0x04
#define CMD_LOCK   0xA0
#define CMD_UNLOCK 0xA1

#define CMD_SUCCEED 0x01
#define CMD_FAIL   0xE1
#endif

