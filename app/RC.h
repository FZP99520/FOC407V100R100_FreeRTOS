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
}RC_CmdReqFlag_TypeDef; //��������ṹ��
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
	FlyStaLock = 0,//����
	FlyStaRdy=1,//������׼��
	FlyStaUnlock =2,//����������������ɽǶȣ�������ſ��Խ�����������ģʽ
	FlyStaFlying=3,//����ģʽ���������
	FlyStaPIDAdj=4,//PID����״̬
	FlyStaAngleErr=5,//�Ƕȴ���
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
	u8 gpssta;   //��λ״̬
	u8 svnum;    //�ɼ�������
	u8 posslnum; //���ڶ�λ������
	float latitude;	//γ��
	u8 nshemi;    //�ϱ�
	float longitude;//����
	u8 ewhemi;    //����
	float altitude;
	float speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ	 
	float speed_angle;    //����ǣ��Ŵ�10��
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

