#ifndef __MPU6050_H
#define __MPU6050_H
#define devAddr  0xD0
#include "stm32f4xx.h"
typedef struct
{
	u8 update_ok;
	u8 offset_ok;
	u16 accel_x;
	u16 accel_y;
	u16 accel_z;
	u16 temp;
	u16 gyro_x;
	u16 gyro_y;
	u16 gyro_z;
}MPU6050_Data;
typedef struct
{
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
}MPU_OFFSET;
void MPU6050_Init(void);//�����ǳ�ʼ����ͬʱ��ʼ��IIC
void MPU6050_Write_Data(u8 addr,u8 dat);
u8 MPU6050_Read_Data(u8 addr);
void MPU6050_UpdateData(void);
s16 MPU6050_GetTemp(void); // �¶ȶ�ȡ����
void MPU6050_Get_offset(void);
// ����MPU6050�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			  0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	MPU_SlaveAddress	0xd0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

extern int16_t Gx_offset,Gy_offset,Gz_offset;
extern float Acc1G_Values;
//extern float Pitch,Roll,Yaw; 
extern MPU6050_Data mpu_data;
//���ⲿ���õ�API
void MPU6050_Init(void);
uint8_t Read_DMP(float* Pitch,float* Roll,float* Yaw);
int Read_Temperature(void);
u16 MPU6050_ReadTwo_Byte(u8 addr);
#endif
