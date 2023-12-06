#include "mpu6050.h"
#include "systick.h"
#include "printf.h"
#include "iic.h"
#include "math.h"
#include "ahrs.h"
#include "mpu9250.h"
MPU6050_Data mpu_data;
MPU_OFFSET mpu_offset;
//float Pitch,Roll,Yaw; 
void MPU6050_Init() //初始化
{
  MPU6050_Write_Data(PWR_MGMT_1, 0x00);
	MPU6050_Write_Data(SMPLRT_DIV, 0x04);//Sample Rate=Output Rate/(1+Sample_div(0x04))
	MPU6050_Write_Data(CONFIG, 0x03);
	MPU6050_Write_Data(GYRO_CONFIG, 0x10);//设置量程+-1000
	MPU6050_Write_Data(ACCEL_CONFIG, 0x08);//+-4g
	MPU6050_Write_Data(0x38,0x01);  //设置中断源，5ms产生一个中断
	//MPU6050_Get_offset();//检测三轴角速度偏差
}
void MPU6050_UpdateData(void)
{
	static float pitch=0.0f;
	static float roll=0.0f;
	float ka=1.0f;
//	mpu_data.accel_x=((MPU6050_Read_Data(ACCEL_XOUT_H)<<8) + MPU6050_Read_Data(ACCEL_XOUT_L));
//	mpu_data.accel_y=((MPU6050_Read_Data(ACCEL_YOUT_H)<<8) + MPU6050_Read_Data(ACCEL_YOUT_L));
//	mpu_data.accel_z=((MPU6050_Read_Data(ACCEL_ZOUT_H)<<8) + MPU6050_Read_Data(ACCEL_ZOUT_L));
//	
//	mpu_data.gyro_x =((MPU6050_Read_Data(GYRO_XOUT_H)<<8) + MPU6050_Read_Data(GYRO_XOUT_L));
//	mpu_data.gyro_y =((MPU6050_Read_Data(GYRO_YOUT_H)<<8) + MPU6050_Read_Data(GYRO_YOUT_L));
//	mpu_data.gyro_z =((MPU6050_Read_Data(GYRO_ZOUT_H)<<8) + MPU6050_Read_Data(GYRO_ZOUT_L));
	mpu_data.accel_x=MPU6050_ReadTwo_Byte(ACCEL_XOUT_H);
	mpu_data.accel_y=MPU6050_ReadTwo_Byte(ACCEL_YOUT_H);
	mpu_data.accel_z=MPU6050_ReadTwo_Byte(ACCEL_ZOUT_H);
	mpu_data.gyro_x=MPU6050_ReadTwo_Byte(GYRO_XOUT_H);
	mpu_data.gyro_y=MPU6050_ReadTwo_Byte(GYRO_YOUT_H);
	mpu_data.gyro_z=MPU6050_ReadTwo_Byte(GYRO_ZOUT_H);
	if(mpu_data.offset_ok==1)
	{
		mpu_data.accel_x = mpu_data.accel_x - mpu_offset.accel_x;
		mpu_data.accel_y = mpu_data.accel_y - mpu_offset.accel_y;
		mpu_data.accel_z = mpu_data.accel_z - mpu_offset.accel_z;
		
		mpu_data.gyro_x  = mpu_data.gyro_x - mpu_offset.gyro_x;
		mpu_data.gyro_y  = mpu_data.gyro_y - mpu_offset.gyro_y;
		mpu_data.gyro_z  = mpu_data.gyro_z - mpu_offset.gyro_z;
	}
	MahonyAHRSupdateIMU(  (short)mpu_data.gyro_x,
			                  (short)mpu_data.gyro_y,
												(short)mpu_data.gyro_z,
		                    (short)mpu_data.accel_x,
												(short)mpu_data.accel_y,
												(short)mpu_data.accel_z );
	mpu_data.update_ok=1;
	Roll=(1-ka)*roll+ka*Roll;
	Pitch=(1-ka)*pitch+ka*Pitch;//低通滤波
	pitch=Pitch;
	roll=Roll;
	return;
}
void MPU6050_Get_offset()
{
	u8 i;
	u8 times=50u;
	long int gyro_x=0,gyro_y=0,gyro_z=0,accel_x=0,accel_y=0;
	long int accel_z=0;
	if(mpu_data.offset_ok==0)
	{
	 for(i=0;i<times;i++)
	 { 
	  MPU6050_UpdateData();//update data
	  gyro_x +=(short)mpu_data.gyro_x;
	  gyro_y +=(short)mpu_data.gyro_y;
	  gyro_z +=(short)mpu_data.gyro_z;
		 
		accel_x+=(short)mpu_data.accel_x;
		accel_y+=(short)mpu_data.accel_y;
		accel_z+=(short)mpu_data.accel_z;
		Delay_ms(50);
	 }
	  mpu_offset.gyro_x=gyro_x/times;
	  mpu_offset.gyro_y=gyro_y/times;
	  mpu_offset.gyro_z=gyro_z/times;
	  mpu_offset.accel_x=accel_x/times;
	  mpu_offset.accel_y=accel_y/times;
	  mpu_offset.accel_z=(accel_z/times)-8192u;
	  mpu_data.offset_ok=1;
  }
	
}
void MPU6050_Write_Data(u8 addr,u8 dat) //写数据
{/*
	IIC_Start();
	IIC_Send_Byte(MPU_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(dat);
	IIC_Wait_Ack();
	IIC_Stop();*/
}
u8 MPU6050_Read_Data(u8 addr) //读取数据
{/*
	s16 temp;
	IIC_Start();
	IIC_Send_Byte(MPU_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(MPU_SlaveAddress+1);
	IIC_Wait_Ack();
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	return temp;*/
}
u16 MPU6050_ReadTwo_Byte(u8 addr)
{/*
	u8 tempH;
	u8 tempL;
	IIC_Start();
	IIC_Send_Byte(MPU_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(MPU_SlaveAddress+1);
	IIC_Wait_Ack();
	tempH=IIC_Read_Byte(1);
	tempL=IIC_Read_Byte(0);
	IIC_Stop();
  return (tempH<<8)+tempL;*/
}
