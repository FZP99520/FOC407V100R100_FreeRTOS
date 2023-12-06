#include "mpu9250.h"
#include "iic.h"
MPU9250 mpu9250_data;
float Pitch;
float Roll;
float Yaw;
u8 MPU9250_Check(void)
{
    u8 temp;
    temp=MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,WHO_AM_I);//ACCEL and GYRO
    if(temp!=0x71) return 0;
    temp=MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,WIA); //AK8963
    if(temp!=0x48) return 0;
    return 1;
}
u8 MPU9250_Init(void)
{
    u8 res;
    MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,INT_PIN_CFG,0x02);//禁用内部IIC主模式，AK8963连接到总线上
    res=MPU9250_Check();
    if(res)
    {
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,SMPLRT_DIV,0x04);
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,CONFIG,0x02);
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,GYRO_CONFIG,0x1B);//2000dps
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,ACCEL_CONFIG,0x08);//+-4G
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,ACCEL_CONFIG2,0x07);//4.8ms延时，低通滤波
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,INT_PIN_CFG,0x82);
        MPU9250_Write_Data(ACCEL_GYRO_SlaveAddress,INT_ENABLE,0x11);//FIFO Overflow,Data Ready int
        return 1;
    }
    else 
        return 0;
}
u8 MPU9250_Read_INT_STA(void)
{
	u8 sta;
	sta=MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,INT_STATUS);
	return sta;
}
void MPU9250_Read(MPU9250* mpu)
{
	mpu->ACCEL_X=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_XOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_XOUT_L);
	mpu->ACCEL_Y=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_YOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_YOUT_L);
	mpu->ACCEL_Z=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_ZOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,ACCEL_ZOUT_L);
	
	mpu->GYRO_X=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_XOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_XOUT_L);
	mpu->GYRO_Y=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_YOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_YOUT_L);
	mpu->GYRO_Z=(s16)MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_ZOUT_H)<<8\
	             |MPU9250_Read_Data(ACCEL_GYRO_SlaveAddress,GYRO_ZOUT_L);
	
	mpu->MAG_X=(s16)MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HXH)<<8|\
	                MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HXL);
	mpu->MAG_Y=(s16)MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HYH)<<8|\
	                MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HYL);
	mpu->MAG_Z=(s16)MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HZH)<<8|\
	                MPU9250_Read_Data(MAGNTOMETER_SlaveAddress,HZL);
	
}
static void MPU9250_Write_Data(u8 slave_addr,u8 addr,u8 data)
{/*
	IIC_Start();
	IIC_Send_Byte(slave_addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();*/
}
static u8 MPU9250_Read_Data(u8 slave_addr,u8 addr) //读取数据
{/*
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(slave_addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(slave_addr+1);
	IIC_Wait_Ack();
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	return temp;*/
}
static void MPU9250_Read_Buff(u8 slave_addr,u8 addr,u8* buff,u8 len)
{/*
	u8 count;
	IIC_Start();
	IIC_Send_Byte(slave_addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(slave_addr+1);
	IIC_Wait_Ack();
	for(count=0;count<len-1;count++)
	{
		buff[count]=IIC_Read_Byte(1);
	}
	buff[count]=IIC_Read_Byte(0);
	IIC_Stop();*/
}
