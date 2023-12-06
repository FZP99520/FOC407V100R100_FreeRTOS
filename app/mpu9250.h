#ifndef _MPU9250_H
#define _MPU9250_H
#include "stm32f4xx.h"
typedef struct
{
	s16 ACCEL_X;
	s16 ACCEL_Y;
	s16 ACCEL_Z;
	s16 TEMP;
	s16 GYRO_X;
	s16 GYRO_Y;
	s16 GYRO_Z;
	s16 MAG_X;
	s16 MAG_Y;
	s16 MAG_Z;
}MPU9250;
extern float Pitch;
extern float Roll;
extern float Yaw;
extern MPU9250 mpu9250_data;
u8 MPU9250_Check(void);
u8 MPU9250_Init(void);
void MPU9250_Read(MPU9250* mpu);
u8 MPU9250_Read_INT_STA(void);
static void MPU9250_Write_Data(u8 slave_addr,u8 addr,u8 dat);
static u8   MPU9250_Read_Data(u8 slave_addr,u8 addr);
static void MPU9250_Read_Buff(u8 slave_addr,u8 addr,u8* buff,u8 len);

#define ACCEL_GYRO_SlaveAddress 0xD0  //从机地址
#define MAGNTOMETER_SlaveAddress 0x18  //从机地址

//INT_STATUS Bit
#define WOM_INT           0x40
#define FIFO_OVERFLOW_INT 0x10
#define FSYNC_INT         0x08
#define RAW_DATA_RDY_INT  0x01

//register map
#define SELF_TEST_X_GYRO  0x00
#define SELF_TEST_Y_GYRO  0x01
#define SELF_TEST_Z_GYRO  0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define XG_OFFSET_H       0x13
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F
#define FIFO_EN           0x23
#define INT_PIN_CFG       0x37
#define INT_ENABLE        0x38
#define INT_STATUS        0x3A
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B  //reset value 0x01
#define PWR_MGMT_2        0x6C
#define FIFO_COUNTH       0x72
#define FIFO_COUNTL       0x73
#define FIFO_R_W          0x74
#define WHO_AM_I          0x75  //reset value 0x68
#define XA_OFFSET_H       0x77
#define XA_OFFSET_L       0x78
#define YA_OFFSET_H       0x7A
#define YA_OFFSET_L       0x7B
#define ZA_OFFSET_H       0x7D
#define ZA_OFFSET_L       0x7E

//magnetometer register
#define WIA               0x00
#define INFO              0x01
#define STA1              0x02
#define HXL               0x03 
#define HXH               0x04 
#define HYL               0x05 
#define HYH               0x06 
#define HZL               0x07
#define HZH               0x08
#define STA2              0x09
#define CONTROL           0x0A
#define RESERVE           0x0B //实际是CONTROL2
#define ASTC              0x0C //self-test
#define TEST1             0x0D
#define TEST2             0x0E
#define I2CDIS            0x0F
#define ASAX              0x10  //X-axis sensitivity adjustment value
#define ASAY              0x11  //Y-axis sensitivity adjustment value
#define ASAZ              0x12  //Z-axis sensitivity adjustment value
#endif



