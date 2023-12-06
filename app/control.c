#include "control.h"
#include "arm_math.h"
#include "math.h"
#include "motor.h"
#include "pwm.h"
#include "mpu9250.h"
#include "mpu6050.h"
PID pid;
MOVEMENT move;
void Set_PID()
{
	pid.Balance_P=300;
	pid.Balance_D=1.0f;
	pid.Velocity_P=80;
	pid.Velocity_I=0.2f;
	pid.Turn_P=100;
	pid.Turn_D=1.0f;
}
void Control_DataHandle(MPU6050_Data* mpu_data,MOVEMENT move)
{
	float Angle;
	int balance_pwm,velocity_pwm,turn_pwm;
	Angle=Pitch;
	balance_pwm=Angle*pid.Balance_P + mpu_data->gyro_x*pid.Balance_D;
	//velocity_pwm
	//turn_pwm=mpu_data->gyro_z*pid->Turn_D;
  motor.pwm1=balance_pwm;
	motor.pwm2=balance_pwm;
	
}
void Set_PWM()
{
	if(motor.pwm1>0) 
	{
		ZF1_H;
		PWM1=motor.pwm1;
	}
	else 
	{
		ZF1_H;
		PWM1 = -motor.pwm1;
	}
	
	if(motor.pwm2>0) 
	{
		ZF2_H;
		PWM2=motor.pwm2;
	}
	else 
	{
		ZF2_H;
		PWM2 = -motor.pwm1;
	}
}

