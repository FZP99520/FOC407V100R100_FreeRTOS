#include "ahrs.h"
#include "math.h"
//#include "mpu6050.h"
#include "mpu9250.h"
#define GYRO_CALIBRATION_COFF 0.030517f		// 该系数和MPU6050中的量程增益对应
#define PI 3.141592f
#define sampleT  0.005f //5ms
#define twoKi 0.01f //0.008
#define twoKp 2*10

float Yaw_Gyro,Pitch_Gyro,Roll_Gyro;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

//具体的解释可以看 https://blog.csdn.net/nemol1990/article/details/21870197
//gx gy gz 单位为rad/s 加速度计和磁力计单位不影响（函数里进行了归一化）
extern float Mag_Yaw; //用于互补计算出偏航角
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {  
    float recipNorm;  
    float s0, s1, s2, s3;  
    float qDot1, qDot2, qDot3, qDot4;  
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;  
    float beta=0.08;              //梯度下降的步长 高速运动下增大
    float gz_deg=0;        //单位度/s 用于互补滤波
    gz_deg=gz;
    gz_deg=gz_deg*GYRO_CALIBRATION_COFF;  
    gx = gx *GYRO_CALIBRATION_COFF * PI / 180;	// 转换为弧度制
    gy = gy *GYRO_CALIBRATION_COFF * PI / 180;
    gz = gz *GYRO_CALIBRATION_COFF * PI / 180;
    
    // Rate of change of quaternion from gyroscope  
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);  
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);  
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);  
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);  
  
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)  
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {  
  
        // Normalise accelerometer measurement  
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
        ax *= recipNorm;  
        ay *= recipNorm;  
        az *= recipNorm;     
  
        // Auxiliary variables to avoid repeated arithmetic  
        _2q0 = 2.0f * q0;  
        _2q1 = 2.0f * q1;  
        _2q2 = 2.0f * q2;  
        _2q3 = 2.0f * q3;  
        _4q0 = 4.0f * q0;  
        _4q1 = 4.0f * q1;  
        _4q2 = 4.0f * q2;  
        _8q1 = 8.0f * q1;  
        _8q2 = 8.0f * q2;  
        q0q0 = q0 * q0;  
        q1q1 = q1 * q1;  
        q2q2 = q2 * q2;  
        q3q3 = q3 * q3;  
  
        // Gradient decent algorithm corrective step  
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;  
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;  
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;  
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;  
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude  
        s0 *= recipNorm;  
        s1 *= recipNorm;  
        s2 *= recipNorm;  
        s3 *= recipNorm;  
  
        // Apply feedback step  
        qDot1 -= beta * s0;  
        qDot2 -= beta * s1;  
        qDot3 -= beta * s2;  
        qDot4 -= beta * s3;  
    }  
  
    // Integrate rate of change of quaternion to yield quaternion  
    q0 += qDot1 * sampleT;  
    q1 += qDot2 * sampleT;  
    q2 += qDot3 * sampleT;  
    q3 += qDot4 * sampleT;  
  
    // Normalise quaternion  
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  
    q0 *= recipNorm;  
    q1 *= recipNorm;  
    q2 *= recipNorm;  
    q3 *= recipNorm;  
    
    Roll = asin(-2 * q1*q3 + 2 * q0*q2) * 180 / PI; 													
    Pitch = atan2(2 * q2*q3 + 2 * q0*q1, -2 *q1q1 - 2 *q2q2 + 1) * 180 / PI; 
    Yaw+=gz_deg*sampleT;
} 
      
void MahonyAHRSupdateIMU(float gx,float gy,float gz,float ax,float ay,float az)
{
    float recipNorm;  
    float q0q0, q0q1, q0q2,  q1q1,  q1q3, q2q2, q2q3, q3q3 ,q1q2, q0q3;
    float halfvx, halfvy, halfvz;  
    float halfex=0.0f, halfey=0.0f, halfez=0.0f;  
    float exInt=0.0f,eyInt=0.0f,ezInt=0.0f;
    float qa, qb, qc;  
    
    Yaw_Gyro=gz*GYRO_CALIBRATION_COFF;    // unit: °/s
    Pitch_Gyro=gx*GYRO_CALIBRATION_COFF;
    Roll_Gyro=gy*GYRO_CALIBRATION_COFF;
    
    gx = gx *GYRO_CALIBRATION_COFF * PI / 180;	// 转换为弧度制
    gy = gy *GYRO_CALIBRATION_COFF * PI / 180;
    gz = gz *GYRO_CALIBRATION_COFF * PI / 180;
   
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
        ax *= recipNorm;  
        ay *= recipNorm;  
        az *= recipNorm;       
        
        q0q0 = q0 * q0;  
        q0q1 = q0 * q1;  
        q0q2 = q0 * q2;  
        q0q3 = q0 * q3;  
        q1q1 = q1 * q1;  
        q1q2 = q1 * q2;  
        q1q3 = q1 * q3;  
        q2q2 = q2 * q2;  
        q2q3 = q2 * q3;  
        q3q3 = q3 * q3;
        
        halfvx = q1q3 - q0q2;  
        halfvy = q0q1 + q2q3;  
        halfvz = q0q0 - 0.5f + q3q3;  
        
        halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
        
        if(twoKi > 0.0f) 
        {  
            exInt += twoKi * halfex * sampleT;    // integral error scaled by Ki  
            eyInt += twoKi * halfey * sampleT;  
            ezInt += twoKi * halfez * sampleT;  
            gx += exInt;  // apply integral feedback  
            gy += eyInt;  
            gz += ezInt;  
        }  
        else
        {  
            exInt = 0.0f; // prevent integral windup  
            eyInt = 0.0f;  
            ezInt = 0.0f;  
        }
        
        gx += twoKp * halfex;  
        gy += twoKp * halfey;  
        gz += twoKp * halfez; 
    }    
    gx *= 0.5f*sampleT;     // pre-multiply common factors  
    gy *= 0.5f*sampleT;  
    gz *= 0.5f*sampleT;  
    qa = q0;  
    qb = q1;  
    qc = q2;  
    q0 += (-qb * gx - qc * gy - q3 * gz);    //根据四元数微分方程更新四元数（用一阶龙格库塔求解）
    q1 += (qa * gx + qc * gz - q3 * gy);  
    q2 += (qa * gy - qb * gz + q3 * gx);  
    q3 += (qa * gz + qb * gy - qc * gx);   
          
    // Normalise quaternion  
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  
    q0 *= recipNorm;  
    q1 *= recipNorm;  
    q2 *= recipNorm;  
    q3 *= recipNorm;  
    
    Yaw = atan2(2 * q1q2 + 2 *q0q3, 1-2*q2q2-2*q3q3) * 180 / PI;	// Yaw
	  Roll = asin(-2 * q1q3 + 2 * q0q2) * 180 / PI; 													
    Pitch = atan2(2 * q2q3 + 2 * q0q1, -2 *q1q1 - 2 *q2q2 + 1) * 180 / PI; 	
    
    /*Yaw_temp+=Yaw_Gyro*sampleT;
    if((Mag_Yaw>90 && Yaw_temp<-90)|| (Mag_Yaw<-90 && Yaw_temp>90))
    {
         Yaw_temp = -Yaw_temp * 0.98f + Mag_Yaw * 0.02f;
    }
	else Yaw_temp = Yaw_temp * 0.98f + Mag_Yaw * 0.02f;

    if(Yaw_temp<0)  Yaw=Yaw_temp+360;
    else  Yaw=Yaw_temp;*/
    
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{  
    float recipNorm;  
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;    
    float hx, hy, bx, bz;  
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;  
    float halfex=0, halfey=0, halfez=0;  
    float exInt=0,eyInt=0,ezInt=0;
    float qa, qb, qc;  
  
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)  
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {  
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);  
        return;  
    }  
  
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)  
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {  
        
        Yaw_Gyro=gz*GYRO_CALIBRATION_COFF;    // unit: °/s
        Pitch_Gyro=gx*GYRO_CALIBRATION_COFF;
        Roll_Gyro=gy*GYRO_CALIBRATION_COFF;
        
       // gx = gx *GYRO_CALIBRATION_COFF * PI / 180;	// 转换为弧度制
        //gy = gy *GYRO_CALIBRATION_COFF * PI / 180;
       // gz = gz *GYRO_CALIBRATION_COFF * PI / 180;
        gx = Pitch_Gyro * PI / 180;	// 转换为弧度制
        gy = Roll_Gyro  * PI / 180;
        gz = Yaw_Gyro   * PI / 180;
        
        // Normalise accelerometer measurement  
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
        ax *= recipNorm;  
        ay *= recipNorm;  
        az *= recipNorm;       
  
        // Normalise magnetometer measurement  
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);  
        mx *= recipNorm;  
        my *= recipNorm;  
        mz *= recipNorm;     
  
        // Auxiliary variables to avoid repeated arithmetic  
        q0q0 = q0 * q0;  
        q0q1 = q0 * q1;  
        q0q2 = q0 * q2;  
        q0q3 = q0 * q3;  
        q1q1 = q1 * q1;  
        q1q2 = q1 * q2;  
        q1q3 = q1 * q3;  
        q2q2 = q2 * q2;  
        q2q3 = q2 * q3;  
        q3q3 = q3 * q3;     
  
        // Reference direction of Earth's magnetic field  
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));  
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));  
        bx = sqrt(hx * hx + hy * hy);  
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));  
  
        // Estimated direction of gravity and magnetic field  
        halfvx = q1q3 - q0q2;  
        halfvy = q0q1 + q2q3;  
        halfvz = q0q0 - 0.5f + q3q3;  
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);  
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);  
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);    
      
        // Error is sum of cross product between estimated direction and measured direction of field vectors  
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);  
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);  
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx); 
  
        // Compute and apply integral feedback if enabled  
        if(twoKi > 0.0f) {  
            exInt += twoKi * halfex * sampleT;    // integral error scaled by Ki  
            eyInt += twoKi * halfey * sampleT;  
            ezInt += twoKi * halfez * sampleT;  
            gx += exInt;  // apply integral feedback  
            gy += eyInt;  
            gz += ezInt;  
        }  
        else {  
            exInt = 0.0f; // prevent integral windup  
            eyInt = 0.0f;  
            ezInt = 0.0f;  
        }  
  
        // Apply proportional feedback  
        gx += twoKp * halfex;  
        gy += twoKp * halfey;  
        gz += twoKp * halfez;  
    }  
      
    // Integrate rate of change of quaternion  
    gx *= 0.5f*sampleT;     // pre-multiply common factors  
    gy *= 0.5f*sampleT;  
    gz *= 0.5f*sampleT;  
    qa = q0;  
    qb = q1;  
    qc = q2;  
    q0 += (-qb * gx - qc * gy - q3 * gz);    //根据四元数微分方程更新四元数（用一阶龙格库塔求解）
    q1 += (qa * gx + qc * gz - q3 * gy);  
    q2 += (qa * gy - qb * gz + q3 * gx);  
    q3 += (qa * gz + qb * gy - qc * gx);   
      
    // Normalise quaternion  
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  
    q0 *= recipNorm;  
    q1 *= recipNorm;  
    q2 *= recipNorm;  
    q3 *= recipNorm;  
    
    Yaw = atan2(2 * q1q2 + 2 *q0q3, 1-2*q2q2-2*q3q3) * 180 / PI + 180;	// Yaw
	Roll = asin(-2 * q1q3 + 2 * q0q2) * 180 / PI; 													
    Pitch = atan2(2 * q2q3 + 2 * q0q1, -2 *q1q1 - 2 *q2q2 + 1) * 180 / PI; 	
}  

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
	
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
	
  return y;
}
