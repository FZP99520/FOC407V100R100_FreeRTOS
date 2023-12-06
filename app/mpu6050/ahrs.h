#ifndef __AHRS_H
#define __AHRS_H

void MahonyAHRSupdateIMU(float gx, float gy, float gz,float ax,float ay,float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);

#endif
