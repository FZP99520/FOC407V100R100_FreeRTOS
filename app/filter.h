#ifndef filter_H
#define filter_H

#include "stm32f4xx.h"
#define Max_Size_Wind   8
typedef struct
{
    float Num_Wind;
    uint8_t Index;
    float Sum;
    float Wind[Max_Size_Wind];
}ST_MoveAverageFilter_t;

typedef struct
{
    float f32Gain;
    float f32a1;
    float f32a2;
    float f32b1;
    float f32Xn_1;
    float f32Yn_1;
}ST_BiquadFilter_t;


float MoveAvarageFilter(ST_MoveAverageFilter_t* pstFilter,float data);
float Biquad_Filter(ST_BiquadFilter_t *pstBiquadFilter, float f32Input);


#endif

