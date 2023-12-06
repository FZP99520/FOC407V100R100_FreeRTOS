#include "filter.h"

float MoveAvarageFilter(ST_MoveAverageFilter_t* pstFilter,float data)
{
    if((pstFilter->Num_Wind>Max_Size_Wind)||(0==pstFilter->Num_Wind))
    {
        pstFilter->Num_Wind=Max_Size_Wind;
    }
    if(pstFilter->Index>=pstFilter->Num_Wind)
    {
        pstFilter->Index=0;
    }   
    pstFilter->Sum-=pstFilter->Wind[pstFilter->Index];
    pstFilter->Wind[pstFilter->Index]=data;
    pstFilter->Sum+=pstFilter->Wind[pstFilter->Index];
    pstFilter->Index++;

    return pstFilter->Sum/pstFilter->Num_Wind;
}

float Biquad_Filter(ST_BiquadFilter_t *pstBiquadFilter, float f32Input)
{
    float f32Output = 0;
    f32Output = pstBiquadFilter->f32Gain*(pstBiquadFilter->f32a1*f32Input + pstBiquadFilter->f32a2*pstBiquadFilter->f32Xn_1) + \
                pstBiquadFilter->f32b1*pstBiquadFilter->f32Yn_1;
    pstBiquadFilter->f32Xn_1 = f32Input;
    pstBiquadFilter->f32Yn_1 = f32Output;

    return f32Output;
}

