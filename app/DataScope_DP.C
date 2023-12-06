#include "DataScope_DP.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "systick.h"
#include "foc.h"
#include "pwm.h"
#include "adc.h"

#include "arm_math.h"

TaskHandle_t hDataScope_Task = NULL;

unsigned char DataScope_OutPut_Buffer[42] = {0};       //串口发送缓冲区

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


void DataScope_SendPwmInfo(u16 u16PhaseA,u16 u16PhaseB,u16 u16PhaseC)
{
    u8 u8Cnt = 0;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xF1;
    DataScope_OutPut_Buffer[u8Cnt++] = 0x6;
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(u16PhaseA);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(u16PhaseA);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(u16PhaseB);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(u16PhaseB);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(u16PhaseC);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(u16PhaseC);

    u8 u8Sum = 0;
    u8 u8Temp = 0;
    for(u8Temp=0;u8Temp<u8Cnt;u8Temp++)
    {
        u8Sum+=DataScope_OutPut_Buffer[u8Temp];
    }
    DataScope_OutPut_Buffer[u8Cnt++] = u8Sum;

    DataScope_SendData(DataScope_OutPut_Buffer,u8Cnt);
}

void DataScope_SendDuty(float f32ta,float f32tb,float f32tc)
{
    u8 u8Cnt = 0;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xF2;
    DataScope_OutPut_Buffer[u8Cnt++] = 12;
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32ta);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32ta);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32ta);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32ta);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32tb);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32tb);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32tb);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32tb);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32tc);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32tc);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32tc);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32tc);

    u8 u8Sum = 0;
    u8 u8Temp = 0;
    for(u8Temp=0;u8Temp<u8Cnt;u8Temp++)
    {
        u8Sum+=DataScope_OutPut_Buffer[u8Temp];
    }
    DataScope_OutPut_Buffer[u8Cnt++] = u8Sum;

    DataScope_SendData(DataScope_OutPut_Buffer,u8Cnt);
}

void DataScope_SendAngleInfo(float f32Data1, float f32Data2,float f32Data3, float f32Data4,  float f32Data5, float f32Data6)
{
    u8 u8Cnt = 0;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xAA;
    DataScope_OutPut_Buffer[u8Cnt++] = 0xF3;
    DataScope_OutPut_Buffer[u8Cnt++] = 0;
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data1);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data1);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data1);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data1);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data2);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data2);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data2);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data2);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data3);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data3);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data3);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data3);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data4);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data4);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data4);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data4);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data5);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data5);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data5);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data5);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE3(f32Data6);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE2(f32Data6);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE1(f32Data6);
    DataScope_OutPut_Buffer[u8Cnt++] = BYTE0(f32Data6);

    u8 u8Sum = 0;
    u8 u8Temp = 0;
    DataScope_OutPut_Buffer[3] = u8Cnt - 4;
    for(u8Temp=0;u8Temp<u8Cnt;u8Temp++)
    {
        u8Sum+=DataScope_OutPut_Buffer[u8Temp];
    }
    DataScope_OutPut_Buffer[u8Cnt++] = u8Sum;

    DataScope_SendData(DataScope_OutPut_Buffer,u8Cnt);
}



void DataScope_Task(void *pvParameters)
{
    u8 u8Count = 0;

    float f32theta = 0;
    float f32SinVal = 0;
    float f32CosVal = 0;
    float f32FreqHz = 0.5f;
    float f32Tcnt = 0;
    float f32Ts = 50;//unit: ms

    float f32Voltage = 0;
    u16   u16Voltage = 0;
    float f32AdcVoltage = 0;
    u16 u16DacValue = 0;

    while(pdTRUE)
    {
        arm_sin_cos_f32(360*f32FreqHz*f32Tcnt, &f32SinVal, &f32CosVal);
        u16Voltage = (f32CosVal+1.0f)/2.0f*3300;
        DAC1_SetVoltage(u16Voltage);

        //u16DacValue = (f32SinVal+1.0f)/2.0f*4095;
        //DAC1_SetValue(u16DacValue);

        f32Tcnt +=f32Ts/1000;
        f32AdcVoltage = (float)stAdcConv.pu16AdcConvVal[0]*3.3f/4096;

        DataScope_Get_Channel_Data(u16Voltage/1000.0f, E_DATA_SCOPE_CHANNEL_1);
        DataScope_Get_Channel_Data(f32AdcVoltage, E_DATA_SCOPE_CHANNEL_2);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_3);
        DataScope_Get_Channel_Data(0,E_DATA_SCOPE_CHANNEL_4);
        DataScope_Get_Channel_Data(0,E_DATA_SCOPE_CHANNEL_5);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_6);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_7);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_8);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_9);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_10);
        u8Count = DataScope_Data_Generate(2);
        DataScope_SendData(DataScope_OutPut_Buffer,u8Count);

        FR_OS_DelayMs(f32Ts);
    }
}

void DataScope_Send(void)
{
    u8 Send_Count;
    float i;
    static float filter=0;
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
    RNG_Cmd(ENABLE);
    for(i=0;i<100;i++)
    {
        DataScope_Get_Channel_Data(10*arm_sin_f32(i), E_DATA_SCOPE_CHANNEL_1);
        DataScope_Get_Channel_Data(10*arm_cos_f32(i), E_DATA_SCOPE_CHANNEL_2);
        DataScope_Get_Channel_Data(10*arm_cos_f32(i)+10*arm_sin_f32(i), E_DATA_SCOPE_CHANNEL_3); 
        DataScope_Get_Channel_Data(0,E_DATA_SCOPE_CHANNEL_4);   
        DataScope_Get_Channel_Data(filter,E_DATA_SCOPE_CHANNEL_5);
        DataScope_Get_Channel_Data(0 , E_DATA_SCOPE_CHANNEL_6);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_7);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_8); 
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_9);  
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_10);
        Send_Count = DataScope_Data_Generate(5);
        DataScope_SendData(DataScope_OutPut_Buffer,Send_Count);
        //Delay_ms(20);
    }
}



//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        switch (Channel)
        {
            case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
            case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
            case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
            case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
            case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
            case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
            case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
            case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
            case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
            case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
        }
  }
}


//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        DataScope_OutPut_Buffer[0] = '$';  //帧头
        switch(Channel_Number)   
        { 
            case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
            case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
            case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
            case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
            case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
            case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
            case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
            case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
            case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
            case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
        }
    }

    return 0;
}




