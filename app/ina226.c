#include "ina226.h"
#include "iic.h"
#include "log.h"
#include "DataScope_DP.h"
#include "oled.h"

#define INA_SLAVE_ADDR              0x88
#define INA_REG_CONFIG              0x00
#define INA_REG_SHUNT_VAL           0x01
#define INA_REG_VBUS_VAL            0x02
#define INA_REG_POWER_VAL           0x03
#define INA_REG_CURRENT_VAL         0x04
#define INA_REG_CALIBRATION         0x05
#define INA_REG_MASK_ENABLE         0x06
#define INA_REG_ALERT_LIMIT         0x07
#define INA_REG_MANUFACTURER_ID     0xFE //0x5449
#define INA_REG_DIE_ID              0xFF //0x2260

#define i2cWrite(u8SlaveAddr,u8RegAddr,pu8Data,u8DataLen)  i2c1_write(u8SlaveAddr,u8RegAddr,pu8Data,u8DataLen)
#define i2cRead(u8SlaveAddr,u8RegAddr,pu8Data,u8DataLen)   i2c1_read(u8SlaveAddr,u8RegAddr,pu8Data,u8DataLen)

u16 u16ManID = 0;
u16 u16DieID = 0;
static u8 _bINA226_InitDone = FALSE;

TaskHandle_t hINA226_Task = NULL;
ST_INA_Info_t stInaInfo;

static u8 _INA_I2C_Read(u8 u8SlaveAddr,u8 u8RegAddr,u16 *pu16Data);
static u8 _INA_I2C_Write(u8 u8SlaveAddr,u8 u8RegAddr,u16 u16Data);

//extern unsigned char DataScope_OutPut_Buffer[42];

void INA226_HandleTask(void *pvParameters)
{
    u8 u8Count;
    u16 u16temp = 0;
    ST_OLED_Disp_t stOledDisp;

    memset(&stOledDisp, 0, sizeof(ST_OLED_Disp_t));
    INA226_Init();
    while(pdTRUE)
    {
#if 0
        _INA_I2C_Read(INA_SLAVE_ADDR, INA_REG_SHUNT_VAL, &stInaInfo.u16VShuntVal);
        stInaInfo.f32VShunt = stInaInfo.u16VShuntVal*2.5f*0.001; //Unit :mV
        stOledDisp.u8x = 0;
        stOledDisp.u8y = 0;
        stOledDisp.f32data = stInaInfo.f32VShunt;
        FR_OS_QueueSend(hOledDisp_Queue, &stOledDisp, 0);
        //debug("f32VShunt = %f mV.\n",stInaInfo.f32VShunt);
        FR_OS_DelayMs(50);
#endif
#if TRUE
        _INA_I2C_Read(INA_SLAVE_ADDR, INA_REG_VBUS_VAL, &stInaInfo.u16VBusVal);
        stInaInfo.f32Vbus = stInaInfo.u16VBusVal*1.25f;          //Unit :mV
        stOledDisp.u8x = 36;
        stOledDisp.u8y = 0;
        stOledDisp.f32data = stInaInfo.f32Vbus*0.001f;//unit: V
        FR_OS_QueueSend(hOledDisp_Queue, &stOledDisp, 0);
        //debug("f32Vbus = %f mV.\n", stInaInfo.f32Vbus);
        FR_OS_DelayMs(50);
#endif
#if TRUE

        _INA_I2C_Read(INA_SLAVE_ADDR, INA_REG_CURRENT_VAL, &stInaInfo.u16CurVal);
        stInaInfo.f32Cur = stInaInfo.u16CurVal*0.02f;//Current   //Unit :mA
        stOledDisp.u8x = 36;
        stOledDisp.u8y = 16;
        stOledDisp.f32data = stInaInfo.f32Cur;//Unit: mA
        FR_OS_QueueSend(hOledDisp_Queue, &stOledDisp, 0);
        //debug("f32Cur = %f mA.\n", stInaInfo.f32Cur);
        FR_OS_DelayMs(50);
#endif
        _INA_I2C_Read(INA_SLAVE_ADDR, INA_REG_POWER_VAL, &stInaInfo.u16PowerVal);
        stInaInfo.f32Power = stInaInfo.u16PowerVal*0.02f*25.0f;    //Unit :mW
        //debug("f32Power = %f mW.\n", stInaInfo.f32Cur);
        
        stOledDisp.u8x = 36;
        stOledDisp.u8y = 32;
        stOledDisp.f32data = stInaInfo.f32Power*0.001f;//unit: W
        FR_OS_QueueSend(hOledDisp_Queue, &stOledDisp, 0);
        FR_OS_DelayMs(50);
#if 0
        DataScope_Get_Channel_Data(stInaInfo.f32Vbus, E_DATA_SCOPE_CHANNEL_1);
        DataScope_Get_Channel_Data(stInaInfo.f32Cur, E_DATA_SCOPE_CHANNEL_2);
        DataScope_Get_Channel_Data(stInaInfo.f32Power,E_DATA_SCOPE_CHANNEL_3);
        DataScope_Get_Channel_Data(stInaInfo.f32VShunt, E_DATA_SCOPE_CHANNEL_4);
        DataScope_Get_Channel_Data(0,E_DATA_SCOPE_CHANNEL_5);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_6);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_7);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_8);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_9);
        DataScope_Get_Channel_Data(0, E_DATA_SCOPE_CHANNEL_10);
        u8Count = DataScope_Data_Generate(4);
        DataScope_SendData(DataScope_OutPut_Buffer,u8Count);
#endif
    }
}

void INA226_Init(void)
{
    if(!_bINA226_InitDone)
    {
        _INA_I2C_Write(INA_SLAVE_ADDR, INA_REG_CONFIG, 0x4527);
        _INA_I2C_Write(INA_SLAVE_ADDR, INA_REG_CALIBRATION, 0x0A00);
        _bINA226_InitDone = TRUE;
    }
}

static u8 _INA_I2C_Read(u8 u8SlaveAddr,u8 u8RegAddr,u16 *pu16Data)
{
    u8 pu8Temp[2];
    EN_I2C_Ret_e eRet;
    eRet = i2cRead(u8SlaveAddr, u8RegAddr, pu8Temp, 2);
    *pu16Data = (u16)pu8Temp[0]<<8|pu8Temp[1];
    return eRet;
}

static u8 _INA_I2C_Write(u8 u8SlaveAddr,u8 u8RegAddr,u16 u16Data)
{
    u8 pu8Temp[2];
    EN_I2C_Ret_e eRet;

    pu8Temp[0] = (u16Data>>8)&0xFF;
    pu8Temp[1] = (u16Data)&0xFF;
    eRet = i2cWrite(u8SlaveAddr, u8RegAddr, pu8Temp, 2);
    return eRet;
}




