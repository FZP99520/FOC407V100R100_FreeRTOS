#include "iic.h"
#include "systick.h"
#include "log.h"


#define I2C1_HW_MODE_ENABLE     FALSE

#if I2C1_HW_MODE_ENABLE
#define I2C1_HW_GPIO_SCL              GPIO_Pin_8
#define I2C1_HW_GPIO_SDA              GPIO_Pin_9
#define I2C1_HW_GPIO_SCL_PIN_SOURCE   GPIO_PinSource8
#define I2C1_HW_GPIO_SDA_PIN_SOURCE   GPIO_PinSource9
#define I2C1_HW_GPIO_PORT  GPIOB

//HW iic
void I2C1_HW_Init(u16 u16ClkSpeed)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);   
    /* Enable I2C1 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(I2C1_HW_GPIO_PORT, I2C1_HW_GPIO_SCL_PIN_SOURCE, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_HW_GPIO_PORT, I2C1_HW_GPIO_SDA_PIN_SOURCE, GPIO_AF_I2C1);
    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = I2C1_HW_GPIO_SCL|I2C1_HW_GPIO_SDA;
    GPIO_Init(I2C1_HW_GPIO_PORT, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = u16ClkSpeed;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_ITConfig(I2C1, I2C_IT_EVT|I2C_IT_ERR, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0; //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

    I2C_Cmd(I2C1, ENABLE);
}


void I2C1_EV_IRQHandler(void)
{
    if(I2C_GetITStatus(I2C1, I2C_IT_TXE)!=RESET)
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_TXE);
    }
}
void I2C1_ER_IRQHandler(void)
{
    /*if(I2C_GetITStatus(I2C1, I2C_IT_TXE)!=RESET)
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_TXE);
    }*/
}

#endif


#define I2C1_SW_GPIO_SCL    GPIO_Pin_8
#define I2C1_SW_GPIO_SDA    GPIO_Pin_9
#define I2C1_SW_GPIO_PORT   GPIOB

//IO操作函数	 
#define _I2C1_SCL_H()     GPIO_SetBits(I2C1_SW_GPIO_PORT,I2C1_SW_GPIO_SCL)//SCL
#define _I2C1_SCL_L()     GPIO_ResetBits(I2C1_SW_GPIO_PORT,I2C1_SW_GPIO_SCL)//SCL
#define _I2C1_SDA_H()     GPIO_SetBits(I2C1_SW_GPIO_PORT,I2C1_SW_GPIO_SDA)//SDA
#define _I2C1_SDA_L()     GPIO_ResetBits(I2C1_SW_GPIO_PORT,I2C1_SW_GPIO_SDA)//SDA
#define _I2C1_SDA_IN()    GPIO_ReadInputDataBit(I2C1_SW_GPIO_PORT,I2C1_SW_GPIO_SDA)  //输入SDA 

#define _I2C1_INTERNAL_DELAY(...) 

#define I2C1_MUTEX_LOCK()   FR_OS_MUTEX_LOCK(hI2C1_Mutex)
#define I2C1_MUTEX_UNLOCK() FR_OS_MUTEX_UNLOCK(hI2C1_Mutex)

TaskHandle_t    hI2C1_Task = NULL;
QueueHandle_t   hI2C1_Queue = NULL;
MutexHandle_t   hI2C1_Mutex = NULL;

static u8 _bI2C1_InitDone = FALSE;

static u8 _au8I2C1_DeviceAddrInfo[10] = {0};

static void _I2C1_delay(u8 u8delay);
static u8   _I2C1_Check_Device_Online(u8 u8SlaveAddr);
static void _I2C1_Start(void);//发送I2C开始信号
static void _I2C1_Stop(void);//发送I2C停止信号
static void _I2C1_Send_Byte(u8 u8Txd);//I2C发送一个字节
static u8   _I2C1_Read_Byte(u8 bAck);//I2C读取一个字节
static u8   _I2C1_Wait_Ack(void);//I2C等待ACK信号
static void _I2C1_Ack(void);//I2C发送ACK信号
static void _I2C1_NAck(void);//I2C不发送ACK信号
static void _I2C1_SDA_SetOutMode(void);
static void _I2C1_SDA_SetInMode(void);

void I2C1_DataHandle_Task(void * pvParameters)
{
    u8 u8Temp = 0;
    u16 u16SlaveAddr2Find = 0;
    BaseType_t xReturn = pdFALSE;
    ST_I2C_Params_t stI2cParams;

    memset(&stI2cParams,0,sizeof(ST_I2C_Params_t));

    I2C1_ScanDevices_ByTask();
    while(pdTRUE)
    {
        xReturn = FR_OS_QueueReceive(hI2C1_Queue, &stI2cParams, portMAX_DELAY);
        
        if(xReturn == pdTRUE)
        {
            if(stI2cParams.eRW_Type == E_I2C_TYPE_WRITE)
            {
                I2C1_SendData(&stI2cParams);
                if(stI2cParams.eRet == E_I2C_OK)
                {
                    DEBUG_INFO("I2C Write ok.\n");
                }
                else
                {
                    DEBUG_ERROR("I2C Write fail, eRet=%d.\n",stI2cParams.eRet);
                }
            }
            else if(stI2cParams.eRW_Type == E_I2C_TYPE_READ)
            {
                I2C1_ReadData(&stI2cParams);
                if(stI2cParams.eRet == E_I2C_OK)
                {
                    DEBUG_INFO("I2C Read ok.\n");
                }
                else
                {
                    DEBUG_ERROR("I2C Read fail, eRet=%d.\n",stI2cParams.eRet);
                }
            }
            else if(stI2cParams.eRW_Type == E_I2C_TYPE_SCAN)
            {
                DEBUG_INFO("Scaning i2c devices......\n")
                for(u16SlaveAddr2Find=0; u16SlaveAddr2Find< 0xFE; u16SlaveAddr2Find +=2)
                {
                    if(_I2C1_Check_Device_Online(u16SlaveAddr2Find&0xFF)==TRUE)
                    {
                        _au8I2C1_DeviceAddrInfo[u8Temp++] = u16SlaveAddr2Find;
                        DEBUG_INFO("Found i2c device 0x%02x.\n", u16SlaveAddr2Find);
                    }
                    FR_OS_DelayMs(20);
                }
                u8Temp = 0;
                u16SlaveAddr2Find = 0;
            }
            else;
        }
    }
}

/*******************************/
void I2C1_SW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PB端口时钟
    GPIO_InitStructure.GPIO_Pin = I2C1_SW_GPIO_SCL|I2C1_SW_GPIO_SDA;	//端口配置
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //推挽输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100M
    GPIO_Init(I2C1_SW_GPIO_PORT, &GPIO_InitStructure);//根据设定参数初始化GPIOB 

    _I2C1_SCL_H();
    _I2C1_SDA_H();
    _bI2C1_InitDone = TRUE;

    DEBUG_TRACE("OK\n");
}
static void _I2C1_SDA_SetOutMode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = I2C1_SW_GPIO_SDA;//端口配置
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //推挽输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100M
    GPIO_Init(I2C1_SW_GPIO_PORT, &GPIO_InitStructure);
}
void _I2C1_SDA_SetInMode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = I2C1_SW_GPIO_SDA;	//端口配置
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN; 
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD; //开漏输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100M
    GPIO_Init(I2C1_SW_GPIO_PORT, &GPIO_InitStructure);
}

static void _I2C1_Start(void)
{
    _I2C1_SDA_SetOutMode();     //sda线输出
    _I2C1_SDA_H();
    _I2C1_SCL_H();
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SDA_L();//START:when CLK is high,DATA change form high to low 
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_L();//钳住I2C总线，准备发送或接收数据 
} 
static void _I2C1_Stop(void)
{
    _I2C1_SDA_SetOutMode();//sda线输出
    _I2C1_SCL_L();
    _I2C1_SDA_L();//STOP:when CLK is high DATA change form low to high
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_H(); 
    _I2C1_delay(10);
    _I2C1_SDA_H();  //发送I2C总线结束信号
    //_I2C1_delay(10);//Delay_us(1);
}
static u8 _I2C1_Wait_Ack(void)
{
    u8 ucErrTime=0;
    _I2C1_SDA_SetInMode();      //SDA设置为输入  
    _I2C1_SDA_H();   
    _I2C1_SCL_H();
    _I2C1_delay(10);//Delay_us(1);
    while(_I2C1_SDA_IN())
    {
        ucErrTime++;
        if(ucErrTime>200)
        {
            _I2C1_Stop();
            return FALSE;
        }
        _I2C1_delay(10);//Delay_us(1);
    }
    _I2C1_SCL_L();//时钟输出0
    return TRUE;  
} 
static void _I2C1_Ack(void)
{
    _I2C1_SCL_L();
    _I2C1_SDA_SetOutMode();
    _I2C1_SDA_L();
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_H();
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_L();
}   
static void _I2C1_NAck(void)
{
    _I2C1_SCL_L();
    _I2C1_SDA_SetOutMode();
    _I2C1_SDA_H();
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_H();
    _I2C1_delay(10);//Delay_us(1);
    _I2C1_SCL_L();
} 
static void _I2C1_Send_Byte(u8 txd)
{                        
    u8 u8Temp;   
    _I2C1_SDA_SetOutMode(); 
    _I2C1_SCL_L();//拉低时钟开始数据传输
    for(u8Temp=0;u8Temp<8;u8Temp++)
    {              
        if(txd&0x80) 
            _I2C1_SDA_H();
        else 
            _I2C1_SDA_L();
        txd<<=1;
        _I2C1_delay(20);
        _I2C1_SCL_H();
        _I2C1_delay(20);
        _I2C1_SCL_L();
        _I2C1_delay(10);
    }
} 

static u8 _I2C1_Read_Byte(u8 bAck)
{
    u8 u8Temp;
    u8 u8Receice = 0;
    _I2C1_SDA_SetInMode();//SDA设置为输入
    for(u8Temp=0;u8Temp<8;u8Temp++)
    {
        _I2C1_SCL_L(); 
        _I2C1_delay(20);
        _I2C1_SCL_H();
        u8Receice <<= 1;
        if(_I2C1_SDA_IN())
            u8Receice++; 
        _I2C1_delay(20);
   }
    if(bAck)
        _I2C1_Ack(); //发送ACK 
    else
        _I2C1_NAck();//发送nACK  
    return u8Receice;
}

static void _I2C1_delay(u8 u8delay)
{
    u8 u8Temp;
    for(u8Temp=0; u8Temp<u8delay; u8Temp++)
    {
        __nop();
    }
}

static u8 _I2C1_Check_Device_Online(u8 u8SlaveAddr)
{
    I2C1_MUTEX_LOCK();

    _I2C1_Start();
    _I2C1_Send_Byte(u8SlaveAddr);
    if(_I2C1_Wait_Ack())
    {
        _I2C1_Stop();
        I2C1_MUTEX_UNLOCK();
        return TRUE;
    }
    else
    {
        _I2C1_Stop();
        I2C1_MUTEX_UNLOCK();
        return FALSE;
    }
}


#define CHECK_ACK(function)         \
    {                                  \
        if(!function)                  \
        {                              \
            *peRet = E_I2C_ERR_NACK;     \
            goto function_end;         \
        }                              \
    }                                  
/****************************************************/
//I2C1 API for external used
EN_I2C_Ret_e I2C1_SendData(ST_I2C_Params_t *pstI2cParams)
{
    u8 u8Temp = 0;
    EN_I2C_Ret_e *peRet = &pstI2cParams->eRet;
    *peRet = E_I2C_OK;//Default ok

    I2C1_MUTEX_LOCK();
    
    _I2C1_Start();
    _I2C1_Send_Byte(pstI2cParams->u8SlaveAddr);
    CHECK_ACK(_I2C1_Wait_Ack());
    if(pstI2cParams->pu8RegAddr != NULL)
    {
        for(u8Temp=0; u8Temp< pstI2cParams->u8RegAddrLen; u8Temp++)
        {
            _I2C1_Send_Byte(pstI2cParams->pu8RegAddr[u8Temp]);
            CHECK_ACK(_I2C1_Wait_Ack());
        }
    }
    if(pstI2cParams->pu8Data != NULL)
    {
        for(u8Temp=0; u8Temp< pstI2cParams->u8DateLen; u8Temp++)
        {
            _I2C1_Send_Byte(pstI2cParams->pu8Data[u8Temp]);
            CHECK_ACK(_I2C1_Wait_Ack());
        }
    }
    else
    {
        *peRet = E_I2C_ERR_INAVLID_DATA;
    }
    _I2C1_Stop();

function_end:
    I2C1_MUTEX_UNLOCK();
    return *peRet;
    
}

EN_I2C_Ret_e I2C1_ReadData(ST_I2C_Params_t *pstI2cParams)
{
    u8 u8Temp = 0;
    u8 bAck = TRUE;
    EN_I2C_Ret_e *peRet = &pstI2cParams->eRet;
    *peRet = E_I2C_OK;//Default ok

    I2C1_MUTEX_LOCK();

    _I2C1_Start();
    _I2C1_Send_Byte(pstI2cParams->u8SlaveAddr);
    CHECK_ACK(_I2C1_Wait_Ack());
    if(pstI2cParams->pu8RegAddr != NULL)
    {
        for(u8Temp=0; u8Temp< pstI2cParams->u8RegAddrLen; u8Temp++)
        {
            _I2C1_Send_Byte(pstI2cParams->pu8RegAddr[u8Temp]);
            CHECK_ACK(_I2C1_Wait_Ack());
        }
    }
    _I2C1_Start();
    _I2C1_Send_Byte(pstI2cParams->u8SlaveAddr+1);
    CHECK_ACK(_I2C1_Wait_Ack());
    if(pstI2cParams->pu8Data != NULL)
    {
        for(u8Temp=0; u8Temp< pstI2cParams->u8DateLen - 1; u8Temp++)
        {
            pstI2cParams->pu8Data[u8Temp] = _I2C1_Read_Byte(TRUE);
        }
        bAck = FALSE;
        pstI2cParams->pu8Data[u8Temp] = _I2C1_Read_Byte(FALSE);
    }
    else
    {
        *peRet = E_I2C_ERR_INAVLID_DATA;
    }
    _I2C1_Stop();

function_end:
    I2C1_MUTEX_UNLOCK();
    return *peRet;
}

EN_I2C_Ret_e i2c1_write(u8 u8SlaveAddr,u8 u8RegAddr,u8 *pu8Data,u8 u8DataLen)
{
    ST_I2C_Params_t stI2cParams;
    memset(&stI2cParams, 0, sizeof(ST_I2C_Params_t));

    stI2cParams.u8SlaveAddr = u8SlaveAddr;
    stI2cParams.pu8RegAddr = &u8RegAddr;
    stI2cParams.u8RegAddrLen = 1;
    stI2cParams.pu8Data = pu8Data;
    stI2cParams.u8DateLen = u8DataLen;
    I2C1_SendData(&stI2cParams);
    return stI2cParams.eRet;
}

EN_I2C_Ret_e i2c1_read(u8 u8SlaveAddr,u8 u8RegAddr,u8 *pu8Data,u8 u8DataLen)
{
    ST_I2C_Params_t stI2cParams;
    memset(&stI2cParams, 0, sizeof(ST_I2C_Params_t));

    stI2cParams.u8SlaveAddr = u8SlaveAddr;
    stI2cParams.pu8RegAddr = &u8RegAddr;
    stI2cParams.u8RegAddrLen = 1;
    stI2cParams.pu8Data = pu8Data;
    stI2cParams.u8DateLen = u8DataLen;
    I2C1_ReadData(&stI2cParams);
    return stI2cParams.eRet;
}

void I2C1_SendData_ByTask(ST_I2C_Params_t *pstI2cParams)
{
    pstI2cParams->eRW_Type = E_I2C_TYPE_WRITE;
    FR_OS_QueueSend(hI2C1_Queue, pstI2cParams, 0);
}

void I2C1_ReadData_ByTask(ST_I2C_Params_t *pstI2cParams)
{
    pstI2cParams->eRW_Type = E_I2C_TYPE_READ;
    FR_OS_QueueSend(hI2C1_Queue, pstI2cParams, 0);
}

void I2C1_ScanDevices_ByTask(void)
{
    ST_I2C_Params_t stI2cParams;
    stI2cParams.eRW_Type = E_I2C_TYPE_SCAN;
    FR_OS_QueueSend(hI2C1_Queue, &stI2cParams, 0);
}

