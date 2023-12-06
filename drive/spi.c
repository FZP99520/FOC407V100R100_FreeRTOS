#include "spi.h"
#include "log.h"

static u8 _bSPI1_InitDone = FALSE;
static u8 _bSPI2_InitDone = FALSE;

//spi1 for flash
//PA5 SCK
//PA6 MISO
//PA7 MOSI
//PC13 CS
u8 SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    DEBUG_TRACE("IN\n");
    if(_bSPI1_InitDone == TRUE)
    {
        DEBUG_ERROR("SPI1 already init.\n");
        return FALSE;
    }
    
    //开启SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //初始化SPI IO 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);//复用为SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);
    SPI_Cmd(SPI1, DISABLE);
    //SPI模式设置
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
#if SPI1_READ_WRITE_DATA_SIZE==1
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
#else
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b; //16bit
#endif
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//8Mbps
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 16;
    SPI_Init(SPI1, &SPI_InitStructure);
    //使能SPI
    SPI_Cmd(SPI1, ENABLE);
    //SPI1_ReadWriteByte(0xff);
 
    _bSPI1_InitDone = TRUE;
    DEBUG_TRACE("OK\n");

    return TRUE;
}

#if (SPI1_READ_WRITE_DATA_SIZE==1)
u8 SPI1_ReadWriteByte(u8 u8data)		 //SPI1读写一个字节
{
    u8 t,data;
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET)	//等待发送缓存器空
    {
        t++;
        if(t>=200)return 0;	//超时返回错误标志	
    }
    SPI_I2S_SendData(SPI1,u8data); //发送数据
    t=0;
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET)	//等待接收
    {
        t++;
        if(t>=200)return 0;	//超时返回错误标志	
    }
    data=SPI_I2S_ReceiveData(SPI1);
    return data; //返回最近SPI1接收的数据			
}
#else
u16 SPI1_ReadWriteByte(u16 u16data)		 //SPI1读写一个字节
{
    u16 t,u16data;
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET)	//等待发送缓存器空
    {
        t++;
        if(t>=200)
        {
            DEBUG_ERROR("SPI TX Over time!\n")
            return 0;   //超时返回错误标志  
        }
    }
    SPI_I2S_SendData(SPI1,u16data); //发送数据
    t=0;
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET)	//等待接收
    {
        t++;
        if(t>=200)
        {
            DEBUG_ERROR("SPI RX Over time!\n")
            return 0;   //超时返回错误标志  
        }
    }
    u16data = SPI_I2S_ReceiveData(SPI1);
    return u16data; //返回最近SPI1接收的数据			
}

#endif


void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)	//设置SPI2的速度
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));	
    SPI1->CR1&=0XFFC7;		 //修改BR[2:0]值
    SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
    SPI_Cmd(SPI1,ENABLE);
}


//spi2
//PB12 CS
//PB13 SCK
//PB14 MISO
//PB15 MOSI
/***************************************************/
u8 SPI2_Init(u16 u16SPI_BaudRatePrescaler)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    DEBUG_TRACE("IN\n");
    /*
    if(_bSPI2_InitDone == TRUE)
    {
        DEBUG_ERROR("SPI2 already init.\n");
        return FALSE;
    }*/
    
    //开启SPI时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //初始化SPI IO 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);//复用为SPI2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);
    SPI_Cmd(SPI2, DISABLE);
    //SPI模式设置
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    //SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b; //16bit for drv8323
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = u16SPI_BaudRatePrescaler;//8Mbps
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 16;
    SPI_Init(SPI2, &SPI_InitStructure);
    //使能SPI
    SPI_Cmd(SPI2, ENABLE);
    //SPI2_ReadWriteByte(0xff);
 
    _bSPI2_InitDone = TRUE;

    DEBUG_TRACE("OK\n");

    return TRUE;
}

EN_SPI_RESULT SPI2_ReadWriteByte(u16 u16DataIn, u16 *pu16DataOut)
{
    u16 t = 0;
    u16 u16ReceiveData = 0;

    if(pu16DataOut == NULL)
    {
        return E_SPI_INVALID_PARAM;
    }

    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET)	//等待发送缓存器空
    {
        t++;
        if(t>2000)
        {
            return E_SPI_TIMEOUT;
        }
    }
    SPI_I2S_SendData(SPI2,u16DataIn); //发送数据
    t=0;
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET)	//等待接收
    {
        t++;
        if(t>2000)
        {
            return E_SPI_TIMEOUT;  //超时返回错误标志  
        }
    }
    *pu16DataOut = SPI_I2S_ReceiveData(SPI2);

    return E_SPI_OK;
}

void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)	//设置SPI2的速度
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));	
    SPI2->CR1&=0XFFC7;		 //修改BR[2:0]值
    SPI2->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
    SPI_Cmd(SPI2,ENABLE);
}


