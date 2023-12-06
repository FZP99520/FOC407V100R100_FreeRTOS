#include "flash.h" 
#include "spi.h"
#include "log.h"


//4Kbytes为一个Sector
//16个扇区为1个Block
//W25X16
//容量为2M字节,共有32个Block,512个Sector 

//选中FLASH
#define FLASH_SPI_CS_LOW()   GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define FLASH_SPI_CS_HIGH()  GPIO_SetBits(GPIOC,GPIO_Pin_13)

//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 


TaskHandle_t hFlash_RW_Task = NULL;
QueueHandle_t hFlash_RW_Queue = NULL;


u16 u16FLASH_ID = 0;
EN_W25QXX_Type_e eFlashType;

static u8 _FLASH_RW_GetAddress(EN_FLASH_RW_OWNER_e eOwner, u32 *pu32Address, u16 *pu16MaxSize);


void FLASH_RW_Task(void *pvParameters)
{
    BaseType_t xRet = pdFALSE;
    u8 u8Temp = 0;
    u8 pu8WriteBuff[10] = {1,2,3,4,5,6,7,8,9,0};
    u8 pu8ReadBuff[10];

    u16 u16DateSize = 0;
    u32 u32FlashAddress = 0;
    u16 u16MaxSize = 0;
    ST_FLASH_RW_t stFlashRW;

    memset(&stFlashRW, 0, sizeof(ST_FLASH_RW_t));

    u16FLASH_ID = FLASH_SPI_ReadID();
    if(u16FLASH_ID != 0)
    {
        DEBUG_INFO("Get Flash ID:0x%x.\n",u16FLASH_ID);
    }
    else
    {
        DEBUG_ERROR("Get Flash ID Error.\n");
    }

    while(pdTRUE)
    {
        xRet = FR_OS_QueueReceive(hFlash_RW_Queue, &stFlashRW, portMAX_DELAY);
        if(xRet == pdTRUE)
        {
            if(_FLASH_RW_GetAddress(stFlashRW.eRW_Owner, &u32FlashAddress, &u16MaxSize) == TRUE)
            {
                u16DateSize = stFlashRW.u16DataSize;
                u16DateSize = u16DateSize>u16MaxSize?u16MaxSize:u16DateSize;
                if(stFlashRW.eRW_Type == E_FLASH_RW_TYPE_READ)
                {
                    FLASH_SPI_Read(stFlashRW.pu8Data, u32FlashAddress, u16DateSize);
                }
                else if(stFlashRW.eRW_Type == E_FLASH_RW_TYPE_WRITE)
                {
                    FLASH_SPI_Write(stFlashRW.pu8Data, u32FlashAddress, u16DateSize);
                }
                else
                {
                }
            }
            else
            {
                DEBUG_PRINT("Unable to get flash address by owner:%d.\n",stFlashRW.eRW_Owner);
            }
            
        }
    }
}

#define FLASH_DATA_MAX_SIZE_NULL 10
#define FLASH_DATA_MAX_SIZE_PID1 0x100
#define FLASH_DATA_MAX_SIZE_PID2 0x100

#define FLASH_DATA_ADDRESS_NULL  0x00000000
#define FLASH_DATA_ADDRESS_PID1  FLASH_DATA_ADDRESS_NULL + FLASH_DATA_MAX_SIZE_PID1
#define FLASH_DATA_ADDRESS_PID2  FLASH_DATA_ADDRESS_PID1 + FLASH_DATA_MAX_SIZE_PID2

static u8 _FLASH_RW_GetAddress(EN_FLASH_RW_OWNER_e eOwner, u32 *pu32Address, u16 *pu16MaxSize)
{
    switch(eOwner)
    {
        case E_FLASH_RW_DATA_OWNER_NULL:
        {
            *pu32Address = FLASH_DATA_ADDRESS_NULL;
            *pu16MaxSize = FLASH_DATA_MAX_SIZE_NULL;
            break;
        }
        case E_FLASH_RW_DATA_OWNER_PID1:
        {
            *pu32Address = FLASH_DATA_ADDRESS_PID1;
            *pu16MaxSize = FLASH_DATA_MAX_SIZE_PID1;
            break;
        }
        case E_FLASH_RW_DATA_OWNER_PID2:
        {
            *pu32Address = FLASH_DATA_ADDRESS_PID2;
            *pu16MaxSize = FLASH_DATA_MAX_SIZE_PID2;
            break;
        } 
        default:
        {
            return FALSE;
        }
    }
    return TRUE;
}

//初始化SPI FLASH的IO口
void FLASH_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//PORTC时钟使能 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//PC13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    FLASH_SPI_CS_HIGH();

    SPI1_Init();//初始化SPI
    SPI1_SetSpeed(SPI_BaudRatePrescaler_2);
 

}  

//读取FLASH_SPI的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
u8 FLASH_SPI_ReadSR(void)   
{  
    u8 byte=0;   
    //FLASH_SPI_CS=0;                            //使能器件
    FLASH_SPI_CS_LOW();   
    SPI1_ReadWriteByte(W25X_ReadStatusReg);    //发送读取状态寄存器命令    
    byte = SPI1_ReadWriteByte(0Xff);             //读取一个字节
    FLASH_SPI_CS_HIGH();  
    //FLASH_SPI_CS=1;                            //取消片选     
    return byte;   
} 
//写FLASH_SPI状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void FLASH_SPI_Write_SR(u8 sr)   
{   
    //FLASH_SPI_CS=0;                            //使能器件 
    FLASH_SPI_CS_LOW();   
    SPI1_ReadWriteByte(W25X_WriteStatusReg);   //发送写取状态寄存器命令    
    SPI1_ReadWriteByte(sr);               //写入一个字节 
    FLASH_SPI_CS_HIGH();  
    //FLASH_SPI_CS=1;                            //取消片选     	      
}   
//FLASH_SPI写使能	
//将WEL置位   
void FLASH_SPI_Write_Enable(void)   
{
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件

    SPI1_ReadWriteByte(W25X_WriteEnable);      //发送写使能  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
} 
//FLASH_SPI写禁止	
//将WEL清零  
void FLASH_SPI_Write_Disable(void)   
{  
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //发送写禁止指令    
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
}    
//读取芯片ID W25X16的ID:0XEF14
u16 FLASH_SPI_ReadID(void)
{
    u16 Temp = 0;
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;
    SPI1_ReadWriteByte(0x90);//发送读取ID命令	    
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);  
    Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
    Temp|=SPI1_ReadWriteByte(0xFF);	 
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;  
    return Temp;
}    
//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void FLASH_SPI_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
    u16 i;    
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReadData);         //发送读取命令   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16));  //发送24bit地址    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
    { 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //循环读数  
    }
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
}  
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void FLASH_SPI_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
    u16 i;  
    FLASH_SPI_Write_Enable();                  //SET WEL 
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_PageProgram);      //发送写页命令   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16)); //发送24bit地址    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((u8)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//循环写数  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选 
    FLASH_SPI_Wait_Busy();//等待写入结束
} 
//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void FLASH_SPI_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{
    u16 pageremain;
    pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
    if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
    while(1)
    {
        FLASH_SPI_Write_Page(pBuffer,WriteAddr,pageremain);
        if(NumByteToWrite==pageremain)break;//写入结束了
        else //NumByteToWrite>pageremain
        {
            pBuffer+=pageremain;
            WriteAddr+=pageremain;	

            NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
            if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
            else pageremain=NumByteToWrite; 	  //不够256个字节了
        }
    }  
} 
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
u8 FLASH_SPI_BUF[4096];
void FLASH_SPI_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
    u32 secpos;
    u16 secoff;
    u16 secremain;	   
    u16 i;    

    secpos=WriteAddr/4096;//扇区地址 0~511 for w25x16

    secoff=WriteAddr%4096;//在扇区内的偏移

    secremain=4096-secoff;//扇区剩余空间大小   

    if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
    while(1) 
    {
        FLASH_SPI_Read(FLASH_SPI_BUF,secpos*4096,4096);//读出整个扇区的内容

        for(i=0;i<secremain;i++)//校验数据
        {
            if(FLASH_SPI_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
        }
        if(i<secremain)//需要擦除
        {
            FLASH_SPI_Erase_Sector(secpos);//擦除这个扇区
            for(i=0;i<secremain;i++)	   //复制
            {
                FLASH_SPI_BUF[i+secoff]=pBuffer[i];	  
            }
            FLASH_SPI_Write_NoCheck(FLASH_SPI_BUF,secpos*4096,4096);//写入整个扇区  

        }else FLASH_SPI_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间.
        if(NumByteToWrite==secremain){break;}//写入结束了
        else//写入未结束
        {
            secpos++;//扇区地址增1
            secoff=0;//偏移位置为0 	 

            pBuffer+=secremain;  //指针偏移
            WriteAddr+=secremain;//写地址偏移	   
            NumByteToWrite-=secremain;				//字节数递减
            if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
            else secremain=NumByteToWrite;			//下一个扇区可以写完了
        }
    };	 	 
}
//擦除整个芯片
//整片擦除时间:
//W25X16:25s 
//W25X32:40s 
//W25X64:40s 
//等待时间超长...
void FLASH_SPI_Erase_Chip(void)   
{                                             
    FLASH_SPI_Write_Enable();                  //SET WEL 
    FLASH_SPI_Wait_Busy();   
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ChipErase);        //发送片擦除命令  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
    FLASH_SPI_Wait_Busy();   				   //等待芯片擦除结束
}   
//擦除一个扇区
//Dst_Addr:扇区地址 0~511 for w25x16
//擦除一个山区的最少时间:150ms
void FLASH_SPI_Erase_Sector(u32 Dst_Addr)   
{   
    Dst_Addr*=4096;
    FLASH_SPI_Write_Enable();                  //SET WEL 	 
    FLASH_SPI_Wait_Busy();   
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_SectorErase);      //发送扇区擦除指令 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16));  //发送24bit地址    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((u8)Dst_Addr);  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
    FLASH_SPI_Wait_Busy();   				   //等待擦除完成
}  
//等待空闲
void FLASH_SPI_Wait_Busy(void)   
{   
    while ((FLASH_SPI_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}  
//进入掉电模式
void FLASH_SPI_PowerDown(void)   
{ 
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_PowerDown);        //发送掉电命令  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
    //delay_us(3);                               //等待TPD  
    FR_OS_DelayMs(1);
}   
//唤醒
void FLASH_SPI_WAKEUP(void)   
{  
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //取消片选     	      
    //delay_us(3);                               //等待TRES1
    FR_OS_DelayMs(1);
}   

