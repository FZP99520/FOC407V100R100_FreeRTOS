#include "flash.h" 
#include "spi.h"
#include "log.h"


//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25X16
//����Ϊ2M�ֽ�,����32��Block,512��Sector 

//ѡ��FLASH
#define FLASH_SPI_CS_LOW()   GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define FLASH_SPI_CS_HIGH()  GPIO_SetBits(GPIOC,GPIO_Pin_13)

//ָ���
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

//��ʼ��SPI FLASH��IO��
void FLASH_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//PORTCʱ��ʹ�� 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//PC13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    FLASH_SPI_CS_HIGH();

    SPI1_Init();//��ʼ��SPI
    SPI1_SetSpeed(SPI_BaudRatePrescaler_2);
 

}  

//��ȡFLASH_SPI��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
u8 FLASH_SPI_ReadSR(void)   
{  
    u8 byte=0;   
    //FLASH_SPI_CS=0;                            //ʹ������
    FLASH_SPI_CS_LOW();   
    SPI1_ReadWriteByte(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������    
    byte = SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�
    FLASH_SPI_CS_HIGH();  
    //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     
    return byte;   
} 
//дFLASH_SPI״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void FLASH_SPI_Write_SR(u8 sr)   
{   
    //FLASH_SPI_CS=0;                            //ʹ������ 
    FLASH_SPI_CS_LOW();   
    SPI1_ReadWriteByte(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
    SPI1_ReadWriteByte(sr);               //д��һ���ֽ� 
    FLASH_SPI_CS_HIGH();  
    //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
}   
//FLASH_SPIдʹ��	
//��WEL��λ   
void FLASH_SPI_Write_Enable(void)   
{
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������

    SPI1_ReadWriteByte(W25X_WriteEnable);      //����дʹ��  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
} 
//FLASH_SPIд��ֹ	
//��WEL����  
void FLASH_SPI_Write_Disable(void)   
{  
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //����д��ָֹ��    
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
}    
//��ȡоƬID W25X16��ID:0XEF14
u16 FLASH_SPI_ReadID(void)
{
    u16 Temp = 0;
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;
    SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);  
    Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
    Temp|=SPI1_ReadWriteByte(0xFF);	 
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;  
    return Temp;
}    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void FLASH_SPI_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
    u16 i;    
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
    { 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //ѭ������  
    }
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void FLASH_SPI_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
    u16 i;  
    FLASH_SPI_Write_Enable();                  //SET WEL 
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram);      //����дҳ����   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16)); //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((u8)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//ѭ��д��  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ 
    FLASH_SPI_Wait_Busy();//�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void FLASH_SPI_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{
    u16 pageremain;
    pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
    if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
    while(1)
    {
        FLASH_SPI_Write_Page(pBuffer,WriteAddr,pageremain);
        if(NumByteToWrite==pageremain)break;//д�������
        else //NumByteToWrite>pageremain
        {
            pBuffer+=pageremain;
            WriteAddr+=pageremain;	

            NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
            if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
            else pageremain=NumByteToWrite; 	  //����256���ֽ���
        }
    }  
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
u8 FLASH_SPI_BUF[4096];
void FLASH_SPI_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
    u32 secpos;
    u16 secoff;
    u16 secremain;	   
    u16 i;    

    secpos=WriteAddr/4096;//������ַ 0~511 for w25x16

    secoff=WriteAddr%4096;//�������ڵ�ƫ��

    secremain=4096-secoff;//����ʣ��ռ��С   

    if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
    while(1) 
    {
        FLASH_SPI_Read(FLASH_SPI_BUF,secpos*4096,4096);//������������������

        for(i=0;i<secremain;i++)//У������
        {
            if(FLASH_SPI_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
        }
        if(i<secremain)//��Ҫ����
        {
            FLASH_SPI_Erase_Sector(secpos);//�����������
            for(i=0;i<secremain;i++)	   //����
            {
                FLASH_SPI_BUF[i+secoff]=pBuffer[i];	  
            }
            FLASH_SPI_Write_NoCheck(FLASH_SPI_BUF,secpos*4096,4096);//д����������  

        }else FLASH_SPI_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
        if(NumByteToWrite==secremain){break;}//д�������
        else//д��δ����
        {
            secpos++;//������ַ��1
            secoff=0;//ƫ��λ��Ϊ0 	 

            pBuffer+=secremain;  //ָ��ƫ��
            WriteAddr+=secremain;//д��ַƫ��	   
            NumByteToWrite-=secremain;				//�ֽ����ݼ�
            if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
            else secremain=NumByteToWrite;			//��һ����������д����
        }
    };	 	 
}
//��������оƬ
//��Ƭ����ʱ��:
//W25X16:25s 
//W25X32:40s 
//W25X64:40s 
//�ȴ�ʱ�䳬��...
void FLASH_SPI_Erase_Chip(void)   
{                                             
    FLASH_SPI_Write_Enable();                  //SET WEL 
    FLASH_SPI_Wait_Busy();   
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
    FLASH_SPI_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ 0~511 for w25x16
//����һ��ɽ��������ʱ��:150ms
void FLASH_SPI_Erase_Sector(u32 Dst_Addr)   
{   
    Dst_Addr*=4096;
    FLASH_SPI_Write_Enable();                  //SET WEL 	 
    FLASH_SPI_Wait_Busy();   
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase);      //������������ָ�� 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((u8)Dst_Addr);  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
    FLASH_SPI_Wait_Busy();   				   //�ȴ��������
}  
//�ȴ�����
void FLASH_SPI_Wait_Busy(void)   
{   
    while ((FLASH_SPI_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void FLASH_SPI_PowerDown(void)   
{ 
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
    //delay_us(3);                               //�ȴ�TPD  
    FR_OS_DelayMs(1);
}   
//����
void FLASH_SPI_WAKEUP(void)   
{  
    FLASH_SPI_CS_LOW(); //FLASH_SPI_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
    FLASH_SPI_CS_HIGH(); //FLASH_SPI_CS=1;                            //ȡ��Ƭѡ     	      
    //delay_us(3);                               //�ȴ�TRES1
    FR_OS_DelayMs(1);
}   

