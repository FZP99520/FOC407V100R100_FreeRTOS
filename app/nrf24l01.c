#include "nrf24l01.h"
#include "spi.h"
NRF_TypeDef NRF;
const u8 TX_ADDRESS_P0[TX_ADR_WIDTH]={0xA3,0xA4,0xA5,0xA6,0xA7}; //发送地址
const u8 RX_ADDRESS_P0[RX_ADR_WIDTH]={0xA3,0xA4,0xA5,0xA6,0xA7};
void NRF24L01_Init(void)
{ 	
    GPIO_InitTypeDef GPIO_InitStructure;
    Delay_ms(100);	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	//CE
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	//初始化指定IO
    NRF_CE_LOW;       //下拉 进入待机模式

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	//CSN
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	//初始化指定IO
    NRF_CSN_HIGH;//上拉		禁止SPI通信		

//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;   //IRQ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //PG13上拉输入 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
    SPI1_Init();    		//初始化SPI
	NRF_CSN_HIGH;			//SPI片选取消  
}
//检测24L01是否存在
//返回值:1，成功;0，失败	
u8 NRF24L01_Check()
{
    u8 buf[5]={0xAA,0xBB,0xCC,0xDD,0xFF};
    u8 buf1[5];
    u8 i;  	 
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
    NRF24L01_Read_Buf(TX_ADDR,buf1,5); //读出写入的地址  
    for(i=0;i<5;i++)
    {
        if(buf1[i]!=buf[i]) break;	
    }
    if(i!=5) return 0;
    return 1;
}
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	  u8 status;	
   	NRF_CSN_LOW;                 //使能SPI传输
  	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
  	SPI1_ReadWriteByte(value);      //写入寄存器的值
  	NRF_CSN_HIGH;                 //禁止SPI传输	   
  	return(status);       			  //返回状态值
}
u8 NRF24L01_Read_Reg(u8 reg)
{
	  u8 reg_val;	    
 		NRF_CSN_LOW;          //使能SPI传输		
  	SPI1_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(0X00);//读取寄存器内容
  	NRF_CSN_HIGH;           //禁止SPI传输		    
  	return reg_val;          //返回状态值
}	
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	  u8 status,i;	       
  	NRF_CSN_LOW;            //使能SPI传输
  	status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	  for(i=0;i<len;i++)
	  {
				 *pBuf=SPI1_ReadWriteByte(0X00);//读出数据
				 pBuf++;
		}
  	NRF_CSN_HIGH;          //关闭SPI传输
  	return status;        //返回读到的状态值
}
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	  NRF_CSN_LOW;          //使能SPI传输
  	status = SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	 {
	       SPI1_ReadWriteByte(*pBuf); //写入数据	
		     pBuf++;		 
	 }
  	NRF_CSN_HIGH;        //关闭SPI传输
  	return status;          //返回读到的状态值
}				 
void NRF24L01_RX_Mode(void)
{
	 NRF_CE_LOW;	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG_NRF24L01, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x03);	//设置地址长度 bit1:0 01:3b,10:4b,11:5b
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_P0,RX_ADR_WIDTH);//写RX节点地址
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,01);	     //设置RF通信频率		  	    
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);//设置TX发射参数,4dbm增益,250Kbps,低噪声增益开启  
  NRF24L01_Write_Reg(NRF_WRITE_REG+DYNPD,0x01); //使能通道0的动态数据长度
	NRF24L01_Write_Reg(NRF_WRITE_REG+FEATURE,0x06);   //使能 应答带数据包和动态数据长度
  
	Delay_ms(5);//waiting into standby 1 mode
  NRF_CE_HIGH;	 //CE为高,进入接收模式 
}					
void NRF24L01_TX_Mode(void)
{														 
	NRF_CE_LOW;	    
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG_NRF24L01,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断	
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x03);	//设置地址长度 bit1:0 01:3b,10:4b,11:5b
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS_P0,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_P0,RX_ADR_WIDTH); //设置RX节点地址,主要为了使能ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x5A);//设置自动重发间隔时间:1500us;最大自动重发次数:10次
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,01);       //设置RF通道为40
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);  //设置TX发射参数,4dbm增益,250Kbps,低噪声增益开启 
  NRF24L01_Write_Reg(NRF_WRITE_REG+DYNPD,0x01);   //使能通道0的动态数据长度
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答 
	NRF24L01_Write_Reg(NRF_WRITE_REG+FEATURE,0x06);   //使能动态数据长度
	Delay_ms(10);//waiting into standby 1 mode
}
u8 NRF24L01_Send_Data(u8 *txBuf, u8 len) //发送数据
{
	 u8 fifo_sta;
	 NRF_CE_LOW;
	 fifo_sta=NRF_R_FIFO_STA();
	 if( fifo_sta&TX_FULL ) NRF_Flush_TX_FIFO();
	 NRF24L01_Write_Buf(WR_TX_PLOAD,txBuf,len);
	 NRF_CE_HIGH;//拉高发送数据
	 return 0;
}
//add on 2019.05.12
void NRF_Flush_TX_FIFO(void)
{
	NRF_CE_LOW;
	NRF24L01_Write_Reg(FLUSH_TX,0x00);
}
void NRF_Flush_RX_FIFO(void)
{
	NRF_CE_LOW;
	NRF24L01_Write_Reg(FLUSH_RX,0x00);
}
u8 NRF_R_FIFO_STA(void)
{
	u8 temp;
	NRF_CE_LOW;
	temp=NRF24L01_Read_Reg(FIFO_STATUS);
	return temp; 
}
u8 NRF_R_STATUS(void)
{
	u8 temp;
	NRF_CE_LOW;
	temp=NRF24L01_Read_Reg(STATUS);
	NRF_CE_HIGH;
	return temp;
}
u8 NRF_W_ACK_PAYLOAD(u8 *buf,u8 Pn,u8 len) //写反馈数据
{
	if(Pn>5) return 0xff;
	 NRF24L01_Write_Buf(W_ACK_PAYLOAD+Pn,buf,len);
	return 1;
}
u8 NRF_R_RX_PL_WID(void)  //读取接收到的数据长度
{
	return NRF24L01_Read_Reg(R_RX_PL_WID);
}
//end of adding
u8 NRF24L01_ReceiveData(NRF_TypeDef* p)
{
	p->RX_Len = NRF_R_RX_PL_WID();
  if(p->RX_Len> 32)
	{
		NRF_Flush_RX_FIFO();
	  return 0;
  }
	else
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,p->RX_BUFF,p->RX_Len);
	  return 1; 
	}
}
