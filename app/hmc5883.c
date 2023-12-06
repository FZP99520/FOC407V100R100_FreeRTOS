#include "hmc5883.h"
#include "math.h"
MAG_Data mag_data;
u8 MAG_DataUpdate(void)
{
	u8 buff[6];
	mag_data.STA=MAG_ReadByte(Reg_Status);//¶ÁÈ¡×´Ì¬
	if(mag_data.STA&MAG_DRY)
	{
	 MAG_GetData(buff);
	 mag_data.MAG_X=(buff[1]<<8)+buff[0];
	 mag_data.MAG_Y=(buff[3]<<8)+buff[2];
	 mag_data.MAG_Z=(buff[5]<<8)+buff[4];
		
		mag_data.Mag_x=(float)mag_data.MAG_X/32767*2.0f;
		mag_data.Mag_y=(float)mag_data.MAG_Y/32767*2.0f;
		mag_data.Mag_z=(float)mag_data.MAG_Z/32767*2.0f;
		
		mag_data.Total_MAG=sqrt(pow(mag_data.Mag_x,2)+pow(mag_data.Mag_y,2)+pow(mag_data.Mag_z,2));
	 mag_data.update_ok=1;
	 return 1;
	}
	else return 0;
}

void MAG_GetData(u8 *buff)
{/*
	u8 i;
	IIC_Start();
	IIC_Send_Byte(Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x00);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(Address+1);
	IIC_Wait_Ack();
	for(i=0;i<6;i++)
	{
		if(i<5) buff[i]=IIC_Read_Byte(1);
		if(i==5) buff[i]=IIC_Read_Byte(0);
	}
	IIC_Stop();*/
}
void MAG_WriteData(u8 addr,u8 dat)
{/*
	IIC_Start();
	IIC_Send_Byte(Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(dat);
	IIC_Wait_Ack();
	IIC_Stop();*/
}
u8 MAG_ReadByte(u8 addr)
{/*
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(Address+1);
	IIC_Wait_Ack();
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	return temp;*/
}
void MAG_Init()
{/*
	#ifdef MAG_HMC5883
	MAG_WriteData(Reg_Config_A,0x10);//Output rate=10Hz
	MAG_WriteData(Reg_Config_B,0xe0);//+-4.5Ga
	MAG_WriteData(Reg_Mode,0x00);//Circulate output
	#endif
	
	#ifdef MAG_QMC5883
	MAG_WriteData(Reg_Config,0x09);//scale:2G,output rate:100Hz,Continuous mode
	MAG_WriteData(Reg_Config2,0x41);//disable interrupt ,pointer roll-over mode
	MAG_WriteData(Reg_Period,0x01); //recommand 0x01
	#endif
	Delay_ms(50);
    */
	
}
