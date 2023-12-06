#include "bmp180.h"
#include "iic.h"
#include "math.h"
float altitude;
long temperature;//0.1c
long pressure;//Pa
long UT,UP;
BMP180 bmp;
void BMP180_Init(void)
{
	u8 delay;
	Delay_ms(100);
	//IIC_Init();
	while(BMP180_Check()==0)
	{
		delay++;
		Delay_ms(10);
		if(delay>=5) break;
	}
}
/*u8 BMP180_Start_Conversion(void)
{
	u8 temp;
	temp=BMP180_Read_Byte(CTRL_MEAS);
	Delay_ms(5);
	if(temp&0x20) 
	{
		BMP180_CaculateData();
		return 1;
	}
	temp|=0x20;
	BMP180_Write_Byte(CTRL_MEAS,temp);
	return 0;
}*/
/*void BMP180_CaculateData(void)
{
	short ac1,ac2,ac3;
	unsigned short ac4,ac5,ac6;
	short b1,b2,mb,mc,md;
	//long UT,UP;
	long x1,x2,b5;
	long b6,b7;
	unsigned long b4;
	long x3,b3,p;
	ac1=(short)(BMP180_Read_Byte(AC1_H)<<8)+BMP180_Read_Byte(AC1_L);
	ac2=(short)(BMP180_Read_Byte(AC2_H)<<8)+BMP180_Read_Byte(AC2_L);
	ac3=(short)(BMP180_Read_Byte(AC3_H)<<8)+BMP180_Read_Byte(AC3_L);
	ac4=(unsigned short)(BMP180_Read_Byte(AC4_H)<<8)+BMP180_Read_Byte(AC4_L);
	ac5=(unsigned short)(BMP180_Read_Byte(AC5_H)<<8)+BMP180_Read_Byte(AC5_L);
	ac6=(unsigned short)(BMP180_Read_Byte(AC6_H)<<8)+BMP180_Read_Byte(AC6_L);
	b1=(short)(BMP180_Read_Byte(B1_H)<<8)+BMP180_Read_Byte(B1_L);
	b2=(short)(BMP180_Read_Byte(B2_H)<<8)+BMP180_Read_Byte(B2_L);
	mb=(short)(BMP180_Read_Byte(MB_H)<<8)+BMP180_Read_Byte(MB_L);
	mc=(short)(BMP180_Read_Byte(MC_H)<<8)+BMP180_Read_Byte(MC_L);
	md=(short)(BMP180_Read_Byte(MD_H)<<8)+BMP180_Read_Byte(MD_L);
	
	BMP180_Write_Byte(CTRL_MEAS,0x74);
	Delay_ms(8);
	UT=(BMP180_Read_Byte(OUT_MSB)<<8)+BMP180_Read_Byte(OUT_LSB);
	
	BMP180_Write_Byte(CTRL_MEAS,0x74);
	Delay_ms(8);
	UT=(BMP180_Read_Byte(OUT_MSB)<<8)+BMP180_Read_Byte(OUT_LSB);
	
	BMP180_Write_Byte(CTRL_MEAS,0x34+(oss<<6));
	Delay_ms(8);
	UP=((BMP180_Read_Byte(OUT_MSB)<<16)+(BMP180_Read_Byte(OUT_LSB)<<8)
	      +BMP180_Read_Byte(OUT_XLSB) )>>(8-oss);
	//Caculate the real temperature
	x1=(UT-ac6)*ac5/pow(2,15);
	x2=mc*pow(2,11)/x1+md;
	b5=x1+x2;
	temperature=(b5+8)/pow(2,4);
	//Caculate the pressuer
	b6=b5-4000;
	x1=(b2*(b6*b6/pow(2,12)))/pow(2,11);
	x2=ac2*b6/pow(2,11);
	x3=x1+x2;
	b3=((((ac1*4+x3))<<oss)+2)/4;//
	x1=ac3*b6/pow(2,13);
	x2=(b1*(b6*b6/pow(2,12)))/pow(2,16);
	x3=((x1+x2)+2)/pow(2,2);
	b4=ac4*(unsigned long)(x3+32768)/pow(2,15);
	b7=((unsigned long)UP-b3)*(50000>>oss);
	if(b7<0x80000000) p=(b7*2)/b4;
	else p=(b7/b4)*2;
	x1=(p/pow(2,8))*(p/pow(2,8));
	x1=(x1*3038)/pow(2,16);
	x2=(-7357*p)/pow(2,16);
	pressure=p+(x1+x2+3791)/16;
	//caculate the high
	altitude=44330u*(1-pow((p/p0),(1.0/5.255)));
}*/
void BMP180_ReadCalibrationData(void)
{
	  bmp.AC1 = BMP180_Read_TwoByte(0xAA);
    bmp.AC2 = BMP180_Read_TwoByte(0xAC);
    bmp.AC3 = BMP180_Read_TwoByte(0xAE);
    bmp.AC4 = BMP180_Read_TwoByte(0xB0);
    bmp.AC5 = BMP180_Read_TwoByte(0xB2);
    bmp.AC6 = BMP180_Read_TwoByte(0xB4);
    bmp.B1  = BMP180_Read_TwoByte(0xB6);
    bmp.B2  = BMP180_Read_TwoByte(0xB8);
    bmp.MB  = BMP180_Read_TwoByte(0xBA);
    bmp.MC  = BMP180_Read_TwoByte(0xBC);
    bmp.MD  = BMP180_Read_TwoByte(0xBE);
}
long BMP180_Read_UT(void)
{
    long temp = 0;
	  #if oss==0
    BMP180_Write_Byte(0xF4,0x2E);
	  #elif oss==1
	  BMP180_Write_Byte(0xF4,0x74);
    #endif
    Delay_ms(5);
    temp = (long)BMP180_Read_TwoByte(0xF6);
    return temp;
}
long BMP180_Read_UP(void)
{
    long pressure = 0;

    BMP180_Write_Byte(0xF4,0x34+(oss<<6));
    Delay_ms(5);
    pressure = (long)BMP180_Read_TwoByte(0xF6);
    //pressure = pressure + BMP_ReadOneByte(0xf8);
    pressure &= 0x0000FFFF;
    return pressure;
}
void BMP_UncompemstatedToTrue(void)
{   
	 // BMP180_ReadCalibrationData();
    bmp.UT = BMP180_Read_UT();//第一次读取错误
    bmp.UT = BMP180_Read_UT();//进行第二次读取修正参数
    bmp.UP = BMP180_Read_UP();

    bmp.X1 = ((bmp.UT - bmp.AC6) * bmp.AC5) >> 15;
    bmp.X2 = (((long)bmp.MC) << 11) / (bmp.X1 + bmp.MD);
    bmp.B5 = bmp.X1 + bmp.X2;
    bmp.Temp  = (bmp.B5 + 8) >> 4;

    bmp.B6 = bmp.B5 - 4000;
    bmp.X1 = ((long)bmp.B2 * (bmp.B6 * bmp.B6 >> 12)) >> 11;
    bmp.X2 = ((long)bmp.AC2) * bmp.B6 >> 11;
    bmp.X3 = bmp.X1 + bmp.X2;

    bmp.B3 = ((((long)bmp.AC1) * 4 + bmp.X3) + 2) /4;
    bmp.X1 = ((long)bmp.AC3) * bmp.B6 >> 13;
    bmp.X2 = (((long)bmp.B1) *(bmp.B6*bmp.B6 >> 12)) >>16;
    bmp.X3 = ((bmp.X1 + bmp.X2) + 2) >> 2;
    bmp.B4 = ((long)bmp.AC4) * (unsigned long)(bmp.X3 + 32768) >> 15;
    bmp.B7 = ((unsigned long)bmp.UP - bmp.B3) * 50000;
    if(bmp.B7 < 0x80000000)
    {
        bmp.p = (bmp.B7 * 2) / bmp.B4;     
    }
    else
    {
        bmp.p = (bmp.B7 / bmp.B4) * 2;
    }
    bmp.X1 = (bmp.p >> 8) * (bmp.p >>8);
    bmp.X1 = (((long)bmp.X1) * 3038) >> 16;
    bmp.X2 = (-7357 * bmp.p) >> 16;
    bmp.p = bmp.p + ((bmp.X1 + bmp.X2 + 3791) >> 4);
    bmp.altitude = 44330 * (1-pow(((bmp.p) / 101325.0),(1.0/5.255)));  
}
void BMP_CaculateData(void)
{
	  bmp.X1 = ((bmp.UT - bmp.AC6) * bmp.AC5) >> 15;
    bmp.X2 = (((long)bmp.MC) << 11) / (bmp.X1 + bmp.MD);
    bmp.B5 = bmp.X1 + bmp.X2;
    bmp.Temp  = (bmp.B5 + 8) >> 4;

    bmp.B6 = bmp.B5 - 4000;
    bmp.X1 = ((long)bmp.B2 * (bmp.B6 * bmp.B6 >> 12)) >> 11;
    bmp.X2 = ((long)bmp.AC2) * bmp.B6 >> 11;
    bmp.X3 = bmp.X1 + bmp.X2;

    bmp.B3 = ((((long)bmp.AC1) * 4 + bmp.X3) + 2) /4;
    bmp.X1 = ((long)bmp.AC3) * bmp.B6 >> 13;
    bmp.X2 = (((long)bmp.B1) *(bmp.B6*bmp.B6 >> 12)) >>16;
    bmp.X3 = ((bmp.X1 + bmp.X2) + 2) >> 2;
    bmp.B4 = ((long)bmp.AC4) * (unsigned long)(bmp.X3 + 32768) >> 15;
    bmp.B7 = ((unsigned long)bmp.UP - bmp.B3) * 50000;
    if(bmp.B7 < 0x80000000)
    {
        bmp.p = (bmp.B7 * 2) / bmp.B4;     
    }
    else
    {
        bmp.p = (bmp.B7 / bmp.B4) * 2;
    }
    bmp.X1 = (bmp.p >> 8) * (bmp.p >>8);
    bmp.X1 = (((long)bmp.X1) * 3038) >> 16;
    bmp.X2 = (-7357 * bmp.p) >> 16;
    bmp.p = bmp.p + ((bmp.X1 + bmp.X2 + 3791) >> 4);
    bmp.altitude = 44330 * (1-pow(((bmp.p) / 101325.0),(1.0/5.255)));  
}
u8 BMP180_Check(void)
{
	u8 id;
	id=BMP180_Read_Byte(BMP180_ID);
	if(id==0x55) return 1;
	else return 0;
}
void BMP180_Write_Byte(u8 addr,u8 data)
{
	IIC_Start();
	IIC_Send_Byte(BMP180_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
  IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();
}
u8 BMP180_Read_Byte(u8 addr)
{
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(BMP180_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(BMP180_SlaveAddress+1);
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	return temp;
}
short BMP180_Read_TwoByte(u8 addr) //address is the msb register
{
	u8 tempH,tempL;
	IIC_Start();
	IIC_Send_Byte(BMP180_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(BMP180_SlaveAddress+1);
	IIC_Wait_Ack();
	tempH=IIC_Read_Byte(1);
	tempL=IIC_Read_Byte(0);
	IIC_Stop();
	return (tempH<<8)+tempL;
}