#include "ili9488.h"
#include "systick.h"
static pFONT *LCD_Fonts;	
void LCD_FillRect(u16 x, u16 y, u16 width, u16 height);
void LCD_SetColor(u32 Color);
struct	
{
	u16 ID;        //控制器ID	
	u16 Width;		// 水平分辨率
	u16 Height;		// 垂直分辨率
	u32 Color;  		//	LCD当前画笔颜色
	u32 BackColor;		//	背景色
	u8  Direction;		//	显示方向
	u8  ShowNum_Mode;	// 数字填充模式
}LCD;
void LCD_DisplayFloat(u16 x,u16 y,float num)
{
	u32 int_=0;
	u16 float_;
	if(num>=0) 
	{
		LCD_DisplayChar(x,y,'+');
		int_ = num;
		float_ = ((u32)(num*100))%(100);
		LCD_DisplayNumber(x+10,y,int_,3);
		LCD_DisplayChar(x+40,y,'.');
		LCD_DisplayNumber(x+48,y,float_,2);
	}
	else
	{
		LCD_DisplayChar(x,y,'-');
		int_ = -num;
		float_ = (short)(-num*100)% 100;
		LCD_DisplayNumber(x+10,y,int_,3);
		LCD_DisplayChar(x+40,y,'.');
		LCD_DisplayNumber(x+48,y,float_,2);
	}
}
void LCD_Init(void)
{
	 LCD_GPIO_Init();
	 
   FSMC_Init();	
	 LCD_Reset(0);
	 Delay_ms(120);
	 LCD_Reset(1);
	 Delay_ms(120);
	 
	 LCD_WR_CMD(0xD3);
	 LCD.ID=LCD_RD_DATA();     //假读
	 LCD.ID=LCD_RD_DATA();     //读0x00
	 LCD.ID=LCD_RD_DATA();     //读0x94
	 LCD.ID<<=8;               //0x94移入高8位
	 LCD.ID|=LCD_RD_DATA();    //读0x88并获取完整ID号
	// printf(" LCD ID:ILI%x\r\n",lcd.ID);
	 //if(LCD.ID!=0x9488)while(1);
	
	 LCD_WR_CMD(0xE0); //P-Gamma
	 LCD_WR_DATA(0x00);
	 LCD_WR_DATA(0x03);
	 LCD_WR_DATA(0x0C);
	 LCD_WR_DATA(0x09);
	 LCD_WR_DATA(0x17);
	 LCD_WR_DATA(0x09);
	 LCD_WR_DATA(0x3E);
	 LCD_WR_DATA(0x89);
	 LCD_WR_DATA(0x49);
	 LCD_WR_DATA(0x08);
	 LCD_WR_DATA(0x0D);
	 LCD_WR_DATA(0x0A);
	 LCD_WR_DATA(0x13);
	 LCD_WR_DATA(0x15);
	 LCD_WR_DATA(0x0F);
	 LCD_WR_CMD(0XE1); //N-Gamma
   LCD_WR_DATA(0x00);
	 LCD_WR_DATA(0x11);
	 LCD_WR_DATA(0x15);
	 LCD_WR_DATA(0x03);
	 LCD_WR_DATA(0x0F);
	 LCD_WR_DATA(0x05);
	 LCD_WR_DATA(0x2D);
	 LCD_WR_DATA(0x34);
	 LCD_WR_DATA(0x41);
	 LCD_WR_DATA(0x02);
	 LCD_WR_DATA(0x0B);
	 LCD_WR_DATA(0x0A);
	 LCD_WR_DATA(0x33);
	 LCD_WR_DATA(0x37);
	 LCD_WR_DATA(0x0F);

	 LCD_WR_CMD(0XC0); //Power Control 1
	 LCD_WR_DATA(0x17); //Vreg1out
	 LCD_WR_DATA(0x15); //Verg2out
	 LCD_WR_CMD(0xC1); //Power Control 2
	 LCD_WR_DATA(0x41); //VGH,VGL
	 LCD_WR_CMD(0xC5); //Power Control 3
	 LCD_WR_DATA(0x00);
	 LCD_WR_DATA(0x12); //Vcom
	 LCD_WR_DATA(0x80);

	 LCD_WR_CMD(0x36); //Memory Access
	 LCD_WR_DATA(0x48);
	 LCD_WR_CMD(0x3A); // Interface Pixel Format
	 LCD_WR_DATA(0x55);
	 LCD_WR_CMD(0XB0); // Interface Mode Control
	 LCD_WR_DATA(0x00);
	 LCD_WR_CMD(0xB1); //Frame rate
	 LCD_WR_DATA(0xA0); //60Hz
	 LCD_WR_CMD(0xB4); //Display Inversion Control
	 LCD_WR_DATA(0x02); //2-dot
	 LCD_WR_CMD(0XB6); //RGB/MCU Interface Control
	 LCD_WR_DATA(0x02); //MCU
	 LCD_WR_DATA(0x02); //Source,Gate scan dieection
	 LCD_WR_CMD(0XE9); // Set Image Function
	 LCD_WR_DATA(0x00); // Disable 24 bit data input
	 LCD_WR_CMD(0xF7);// Adjust Control
	 LCD_WR_DATA(0xA9);
	 LCD_WR_DATA(0x51);
	 LCD_WR_DATA(0x2C);
	 LCD_WR_DATA(0x82);
	// D7 stream, loose
	 LCD_WR_CMD(0x11); //Sleep out
	 Delay_ms(120);
	 LCD_WR_CMD(0x29); //Display on
	
	 LCD_SetBackColor(LCD_BLACK); 	// 设置背景色
  	LCD_SetColor(LCD_WHITE);		// 设置画笔颜色
	 LCD_SetFont(&Font20);  			// 设置默认字体
	 LCD_ShowNumMode(Fill_Space);	// 设置数字填充模式
	 LCD_DisplayMode(Mode_H);		// 设置显示方向
	 LCD_Clear();		// 清屏
	 LCD_BackLight(1);
}


void LCD_GPIO_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure; //结构体变量定义
	RCC_AHB1PeriphClockCmd(LCD_CLK,ENABLE);//使能GPIO的复用功能时钟
	
 	/*--------------------- ---初始化FSMC引脚-----------------------------------*/ 
	
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉

	GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN;				 
 	GPIO_Init(LCD_CS_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_CS_PORT,LCD_CS_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;				 
 	GPIO_Init(LCD_RS_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_RS_PORT,LCD_RS_PINS,GPIO_AF_FSMC);
	
	GPIO_InitStructure.GPIO_Pin = LCD_WR_PIN;				 
 	GPIO_Init(LCD_WR_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_WR_PORT,LCD_WR_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_RD_PIN;				 
 	GPIO_Init(LCD_RD_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_RD_PORT,LCD_RD_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D0_PIN;				 
 	GPIO_Init(LCD_D0_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D0_PORT,LCD_D0_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D1_PIN;				 
 	GPIO_Init(LCD_D1_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D1_PORT,LCD_D1_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D2_PIN;				 
 	GPIO_Init(LCD_D2_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D2_PORT,LCD_D2_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D3_PIN;				 
 	GPIO_Init(LCD_D3_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D3_PORT,LCD_D3_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D4_PIN;				 
 	GPIO_Init(LCD_D4_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D4_PORT,LCD_D4_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D5_PIN;				 
 	GPIO_Init(LCD_D5_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D5_PORT,LCD_D5_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D6_PIN;				 
 	GPIO_Init(LCD_D6_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D6_PORT,LCD_D6_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D7_PIN;				 
 	GPIO_Init(LCD_D7_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D7_PORT,LCD_D7_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D8_PIN;				 
 	GPIO_Init(LCD_D8_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D8_PORT,LCD_D8_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D9_PIN;				 
 	GPIO_Init(LCD_D9_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D9_PORT,LCD_D9_PINS,GPIO_AF_FSMC);
	
	GPIO_InitStructure.GPIO_Pin = LCD_D10_PIN;				 
 	GPIO_Init(LCD_D10_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D10_PORT,LCD_D10_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D11_PIN;				 
 	GPIO_Init(LCD_D11_PORT, &GPIO_InitStructure); 
	GPIO_PinAFConfig(LCD_D11_PORT,LCD_D11_PINS,GPIO_AF_FSMC);
	
	GPIO_InitStructure.GPIO_Pin = LCD_D12_PIN;				 
 	GPIO_Init(LCD_D12_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D12_PORT,LCD_D12_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D13_PIN;				 
 	GPIO_Init(LCD_D13_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D13_PORT,LCD_D13_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D14_PIN;				 
 	GPIO_Init(LCD_D14_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D14_PORT,LCD_D14_PINS,GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = LCD_D15_PIN;				 
 	GPIO_Init(LCD_D15_PORT, &GPIO_InitStructure); 
   GPIO_PinAFConfig(LCD_D15_PORT,LCD_D15_PINS,GPIO_AF_FSMC);
	
 	/*-------------------------初始化其他控制引脚--------------------------------*/ 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//2MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	
	//初始化复位引脚
 	GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN;				
 	GPIO_Init(LCD_RST_PORT, &GPIO_InitStructure);	
	
	//初始化背光引脚
 	GPIO_InitStructure.GPIO_Pin = LCD_BL_PIN;			 
 	GPIO_Init(LCD_BL_PORT, &GPIO_InitStructure);		
	GPIO_SetBits(LCD_BL_PORT,LCD_BL_PIN);			//开启LCD背光	
	
	
}
void LCD_BackLight(u8 sta)
{
		if(sta)	
			GPIO_SetBits(GPIOA, GPIO_Pin_11);
		else
			GPIO_ResetBits(GPIOA, GPIO_Pin_11);
}
void LCD_Reset(u8 sta)
{
		if(sta)	
			GPIO_SetBits(GPIOA, GPIO_Pin_12);
		else
			GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}
void LCD_WR_CMD(u16 Index)
{
		*(__IO u16*)FSMC_CMD = Index;
}
void LCD_WR_DATA(u16 Index)
{
		*(__IO u16*)FSMC_DATA = Index;
}
u16 LCD_RD_DATA(void)
{
	u16 t;
	t= *(__IO u16*)FSMC_DATA;
	return t;
}

//******************************************
void LCD_SetCursor(u16 x, u16 y)
{	   
   //ILI9488设置坐标时必须把END坐标也写进去。因为END坐标通常不确定，这里直接写最大坐标
	LCD_WR_CMD(0x2A); 							
	LCD_WR_DATA(x>>8);LCD_WR_DATA(x&0XFF); 	//设置水平坐标，高位在前	
	LCD_WR_DATA(0x01);   // 设置最大X坐标为480
	LCD_WR_DATA(0xDF); 	
	LCD_WR_CMD(0x2B); 						
	LCD_WR_DATA(y>>8);LCD_WR_DATA(y&0XFF); 	//设置垂直坐标，高位在前	
	LCD_WR_DATA(0x01);   // 设置最大Y坐标为480
	LCD_WR_DATA(0xDF); 
} 

/*****************************************************************************************
*	函 数 名:	 LCD_SetColor
*	入口参数:	 Color - 要设置的颜色
*	返 回 值: 无
*	函数功能: 设置画笔的颜色，用于显示字符、绘制图形等
*	说    明: 虽然反客3.5寸的屏幕使用的颜色格式是16位的RGB565，但是这里为了方便用户使用，
*				 入口参数使用的是24位的RGB888颜色，然后再通过代码转换成16位色。用户可以很方便
*				 的在电脑用取色器获取24位的颜色，再将此24位颜色值输入该函数即可。		 
******************************************************************************************/

void LCD_SetColor(u32 Color)
{
	u16 Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值
	
	Red_Value   = (u16)((Color&0xF80000)>>8);		// 提取红色的值
	Green_Value = (u16)((Color&0x00FC00)>>5);		// 提取绿色的值
	Blue_Value  = (u16)((Color&0x0000F8)>>3);		// 提取蓝色的值
	
	LCD.Color = (u16)(Red_Value | Green_Value | Blue_Value);		// 合成16位色，颜色格式RGB565
}

/*****************************************************************************************
*	函 数 名:	 LCD_SetBackColor
*	入口参数:	 Color - 要设置的颜色
*	返 回 值: 无
*	函数功能: 用于清屏或字体的显示背景颜色
*	说    明: 虽然反客3.5寸的屏幕使用的颜色格式是16位的RGB565，但是这里为了方便用户使用，
*				 入口参数使用的是24位的RGB888颜色，然后再通过代码转换成16位色。用户可以很方便
*				 的在电脑用取色器获取24位的颜色，再将此24位颜色值输入该函数即可。		 
******************************************************************************************/

void LCD_SetBackColor(u32 Color)
{
	u16 Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值
	
	Red_Value   = (u16)((Color&0xF80000)>>8);	// 提取红色的值
	Green_Value = (u16)((Color&0x00FC00)>>5); // 提取绿色的值
	Blue_Value  = (u16)((Color&0x0000F8)>>3); // 提取蓝色的值
	
	LCD.BackColor = (u16)(Red_Value | Green_Value | Blue_Value);	// 合成16位色，颜色格式RGB565
}

/*****************************************************************************************
*	函 数 名:	 LCD_DisplayMode
*	入口参数:	 direction - 横屏或竖屏显示，可选参数 Mode_H 横屏，Mode_V 竖直
*	返 回 值: 无
*	函数功能: 设置液晶屏的显示方向，横屏或竖屏
*	说    明: 无	 
******************************************************************************************/

void LCD_DisplayMode(u8 direction)
{
	LCD.Direction = direction;		// 修改全局变量，方向标志位
	
	if (LCD.Direction == Mode_H) 		// 横屏显示
	{
		LCD_WR_CMD(0X36);		// 扫描方向设置寄存器
		LCD_WR_DATA(0XE8); 	//	扫描方向，从上到下	，从右到左
		
		LCD_WR_CMD(0x2A); 	
		LCD_WR_DATA(0x00);	// 设置x坐标（在这里没有作用，只是需要先写入而已）
		LCD_WR_DATA(0x00); 			
		LCD_WR_DATA(0x01);	// 设置最大x坐标为320
		LCD_WR_DATA(0XDF); 
			
		LCD_WR_CMD(0x2B); 
		LCD_WR_DATA(0x00);	// 设置y坐标（在这里没有作用，只是需要先写入而已）
		LCD_WR_DATA(0x00); 		
		LCD_WR_DATA(0x01);	// 设置最大y坐标为240
		LCD_WR_DATA(0x3F); 		
		
		LCD.Width  = 480;		// 水平宽度320
		LCD.Height = 320;		// 垂直宽度240
	}
	else if(LCD.Direction == Mode_V)	// 竖屏显示
	{
		LCD_WR_CMD(0X36);		// 扫描方向设置寄存器
		LCD_WR_DATA(0X48); 	//	扫描方向，从左到右，从上到下	
		
		LCD_WR_CMD(0x2A); 	
		LCD_WR_DATA(0x00);	// 设置x坐标（在这里没有作用，只是需要先写入而已）
		LCD_WR_DATA(0x00); 			
		LCD_WR_DATA(0x01);   // 设置最大x坐标为240
		LCD_WR_DATA(0x3F); 				
                           
		LCD_WR_CMD(0x2B);    
		LCD_WR_DATA(0x00);   // 设置y坐标（在这里没有作用，只是需要先写入而已）
		LCD_WR_DATA(0x00); 			
		LCD_WR_DATA(0x01);   // 设置最大y坐标为320
		LCD_WR_DATA(0XDF); 			
		                     
		LCD.Width  = 320;    // 水平宽度240
		LCD.Height = 480;		// 垂直宽度320
	}
}
/*****************************************************************************************
*	函 数 名:	 LCD_Clear
*	入口参数: 无
*	返 回 值: 无
*	函数功能: 清除整个屏幕的显示
*	说    明: 清除的颜色为背景色，用LCD_SetBackColor设置
******************************************************************************************/

void LCD_Clear(void)
{ 
	u32 i = 0;
	
	LCD_SetCursor(0,0);	// 设置起始坐标
	LCD_WR_CMD(0X2C);		// 开始写入GRAM

	for(i=0;i<320*480;i++)	// 总共需要写入320*240个点，ILI9341会根据屏幕的分辨率自动换行显示
	{
		*(__IO u16*)FSMC_DATA = LCD.BackColor;	// 写入背景色
	}
}

/*****************************************************************************************
*	函 数 名:	 LCD_DrawPoint
*	入口参数:	 x - 水平坐标
*			  	 y - 垂直坐标
*				 color - 要显示的颜色
*	返 回 值: 无
*	函数功能: 在制定坐标处绘制指定颜色的点
*	说    明: 无
******************************************************************************************/

void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_SetCursor(x,y);	// 设置坐标
	LCD_WR_CMD(0X2C);		// 写入GRAM
	*(__IO u16*)FSMC_DATA  = color; 	// 写入颜色
}

/*****************************************************************************************
*	函 数 名:	 LCD_SetFont
*	入口参数:	 *fonts - 相应字体结构体指针的地址
*	返 回 值: 无
*	函数功能: 设置字体
*	说    明: 需要事先取模，字体选择，Font32/Font24/Font20/Font16/Font12
******************************************************************************************/

void LCD_SetFont(pFONT *fonts)
{
	LCD_Fonts = fonts;
}

/*****************************************************************************************
*	函 数 名:	 LCD_DisplayChar
*	入口参数:	 x - 水平坐标
*			  	 y - 垂直坐标
*				 c - 字符的ASCII值
*	返 回 值: 无
*	函数功能: 在指定坐标处显示单个ASCII字符
*	说    明: 可通过 LCD_SetFont() 函数设置相应的字体
******************************************************************************************/

void LCD_DisplayChar(u16 x, u16 y,u8 add)
{
	u16  index = 0, counter = 0;
   u8   disChar;	//字模的值
	u16  Xaddress = x; //水平坐标
	
	add = add - 32; 
	for(index = 0; index < LCD_Fonts->Sizes; index++)
	{
		disChar = LCD_Fonts->table[add*LCD_Fonts->Sizes + index]; //获取字符的模值
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
				LCD_DrawPoint(Xaddress,y,LCD.Color);	//当前模值不为0时，使用画笔色绘点
			}
			else		
			{		
				LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//否则使用背景色绘制点
			}
			disChar >>= 1;
			Xaddress++;  //水平坐标自加
			
			if( (Xaddress - x)==LCD_Fonts->Width ) //如果水平坐标达到了字符宽度，则退出当前循环
			{													//进入下一行的绘制
				Xaddress = x;
				y++;
				break;
			}
		}	
	}	
}

/*****************************************************************************************
*	函 数 名:	 LCD_DisplayString
*	入口参数:	 x - 水平坐标
*			  	 y - 垂直坐标
*				 *p - 要显示的字符串
*	返 回 值: 无
*	函数功能: 在指定坐标处显示字符串
*	说    明: 可通过 LCD_SetFont() 函数设置相应的字体
******************************************************************************************/

void LCD_DisplayString( u16 x, u16 y, u8 *p) 
{  
	while( (x<LCD.Width)&&(*p != 0) )	//判断显示坐标是否超出显示区域并且字符是否为空字符
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_Fonts->Width; //显示下一个字符
		 p++;	//取下一个字符地址
	}
}

/*****************************************************************************************
*	函 数 名:	 GetNumber
*	入口参数:	 number - 目标整数
*			  	 size - 目标数的长度
*	返 回 值: 取整数的第N位的数，用于拆分整数以进行显示
*	函数功能: 取出的第N位的数
*	说    明: 在 LCD_DisplayNumber()里被调用
******************************************************************************************/

u8 GetNumber(u32 num,u8 size)
{
	u32 numPow = 1;
	u8  vaule;
	
	while(size>0)
	{
		numPow *=10;
		size--;
	}
	vaule = (num/numPow)%10;
	return vaule;	
}

/*****************************************************************************************
*	函 数 名:	 LCD_ShowNumMode
*	入口参数:	 mode - 数字填充模式，Fill_Space 填充空格，Fill_Zero 填充零
*	返 回 值: 无
*	函数功能: 设置数字显示的填充模式
*	说    明: 使用 LCD_DisplayNumber() 显示数字时，调用此函数设置高位的填充模式，
*				 通常在显示时间或日期的时候调用
******************************************************************************************/

void LCD_ShowNumMode(u8 mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************
*	函 数 名:	 LCD_DisplayNumber
*	入口参数:	 x - 水平坐标
*			  	 y - 垂直坐标
*				 number - 要显示的数字
*				 len - 数字的长度
*	返 回 值: 无
*	函数功能: 显示整数变量
*	说    明: 需要指定显示的位数
******************************************************************************************/

void LCD_DisplayNumber( u16 x, u16 y, u32 number, u8 len) 
{  
	u8 i,value;
	u8 zero_Flag = 0;

	for(i=0;i<len;i++)
	{
		value = GetNumber(number,len-i-1);	//获取当前整数的第N位数
		if( zero_Flag==0 && (i<len-1) )	//判断前面为0的部分
		{
			if(value == 0)
			{
				if(LCD.ShowNum_Mode == 0)	
					LCD_DisplayChar( x+i*LCD_Fonts->Width, y,48);	//填充0
				else
					LCD_DisplayChar( x+i*LCD_Fonts->Width, y,32);	//填充空格
				continue;
			}
			else
				zero_Flag = 1;	//当遇到第一个非0数时置1
		}									
		LCD_DisplayChar( x+i*LCD_Fonts->Width, y, value+48 );	//将拆分的数进行显示
	}
}

/*****************************************************************************************
*	函 数 名:	 LCD_DrawLine
*	入口参数:	 x1、y1 - 起点坐标
*			  	 x2、y2 - 终点坐标
*	返 回 值: 无
*	函数功能: 绘制直线
*	说    明: 无
******************************************************************************************/

#define ABS(X)  ((X) > 0 ? (X) : -(X))    

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;

	deltax = ABS(x2 - x1);        /* The difference between the x's */
	deltay = ABS(y2 - y1);        /* The difference between the y's */
	x = x1;                       /* Start x off at the first pixel */
	y = y1;                       /* Start y off at the first pixel */

	if (x2 >= x1)                 /* The x-values are increasing */
	{
	 xinc1 = 1;
	 xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
	 xinc1 = -1;
	 xinc2 = -1;
	}

	if (y2 >= y1)                 /* The y-values are increasing */
	{
	 yinc1 = 1;
	 yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
	 yinc1 = -1;
	 yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
	 xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	 yinc2 = 0;                  /* Don't change the y for every iteration */
	 den = deltax;
	 num = deltax / 2;
	 numadd = deltay;
	 numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
	 xinc2 = 0;                  /* Don't change the x for every iteration */
	 yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	 den = deltay;
	 num = deltay / 2;
	 numadd = deltax;
	 numpixels = deltay;         /* There are more y-values than x-values */
	}
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
	 LCD_DrawPoint(x,y,LCD.Color);             /* Draw the current pixel */
	 num += numadd;              /* Increase the numerator by the top of the fraction */
	 if (num >= den)             /* Check if numerator >= denominator */
	 {
		num -= den;               /* Calculate the new numerator value */
		x += xinc1;               /* Change the x as appropriate */
		y += yinc1;               /* Change the y as appropriate */
	 }
	 x += xinc2;                 /* Change the x as appropriate */
	 y += yinc2;                 /* Change the y as appropriate */
	}
}

/*****************************************************************************************
*	函 数 名:	 LCD_DrawRect
*	入口参数:	 x、y - 起点坐标
*			  	 width - 矩形长度
*				 height - 矩形宽度
*	返 回 值: 无
*	函数功能: 绘制矩形框
*	说    明: 无
******************************************************************************************/

void LCD_DrawRect(u16 x, u16 y, u16 width, u16 height)
{
	/* draw horizontal lines */
	LCD_DrawLine(x, y, x+width, y);
	LCD_DrawLine(x, y+height, x+width, y+height);

	/* draw vertical lines */
	LCD_DrawLine(x, y, x, y+height);
	LCD_DrawLine(x+width, y, x+width, y+height);
}

/*****************************************************************************************
*	函 数 名:	 LCD_DrawCircle
*	入口参数:	 x、y - 起点坐标
*			  	 r - 半径
*	返 回 值: 无
*	函数功能: 绘制圆形框
*	说    明: 半径不能大于xy坐标的值
******************************************************************************************/

void LCD_DrawCircle(u16 x, u16 y, u16 r)
{
	int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
	do {   

		LCD_DrawPoint(x-Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y-Yadd,LCD.Color);
		LCD_DrawPoint(x-Xadd,y-Yadd,LCD.Color);
		
		e2 = err;
		if (e2 <= Yadd) {
			err += ++Yadd*2+1;
			if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
		}
		if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0); 
}

/*****************************************************************************************
*	函 数 名:	 LCD_DrawEllipse
*	入口参数:	 x、y - 中心坐标
*			  	 r1、r2 - 分别为水平和垂直半轴的长度
*	返 回 值: 无
*	函数功能: 绘制椭圆框
*	说    明: 无
******************************************************************************************/

void LCD_DrawEllipse(int x, int y, int r1, int r2)
{
  int Xadd = -r1, Yadd = 0, err = 2-2*r1, e2;
  float K = 0, rad1 = 0, rad2 = 0;
   
  rad1 = r1;
  rad2 = r2;
  
  if (r1 > r2)
  { 
    do {
      K = (float)(rad1/rad2);
		 
		LCD_DrawPoint(x-Xadd,y+(u16)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y+(u16)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y-(u16)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x-Xadd,y-(u16)(Yadd/K),LCD.Color);     
		 
      e2 = err;
      if (e2 <= Yadd) {
        err += ++Yadd*2+1;
        if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
      }
      if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
  }
  else
  {
    Yadd = -r2; 
    Xadd = 0;
    do { 
      K = (float)(rad2/rad1);

		LCD_DrawPoint(x-(u16)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(u16)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(u16)(Xadd/K),y-Yadd,LCD.Color);
		LCD_DrawPoint(x-(u16)(Xadd/K),y-Yadd,LCD.Color);  
		 
      e2 = err;
      if (e2 <= Xadd) {
        err += ++Xadd*3+1;
        if (-Yadd == Xadd && e2 <= Yadd) e2 = 0;
      }
      if (e2 > Yadd) err += ++Yadd*3+1;     
    }
    while (Yadd <= 0);
  }
}

/*****************************************************************************************
*	函 数 名:	 LCD_FillRect
*	入口参数:	 x、y - 中心坐标
*				 width - 长度
*				 height - 宽度	
*	返 回 值: 无
*	函数功能: 填充矩形区域
*	说    明: 无
******************************************************************************************/

void LCD_FillRect(u16 x, u16 y, u16 width, u16 height)
{
	u16 i = 0,j = 0;

	for(i=0;i<height;i++)
	{
		LCD_SetCursor(x,y+i);   // 设置光标位置 
		LCD_WR_CMD(0X2C);			// 开始写入GRAM
		for(j=0;j<width;j++)
		{
			*(__IO u16*)FSMC_DATA = LCD.Color;	//写入数据 
		}
	}
}

/*****************************************************************************************
*	函 数 名:	 LCD_FillCircle
*	入口参数:	 x、y - 中心坐标
*				 r - 半径
*	返 回 值: 无
*	函数功能: 填充圆形区域
*	说    明: 无
******************************************************************************************/

void LCD_FillCircle(u16 x, u16 y, u16 r)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (r << 1);
  
  CurX = 0;
  CurY = r;
  
  while (CurX <= CurY)
  {
    if(CurY > 0) 
    { 
		LCD_DrawLine(x - CurX, y - CurY,x - CurX,y - CurY + 2*CurY);
		LCD_DrawLine(x + CurX, y - CurY,x + CurX,y - CurY + 2*CurY); 
    }
    
    if(CurX > 0) 
    {
		LCD_DrawLine(x - CurY, y - CurX,x - CurY,y - CurX + 2*CurX);
		LCD_DrawLine(x + CurY, y - CurX,x + CurY,y - CurX + 2*CurX); 		 
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  
  LCD_DrawCircle(x, y, r);  
}


//以下是测试函数
void LCD_ClearTest(void)
{
	u16 time = 1000;	//延时时间

	LCD_SetFont(&Font24);		//设置字体
	LCD_SetColor(LCD_BLACK);	//设置画笔颜色

	LCD_SetBackColor(LCD_RED);    LCD_Clear();  Delay_ms(time);
	LCD_SetBackColor(LCD_GREEN);  LCD_Clear();  Delay_ms(time);
	LCD_SetBackColor(LCD_BLUE);   LCD_Clear();  Delay_ms(time);
	LCD_SetBackColor(LCD_GREY);   LCD_Clear();  Delay_ms(time);	
	LCD_SetBackColor(LCD_WHITE);   LCD_Clear();  Delay_ms(time);
	LCD_SetBackColor(LCD_BLACK);   LCD_Clear();  Delay_ms(time);
}
void LCD_TextTest(void)
{
	u16 time = 100;
	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色

	LCD_SetColor(LCD_WHITE);
	LCD_SetFont(&Font32); LCD_DisplayString(0, 0,"!#$%&'()*+,-.01"); 				Delay_ms(time);		
	LCD_SetFont(&Font24); LCD_DisplayString(0,32,"!#$%&'()*+,-.01234"); 			Delay_ms(time);
	LCD_SetFont(&Font20); LCD_DisplayString(0,56,"!#$%&'()*+,-.012345"); 		Delay_ms(time);	
	LCD_SetFont(&Font16); LCD_DisplayString(0,76,"!#$%&'()*+,-.01234567"); 		Delay_ms(time);	
	LCD_SetFont(&Font12); LCD_DisplayString(0,92,"!#$%&'()*+,-.0123456789:"); 	Delay_ms(time);	

	LCD_SetColor(LCD_CYAN);                                                      
	LCD_SetFont(&Font12); LCD_DisplayString(0,104,"!#$%&'()*+,-.0123456789:"); Delay_ms(time);	
	LCD_SetFont(&Font16); LCD_DisplayString(0,116,"!#$%&'()*+,-.01234567"); 	Delay_ms(time);	
	LCD_SetFont(&Font20); LCD_DisplayString(0,132,"!#$%&'()*+,-.012345"); 		Delay_ms(time);		
	LCD_SetFont(&Font24); LCD_DisplayString(0,152,"!#$%&'()*+,-.01234"); 		Delay_ms(time);		
	LCD_SetFont(&Font32); LCD_DisplayString(0,176,"!#$%&'()*+,-.01"); 			Delay_ms(time);	

	LCD_SetFont(&Font32);
	LCD_SetColor(LCD_YELLOW);
	LCD_DisplayNumber( 0,220,429496729,9);   Delay_ms(time);	
	LCD_ShowNumMode(Fill_Zero);	
	LCD_DisplayNumber( 0,252,123456,9);	     Delay_ms(time);
	LCD_ShowNumMode(Fill_Space);	
	LCD_DisplayNumber( 0,284,1234,9);		  Delay_ms(time);		

	Delay_ms(2000);	
}

void LCD_FillTest(void)
{
	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色

	LCD_SetFont(&Font16);
	LCD_SetColor(LCD_BLUE);	  	  LCD_FillRect(110,  3,130,16);	LCD_DisplayString(0,  3,"LCD_BLUE");		
	LCD_SetColor(LCD_GREEN);  	  LCD_FillRect(110, 25,130,16);  LCD_DisplayString(0, 25,"LCD_GREEN");		
	LCD_SetColor(LCD_RED);    	  LCD_FillRect(110, 47,130,16);  LCD_DisplayString(0, 47,"LCD_RED");			
	LCD_SetColor(LCD_CYAN);   	  LCD_FillRect(110, 69,130,16);  LCD_DisplayString(0, 69,"LCD_CYAN");		
	LCD_SetColor(LCD_MAGENTA);	  LCD_FillRect(110, 91,130,16);  LCD_DisplayString(0, 91,"LCD_MAGENTA");	
	LCD_SetColor(LCD_YELLOW); 	  LCD_FillRect(110,113,130,16);  LCD_DisplayString(0,113,"LCD_YELLOW");		
	LCD_SetColor(LCD_GREY);   	  LCD_FillRect(110,135,130,16);	LCD_DisplayString(0,135,"LCD_GREY");		
                                      
	LCD_SetColor(LIGHT_BLUE);	  LCD_FillRect(110,157,130,16);  LCD_DisplayString(0,157,"LIGHT_BLUE");		
	LCD_SetColor(LIGHT_GREEN);   LCD_FillRect(110,179,130,16);  LCD_DisplayString(0,179,"LIGHT_GREEN");	
	LCD_SetColor(LIGHT_RED);     LCD_FillRect(110,201,130,16);  LCD_DisplayString(0,201,"LIGHT_RED");	   
	LCD_SetColor(LIGHT_CYAN);    LCD_FillRect(110,223,130,16);  LCD_DisplayString(0,223,"LIGHT_CYAN");	   
	LCD_SetColor(LIGHT_MAGENTA); LCD_FillRect(110,245,130,16);  LCD_DisplayString(0,245,"LIGHT_MAGENTA");	
	LCD_SetColor(LIGHT_YELLOW);  LCD_FillRect(110,267,130,16);  LCD_DisplayString(0,267,"LIGHT_YELLOW");	
	LCD_SetColor(LIGHT_GREY);    LCD_FillRect(110,289,130,16);	LCD_DisplayString(0,289,"LIGHT_GREY");  	
                                                     
	Delay_ms(2000);
}
void LCD_ColorTest(void)
{
	u16 i;

	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色
	LCD_SetFont(&Font32);
	LCD_SetColor(LCD_BLACK);

	LCD_SetBackColor(LIGHT_MAGENTA); LCD_DisplayString(0,  0,"   Color Test   "); 
	LCD_SetBackColor(LCD_YELLOW);  	LCD_DisplayString(0, 40,"   Color Test   ");
	LCD_SetBackColor(LCD_CYAN);  	 	LCD_DisplayString(0, 80,"   Color Test   ");

	//使用画线函数绘制三基色色条
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_RED-(i<<16) );
		LCD_DrawLine(i,150,i,190);			
	}
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_GREEN-(i<<8) );
		LCD_DrawLine(i,200,i,240);	
	}
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_BLUE-i );
		LCD_DrawLine(i,250,i,290);	
	}	
	Delay_ms(2000);	
}
void LCD_GrahicTest(void)
{
	u16 time = 80;

	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色	

	LCD_SetColor(LCD_RED);
	LCD_DrawLine(0,10,240,10);	Delay_ms(time);	//画直线
	LCD_DrawLine(0,20,240,20);	Delay_ms(time);
	LCD_DrawLine(0,30,240,30); Delay_ms(time);
	LCD_DrawLine(0,40,240,40);	Delay_ms(time);	
	
	LCD_SetColor(LCD_YELLOW);	
	LCD_DrawRect( 5, 85,240,150); Delay_ms(time);	//绘制矩形
	LCD_DrawRect(30,100,190,120); Delay_ms(time);
	LCD_DrawRect(55,115,140,90);  Delay_ms(time);
	LCD_DrawRect(80,135,90,60);   Delay_ms(time);	

	LCD_SetColor(LIGHT_CYAN);
	LCD_DrawCircle(120,170,100);	Delay_ms(time);	//绘制圆形
	LCD_DrawCircle(120,170,80);   Delay_ms(time);
	LCD_DrawCircle(120,170,60);   Delay_ms(time);
	LCD_DrawCircle(120,170,40);   Delay_ms(time);

	LCD_SetColor(DARK_CYAN);
	LCD_DrawLine(0,285,240,285);	Delay_ms(time);	//画直线
	LCD_DrawLine(0,295,240,295);	Delay_ms(time);
	LCD_DrawLine(0,305,240,305);  Delay_ms(time);
	LCD_DrawLine(0,315,240,315);	Delay_ms(time);	
	Delay_ms(1000);

	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色	
	
	LCD_SetColor(LCD_RED);    LCD_FillCircle( 60,80,60);		//填充圆形
	LCD_SetColor(LCD_GREEN);  LCD_FillCircle(120,80,60); 	
	LCD_SetColor(LCD_BLUE);   LCD_FillCircle(160,80,60);  	

	LCD_SetColor(LIGHT_BLUE);
	LCD_DrawEllipse(120,250,110,50);  Delay_ms(time);	//绘制椭圆
	LCD_DrawEllipse(120,250, 95,40);  Delay_ms(time);
	LCD_DrawEllipse(120,250, 80,30);  Delay_ms(time);
	LCD_DrawEllipse(120,250, 65,20);  Delay_ms(time);

	Delay_ms(2000);	
}
void LCD_HorizontalText(void)
{
	u16 time = 100;
	
	LCD_ClearTest();	// 清屏测试函数

	
	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色
	LCD_SetColor(LCD_WHITE);
	LCD_SetFont(&Font32); LCD_DisplayString(0, 0,"!#$%&'()*+,-.0123456"); 		Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font24); LCD_DisplayString(0,32,"!#$%&'()*+,-.0123456789:"); 	Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font20); LCD_DisplayString(0,56,"!#$%&'()*+,-.0123456789:"); 	Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font16); LCD_DisplayString(0,76,"!#$%&'()*+,-.0123456789:"); 	Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font12); LCD_DisplayString(0,92,"!#$%&'()*+,-.0123456789:"); 	Delay_ms(time);	// 字符显示测试
	LCD_SetColor(LIGHT_YELLOW);                                                      
	LCD_SetFont(&Font12); LCD_DisplayString(0,104,"!#$%&'()*+,-.0123456789:"); Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font16); LCD_DisplayString(0,116,"!#$%&'()*+,-.0123456789:"); Delay_ms(time);	// 字符显示测试
	LCD_SetFont(&Font20); LCD_DisplayString(0,132,"!#$%&'()*+,-.0123456789:"); Delay_ms(time);	// 字符显示测试	
	LCD_SetFont(&Font24); LCD_DisplayString(0,152,"!#$%&'()*+,-.0123456789:"); Delay_ms(time);	// 字符显示测试	
	LCD_SetFont(&Font32); LCD_DisplayString(0,176,"!#$%&'()*+,-.0123456"); 		Delay_ms(time);	// 字符显示测试
	Delay_ms(2000);		
	
	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色
	LCD_SetFont(&Font16);
	LCD_SetColor(LCD_BLUE);	  	  LCD_FillRect(110,  3,210,16);	LCD_DisplayString(0,  3,"LCD_BLUE");		// 矩形填充测试
	LCD_SetColor(LCD_GREEN);  	  LCD_FillRect(110, 25,210,16);  LCD_DisplayString(0, 25,"LCD_GREEN");		// 矩形填充测试
	LCD_SetColor(LCD_RED);    	  LCD_FillRect(110, 47,210,16);  LCD_DisplayString(0, 47,"LCD_RED");			// 矩形填充测试
	LCD_SetColor(LCD_CYAN);   	  LCD_FillRect(110, 69,210,16);  LCD_DisplayString(0, 69,"LCD_CYAN");		// 矩形填充测试
	LCD_SetColor(LCD_MAGENTA);	  LCD_FillRect(110, 91,210,16);  LCD_DisplayString(0, 91,"LCD_MAGENTA");	// 矩形填充测试
	LCD_SetColor(LCD_YELLOW); 	  LCD_FillRect(110,113,210,16);  LCD_DisplayString(0,113,"LCD_YELLOW");		// 矩形填充测试
	LCD_SetColor(LCD_GREY);   	  LCD_FillRect(110,135,210,16);	LCD_DisplayString(0,135,"LCD_GREY");		// 矩形填充测试
                                                     
	LCD_SetColor(LIGHT_BLUE);	  LCD_FillRect(110,157,210,16);  LCD_DisplayString(0,157,"LIGHT_BLUE");		// 矩形填充测试
	LCD_SetColor(LIGHT_GREEN);   LCD_FillRect(110,179,210,16);  LCD_DisplayString(0,179,"LIGHT_GREEN");	// 矩形填充测试
	LCD_SetColor(LIGHT_RED);     LCD_FillRect(110,201,210,16);  LCD_DisplayString(0,201,"LIGHT_RED");	   // 矩形填充测试
	LCD_SetColor(LIGHT_CYAN);    LCD_FillRect(110,223,210,16);  LCD_DisplayString(0,223,"LIGHT_CYAN");	   // 矩形填充测试                                         
	Delay_ms(2000);	
	
	LCD_SetBackColor(LCD_BLACK); //设置背景色
	LCD_Clear(); //清屏，刷背景色		
	LCD_SetColor(LCD_RED);    LCD_FillCircle( 80,120,80);		//填充圆形
	LCD_SetColor(LCD_GREEN);  LCD_FillCircle(160,120,80); 	//填充圆形
	LCD_SetColor(LCD_BLUE);   LCD_FillCircle(240,120,80);  	//填充圆形
	Delay_ms(2000);		
}
