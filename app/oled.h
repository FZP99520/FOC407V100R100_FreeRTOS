#ifndef _LQOLED_H
#define _LQOLED_H
#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"
//#include "userinc/gpio.h"

#include "ina226.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"

typedef struct
{
    u8 u8x;
    u8 u8y;
    float f32data;
}ST_OLED_Disp_t;

extern TaskHandle_t  hOledDisp_Task;
extern QueueHandle_t hOledDisp_Queue;


//汉字大小，英文数字大小
#define 	TYPE8X16		1
#define 	TYPE16X16		2
#define 	TYPE6X8			3

//-----------------OLED端口定义----------------


#define OLED_SCL_CLR()      GPIO_ResetBits(GPIOD,GPIO_Pin_0)
#define OLED_SCL_SET()      GPIO_SetBits(GPIOD,GPIO_Pin_0)

#define OLED_SDA_CLR()      GPIO_ResetBits(GPIOD,GPIO_Pin_1)
#define OLED_SDA_SET()      GPIO_SetBits(GPIOD,GPIO_Pin_1)

#define OLED_RST_CLR()      GPIO_ResetBits(GPIOD,GPIO_Pin_2)
#define OLED_RST_SET()      GPIO_SetBits(GPIOD,GPIO_Pin_2)

#define OLED_DC_CLR()       GPIO_ResetBits(GPIOD,GPIO_Pin_3)
#define OLED_DC_SET()       GPIO_SetBits(GPIOD,GPIO_Pin_3)

#define OLED_CS_CLR()       GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define OLED_CS_SET()       GPIO_SetBits(GPIOD,GPIO_Pin_4)

//CS管脚请接地

extern void OLED_Init(void);
extern void OLED_CLS(void);
extern void OLED_CLS_y(char y);
extern void OLED_CLS_line_area(u8 start_x,u8 start_y,u8 width);
extern void OLED_P6x8Str(u8 x,u8 y,u8 *ch,const u8 *F6x8);
extern void OLED_P8x16Str(u8 x,u8 y,u8 *ch,const u8 *F8x16);
extern void OLED_P14x16Str(u8 x,u8 y,u8 ch[],const u8 *F14x16_Idx,const u8 *F14x16);
extern void OLED_P16x16Str(u8 x,u8 y,u8 *ch,const u8 *F16x16_Idx,const u8 *F16x16);
//extern void OLED_Print(u8 x, u8 y, u8 *ch);
extern void OLED_PutPixel(u8 x,u8 y);
extern void OLED_Print(u8 x, u8 y, u8 *ch,u8 char_size, u8 ascii_size);
extern void OLED_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);
extern void Draw_BMP(u8 x,u8 y,const u8 *bmp); 
extern void OLED_Fill(u8 dat);

void OLED_Display_Task(void *pvParameters);


#endif

