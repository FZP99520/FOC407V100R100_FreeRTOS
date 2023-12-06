#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f4xx.h"

#define  KEY_MAX_CNT 100 //按下确认时长

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef BIT
#define BIT(x)  (1<<x)
#endif

typedef enum
{
    E_KEY_LEFT_LEFT   = BIT(0),
    E_KEY_LEFT_DOWN   = BIT(1),
    E_KEY_LEFT_RIGHT  = BIT(2),
    E_KEY_LEFT_UP     = BIT(3),
    E_KEY_RIGHT_LEFT  = BIT(4),
    E_KEY_RIGHT_DOWN  = BIT(5),
    E_KEY_RIGHT_RIGHT = BIT(6),
    E_KEY_RIGHT_UP    = BIT(7),
    E_KEY_LEFT_UPPER  = BIT(8),
    E_KEY_RIGHT_UPPER = BIT(9)
}EN_KEY_FLAG;

typedef enum
{
    E_KEY_EVENT_LEFT_LEFT   = BIT(0),
    E_KEY_EVENT_LEFT_DOWN   = BIT(1),
    E_KEY_EVENT_LEFT_RIGHT  = BIT(2),
    E_KEY_EVENT_LEFT_UP     = BIT(3),
    E_KEY_EVENT_RIGHT_LEFT  = BIT(4),
    E_KEY_EVENT_RIGHT_DOWN  = BIT(5),
    E_KEY_EVENT_RIGHT_RIGHT = BIT(6),
    E_KEY_EVENT_RIGHT_UP    = BIT(7),
    E_KEY_EVENT_LEFT_UPPER  = BIT(8),
    E_KEY_EVENT_RIGHT_UPPER = BIT(9)
}EN_KeyEvent;


typedef struct
{
    u8 bInitDone;
    u32 u32KeyFlag;    
}ST_KeyStatus_t;

//extern ST_KeyStatus_t stKeyStatus;

#define KEY_READ_LEFT_LEFT      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)?FALSE:TRUE
#define KEY_READ_LEFT_DOWN      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)?FALSE:TRUE
#define KEY_READ_LEFT_RIGHT     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)?FALSE:TRUE
#define KEY_READ_LEFT_UP        GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)?FALSE:TRUE

#define KEY_READ_RIGHT_LEFT     GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)?FALSE:TRUE
#define KEY_READ_RIGHT_DOWN     GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)?FALSE:TRUE
#define KEY_READ_RIGHT_RIGHT    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)?FALSE:TRUE
#define KEY_READ_RIGHT_UP       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)?FALSE:TRUE

#define KEY_READ_LEFT_UPPER     GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)?FALSE:TRUE
#define KEY_READ_RIGHT_UPPER    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)?FALSE:TRUE

void KEY_Init(void);//IO初始化
void KEY_Scan(ST_KeyStatus_t* pstKeyStatus);
void KEY_StatusClearBit(ST_KeyStatus_t* pstKeyStatus, EN_KEY_FLAG eKeyFlag);
ST_KeyStatus_t* KEY_GetInfo(void);



#endif

