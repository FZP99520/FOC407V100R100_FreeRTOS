#include "key.h"
#include "log.h"

static ST_KeyStatus_t _stKeyStatus;

//������ʼ������
void KEY_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    DEBUG_TRACE("IN\n");
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5; //KEY ��Ӧ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�Ѿ������������
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE

    _stKeyStatus.bInitDone = TRUE;

    DEBUG_TRACE("OK\n");
} 

void KEY_StatusClearBit(ST_KeyStatus_t * pstKeyStatus, EN_KEY_FLAG eKeyFlag)
{
    pstKeyStatus->u32KeyFlag &= ~eKeyFlag;
}
ST_KeyStatus_t* KEY_GetInfo(void)
{
    return &_stKeyStatus;
}

void KEY_Scan(ST_KeyStatus_t* pstKeyStatus)
{
    if(KEY_READ_LEFT_LEFT)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_LEFT_LEFT;
    }
    if(KEY_READ_LEFT_DOWN)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_LEFT_DOWN;
    }
    if(KEY_READ_LEFT_RIGHT)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_LEFT_RIGHT;
    }
    if(KEY_READ_LEFT_UP)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_LEFT_UP;
    }
    if(KEY_READ_RIGHT_LEFT)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_RIGHT_LEFT;
    }
    if(KEY_READ_RIGHT_DOWN)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_RIGHT_DOWN;
    }
    if(KEY_READ_RIGHT_RIGHT)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_RIGHT_RIGHT;
    }
    if(KEY_READ_RIGHT_UP)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_RIGHT_UP;
    }
    if(KEY_READ_LEFT_UPPER)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_LEFT_UPPER;
    }
    if(KEY_READ_RIGHT_UPPER)
    {
        pstKeyStatus->u32KeyFlag |= E_KEY_RIGHT_UPPER;
    }
    
}

