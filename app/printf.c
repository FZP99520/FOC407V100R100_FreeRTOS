#include "printf.h"
/**STM32F407VGT6*/
/*No mircolib */
int fputc(int ch, FILE* f)
{
    USART_SendData(USART1, (u8)ch);
	  while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET)
		{
		}			
    return ch;
}
int fgetc(FILE *f)
{
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return (int)USART_ReceiveData(USART1);
}
void PRINTF_Init(u32 baudrate)
{
   USART_InitTypeDef USART_InitStructure;
   GPIO_InitTypeDef GPIO_InitStructure;
   /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);   
   /* Enable USART clock */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
   /* Configure USART Tx and Rx as alternate function push-pull */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* USARTx configuration ----------------------------------------------------*/
   /* Enable the USART OverSampling by 8 */
   //USART_OverSampling8Cmd(USART1, ENABLE); 
   USART_InitStructure.USART_BaudRate = baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   /* When using Parity the word length must be configured to 9 bits */
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART1, &USART_InitStructure);
	 USART_ClearFlag(USART1,USART_FLAG_TC);
   USART_Cmd(USART1,ENABLE);
}
