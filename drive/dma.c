#include "dma.h"
#include "stm32f4xx_dma.h"
#include "foc.h"
#include "adc.h"


void DMA_USART1_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr)
{
    //DMA2 Stream7 Channel4 USART1_TX
    //DMA2 Stream5 channel4 USART1_RX
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_InitStructure.DMA_BufferSize =(u32)USART1_BUFF_SIZE;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)(&(USART1->DR)) ;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* Configure TX DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)u32TxBuffAddr ;
    DMA_Init(DMA2_Stream7,&DMA_InitStructure);
    DMA_Cmd(DMA2_Stream7, DISABLE);
    /* Configure RX DMA */
    DMA_DeInit(DMA2_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)u32RxBuffAddr; 
    DMA_Init(DMA2_Stream5,&DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5,ENABLE);
    /*  interrupt  */
    DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);//传输完成中断  TX

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Enable USART to DMA Request */
    USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);   
    USART_Cmd(USART1,ENABLE);   
}

void DMA2_Stream7_IRQHandler(void)//USART1_DMA IRQ_Handler
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET) //USART1_TX
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
        USART_ITConfig(USART1,USART_IT_TC, ENABLE);
        DMA_Cmd(DMA2_Stream7, DISABLE);
    }
}

void DMA_ADC1_Init(u16 *pu16AdcConvBuff)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_InitStructure.DMA_BufferSize =(u32)ADC_CHANNEL_INUSE_NUM;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)(&(ADC1->DR)) ;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* Configure ADC1 DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_0 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)pu16AdcConvBuff;
    DMA_Init(DMA2_Stream0,&DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);
    /*  interrupt  */
    DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);//传输完成中断  TX

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Enable ADC1 to DMA Request */
    ADC_DMACmd(ADC1, ENABLE);   

}

void DMA2_Stream0_IRQHandler() //ADC1 DMA IRQ_Handler
{
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET) //ADC1 DMA 
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        //ADC_Data_Handle();
        #if 0
        if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            if(hAdcConv_Queue!= NULL)
            {
                FR_OS_QueueSendFromISR(hAdcConv_Queue, &stAdcConv);
            }
        }
        #endif
    }
}

/*************************************************************************/
void DMA_USART2_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr)
{
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream6);//TX
    DMA_InitStructure.DMA_Memory0BaseAddr=(u32)u32TxBuffAddr;//发送地址
    DMA_InitStructure.DMA_BufferSize = USART2_BUFF_SIZE;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(USART2->DR)) ;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* Configure TX DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)u32TxBuffAddr ;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
   /* Configure RX DMA */
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)u32RxBuffAddr; 
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);//开启中断
    /*Enable USART to DMA Request */
    USART_DMACmd(USART2, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);      
    /* Enable USART */
    USART_Cmd(USART2, ENABLE);
}
/************************/
void DMA1_Stream6_IRQHandler(void)//USART2_TX DMA
{
    if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET) 
    {
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA1_Stream6, DISABLE);
        USART_ITConfig(USART2, USART_IT_TC, ENABLE);
    }
}

/******************USART3 DMA**************************/
void DMA_USART3_Init(u8 *u32TxBuffAddr, u8 *u32RxBuffAddr)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_InitStructure.DMA_BufferSize = USART3_BUFF_SIZE;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(USART3->DR)) ;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* Configure TX DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)u32TxBuffAddr;
    DMA_Init(DMA1_Stream3,&DMA_InitStructure);
    DMA_Cmd(DMA1_Stream3, DISABLE);
    /* Configure RX DMA */
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(u32)u32RxBuffAddr; 
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1,ENABLE);
    /*  interrupt  */
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);//传输完成中断  TX

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /*Enable USART to DMA Request */
    USART_DMACmd(USART3, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);   
    USART_Cmd(USART3, ENABLE);   
}

void DMA1_Stream3_IRQHandler(void)//USART3_TX DMA
{
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET) //USART3_TX
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
        USART_ITConfig(USART3, USART_IT_TC,ENABLE);  //发送完成中断
        DMA_Cmd(DMA1_Stream3, DISABLE); //关闭传输
    }
}
/**************************/

#if 0
void DMA_USART6_Init(void)
{
	 	NVIC_InitTypeDef NVIC_InitStructure;
	 DMA_InitTypeDef  DMA_InitStructure;
	   /* Enable the DMA clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	 DMA_InitStructure.DMA_BufferSize =USART6_BUFF_SIZE;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(USART6->DR)) ;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   /* Configure TX DMA */
   DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
   DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)USART6_TX_Buff;
   DMA_Init(DMA2_Stream6,&DMA_InitStructure);
	 DMA_Cmd(DMA2_Stream6, DISABLE);
 /* Configure RX DMA */
	 DMA_DeInit(DMA2_Stream1);
   DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
   DMA_InitStructure.DMA_Memory0BaseAddr =(u32)USART6_RX_Buff; 
   DMA_Init(DMA2_Stream1,&DMA_InitStructure);
	 DMA_Cmd(DMA2_Stream1,ENABLE);
	 /*  interrupt  */
	  DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);//传输完成中断  TX
		
	 NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 
	 /*Enable USART to DMA Request */
   USART_DMACmd(USART6, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);   
   USART_Cmd(USART6,ENABLE);   
}
void DMA2_Stream6_IRQHandler(void)//USART6_TX DMA
{
	if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET) //USART3_TX
	{
		DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);
		USART_ITConfig(USART6,USART_IT_TC,ENABLE);  //发送完成中断
		DMA_Cmd(DMA2_Stream6,DISABLE); //关闭传输
	}
}
#endif

