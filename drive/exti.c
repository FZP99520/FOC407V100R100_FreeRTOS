#include "exti.h"
#include "nrf24l01.h"
#include "pwm.h"
#include "mpu9250.h"
#include "led.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
void EXTI0_Init()//For nrf24L01
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��
		EXTI_DeInit();
		//�� SYSCFGʱ�ӣ�����Io�����жϵ�ӳ���ϵ
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
//	  GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		//PB8�ж�ģʽ ����
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource0);
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		//���ж�����
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}
void EXTI1_Init(void)//For MPU9250 Int
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��
		//EXTI_DeInit();
		//�� SYSCFGʱ�ӣ�����Io�����жϵ�ӳ���ϵ
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	  GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		//PB8�ж�ģʽ ����
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource1);
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		//���ж�����
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}
void EXTI1_IRQHandler(void) 
{
	u8 sta;
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		sta=MPU9250_Read_INT_STA();
		if(sta&RAW_DATA_RDY_INT) MPU9250_Read(&mpu9250_data);
		#if 1
		if(LED2_Status) LED2_ON;
		else LED2_OFF;
		#endif
	}
}
void EXTI0_IRQHandler(void)
{
		u8 STA; 
	if( EXTI_GetITStatus(EXTI_Line0)!=RESET) //�ж��жϷ���
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		STA = NRF_R_STATUS();//�ȶ�ȡ״̬�Ĵ���
	 if(STA&TX_OK)//�������
	 {          
     		 //PTX ģʽ�£�ʹ����ǿģʽ�����һ�����ݽ���
		 NRF_CE_LOW;
		 NRF.TX_Cnt++;
		 NRF.tx_OK=1;
		 if(STA&RX_OK)
		 {
			 NRF.RX_Cnt++;
			 NRF.rx_OK=1;
       NRF24L01_ReceiveData(&NRF);
			 NRF.MAX_cnt=0;
		 }
	 }
	 if(STA&MAX_TX)//�ﵽ����ʹ���
	 {
		 NRF_CE_LOW;
		 NRF.MAX_cnt++;
		 NRF.TX_Max=1;
	 }
	 NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,STA);//clear interrupt pending
	}
}
