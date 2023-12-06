#include "timer.h"
#include "led.h"
#include "mpu6050.h"
#include "ANO_DT.h"
#include "nrf24l01.h"
#include "DataExc.h"
#include "key.h"
#include "adc.h"
#include "display.h"
#include "main.h"
#include "input_ctrl.h"
void TIM2_Int_Init(void) //20ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
  TIM_TimeBaseInitStructure.TIM_Period = 20000-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM3_Int_Init(void) //5ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 5000-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_Int_Init(void) //10ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM2ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 10000-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM4
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��4
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//��ʱ��2�жϷ������
//void TIM2_IRQHandler(void)   //TIM2�ж� T=20ms 1 2
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
//	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //���TIM2�����жϱ�־ 
//}
void NRF_Data_Handle(void);

void TIM3_IRQHandler(void)//5ms 1 3
{
	static u16 cnt=0;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if(cnt%4==1) NRF_Data_Handle();//T=4*5ms=20ms
		if(cnt%4==0) ScanInput_Handle();//T=4*5ms=20ms
		if(cnt%2==1) Show_Update();     //T=2*5ms=10ms
		cnt++;
		
	}
}
//void TIM4_IRQHandler(void)//T=10ms ˢ����ʾ����  1 3
//{
//	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
//	{
//		Show_Update();
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	}
//}
void NRF_Data_Handle(void)
{
	static u8 tx_cnt=0,tx_max=0;
	static u8 temp=0;
	temp++;
	/******************ͳ���ź�����***************/
	 if(temp == 50)
		{
			temp = 0;
			NRF.Signal = (float)tx_cnt/(tx_cnt + tx_max);
			tx_cnt=0;
			tx_max=0;
		}
		/****************����շ����*****************/
		if(NRF.rx_OK==1)
		{
			NRF.rx_OK=0;//���������ɱ�־λ
			RC_Anl_BUFF(NRF.RX_BUFF,NRF.RX_Len);//�������յ�������
		}
		if(NRF.tx_OK)
		{
			tx_cnt++;//��¼1s�����ݷ�����ɴ���
			NRF.tx_OK=0;
		}
		if(NRF.TX_Max)
		{
			tx_max++; //��¼1s�����ݷ���ʧ�ܴ���
			NRF.TX_Max=0;
			NRF_Flush_TX_FIFO();
			if(NRF.MAX_cnt>100)
			{
				NRF.MAX_cnt=0;
				Show_Page1_Log("NRF connection is lose! Trying to reconnect.");
			}
		}
	  RC_Send_Handle();//����Ҫ���͵�����
		if(Page_Now==1)Show_NRFData_Update(NRF);         //������ʾ��Ƶͨ������
}
