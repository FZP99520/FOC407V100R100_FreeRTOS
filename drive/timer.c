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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM2时钟
  TIM_TimeBaseInitStructure.TIM_Period = 20000-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器2更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器2
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM3_Int_Init(void) //5ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 5000-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_Int_Init(void) //10ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM2时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 10000-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//定时器2中断服务程序
//void TIM2_IRQHandler(void)   //TIM2中断 T=20ms 1 2
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
//	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //清除TIM2更新中断标志 
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
//void TIM4_IRQHandler(void)//T=10ms 刷新显示数据  1 3
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
	/******************统计信号质量***************/
	 if(temp == 50)
		{
			temp = 0;
			NRF.Signal = (float)tx_cnt/(tx_cnt + tx_max);
			tx_cnt=0;
			tx_max=0;
		}
		/****************检查收发情况*****************/
		if(NRF.rx_OK==1)
		{
			NRF.rx_OK=0;//清楚接收完成标志位
			RC_Anl_BUFF(NRF.RX_BUFF,NRF.RX_Len);//分析接收到的数据
		}
		if(NRF.tx_OK)
		{
			tx_cnt++;//记录1s内数据发送完成次数
			NRF.tx_OK=0;
		}
		if(NRF.TX_Max)
		{
			tx_max++; //记录1s内数据发送失败次数
			NRF.TX_Max=0;
			NRF_Flush_TX_FIFO();
			if(NRF.MAX_cnt>100)
			{
				NRF.MAX_cnt=0;
				Show_Page1_Log("NRF connection is lose! Trying to reconnect.");
			}
		}
	  RC_Send_Handle();//处理要发送的数据
		if(Page_Now==1)Show_NRFData_Update(NRF);         //更新显示射频通信数据
}
