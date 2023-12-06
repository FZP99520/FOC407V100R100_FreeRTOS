#include "input_ctrl.h"
#include "key.h"
#include "adc.h"
#include "rc.h"
#include "display.h"
#include "ili9488.h"
#include "stdio.h"
static void Cmd_Handle(void);
static void PID_Data_Handle(void);
Input_TypeDef INPUT;

u8 PID_R_Index=1; //指向的PID数据位置
u8 PID_C_Index=1;
float* PID_Point;//指针指向被修改的某个PID参数

//扫描输入数据处理
void ScanInput_Handle(void) //T=20ms
{
	//KEY_Scan(); //扫描按键，结果存放在结构体中
		//根据按键扫描结果，完成相关操作
	//换页操作
	if(INPUT.LEFT_Upper_cnt>=KEY_MAX_CNT) 	
	{
		INPUT.LEFT_Upper_cnt=0;
		if(Page_Now ==1)
			Page_Next = 2;
		else if(Page_Now ==2)
			Page_Next = 1;
		else;
	}
  if(Page_Now==1) Cmd_Handle();
	if(Page_Now==2) PID_Data_Handle();
	
	GetThro_Data();
}
void Cmd_Handle(void)//T=20ms
{
	static u16 cnt_=0;
	if(INPUT.Thro_High<1100u && INPUT.Thro_Yaw<1100u)
	{
		cnt_++;
		if(cnt_>150 && RC_status.flysta!=FlyStaFlying) 
		{
				RC_CmdReqFlag.Unlock=1;//非飞行状态才允许解锁
				Show_Page1_Log("Request to unlock the drone!!!");
			  cnt_=0;
		}
	}
	else if(INPUT.Thro_High<1100u && INPUT.Thro_Yaw>1800u)
	{
		cnt_++;
		if(cnt_>50) 
		{
			RC_CmdReqFlag.Lock=1;
			cnt_=0;
			Show_Page1_Log("Request to lock the drone.");
		}
	}
	else
		cnt_=0;
//	if( INPUT.RIGHT_Upper_cnt>=KEY_MAX_CNT)//解锁
//	{
//		INPUT.RIGHT_Upper_cnt=0;
//		RC_CmdReqFlag.Unlock=1;
//		Show_Page1_Log("Request to unlock the drone!!!");
//	}
//	
//	if( INPUT.LEFT_Right_cnt>=KEY_MAX_CNT)//解锁
//	{
//		INPUT.LEFT_Right_cnt=0;
//		RC_CmdReqFlag.Lock=1;
//		Show_Page1_Log("Request to lock the drone.");
//	}
	/*****************************/
	if(INPUT.RIGHT_Down_cnt>=KEY_MAX_CNT) 	//右下按键按下 气压计校准	
	{
		  INPUT.RIGHT_Down_cnt=0;
		  RC_CmdReqFlag.BAR = 1 ;
		  RC_CmdReqFlag.cmd_sta =1;
		  LCD_SetColor(LCD_BLUE);
		  LCD_FillCircle(440,5*20+10,7);
		  Show_Page1_Log("Request to calibtate ms5611.");
	}
	if(INPUT.RIGHT_Up_cnt>=KEY_MAX_CNT) 	//右下按键按下 气压计校准	
	{
		  INPUT.RIGHT_Up_cnt=0;
		  RC_CmdReqFlag.ACCEL = 1 ;
		  RC_CmdReqFlag.cmd_sta =1;
		  LCD_SetColor(LCD_BLUE);
		  LCD_FillCircle(440,2*20+10,7);
		  Show_Page1_Log("Request to calibtate MPU6050 acceleration.");
	}
	if(INPUT.RIGHT_Left_cnt>=KEY_MAX_CNT) 	//右下按键按下 气压计校准	
	{
	    INPUT.RIGHT_Left_cnt=0;
		  RC_CmdReqFlag.GYRO = 1 ;
		  RC_CmdReqFlag.cmd_sta =1;
		  LCD_SetColor(LCD_BLUE);
		  LCD_FillCircle(440,3*20+10,7);
		  Show_Page1_Log("Request to calibtate MPU6050 gyroscope.");
	}
	if(INPUT.RIGHT_Right_cnt>=KEY_MAX_CNT) 	//右下按键按下 校准	
	{
		INPUT.RIGHT_Right_cnt=0;
		RC_CmdReqFlag.MAG = 1 ;
		RC_CmdReqFlag.cmd_sta =1;
		LCD_SetColor(LCD_BLUE);
		LCD_FillCircle(440,4*20+10,7);
		Show_Page1_Log("Request to calibtate QMC5883.");
	}	 
}
/****************/
static float* Index_To_DataAddr(u8 R,u8 C)
{
	float* p;
	if(C==1)
	{
		if(R==1)      p=&RC_Roll_angle_PID.P;
		else if(R==2) p=&RC_Pitch_angle_PID.P;
		else if(R==3) p=&RC_Yaw_angle_PID.P;
		else if(R==4) p=&RC_High_dis_PID.P;
		else if(R==5) p=&RC_Roll_rate_PID.P;
		else if(R==6) p=&RC_Pitch_rate_PID.P;
		else if(R==7) p=&RC_Yaw_rate_PID.P;
		else if(R==8) p=&RC_High_v_PID.P;
		else;
	}
	else if(C==2)
	{
		if(R==1)      p=&RC_Roll_angle_PID.I;
		else if(R==2) p=&RC_Pitch_angle_PID.I;
		else if(R==3) p=&RC_Yaw_angle_PID.I;
		else if(R==4) p=&RC_High_dis_PID.I;
		else if(R==5) p=&RC_Roll_rate_PID.I;
		else if(R==6) p=&RC_Pitch_rate_PID.I;
		else if(R==7) p=&RC_Yaw_rate_PID.I;
		else if(R==8) p=&RC_High_v_PID.I;
		else;
	}
	else if(C==3)
	{
		if(R==1)      p=&RC_Roll_angle_PID.D;
		else if(R==2) p=&RC_Pitch_angle_PID.D;
		else if(R==3) p=&RC_Yaw_angle_PID.D;
		else if(R==4) p=&RC_High_dis_PID.D;
		else if(R==5) p=&RC_Roll_rate_PID.D;
		else if(R==6) p=&RC_Pitch_rate_PID.D;
		else if(R==7) p=&RC_Yaw_rate_PID.D;
		else if(R==8) p=&RC_High_v_PID.D;
		else;
	}
	return p;
}
void PID_Data_Handle(void)
{
	char temp[10];
	static u8 flag_new=0;
	static u8 flag_pos_new=1;
	static u8 flag1,flag2;
	static u8 f1,f2,f3,f4;
	static u16 flag_send_cnt=0;
	static u8 flag_send;
	if( INPUT.RIGHT_Upper_cnt>=KEY_MAX_CNT)//读取PID数据
	{
		INPUT.RIGHT_Upper_cnt=0;
		RC_CmdReqFlag.Read_pid=1;
		Show_PID_Log("Request To Read Drone PID Data.");
	}
	/*********************/
	if(INPUT.Thro_Pitch<1100u)
	{
		flag_send_cnt++;
		if(flag_send_cnt>50) flag_send=1;
	}
	else 
		flag_send_cnt=0;
	if(flag_send)
	{
		if(INPUT.Thro_Pitch>1400u)
		{
			flag_send=0;
			flag_send_cnt=0;
			RC_CmdReqFlag.Send_pid1=1;
			RC_CmdReqFlag.Send_pid2=1;
			RC_CmdReqFlag.Send_pid3=1;
			Show_PID_Log("Ready To Send PID Data!");
		}
	}
	/**********************************/
    /*if(!LEFT_Left_STA) flag1=1;//按下松开
    if(flag1){if(LEFT_Left_STA) {*PID_Point+=0.1f;flag1=0;flag_new=1;}}
    if(!LEFT_Right_STA) flag2=1;//按下松开
    if(flag2){if(LEFT_Right_STA) {*PID_Point-=0.1f;flag2=0;flag_new=1;}}

    if(!LEFT_Left_STA) {*PID_Point+=0.1f;flag_new=1;}
    if(!LEFT_Right_STA){*PID_Point-=0.1f;flag_new=1;}
    if(!LEFT_Up_STA)   {*PID_Point+=0.001f;flag_new=1;}//按下
    if(!LEFT_Down_STA) {*PID_Point-=0.001f;flag_new=1;}//按下*/
    /************指向需要修改的数据**************/
    /*if(!RIGHT_Left_STA) f1=1; //1
    if(f1)
    {
    if(RIGHT_Left_STA)
     {
			 flag_pos_new=1;
			 f1=0;
			 PID_C_Index--;
			 if(PID_C_Index==0) PID_C_Index=3;
		 }
	}
	if(!RIGHT_Right_STA) f2=1; //2
	if(f2)
  {
	   if(RIGHT_Right_STA)
     {
			 flag_pos_new=1;
			 f2=0;
			 PID_C_Index++;
			 if(PID_C_Index==4) PID_C_Index=1;
		 }
	}
	if(!RIGHT_Up_STA) f3=1; //3
	if(f3)
  {
	   if(RIGHT_Up_STA)
     {
			 flag_pos_new=1;
			 f3=0;
			 PID_R_Index--;
			 if(PID_R_Index==0) PID_R_Index=8;
		 }
	}
	//if(!RIGHT_Down_STA) f4=1;//4
	if(f4)
  {
	   if(RIGHT_Down_STA)
     {
			 flag_pos_new=1;
			 f4=0;
			 PID_R_Index++;
			 if(PID_R_Index==9) PID_R_Index=1;
		 }
	}
	PID_Point = Index_To_DataAddr(PID_R_Index,PID_C_Index);
	/*************************/
	/*if(flag_pos_new)
	{
		flag_pos_new=0;
		Show_RC_PID_Update();
		LCD_SetColor(LCD_YELLOW);
		sprintf(temp,"%7.3f",*PID_Point);
	  LCD_DisplayString(150+(PID_C_Index-1)*90,(4+PID_R_Index-1)*20,temp);
		Show_PID_Log("PID Data Pointer is Change!");
	}
	if(flag_new) 
  {
	  flag_new=0;
	  Show_RC_PID_Update();
		LCD_SetColor(LCD_YELLOW);
		sprintf(temp,"%7.3f",*PID_Point);
	  LCD_DisplayString(150+(PID_C_Index-1)*90,(4+PID_R_Index-1)*20,temp);
		Show_PID_Log("Data is NEW! Send it?");
	}*/
}
