#include "fsmc.h"

void FSMC_Init(void)
{
	FSMC_NORSRAMTimingInitTypeDef  SET;	
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;	
	
   RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);	//使能FSMC时钟
	
	SET.FSMC_AddressSetupTime = 0x00;	 		//地址建立时间
	SET.FSMC_AddressHoldTime = 0x00;		 		//地址保持时间
	SET.FSMC_DataSetupTime = 0x03;		 		//数据保存时间
	SET.FSMC_BusTurnAroundDuration = 0x00;
	SET.FSMC_CLKDivision = 0x00;
	SET.FSMC_DataLatency = 0x00;
	SET.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 

	FSMC_NORSRAMInitStructure.FSMC_Bank 				  = FSMC_Bank1_NORSRAM1;			
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux 	  = FSMC_DataAddressMux_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_MemoryType 		  = FSMC_MemoryType_SRAM;			
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth 	  = FSMC_MemoryDataWidth_16b;		
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode 	  = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait	  = FSMC_AsynchronousWait_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WrapMode 			  = FSMC_WrapMode_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive   = FSMC_WaitSignalActive_BeforeWaitState;  
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation 	  = FSMC_WriteOperation_Enable;					
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal 		  = FSMC_WaitSignal_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode		  = FSMC_ExtendedMode_Enable; 	
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst 		  = FSMC_WriteBurst_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct  = &SET;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  // 使能BANK1  
}
