#include "hard_stop.h"


 u8 hard_stop_state = HARD_STOP_UNPUSH;

void hard_stop_init(void)
{
		EXTI_InitTypeDef   EXTI_InitStructure;
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE);
	  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	  GPIO_Init( GPIOC , &GPIO_InitStructure);

	  SYSCFG_EXTILineConfig(  EXTI_PortSourceGPIOC, EXTI_PinSource11   );

	  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  //NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	 // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0; //等级最高
	  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	 // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  //NVIC_Init(&NVIC_InitStructure);
	  
	  EXTI_ClearITPendingBit( EXTI_Line11 ); 	

}