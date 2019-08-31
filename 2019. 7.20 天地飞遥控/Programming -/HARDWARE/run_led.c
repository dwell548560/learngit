#include "run_led.h"


u8 led1_flag = 0;

u8 cnt_led = 0;
void run_led_gpio_init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;



	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //



}


 //定时1s中断一次,在定时器中断中反转LED1
void run_led_tim11_init(void)
{
	u16 arr;
	u16 psc;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	arr=10000-1;                 
	psc=16800-1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE); //??? TIM3 ??
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; //??????
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc; //?????
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //??????
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);// ??????? TIM3
	TIM_ITConfig(TIM11,TIM_IT_Update,ENABLE); //?????? 3 ????
	
	NVIC_InitStructure.NVIC_IRQChannel= TIM1_TRG_COM_TIM11_IRQn; //??? 3 ??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //????? 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //????? 3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);// ???? NVIC
	
	TIM_Cmd(TIM11,ENABLE); //?????? 3

}


void TIM1_TRG_COM_TIM11_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM11,TIM_IT_Update)==SET)
	{
		led1_flag = !led1_flag;
		if(led1_flag == 1)
		{
			LED1_ON;
		}
		else 
		LED1_OFF;
		
						
	}

	TIM_ClearITPendingBit(TIM11,TIM_IT_Update);

}




//未用到
void systick_robot_init(void)
{
	SysTick_Config(SystemCoreClock / 100);//10ms进入一次中断，这里相当于赋值ARR为168000000/100

}

