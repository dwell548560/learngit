#include "common_exit.h"

CRASH_STU crash_sta_type;


void EXTI15_10_IRQHandler(void)
{
	////人工/自动切换
	if(EXTI_GetITStatus( EXTI_Line15 ) != RESET) //PF15 人工/自主切换
	{
		if(SWITCH_GPIO == 0)
			{
				switch_button_flag = 1;//置1，进行人工控制
			
			}
			else if(SWITCH_GPIO == 1)
			{
				switch_button_flag = 0;//标志置0，自动模式

			}
		EXTI_ClearITPendingBit( EXTI_Line15 );
	}
	
	
	//反转复位 右
	if(EXTI_GetITStatus( EXTI_Line12 ) != RESET)
	{
		if(PA12 == 0)//如果触发了 反转极限触发
		{ 		
			if(trigger == STOP_TRIGGER_FLAG)//已经复位过了
			{							
				MOTOR_PARA .Limit_switch_reverse_flag =1;//反转限位触发标志
				MOTOR_PARA.ori_flag=2;//正转标志					
			}		
			else if(trigger == FIR_TRIGGER_FLAG)
			{
				vel_mode_run(RIGHT_TURN);
				MOTOR_PARA.ori_flag=2;//正转标志	
			}
			MOTOR_PARA.ori_flag=2;//正转标志
			
		}		
	EXTI_ClearITPendingBit( EXTI_Line12 );
	}
	
	
	if(EXTI_GetITStatus( EXTI_Line13 ) != RESET) //PF15 人工/自主切换
	{
		  if(MAN_DIR_GPIO == 0)
			{
				manual_motor_dir_flag = 1;//置1，进行人工控制		
			}
			else if(MAN_DIR_GPIO == 1)
			{
				manual_motor_dir_flag = 0;//标志置0，自动模式
			}
		EXTI_ClearITPendingBit( EXTI_Line13 );
	}
	
	
	if(EXTI_GetITStatus( EXTI_Line11 ) != RESET) 
	{
		delay_ms(10);
		if(GPIO_HARD_STOP == HARD_STOP_TRI)
		{
			hard_stop_state = HARD_STOP_PUSH;
		}
		else if(GPIO_HARD_STOP == HARD_STOP_UNTRI)//弹起时复位前轮
		{
				mcu_reset();//复位主板
		}
		EXTI_ClearITPendingBit( EXTI_Line11 );
		
	}
	
}


