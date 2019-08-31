#include "common_exit.h"

CRASH_STU crash_sta_type;


void EXTI15_10_IRQHandler(void)
{
	////�˹�/�Զ��л�
	if(EXTI_GetITStatus( EXTI_Line15 ) != RESET) //PF15 �˹�/�����л�
	{
		if(SWITCH_GPIO == 0)
			{
				switch_button_flag = 1;//��1�������˹�����
			
			}
			else if(SWITCH_GPIO == 1)
			{
				switch_button_flag = 0;//��־��0���Զ�ģʽ

			}
		EXTI_ClearITPendingBit( EXTI_Line15 );
	}
	
	
	//��ת��λ ��
	if(EXTI_GetITStatus( EXTI_Line12 ) != RESET)
	{
		if(PA12 == 0)//��������� ��ת���޴���
		{ 		
			if(trigger == STOP_TRIGGER_FLAG)//�Ѿ���λ����
			{							
				MOTOR_PARA .Limit_switch_reverse_flag =1;//��ת��λ������־
				MOTOR_PARA.ori_flag=2;//��ת��־					
			}		
			else if(trigger == FIR_TRIGGER_FLAG)
			{
				vel_mode_run(RIGHT_TURN);
				MOTOR_PARA.ori_flag=2;//��ת��־	
			}
			MOTOR_PARA.ori_flag=2;//��ת��־
			
		}		
	EXTI_ClearITPendingBit( EXTI_Line12 );
	}
	
	
	if(EXTI_GetITStatus( EXTI_Line13 ) != RESET) //PF15 �˹�/�����л�
	{
		  if(MAN_DIR_GPIO == 0)
			{
				manual_motor_dir_flag = 1;//��1�������˹�����		
			}
			else if(MAN_DIR_GPIO == 1)
			{
				manual_motor_dir_flag = 0;//��־��0���Զ�ģʽ
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
		else if(GPIO_HARD_STOP == HARD_STOP_UNTRI)//����ʱ��λǰ��
		{
				mcu_reset();//��λ����
		}
		EXTI_ClearITPendingBit( EXTI_Line11 );
		
	}
	
}


