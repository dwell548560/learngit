//#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "common_exit.h"
#include "run_led.h"
#include "IWDG.h"
#include "my_reset.h"
#include "reset_motor.h"
#include "wfly.h"

void Motor_Start_Init(void)
{
	CAN2_RPDO_Speed(0, 0x00, 6,1);//Ĭ���Զ�ģʽ���͸���λ��
	delay_ms(1);
	  
	 //����ֹͣ
	 CAN2_RPDO_Speed(0, 0x01, 1,4);
	delay_ms(2);
	 
	 //ǰ�ֻ���
	CAN2_RPDO_Positon( 0, 2, 1, 4 );
	delay_us(10);
	CAN2_RPDO_Positon(0x2f, 2, 2, 2 );
	delay_us(10);
	CAN2_RPDO_Positon(0x3f, 2, 2, 2 );
	delay_us(10);


}


int main(void)
{ 
	trigger = UN_TRIGGER_FLAG;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
   
	//���еƳ�ʼ��
	run_led_gpio_init();
	//run_led_tim11_init();//1s�ж�
	
	//Ӳ��ͣ
	hard_stop_init();
	
	//����   �˹�����ʱ��
	Accelerate_ADC_Init();
	Accelerate_DMA();

	//�˹�/�Զ��л���ť��ʼ��
  //switch_gpio_init();

	//CANOPEN��ʼ�� RobotQ������   �������ű�������Ϊ1s���������������1s�ڽ��ղ����µ��ٶȾ͸�����ٶ���0
	CAN2__ROBOTQ_Configuration();
	
	Wfly_Usart_Init();
	Wfly_DMA_Init();


	//�ڵײ�����н������������Ӧ������ʼ���̶�
  delay_ms(2000);//�ϵ�ʱ�ȴ�2s�ٽ���ǰ�ָ�λ������
  Swtich_Init();//�ϵ縴λ��ʼ��������ʼ��
  delay_ms(1000);
 
#if 1
 //���Ʋ���
 
	Check_Switch_hanle();//�ϵ��ȼ����������ǲ��Ǹպ�ĳ һ�����ڴ���״̬
	if(trigger == UN_TRIGGER_FLAG)//���������������
	{
		Reset_hal();//������λ
		
	}
    while(trigger != STOP_TRIGGER_FLAG)// ��Ҫ
   {		 
				
   }//�ȵ�ǰ�ָ�λ��ɣ����򲻽��й���
	 
	point_reset_hal();//��������SDO��ʹ���м����λ�����ϵ�ʱ��ԭ���غ�.���м�Ĺ��Ϊʵ���˶�����㣬�ϵ�ʱ������������䲻�غ�
	driver_init_hal();//������������ ����ģʽ��ʼ��
	start_pdo(0x000, 0x0001, 2);//�������нڵ����pdoͨ��
	
	Motor_Start_Init();//������λ������ֹͣ��ǰ�ֹ���
	 
#endif

	while(1)
	{	
		manual_hal();
		
		Get_Ch_Hal();//���ң������

		reset_handle();//ң�ظ�λ��������λ�����Ϳ���ָ�λ
		
	}
		

}


