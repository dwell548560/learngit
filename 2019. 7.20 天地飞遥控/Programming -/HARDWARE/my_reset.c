#include "my_reset.h"

void mcu_reset(void)
{
  __set_FAULTMASK(1); //�ر������ж�
  NVIC_SystemReset(); //��λ

}


//������յ�can�·�������Ϊָ��ֵʱ�ͽ��и�λ��ps3�·�ָ��
void reset_handle(void)
{
   if(rx_can2_id == 0x588)//���ػ�����ָ��
	 {
	    if(( can2_rx_data[0] == 0x77)&&( can2_rx_data[1] == 0x99))
			{
				 rx_can2_id = 0;
				 can2_rx_data[0] = 0;
				 can2_rx_data[1] = 0;
			   mcu_reset();//��λ			   
			}
	 }
//	 if(my_wfly.ch_data_buf[4] == 341)//��ط�ң������ָ�� ����E���ϵ��£��µ�ֵΪ341
//	 {
//			
//			if(my_wfly.open_wfly_reset_flag == OPEN_RESET_WBUS)mcu_reset();//��λ		ң�ؿ��Ÿ�λ����ʱ�ĸ�λ
//				
//	 }

}

