#include "my_reset.h"

void mcu_reset(void)
{
  __set_FAULTMASK(1); //关闭所有中断
  NVIC_SystemReset(); //复位

}


//如果接收到can下发的数据为指定值时就进行复位，ps3下发指令
void reset_handle(void)
{
   if(rx_can2_id == 0x588)//工控机发送指令
	 {
	    if(( can2_rx_data[0] == 0x77)&&( can2_rx_data[1] == 0x99))
			{
				 rx_can2_id = 0;
				 can2_rx_data[0] = 0;
				 can2_rx_data[1] = 0;
			   mcu_reset();//复位			   
			}
	 }
//	 if(my_wfly.ch_data_buf[4] == 341)//天地飞遥控器发指令 拨杆E由上到下，下的值为341
//	 {
//			
//			if(my_wfly.open_wfly_reset_flag == OPEN_RESET_WBUS)mcu_reset();//复位		遥控开放复位功能时的复位
//				
//	 }

}

