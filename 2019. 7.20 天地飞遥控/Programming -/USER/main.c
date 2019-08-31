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
	CAN2_RPDO_Speed(0, 0x00, 6,1);//默认自动模式发送给上位机
	delay_ms(1);
	  
	 //后轮停止
	 CAN2_RPDO_Speed(0, 0x01, 1,4);
	delay_ms(2);
	 
	 //前轮回零
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
   
	//运行灯初始化
	run_led_gpio_init();
	//run_led_tim11_init();//1s中断
	
	//硬急停
	hard_stop_init();
	
	//油门   人工操作时用
	Accelerate_ADC_Init();
	Accelerate_DMA();

	//人工/自动切换按钮初始化
  //switch_gpio_init();

	//CANOPEN初始化 RobotQ驱动器   驱动器脚本中设置为1s保护，如果驱动器1s内接收不到新的速度就给电机速度设0
	CAN2__ROBOTQ_Configuration();
	
	Wfly_Usart_Init();
	Wfly_DMA_Init();


	//在底层程序中将电机驱动器相应参数初始化固定
  delay_ms(2000);//上电时等待2s再进行前轮复位不可少
  Swtich_Init();//上电复位初始化，光电初始化
  delay_ms(1000);
 
#if 1
 //控制部分
 
	Check_Switch_hanle();//上电先检测三个光电是不是刚好某 一个处于触发状态
	if(trigger == UN_TRIGGER_FLAG)//如果不是上面的情况
	{
		Reset_hal();//开机复位
		
	}
    while(trigger != STOP_TRIGGER_FLAG)// 需要
   {		 
				
   }//等到前轮复位完成，否则不进行工作
	 
	point_reset_hal();//下面四条SDO是使得中间光电的位置与上电时的原点重合.以中间的光电为实际运动的零点，上电时的两点可能与其不重合
	driver_init_hal();//驱动器控制字 工作模式初始化
	start_pdo(0x000, 0x0001, 2);//启动所有节点进行pdo通信
	
	Motor_Start_Init();//开机复位，后轮停止，前轮归零
	 
#endif

	while(1)
	{	
		manual_hal();
		
		Get_Ch_Hal();//获得遥控数据

		reset_handle();//遥控复位操作，上位机发送控制指令复位
		
	}
		

}


