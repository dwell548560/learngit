#include "reset_motor.h"



TPYE_CANOPEN thrrot_type;

 u16 rx_can2_id = 0;

 u8 can2_rx_data[8] = {0};
 
__packed union rx_robotq_para{

u8 rx_can2[8];

s32 can2_speed;


}para_robot;

int trigger;

motor_type MOTOR_PARA;

u8 type = 0;


const unsigned int CAN_baud_table[CAN_BAUD_NUM][5] = 
{
//波特率， CAN_SJW，   CAN_BS1，    CAN_BS2，CAN_Prescaler 
	{5,   CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,450},		//未通			
	{10,  CAN_SJW_1tq,CAN_BS1_6tq,CAN_BS2_2tq, 400},		//未通			
	{15,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,150},		//15K  未通
	{20,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,200},		//20k //未通
	{25,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,112},		//25k  未通
	{40,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,100},		//40k  未通
	{50,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,56},			//50k	ok
	{62,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,36},			//62.5k
	{80,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,50},			//80k   未通
	{100, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,28},			//100K	ok
	{125, CAN_SJW_1tq,CAN_BS1_13tq, CAN_BS2_2tq,18},		//125K 未通
	{200, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,14},			//200K  ok
	{250, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,8},		    //250k  ok
	{400, CAN_SJW_1tq,CAN_BS1_15tq, CAN_BS2_5tq,5},			//400K  ok
	{500, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,4},			//500K	ok
	{666, CAN_SJW_1tq,CAN_BS1_5tq, CAN_BS2_2tq,8},			//未通
	{800, CAN_SJW_1tq,CAN_BS1_8tq, CAN_BS2_3tq,14},			//800K 未通
	{1000,CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,2},			//1000K	ok
};


void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
{
	unsigned int i = 0;
	for(i = 0;i < CAN_BAUD_NUM;i ++)
	{
		if(Baud == CAN_baud_table[i][0])
		{
			CAN_InitStructure->CAN_SJW = CAN_baud_table[i][1];
			CAN_InitStructure->CAN_BS1 = CAN_baud_table[i][2];
			CAN_InitStructure->CAN_BS2 = CAN_baud_table[i][3];
			CAN_InitStructure->CAN_Prescaler = CAN_baud_table[i][4];
			return;	
		}
	}	

}











void CAN2__ROBOTQ_Configuration(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);//CAN2是从CAN1，需要先打开CAN1的时钟
	
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 


	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */


	/* CAN register init */
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	CAN_Baud_Process(500,&CAN_InitStructure);

	CAN_Init(CAN2, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 15;	   //CAN1滤波器号从0到13   CAN2的滤波器组是14-27

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //滤波屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;	//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;		//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

//CAN2_WriteData(0x20,SET_VECLOCITY_OB,0x01,0x00000064,0x01);

//中断接收函数
void CAN2_RX0_IRQHandler(void)
{
  u8 i;
	CanRxMsg RxMessage;
	
  CAN_Receive(CAN2, 0, &RxMessage);

	//保存低4字节，为速度值，高四字节暂时没有用到，为0
	if(RxMessage.StdId == 0x588)//如果ID为0x181  TPDO1的COB_ID为0x181     Robotq使用的是标准帧ID
	{
		
		for( i =0 ; i<8 ;i++)
		{
		
			para_robot.rx_can2[i] = RxMessage.Data[i];
			
		  can2_rx_data[i] = RxMessage.Data[i];//5.9 .2019
		}
		
		thrrot_type.speed_robotq = para_robot.can2_speed;
		rx_can2_id = 0x588;//5.9 .2019
	}
		CAN_ClearITPendingBit(CAN2,CAN_IT_FF0);
}


//SDO一般只进行配置参数，实际下发速度等使用RPDO
//cmd 是设置/咨询 剩余字节数的8位数据
//index 索引，这个参数Robotq提供的手册查询
//sub_index 子索引
//data SDO 发送的数据
//ID  节点号
void CAN2_SDO_WriteData(u8 cmd,u16 index,u8 sub_index,int data,u8 id)
{
	u16 i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	SET_SDO_HAL( cmd, index, sub_index, data );

	TxMessage.StdId = 0x600+id;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	
	//Progress(buf);
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = thrrot_type.payload_data[i];//发送数据
	}
	
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
	while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i !=  0xFFFF))
	{
		i ++;
	}

}


//发送RPDO数据到驱动器   发送的数据是一个S32位数据，所以RPDO一帧数据可以发两个数据源，比如电机1和电机2的速度，只是事先要在脚本中加入RPDO设置
//RPDO_data_low:要发送数据的低4字节
//RPDO_data_high 高4字节
//id ：CAN_ID  节点1 id=1
//RPDO_num :第几个RPDO，总共有四个，RPDO1-RPDO4  COB_ID也不同
void CAN2_RPDO_WriteData(s32 RPDO_data_low, s32 RPDO_data_high, u8 id, u8 RPDO_num, u8 send_len)
{
	u16 i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
		
	switch(RPDO_num)
	{
		case  0://启动节点PDO
		{
			TxMessage.StdId = 0x000 + id;
		  break;
		}
		case  1 ://后轮速度
		{
			TxMessage.StdId = 0x200+id;//对应步科电机写 速度或位置
			break;
		}
		case  2:
		{
			TxMessage.StdId = 0x300+id;//对应步科电机写 控制字
			break;
		}

		case  3:
		{
			TxMessage.StdId = 0x400+id;
			break;
		}

		case 4 :
		{
			TxMessage.StdId = 0x500+id;
			break;
		}

		case 5://进入手动模式时，主板向PC发送的COB-ID
		{
			TxMessage.StdId = 0x666;
			break;

		}

		case 6://进入自动时，主板向PC发送的COB-ID
		{
			TxMessage.StdId = 0x667;
			break;

		}



		default:break;

	}

	//TxMessage.IDE = CAN_ID_STD;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = send_len;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	
	
	//Progress(buf);
	for(i = 0;i < 8; i ++)
	{
		if(i<4)TxMessage.Data[i] =  (u8)(RPDO_data_low>>(8*i));
		else if(i >= 4)TxMessage.Data[i] =  (u8)(RPDO_data_high>>(8*(i-4)));
	
	}
		RPDO_data_low = 0;
	  RPDO_data_high = 0;
	
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
	
	//while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i !=  0xFFFF))
//	{
	//	i ++;
		
	//}

}



//设置SDO 8字节数据
void SET_SDO_HAL(u8 cmd,u16 index,u8 sub_index,u32 data )
{
	
//	thrrot_type.TPYE_CANOPEN.cmd =cmd;
//	
//	thrrot_type.TPYE_CANOPEN.fir_index.low_index = (u8) index;
//	thrrot_type.TPYE_CANOPEN.fir_index.high_index =(u8) (index>>8);
//	thrrot_type.TPYE_CANOPEN.sub_index = sub_index;

//	thrrot_type.TPYE_CANOPEN.data[0] = (u8)data;
//	thrrot_type.TPYE_CANOPEN.data[1] = (u8)(data>>8);
//	thrrot_type.TPYE_CANOPEN.data[2] = (u8)(data>>16);
//	thrrot_type.TPYE_CANOPEN.data[3] = (u8)(data>>24);

	
	
	thrrot_type.payload_data[0] = cmd;
	thrrot_type.payload_data[1] = (u8) index;
	thrrot_type.payload_data[2] = (u8) (index>>8);
	thrrot_type.payload_data[3] = sub_index;
	
	thrrot_type.payload_data[4] = (u8)data;
	thrrot_type.payload_data[5] = (u8)(data>>8);
	thrrot_type.payload_data[6] = (u8)(data>>16);
	thrrot_type.payload_data[7] = (u8)(data>>24);


	

}

//未用
void CAN2_RPDO_SET( u16 index,u8 sub_index,u32 data)
{

	u16 index_rb;

	index_rb = index;

	switch(index_rb)
	{
		case READ_MOTOR_CURRENT_OB :  //获得电机输出电流 0x2100
		{
			CAN2_SDO_WriteData(0x48,index,sub_index,data,1);
			break;
		}

		case FEED_BACK:
		{
			CAN2_SDO_WriteData(0x48,index,sub_index,data,1);
			break;
		}

		default:break;
	}
	 
//SET_SDO_HAL( 0X04, index, sub_index, data);//接收，咨询命令


}


//下发速度 RPDO1  不用
void CAN2_RPDO1_Speed(s32 low_loop_speed, s32 high_data,int can_id, int numb, u8 send_len)
{

	CAN2_RPDO_WriteData(low_loop_speed , high_data ,can_id , numb,  send_len);//速度loop_speed，节点1 RPDO1


}





void CAN2_RPDO_Speed(s32 get_speed, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_speed , 0 ,can_id , pdo_numb,  send_len);//速度

}



void CAN2_RPDO_Positon(s32 get_positon, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_positon , 0 ,can_id , pdo_numb, send_len);//位置

}


void CAN2_RPDO_Control_Word(u16 get_word, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_word , 0 ,can_id , pdo_numb,  send_len);//位置



}

void start_pdo(int get_cob_id, int data, u8 send_len)
{

    CAN2_RPDO_WriteData(data , 0 ,get_cob_id , 0,  send_len);//



}




void vel_mode_run(u8 dir_turn)
{
  if(dir_turn == RIGHT_TURN)//正转
	{
	  CAN2_SDO_WriteData(0x2f,0x6060, 0, -3, 2);//速度模式，id是2 前轮驱动器，正向运动，-3立即速度
		delay_ms(100);
		CAN2_SDO_WriteData(0x23,0x60ff, 0, 0x00045555, 2);//设置速度值 ,正转
		delay_ms(100);					
		CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 2);//启动
	
	}
	else if(dir_turn == LEFT_TURN)//反转
	{
	  CAN2_SDO_WriteData(0x2f,0x6060, 0, -3, 2);//速度模式，id是2 前轮驱动器，正向运动，-3立即速度
		delay_ms(100);
		CAN2_SDO_WriteData(0x23,0x60ff, 0, -0x00045555, 2);//设置速度值 ,反转
		delay_ms(100);					
		CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 2);//启动
	}


}



void point_reset_hal(void)
{
	CAN2_SDO_WriteData(0x2f,0x6060, 0x00, 0x06,2);
			delay_ms(100);
  CAN2_SDO_WriteData(0x2b,0x6040, 0x00, 0x06, 2);
		delay_ms(100);
	CAN2_SDO_WriteData(0x2b,0x6040, 0x00, 0x0f, 2);
	delay_ms(100);
	CAN2_SDO_WriteData(0x2b,0x6040, 0x00, 0x1f, 2);
			delay_ms(100);

}


void driver_init_hal(void)
{
	CAN2_SDO_WriteData(0x2f, 0x6060, 0x00, -3, 1);//节点1 工作模式 立即速度模式
			
	CAN2_SDO_WriteData(0x2f, 0x6060, 0x00, 1,2);//节点2 工作模式 位置模式
			
	CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 1);//节点1紧轴
			
	CAN2_SDO_WriteData(0x23, 0x6081, 0x00, 0xc8000, 2);//梯形速度300rpm

}















void Reset_hal(void)
{

   if(PA4 == 1)//如果不处于中间光电触发的位置
	{
		if(MOTOR_PARA.reset_flag == 0)//用于第一次上电复位，还未复位过
		{
			
			
			//前轮运动
			vel_mode_run(RIGHT_TURN);
			MOTOR_PARA.ori_flag = 2;//正转标志
			MOTOR_PARA.ab_angle = 0;
		}
		else //如果复位过了不做动作
		{
			if(MOTOR_PARA.ab_angle >= 0)
			{
				MOTOR_PARA.reset_active_flag = 1;
				MOTOR_PARA.reset_active_flag = 0;
			}

			else if(MOTOR_PARA.ab_angle < 0)
			{
				MOTOR_PARA.reset_active_flag = 1;
				
				MOTOR_PARA.reset_active_flag = 0;
			}
			MOTOR_PARA.ab_angle = 0;
			

		}
		
	}
	

}




//三个光电开关   PA3 PA4 PA12     PA12反转极限   PA3 正转极限   PA4中间光电
void Swtich_Init(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_12;//光电开关触发时输出高电平     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);   

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;   //用于人工时的方向控制
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
	 GPIO_Init( GPIOF , &GPIO_InitStructure);
	
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource13|EXTI_PinSource15);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3|EXTI_PinSource4|EXTI_PinSource12);
  
	EXTI_InitStructure.EXTI_Line = EXTI_Line12|EXTI_Line4|EXTI_Line3;//
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//????
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //?????
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//?? LINE0
	EXTI_Init(&EXTI_InitStructure);

//  EXTI_InitStructure.EXTI_Line = EXTI_Line13|EXTI_Line15;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //下降沿触发进入人工模式，上升沿触发进入自动模式
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
	

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级和响应优先级都要比人体感应要高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
	NVIC_Init(&NVIC_InitStructure);//?? NVIC

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级和响应优先级都要比人体感应要高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
	NVIC_Init(&NVIC_InitStructure);//?? NVIC
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级和响应优先级都要比人体感应要高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
	NVIC_Init(&NVIC_InitStructure);//?? NVIC
	

	GPIO_SetBits(GPIOA,GPIO_Pin_3);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);

}


void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus( EXTI_Line3 ) != RESET)
	{
		if(PA3==0)//正转限位触发  驾驶员视角下右转 导致左边的光电触发
		{
			trigger = FIR_TRIGGER_FLAG;//第一次触发
			vel_mode_run(LEFT_TURN);//左转					
    }
  }
	
	EXTI_ClearITPendingBit(EXTI_Line3);
}


void EXTI4_IRQHandler(void)
{
   	if((EXTI_GetITStatus( EXTI_Line4 ) != RESET) && (trigger != UN_TRIGGER_FLAG))//
		{
			if((PA4 == 0) && (trigger == FIR_TRIGGER_FLAG))//如果是在中间经过了中间位置，并且是在上电第一次复位阶段,且必须是反转（司机视角左转）经过
			{
				delay_ms(500);//继续之前的方向运行500ms	
				vel_mode_run(RIGHT_TURN);
				trigger = SEC_TRIGGER_FLAG;//为第二次触发准备
			}
			else if((PA4 == 0) && (trigger == SEC_TRIGGER_FLAG))//第2次到达光电复位，复位结束
			{          
			  CAN2_SDO_WriteData(0x23,0x60ff, 0, 0, 2);	//这里只能用梯形速度为0让前轮停下，因为还没有进行设置停止点为本次上电零点的配置操作，因为上电时的点为驱动器默认零点，后面要用指令设置新的零点，该操作是在停下电机之后的
				trigger = STOP_TRIGGER_FLAG;				
			}		  					
  }
	EXTI_ClearITPendingBit(EXTI_Line4);
}



//上电时三个光电位置检测
void Check_Switch_hanle(void) //上电先检查每一个开关电平，电机才运动
{
	if(PA4 == 0)//复位开关，上电时如果处在已经复位的位置，也需要进行运动复位  5.6 2019
	{
		trigger = UN_TRIGGER_FLAG ;//
		vel_mode_run(RIGHT_TURN);
		MOTOR_PARA.ori_flag = 2;//正向标志
		
	}

	else if(PA3 == 0)//正转极限
	{
		trigger = FIR_TRIGGER_FLAG;//触发一次光电
		vel_mode_run(LEFT_TURN);
		MOTOR_PARA.ori_flag = 3;//反向标志
	}

	else if(PA12 == 0)//反转极限
	{
		trigger = UN_TRIGGER_FLAG;//第一次触发必须是触发PA3，即右转极限触发
		vel_mode_run(RIGHT_TURN);
		MOTOR_PARA.ori_flag = 2;//正向标志
		
	}

}



