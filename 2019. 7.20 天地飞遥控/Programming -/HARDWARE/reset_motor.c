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
//�����ʣ� CAN_SJW��   CAN_BS1��    CAN_BS2��CAN_Prescaler 
	{5,   CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,450},		//δͨ			
	{10,  CAN_SJW_1tq,CAN_BS1_6tq,CAN_BS2_2tq, 400},		//δͨ			
	{15,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,150},		//15K  δͨ
	{20,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,200},		//20k //δͨ
	{25,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,112},		//25k  δͨ
	{40,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,100},		//40k  δͨ
	{50,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,56},			//50k	ok
	{62,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,36},			//62.5k
	{80,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,50},			//80k   δͨ
	{100, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,28},			//100K	ok
	{125, CAN_SJW_1tq,CAN_BS1_13tq, CAN_BS2_2tq,18},		//125K δͨ
	{200, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,14},			//200K  ok
	{250, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,8},		    //250k  ok
	{400, CAN_SJW_1tq,CAN_BS1_15tq, CAN_BS2_5tq,5},			//400K  ok
	{500, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,4},			//500K	ok
	{666, CAN_SJW_1tq,CAN_BS1_5tq, CAN_BS2_2tq,8},			//δͨ
	{800, CAN_SJW_1tq,CAN_BS1_8tq, CAN_BS2_3tq,14},			//800K δͨ
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);//CAN2�Ǵ�CAN1����Ҫ�ȴ�CAN1��ʱ��
	
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

	CAN_FilterInitStructure.CAN_FilterNumber = 15;	   //CAN1�˲����Ŵ�0��13   CAN2���˲�������14-27

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //�˲�����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;	//�������κ�ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;		//�������κ�ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

//CAN2_WriteData(0x20,SET_VECLOCITY_OB,0x01,0x00000064,0x01);

//�жϽ��պ���
void CAN2_RX0_IRQHandler(void)
{
  u8 i;
	CanRxMsg RxMessage;
	
  CAN_Receive(CAN2, 0, &RxMessage);

	//�����4�ֽڣ�Ϊ�ٶ�ֵ�������ֽ���ʱû���õ���Ϊ0
	if(RxMessage.StdId == 0x588)//���IDΪ0x181  TPDO1��COB_IDΪ0x181     Robotqʹ�õ��Ǳ�׼֡ID
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


//SDOһ��ֻ�������ò�����ʵ���·��ٶȵ�ʹ��RPDO
//cmd ������/��ѯ ʣ���ֽ�����8λ����
//index �������������Robotq�ṩ���ֲ��ѯ
//sub_index ������
//data SDO ���͵�����
//ID  �ڵ��
void CAN2_SDO_WriteData(u8 cmd,u16 index,u8 sub_index,int data,u8 id)
{
	u16 i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	SET_SDO_HAL( cmd, index, sub_index, data );

	TxMessage.StdId = 0x600+id;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA; /* ����Ϊ����֡ */
	TxMessage.DLC = 8;            /* ���ݳ���, can���Ĺ涨�������ݳ���Ϊ8�ֽ� */
	
	//Progress(buf);
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = thrrot_type.payload_data[i];//��������
	}
	
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* ���������Ϣ�����͵������0,1,2��û���������뷢��no_box */	
	while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i !=  0xFFFF))
	{
		i ++;
	}

}


//����RPDO���ݵ�������   ���͵�������һ��S32λ���ݣ�����RPDOһ֡���ݿ��Է���������Դ��������1�͵��2���ٶȣ�ֻ������Ҫ�ڽű��м���RPDO����
//RPDO_data_low:Ҫ�������ݵĵ�4�ֽ�
//RPDO_data_high ��4�ֽ�
//id ��CAN_ID  �ڵ�1 id=1
//RPDO_num :�ڼ���RPDO���ܹ����ĸ���RPDO1-RPDO4  COB_IDҲ��ͬ
void CAN2_RPDO_WriteData(s32 RPDO_data_low, s32 RPDO_data_high, u8 id, u8 RPDO_num, u8 send_len)
{
	u16 i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
		
	switch(RPDO_num)
	{
		case  0://�����ڵ�PDO
		{
			TxMessage.StdId = 0x000 + id;
		  break;
		}
		case  1 ://�����ٶ�
		{
			TxMessage.StdId = 0x200+id;//��Ӧ���Ƶ��д �ٶȻ�λ��
			break;
		}
		case  2:
		{
			TxMessage.StdId = 0x300+id;//��Ӧ���Ƶ��д ������
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

		case 5://�����ֶ�ģʽʱ��������PC���͵�COB-ID
		{
			TxMessage.StdId = 0x666;
			break;

		}

		case 6://�����Զ�ʱ��������PC���͵�COB-ID
		{
			TxMessage.StdId = 0x667;
			break;

		}



		default:break;

	}

	//TxMessage.IDE = CAN_ID_STD;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA; /* ����Ϊ����֡ */
	TxMessage.DLC = send_len;            /* ���ݳ���, can���Ĺ涨�������ݳ���Ϊ8�ֽ� */
	
	
	//Progress(buf);
	for(i = 0;i < 8; i ++)
	{
		if(i<4)TxMessage.Data[i] =  (u8)(RPDO_data_low>>(8*i));
		else if(i >= 4)TxMessage.Data[i] =  (u8)(RPDO_data_high>>(8*(i-4)));
	
	}
		RPDO_data_low = 0;
	  RPDO_data_high = 0;
	
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* ���������Ϣ�����͵������0,1,2��û���������뷢��no_box */	
	
	//while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i !=  0xFFFF))
//	{
	//	i ++;
		
	//}

}



//����SDO 8�ֽ�����
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

//δ��
void CAN2_RPDO_SET( u16 index,u8 sub_index,u32 data)
{

	u16 index_rb;

	index_rb = index;

	switch(index_rb)
	{
		case READ_MOTOR_CURRENT_OB :  //��õ��������� 0x2100
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
	 
//SET_SDO_HAL( 0X04, index, sub_index, data);//���գ���ѯ����


}


//�·��ٶ� RPDO1  ����
void CAN2_RPDO1_Speed(s32 low_loop_speed, s32 high_data,int can_id, int numb, u8 send_len)
{

	CAN2_RPDO_WriteData(low_loop_speed , high_data ,can_id , numb,  send_len);//�ٶ�loop_speed���ڵ�1 RPDO1


}





void CAN2_RPDO_Speed(s32 get_speed, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_speed , 0 ,can_id , pdo_numb,  send_len);//�ٶ�

}



void CAN2_RPDO_Positon(s32 get_positon, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_positon , 0 ,can_id , pdo_numb, send_len);//λ��

}


void CAN2_RPDO_Control_Word(u16 get_word, int can_id, u8 pdo_numb, u8 send_len)
{

	CAN2_RPDO_WriteData(get_word , 0 ,can_id , pdo_numb,  send_len);//λ��



}

void start_pdo(int get_cob_id, int data, u8 send_len)
{

    CAN2_RPDO_WriteData(data , 0 ,get_cob_id , 0,  send_len);//



}




void vel_mode_run(u8 dir_turn)
{
  if(dir_turn == RIGHT_TURN)//��ת
	{
	  CAN2_SDO_WriteData(0x2f,0x6060, 0, -3, 2);//�ٶ�ģʽ��id��2 ǰ���������������˶���-3�����ٶ�
		delay_ms(100);
		CAN2_SDO_WriteData(0x23,0x60ff, 0, 0x00045555, 2);//�����ٶ�ֵ ,��ת
		delay_ms(100);					
		CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 2);//����
	
	}
	else if(dir_turn == LEFT_TURN)//��ת
	{
	  CAN2_SDO_WriteData(0x2f,0x6060, 0, -3, 2);//�ٶ�ģʽ��id��2 ǰ���������������˶���-3�����ٶ�
		delay_ms(100);
		CAN2_SDO_WriteData(0x23,0x60ff, 0, -0x00045555, 2);//�����ٶ�ֵ ,��ת
		delay_ms(100);					
		CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 2);//����
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
	CAN2_SDO_WriteData(0x2f, 0x6060, 0x00, -3, 1);//�ڵ�1 ����ģʽ �����ٶ�ģʽ
			
	CAN2_SDO_WriteData(0x2f, 0x6060, 0x00, 1,2);//�ڵ�2 ����ģʽ λ��ģʽ
			
	CAN2_SDO_WriteData(0x2b,0x6040, 0, 0x0f, 1);//�ڵ�1����
			
	CAN2_SDO_WriteData(0x23, 0x6081, 0x00, 0xc8000, 2);//�����ٶ�300rpm

}















void Reset_hal(void)
{

   if(PA4 == 1)//����������м��紥����λ��
	{
		if(MOTOR_PARA.reset_flag == 0)//���ڵ�һ���ϵ縴λ����δ��λ��
		{
			
			
			//ǰ���˶�
			vel_mode_run(RIGHT_TURN);
			MOTOR_PARA.ori_flag = 2;//��ת��־
			MOTOR_PARA.ab_angle = 0;
		}
		else //�����λ���˲�������
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




//������翪��   PA3 PA4 PA12     PA12��ת����   PA3 ��ת����   PA4�м���
void Swtich_Init(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_12;//��翪�ش���ʱ����ߵ�ƽ     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);   

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;   //�����˹�ʱ�ķ������
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
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //�½��ش��������˹�ģʽ�������ش��������Զ�ģʽ
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
	

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�����Ӧ���ȼ���Ҫ�������ӦҪ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
	NVIC_Init(&NVIC_InitStructure);//?? NVIC

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�����Ӧ���ȼ���Ҫ�������ӦҪ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
	NVIC_Init(&NVIC_InitStructure);//?? NVIC
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//???? 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�����Ӧ���ȼ���Ҫ�������ӦҪ��
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
		if(PA3==0)//��ת��λ����  ��ʻԱ�ӽ�����ת ������ߵĹ�紥��
		{
			trigger = FIR_TRIGGER_FLAG;//��һ�δ���
			vel_mode_run(LEFT_TURN);//��ת					
    }
  }
	
	EXTI_ClearITPendingBit(EXTI_Line3);
}


void EXTI4_IRQHandler(void)
{
   	if((EXTI_GetITStatus( EXTI_Line4 ) != RESET) && (trigger != UN_TRIGGER_FLAG))//
		{
			if((PA4 == 0) && (trigger == FIR_TRIGGER_FLAG))//��������м侭�����м�λ�ã����������ϵ��һ�θ�λ�׶�,�ұ����Ƿ�ת��˾���ӽ���ת������
			{
				delay_ms(500);//����֮ǰ�ķ�������500ms	
				vel_mode_run(RIGHT_TURN);
				trigger = SEC_TRIGGER_FLAG;//Ϊ�ڶ��δ���׼��
			}
			else if((PA4 == 0) && (trigger == SEC_TRIGGER_FLAG))//��2�ε����縴λ����λ����
			{          
			  CAN2_SDO_WriteData(0x23,0x60ff, 0, 0, 2);	//����ֻ���������ٶ�Ϊ0��ǰ��ͣ�£���Ϊ��û�н�������ֹͣ��Ϊ�����ϵ��������ò�������Ϊ�ϵ�ʱ�ĵ�Ϊ������Ĭ����㣬����Ҫ��ָ�������µ���㣬�ò�������ͣ�µ��֮���
				trigger = STOP_TRIGGER_FLAG;				
			}		  					
  }
	EXTI_ClearITPendingBit(EXTI_Line4);
}



//�ϵ�ʱ�������λ�ü��
void Check_Switch_hanle(void) //�ϵ��ȼ��ÿһ�����ص�ƽ��������˶�
{
	if(PA4 == 0)//��λ���أ��ϵ�ʱ��������Ѿ���λ��λ�ã�Ҳ��Ҫ�����˶���λ  5.6 2019
	{
		trigger = UN_TRIGGER_FLAG ;//
		vel_mode_run(RIGHT_TURN);
		MOTOR_PARA.ori_flag = 2;//�����־
		
	}

	else if(PA3 == 0)//��ת����
	{
		trigger = FIR_TRIGGER_FLAG;//����һ�ι��
		vel_mode_run(LEFT_TURN);
		MOTOR_PARA.ori_flag = 3;//�����־
	}

	else if(PA12 == 0)//��ת����
	{
		trigger = UN_TRIGGER_FLAG;//��һ�δ��������Ǵ���PA3������ת���޴���
		vel_mode_run(RIGHT_TURN);
		MOTOR_PARA.ori_flag = 2;//�����־
		
	}

}



