#include "wfly.h"

wfly_para my_wfly;
 int last_thorrt = 0 ;
 int last_sheer = 0;
	


	
void Wfly_Usart_Init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);//
//	
//		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 |GPIO_Pin_9; //
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //

//			
//	//SBUSͨ��
//	USART_InitStructure.USART_BaudRate = 100000;//������
//	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//��Ϊ��һλżУ������9λ,��˼�ع̶�8λ����λ���ټ���һλУ��λ(żУ��)������9λ�ֳ���f407�������ģ�����оƬ�����Ƿֿ���
//	USART_InitStructure.USART_StopBits = USART_StopBits_2;//���ڰ�˼�ص��������ڲ�����λ115200ʱ������żУ��ʱ��ֹͣλΪ2λ
//	USART_InitStructure.USART_Parity = USART_Parity_Even;//
//	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//��Ӳ��������
//	USART_InitStructure.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;//�շ�ģʽ
//	USART_Init(USART1, &USART_InitStructure); //
//	
//	USART_ClearFlag(USART1, USART_FLAG_TC);
//	
//	
//	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//??????


//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//??????

//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); 
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 

//	USART_ClearITPendingBit(USART1, USART_IT_TC);//����ж�TCλ	

//	USART_Cmd(USART1, ENABLE); //



//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//????? 3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;  //????? 3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ????
//	NVIC_Init(&NVIC_InitStructure);  //?????????? VIC ????          

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //PA
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //PC6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);//PC7
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //
	
	
//USART6  SBUS
	USART_InitStructure.USART_BaudRate = 100000;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//
	USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;//�շ�ģʽ
	USART_Init(USART6, &USART_InitStructure); //
	USART_Cmd(USART6, ENABLE); //
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	
	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);//�����ж�
 	USART_ITConfig(USART6, USART_IT_TC, DISABLE); 
	
// 	//����DMA��ʽ����  
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  
	//����DMA��ʽ����  
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE); 
	USART_ClearITPendingBit(USART6, USART_IT_TC);//����ж�TCλ	
	
	

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//????? 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  //????? 3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ????
	NVIC_Init(&NVIC_InitStructure);  //?????????? VIC ????

	USART_Cmd(USART6, ENABLE); 




}

void Wfly_DMA_Init(void)
{
//	DMA_InitTypeDef DMA_InitStructure;
//	NVIC_InitTypeDef  NVIC_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  

//    DMA_DeInit(DMA2_Stream5);
//	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}//?? DMA ???
//    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);   
//    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(my_wfly.rx_sbus_buf);
//    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
//    DMA_InitStructure.DMA_BufferSize         = MAX_RX_SBUS;
//    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //ѭ��ģʽ
//    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
//    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
//    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull ;
//    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
//    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA2_Stream5, &DMA_InitStructure); 
//                     
//    DMA_Cmd(DMA2_Stream5, ENABLE); 


	DMA_InitTypeDef DMA_InitStructure;
	//����DMAʱ��  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  

	DMA_DeInit(DMA2_Stream2);
    DMA_DeInit(DMA2_Stream2);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
	

	//����
    DMA_DeInit(DMA2_Stream2);					//���չܵ�
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(my_wfly.rx_sbus_buf);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = MAX_RX_SBUS;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//ѭ��ģʽ
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);                      
   
	     
    DMA_Cmd(DMA2_Stream2, ENABLE);  

}


void USART6_IRQHandler(void)
{

	u8 clearflag = 0;
	u16 len = 0;

	if(USART_GetITStatus(USART6, USART_IT_IDLE)==SET)
	{
		clearflag = USART6->SR;
		clearflag = USART6->DR;//�ȶ�SR  �ٶ�DR���IDLE��־

		DMA_Cmd(DMA2_Stream2, DISABLE);

		//DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //���������ɱ�־λ��������봫������ж�
		while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}

		len = MAX_RX_SBUS-DMA_GetCurrDataCounter(DMA2_Stream2);

		if(len>0)
		{
			my_wfly.rx_DMA_finish_flag = RX_SBUS_FINISH_FLAG;
			my_wfly.rx_DMA_Len = len;
		
			
		}
		else 
		{
			
			SBUS_reopen_rx();
							
		}
				
	}
	//USART_ClearITPendingBit(USART1,USART_IT_IDLE);
}



int SBUS_Swap_Motor(u16 swap_data, u8 data_class)
{
	switch (data_class)
	{
		case CH3_THORRT: 
		{
			if((swap_data >= MIN_THORRT_SBUS)&&(swap_data < MEDIUM_THORRT_SBUS - DEAD_ZONE))//�����ٶ�  333 - 1014
			{
				my_wfly.motor_rate = (double)((double)(((MEDIUM_THORRT_SBUS - DEAD_ZONE) - swap_data ) * MAX_MOTOT_DRIVER_RPM ) / ((MEDIUM_THORRT_SBUS - DEAD_ZONE) - MIN_THORRT_SBUS));
				my_wfly.motor_rate = -(double) ((my_wfly.motor_rate * 512 * KINCO_RATIO) / 1875);
			}
			else if((swap_data > (MEDIUM_THORRT_SBUS + DEAD_ZONE))&&(swap_data <= MAX_THORRT_SBUS ))//�����ٶ�
			{
				my_wfly.motor_rate = (double)((double)((double)(swap_data - (MEDIUM_THORRT_SBUS + DEAD_ZONE)) * MAX_MOTOT_DRIVER_RPM ) / (double)(MAX_THORRT_SBUS - (MEDIUM_THORRT_SBUS + DEAD_ZONE)));
				my_wfly.motor_rate = (double)((my_wfly.motor_rate * 512 * KINCO_RATIO) / 1875);
			}
			else if((swap_data >=  MEDIUM_THORRT_SBUS - DEAD_ZONE) && (swap_data <= MEDIUM_THORRT_SBUS + DEAD_ZONE))
			{
				my_wfly.motor_rate = 0;
			}
			else if((swap_data < MIN_THORRT_SBUS) || (swap_data > MAX_THORRT_SBUS))//���ͨ������С��333�������1706  
			{
				my_wfly.motor_rate = 0;
			}
			
			return my_wfly.motor_rate;
		}
		
		case CH4_STEER:
		{
			if((swap_data >= MIN_STEER_SBUS) && (swap_data < MEDIUM_STEER_SBUS - DEAD_ZONE))
			{
				my_wfly.steer_angle_pulse = (double)((((MEDIUM_STEER_SBUS - DEAD_ZONE) - swap_data) * MAX_ANGLE_PULSE) / ((MEDIUM_STEER_SBUS - DEAD_ZONE) - MIN_STEER_SBUS));
				
			}
			else if((swap_data > MEDIUM_STEER_SBUS + DEAD_ZONE) && (swap_data <= MAX_STEER_SBUS))
			{
				my_wfly.steer_angle_pulse = (double)(((swap_data - (MEDIUM_STEER_SBUS + DEAD_ZONE)) * MAX_ANGLE_PULSE) / (MAX_STEER_SBUS - (MEDIUM_STEER_SBUS + DEAD_ZONE)));
				my_wfly.steer_angle_pulse = -my_wfly.steer_angle_pulse;
			
			}
			else if((swap_data >= MEDIUM_STEER_SBUS - DEAD_ZONE ) && (swap_data <= MEDIUM_STEER_SBUS + DEAD_ZONE))
			{
				my_wfly.steer_angle_pulse = 0;

			}
			else if((swap_data < MIN_STEER_SBUS) || (swap_data > MAX_STEER_SBUS))
			{
				my_wfly.steer_angle_pulse = 0;
			}
			
			return my_wfly.steer_angle_pulse;
		}

		default: break;

	}

}



void SBUS_Motor_Hal(u16 thorrt_ch3, u16 sheer_ch4)
{
	static u8 state_swap = 1;

	if(state_swap == 1)
	{
	
		if(last_thorrt != thorrt_ch3)
			CAN2_RPDO_Speed( SBUS_Swap_Motor(thorrt_ch3, CH3_THORRT ), 1, 1, 4 );
		last_thorrt = thorrt_ch3;
		state_swap = 2;
	}
	if(state_swap == 2)
	{
		
		if(last_sheer != sheer_ch4)
		{
				CAN2_RPDO_Positon( SBUS_Swap_Motor(sheer_ch4, CH4_STEER ), 2, 1, 4 );
				delay_us(5);
				CAN2_RPDO_Positon(0x2f, 2, 2, 2 );
				delay_us(5);
				CAN2_RPDO_Positon(0x3f, 2, 2, 2 );
				delay_us(5);
			
				if((last_sheer > 0) && (last_sheer != MEDIUM_STEER_SBUS))my_wfly.open_wfly_reset_flag = OPEN_RESET_WBUS;//�ӱ������帴λ��ʼ��ң���˶���ǰ���򿪷�ң�ظ�λ����Ĺ��ܣ���λ���� E �������²�һ���ٲ���ȥ
		
				
		}
		
		last_sheer = sheer_ch4;
		state_swap = 1;
	}
}



void Get_Ch_Hal(void)
{
	static u8 atuo_manul_flag = 1;//�Զ��������˹����Ʊ�־���˹����ư������� ң�غ��˿�����Ĭ�Ͽ���ʱ�����Զ�����ģʽ
	
	if (switch_button_flag == MAN_TRI_FAIL)//��������˹������ſ�ʼ����ң��ģʽ
	{
		if(Wfly_SBUS_Check())
		{	
				my_wfly.check_success_flag = SBUS_CHECK_RESET;
				my_wfly.rx_DMA_finish_flag = RX_SBUS_FINISH_RESET;
				
				Wfly_Data_Channel(my_wfly.avail_buf);//���ң��ͨ������
			
			
			//�ж���û�п����������Ž����·������� ң����E���ˣ����²�Ϊ����ң��
				if(my_wfly.ch_data_buf[4] == 341 )
				{
					
					SBUS_Motor_Hal(my_wfly.ch_data_buf[2], my_wfly.ch_data_buf[3] );//�·������������
					
					if(!atuo_manul_flag)	//ֻ����һ��
					{
						//�л�Ϊ�ֶ�ģʽң����ʱ�����֣�ǰ�ֻ���
						CAN2_RPDO_Speed(0, 0x01, 1,4);
						delay_ms(20);
						
						CAN2_RPDO_Positon( 0, 2, 1, 4 );
						delay_us(10);
						CAN2_RPDO_Positon(0x2f, 2, 2, 2 );
						delay_us(10);
						CAN2_RPDO_Positon(0x3f, 2, 2, 2 );
						delay_us(10);
					
						atuo_manul_flag = 1;
						CAN2_RPDO_Speed(0, 0x00, 5,1);//�����ֶ�ң��ģʽ�ϱ���Ϣ
					
					}
					
				}
				
				else if((my_wfly.ch_data_buf[4] == 1706) && (atuo_manul_flag))
				{
					atuo_manul_flag = 0; 
					CAN2_RPDO_Speed(0, 0x00, 6,1);//�����Զ�����ģʽ���͸���λ����Ϣ����������CANOPEN��COB-ID��Ϊ��ʶ�ģ���λ���������COB-ID����ʶ��
					
				}	
				
				//SBUS_Motor_Hal(my_wfly.ch_data_buf[2], my_wfly.ch_data_buf[3] );//�·������������

				SBUS_reopen_rx();		
		}
		else SBUS_reopen_rx();
	}
	


}

void Wfly_Data_Channel(  const u8 *sBusData)
{

        
        my_wfly.ch_data_buf[0] = ( (sBusData[ 2]&0x07) << 8 ) +  sBusData[ 1]; //ͨ��1  Channel1                                                                       //sBus[ 2] low3 + sBus[ 1] low8 
        my_wfly.ch_data_buf[1] = ( (sBusData[ 3]&0x3F) << 5 ) + (sBusData[ 2] >> 3 );    //ͨ��1  Channel2                                                                 //sBus[ 3] low6 + sBus[ 2] high5 
        my_wfly.ch_data_buf[2] = ( (sBusData[ 5]&0x01) << 10) + (sBusData[ 4] << 2 ) + (sBusData[ 3] >> 6);                //sBus[ 5] low1 + sBus[ 4] low8  + sBus[ 3] high2 
        my_wfly.ch_data_buf[3] = ( (sBusData[ 6]&0x0F) << 7 ) + (sBusData[ 5] >> 1 );                                                                //sBus[ 6] low4 + sBus[ 5] high7 
        my_wfly.ch_data_buf[4] = ( (sBusData[ 7]&0x7F) << 4 ) + (sBusData[ 6] >> 4 );                                                                //sBus[ 7] low7 + sBus[ 6] high4 
        my_wfly.ch_data_buf[5] = ( (sBusData[ 9]&0x03) << 9 ) + (sBusData[ 8] << 1 ) + (sBusData[ 7] >> 7);                //sBus[ 9] low2 + sBus[ 8] low8  + sBus[ 7] high1 
        my_wfly.ch_data_buf[6] = ( (sBusData[10]&0x1F) << 6 ) + (sBusData[ 9] >> 2 );                                                                //sBus[10] low5 + sBus[ 9] high6 
        my_wfly.ch_data_buf[7] = ( (sBusData[11]&0xFF) << 3 ) + (sBusData[10] >> 5 );                                                                //sBus[11] low8 + sBus[10] high3 
        my_wfly.ch_data_buf[8] = ( (sBusData[13]&0x07) << 8 ) +  sBusData[12];                                                                        //sBus[13] low3 + sBus[12] low8 
        my_wfly.ch_data_buf[9] = ( (sBusData[14]&0x3F) << 5 ) + (sBusData[13] >> 3 );                                                                //sBus[14] low6 + sBus[13] high5 
        my_wfly.ch_data_buf[10] = ( (sBusData[16]&0x01) << 10) + (sBusData[15] << 2 ) + (sBusData[14] >> 6);                //sBus[16] low1 + sBus[15] low8  + sBus[14] high2 
        my_wfly.ch_data_buf[11] = ( (sBusData[17]&0x0F) << 7 ) + (sBusData[16] >> 1 );                                                                //sBus[17] low4 + sBus[16] high7 
        my_wfly.ch_data_buf[12] = ( (sBusData[18]&0x7F) << 4 ) + (sBusData[17] >> 4 );                                                                //sBus[18] low7 + sBus[17] high4 
        my_wfly.ch_data_buf[13] = ( (sBusData[20]&0x03) << 9 ) + (sBusData[19] << 1 ) + (sBusData[18] >> 7);                //sBus[20] low2 + sBus[19] low8  + sBus[18] high1 
        my_wfly.ch_data_buf[14] = ( (sBusData[21]&0x1F) << 6 ) + (sBusData[20] >> 2 );                                                                //sBus[21] low5 + sBus[20] high6 
        my_wfly.ch_data_buf[15] = ( (sBusData[22]&0xFF) << 3 ) + (sBusData[21] >> 5 );                                                                //sBus[22] low8 + sBus[21] high3 
	
		
}


//У������SBUS
u8 Wfly_SBUS_Check(void)
{
	u16 i = 0;
	if(my_wfly.rx_DMA_finish_flag == 1)
	{
		for(i = 0; i < my_wfly.rx_DMA_Len; i++)
		{
			if(my_wfly.rx_sbus_buf[i] == 0x0f && my_wfly.rx_sbus_buf[i + 24] == 0x00)
			{
				my_wfly.check_success_flag = SBUS_CHECK_SUCCESS_FLAG;
				memcpy(my_wfly.avail_buf, &my_wfly.rx_sbus_buf[i], SBUS_DATA_LEN);
				return my_wfly.check_success_flag;
			}
		}
		if(my_wfly.check_success_flag == 0)
		{
			SBUS_reopen_rx();
			return 0;
			
		}
		
	}
	else return 0;
}




void SBUS_reopen_rx(void)
{
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //���������ɱ�־λ��������봫������ж�
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
	
	DMA_SetCurrDataCounter(DMA2_Stream2, MAX_RX_SBUS);
	DMA_Cmd(DMA2_Stream2, ENABLE);


}