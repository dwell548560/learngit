#include "kulun.h"


KL_type KULUN_DATA;//���ؼƲ����ṹ��


void Kulun_Usart6_Init(void)
{
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
	//PD3  RE
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //SP485оƬ�ķ��ͽ���ʹ�ܶ�
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //?? 100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //????
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
//	GPIO_Init(GPIOD,&GPIO_InitStructure); //PD3
		
	
//USART3
	USART_InitStructure.USART_BaudRate = 9600;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��Ϊ��һλżУ������9λ,��˼�ع̶�8λ����λ���ټ���һλУ��λ(żУ��)������9λ�ֳ���f407�������ģ�����оƬ�����Ƿֿ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//���ڰ�˼�ص��������ڲ�����λ115200ʱ������żУ��ʱ��ֹͣλΪ2λ
	USART_InitStructure.USART_Parity = USART_Parity_No;//żУ��
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

void DMA_Usart6_Init(void)
{

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
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(KULUN_DATA.kl_receive_data);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = USART6_RE_BUF;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;//ѭ��ģʽ
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);                      
   
	//NVIC_EnableIRQ(DMA_Channel_4);     
    DMA_Cmd(DMA2_Stream2, ENABLE);  

	//����
//    DMA_DeInit(DMA2_Stream6);
//    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;//���͹ܵ�
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
//	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(&person_val.DMA_Tx_Data);
//    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
//    DMA_InitStructure.DMA_BufferSize         = 7;
//    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;//ѭ��ģʽ
//    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
//    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull ;
//    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
//    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA2_Stream6, &DMA_InitStructure);  


}

void USART6_IRQHandler(void)
{

	u8 clearflag=0;
	u16 len =0;

	if(USART_GetITStatus(USART6,USART_IT_IDLE)==SET)
	{
		clearflag = USART6->SR;
		clearflag = USART6->DR;//�ȶ�SR  �ٶ�DR���IDLE��־

		DMA_Cmd(DMA2_Stream2, DISABLE);

		//DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //���������ɱ�־λ��������봫������ж�
		while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}

		len = USART6_RE_BUF-DMA_GetCurrDataCounter(DMA2_Stream2);

		if(len>0)
		{
			KULUN_DATA.rx_done_flag = 1;//��λ��һ֡���ݽ�����ɱ�־
			KULUN_DATA.rx_len_data = len;//ʵ�ʽ��յ������ݳ���

			//Usart6_reopen_rx();
			//USART_Cmd(USART6, DISABLE); 
		}
		else 
		{
			//memset( ( uint8_t* )KULUN_DATA.kl_receive_data, 0 , sizeof(KULUN_DATA.kl_receive_data) );
		//	KULUN_DATA.rx_done_flag=0;
			Usart6_reopen_rx();
							
		}
				
	}
	//USART_ClearITPendingBit(USART1,USART_IT_IDLE);
}

//������ȡ
void Kulun_handle(void)
{
	if(KULUN_DATA.rx_done_flag == 1)
	{		
		KULUN_DATA.rx_done_flag = 0;
		Kulun_Check_handle();
		if(KULUN_DATA.rx_check_flag == 1)
		{
			memcpy(KULUN_DATA.kl_success_data,KULUN_DATA.kl_receive_data,KULUN_DATA_LEN);//����ȷ���ݿ���������

			KULUN_DATA.battery_percent = KULUN_DATA.kl_success_data[3];

			KULUN_DATA.battery_voltage = 0;//����ϴε�����
			KULUN_DATA.battery_voltage = KULUN_DATA.battery_voltage |KULUN_DATA.kl_success_data[4];
			KULUN_DATA.battery_voltage <<= 8;
			KULUN_DATA.battery_voltage |= KULUN_DATA.kl_success_data[5];

			KULUN_DATA.battery_electricity = 0;
			KULUN_DATA.battery_electricity = KULUN_DATA.battery_voltage |KULUN_DATA.kl_success_data[6];
			KULUN_DATA.battery_electricity<<=8;
			KULUN_DATA.battery_electricity |= KULUN_DATA.kl_success_data[7];
	
			KULUN_DATA.set_battery = 0;
			KULUN_DATA.set_battery = KULUN_DATA.set_battery |KULUN_DATA.kl_success_data[8];
			KULUN_DATA.set_battery <<= 8;
			KULUN_DATA.set_battery |=  KULUN_DATA.kl_success_data[9];
				
			KULUN_DATA.electricity_dir = KULUN_DATA.kl_success_data[12];

			
			KULUN_DATA.rx_check_flag = 0;
			//memset(KULUN_DATA.kl_success_data,0,USART6_RE_BUF);
			//USART_Cmd(USART6, ENABLE); 
			Usart6_reopen_rx();//��ȡ�ɹ������¿����ڿ����ж�DMA
		}
		//USART_Cmd(USART6, DISABLE); 
		//memset(KULUN_DATA.kl_receive_data,0,USART6_RE_BUF);
		
		Usart6_reopen_rx();//У�鲻�ɹ������¿����ڿ����ж�DMA
	}
		
	//Usart6_reopen_rx();
}


//У���������
void Kulun_Check_handle(void)
{	
	u8 i = 0,j = 0;
	u8 sum = 0;

	for(i = 0;i < KULUN_DATA.rx_len_data ; i++)
	{
		if((KULUN_DATA.kl_receive_data[i] == 0x5a)&&(KULUN_DATA.kl_receive_data[i+1] == 0xa5)&&(KULUN_DATA.kl_receive_data[i+2] == 0x10))
		{
			for(j = i; j < KULUN_DATA.rx_len_data-i-1;j++)
			{
				sum += KULUN_DATA.kl_receive_data[j];
			}
			if(sum == KULUN_DATA.kl_receive_data[j])
			{			
				KULUN_DATA.rx_check_flag = 1;
			}
			//else memset(KULUN_DATA.kl_receive_data,0,USART6_RE_BUF);
		}
		//else memset(KULUN_DATA.kl_receive_data,0,USART6_RE_BUF);
	}
}


void Usart6_reopen_rx(void)
{


	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //���������ɱ�־λ��������봫������ж�
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
	
	DMA_SetCurrDataCounter(DMA2_Stream2,USART6_RE_BUF);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	






}
