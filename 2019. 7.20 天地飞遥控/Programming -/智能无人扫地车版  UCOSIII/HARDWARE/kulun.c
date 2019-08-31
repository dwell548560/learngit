#include "kulun.h"


KL_type KULUN_DATA;//库仑计参数结构体


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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //
	//PD3  RE
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //SP485芯片的发送接收使能端
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //?? 100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //????
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
//	GPIO_Init(GPIOD,&GPIO_InitStructure); //PD3
		
	
//USART3
	USART_InitStructure.USART_BaudRate = 9600;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//因为有一位偶校验所以9位,艾思控固定8位数据位，再加上一位校验位(偶校验)，就是9位字长，f407是这样的，其他芯片可能是分开的
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//对于艾思控的驱动器在波特率位115200时，无奇偶校验时，停止位为2位
	USART_InitStructure.USART_Parity = USART_Parity_No;//偶校验
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//无硬件控制流
	USART_InitStructure.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;//收发模式
	USART_Init(USART6, &USART_InitStructure); //
	USART_Cmd(USART6, ENABLE); //
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	
	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);//空闲中断
 	USART_ITConfig(USART6, USART_IT_TC, DISABLE); 
	
// 	//采用DMA方式发送  
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  
	//采用DMA方式接收  
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE); 
	USART_ClearITPendingBit(USART6, USART_IT_TC);//清除中断TC位	
	
	

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
	//启动DMA时钟  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  

	DMA_DeInit(DMA2_Stream2);
    DMA_DeInit(DMA2_Stream2);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
	

	//接收
    DMA_DeInit(DMA2_Stream2);					//接收管道
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(KULUN_DATA.kl_receive_data);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = USART6_RE_BUF;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;//循环模式
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);                      
   
	//NVIC_EnableIRQ(DMA_Channel_4);     
    DMA_Cmd(DMA2_Stream2, ENABLE);  

	//发送
//    DMA_DeInit(DMA2_Stream6);
//    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;//发送管道
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
//	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(&person_val.DMA_Tx_Data);
//    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
//    DMA_InitStructure.DMA_BufferSize         = 7;
//    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;//循环模式
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
		clearflag = USART6->DR;//先读SR  再读DR清除IDLE标志

		DMA_Cmd(DMA2_Stream2, DISABLE);

		//DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //清除传输完成标志位，否则进入传输完成中断
		while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}

		len = USART6_RE_BUF-DMA_GetCurrDataCounter(DMA2_Stream2);

		if(len>0)
		{
			KULUN_DATA.rx_done_flag = 1;//上位机一帧数据接收完成标志
			KULUN_DATA.rx_len_data = len;//实际接收到的数据长度

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

//数据提取
void Kulun_handle(void)
{
	if(KULUN_DATA.rx_done_flag == 1)
	{		
		KULUN_DATA.rx_done_flag = 0;
		Kulun_Check_handle();
		if(KULUN_DATA.rx_check_flag == 1)
		{
			memcpy(KULUN_DATA.kl_success_data,KULUN_DATA.kl_receive_data,KULUN_DATA_LEN);//将正确数据拷贝到缓存

			KULUN_DATA.battery_percent = KULUN_DATA.kl_success_data[3];

			KULUN_DATA.battery_voltage = 0;//清除上次的数据
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
			Usart6_reopen_rx();//提取成功，重新开串口空闲中断DMA
		}
		//USART_Cmd(USART6, DISABLE); 
		//memset(KULUN_DATA.kl_receive_data,0,USART6_RE_BUF);
		
		Usart6_reopen_rx();//校验不成功，重新开串口空闲中断DMA
	}
		
	//Usart6_reopen_rx();
}


//校验接收数据
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
	DMA_ClearFlag( DMA2_Stream2, DMA_FLAG_TCIF2 );    //清除传输完成标志位，否则进入传输完成中断
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}
	
	DMA_SetCurrDataCounter(DMA2_Stream2,USART6_RE_BUF);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	






}
