#include "manul_hal.h"

ACCELERATE_TYPE  acce_data;
u8 switch_button_flag;
u8 manual_motor_dir_flag;
u8 manul_trigger_flag = 0;




int64_t speed_type = 0;

void Accelerate_ADC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	//??? ADC ? GPIO ???????? GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//?? GPIOA ??
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //?? ADC1 ??
	//???? ADC1 ?? 5 IO ?

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 ?? 5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//?????
	GPIO_Init(GPIOA, &GPIO_InitStructure);//???
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC1 复位开始
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //复位结束
	

	ADC1->CR2|=0x00000300;//开启DMA模式和DMA禁止选择，是CR2的 位8 和位9置1
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//????
	ADC_CommonInitStructure.ADC_TwoSamplingDelay =ADC_TwoSamplingDelay_20Cycles;//??????????? 5 ???
	ADC_CommonInitStructure.ADC_DMAAccessMode =ADC_DMAAccessMode_Disabled; //DMA ??
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//??? 4 ???
	//ADCCLK=PCLK2/4=84/4=21Mhz,ADC ???????? 36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);//???
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12 ???
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//?????
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//??????
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//??????,??????
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//??? 
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1 ?????????
	ADC_Init(ADC1, &ADC_InitStructure);//ADC ???
	//??? ADC ??
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles ); 
	
	
	ADC_Cmd(ADC1, ENABLE);//?? AD ???
	//ADC_SoftwareStartConv(ADC1); //使能指定的 ADC1 的软件转换启动功能  这个必须放在最后，否则采集不到数据，这个是开启ADC采集，人工模式开启后才打开

}


void Accelerate_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&ADC1->DR);	
  // 存储器地址，实际上就是一个内部SRAM的变量	
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)acce_data.accelerate_re;  //内存地址
  // 数据传输方向为外设到存储器	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	// 缓冲区大小为，指一次传输的数据量
	DMA_InitStructure.DMA_BufferSize = ACCELERATE_ADC_NUM;//
	// 外设寄存器只有一个，地址不用递增
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // 存储器地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //存储器地址增加
  // // 外设数据大小为半字，即两个字节 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  //	存储器数据大小也为半字，跟外设数据大小相同
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	
	// 循环传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  // 禁止DMA FIFO	，使用直连模式
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
  // FIFO 大小，FIFO模式禁止时，这个不用配置	
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	// 选择 DMA 通道，通道存在于流中
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
  //初始化DMA流，流相当于一个大的管道，管道里面有很多通道
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	// 使能DMA流
	DMA_Cmd(DMA2_Stream0, ENABLE);

}



//实测油门ADC采集结果，不睬油门时为0.83V,油门踩到底是2.56V。
void ADC_average_handle(void)
{
	u8 i;
	u32 sum = 0;
	for(i = 0 ;i < ACCELERATE_ADC_NUM; i++)
	{
		sum += acce_data.accelerate_re[i];
	}

	acce_data.adc_result_type = (float)((float)sum/ACCELERATE_ADC_NUM);

	acce_data.adc_voltage = (float)((3.3 * acce_data.adc_result_type)/4096 );

}


//人工控制
void manual_hal(void)
{
	
	
	static u8 star_type = 0  ; 
	static u8 swap_type = 0;//必须加， 防止不断发送RPDO占用总线
	static u8 last_speed_type = 0;//用于保存上次速度值
	
//由于硬件干扰，这里采用轮询的方式判断人工切换状态
	if(SWITCH_GPIO == MAN_BUTTON_TRI)//触发
	{
		switch_button_flag = MAN_TRI_SUCCESS;//置1，进行人工控制
		
	
		
		
	}
	else if(SWITCH_GPIO == UN_MAN_BUTTON_TRI)
	{
		switch_button_flag = MAN_TRI_FAIL;//
		
	}
		
		
	if(MAN_DIR_GPIO == DIR_MAN_TRI)//触发 方向
	{
		manual_motor_dir_flag = MAN_DIR_BACK;//	
	}
	else if(MAN_DIR_GPIO == UN_DIR_MAN_TRI)
	{
		manual_motor_dir_flag = MAN_DIR_FOR;//标志置0，自动模式
	}
		
		
	if(switch_button_flag == MAN_TRI_SUCCESS )//如果是人工模式
	{
			if(star_type == 0)
			{	

				CAN2_RPDO_Speed(0, 0x00, 5,1);//进入手动模式 向上位机发送一个COB-ID为0x666的数据，数据本身无意义，这里是手动中的人开模式
				delay_ms(2);

				ADC_Cmd(ADC1, ENABLE);
				ADC_SoftwareStartConv(ADC1);//进入人工模式了才打开
				CAN2_RPDO_Control_Word(0x06, 0x02, 2, 2); //前轮解锁
				
			}
			ADC_average_handle();//油门采集数据
			delay_us(10);

			if((acce_data.adc_voltage >= ADC_ZERO_VAL) && (acce_data.adc_voltage <= 3.3))//
			{
				speed_type = MAX_VEL_RPM * (float)((acce_data.adc_voltage - ADC_ZERO_VAL) / 3.3);//因为最低是0.83V左右 
						
				if(speed_type <= MAX_VEL_RPM)
				{
					speed_type = (speed_type * 512 * KINCO_RATIO)/1875;//步科电机速度转化公式
					if(manual_motor_dir_flag == 0 )	CAN2_RPDO_Speed(speed_type, 0x01, 1,4);//
					else if(manual_motor_dir_flag == 1)	CAN2_RPDO_Speed(-speed_type, 0x01, 1,4);//后轮电机反转，倒车4.28 .2019
				
				}
			}	
			else
			{
				CAN2_RPDO_Speed(0, 0x01, 1,4);
				
			}
			last_speed_type = speed_type;
					
			star_type = 1;
			swap_type = 1;
			delay_ms(2);//需要加入延时，否则发送给总线的速度0x201 id太多占用总线，导致驱动器的上传的TPDO1 TPDO2无法上传
	}

	else if (switch_button_flag == MAN_TRI_FAIL)//自动模式下就关闭AD采集
	{	
		ADC_Cmd(ADC1, DISABLE);
		
		if(swap_type != 0)//由于前轮回零无法消除前轮的摩擦问题，所以从手动切回到自动需要进行前轮复位操作
		{	
			//CAN2_RPDO_Speed(0, 0x00, 6,1);//进入自动模式的COB-ID,数据本身无意义
			//delay_ms(2);
			CAN2_RPDO_Speed(0, 0x01, 1,4);//停止后轮电机
		
			mcu_reset();//复位主板
		}
		
		acce_data.adc_voltage = 0;
		star_type = 0;	
		swap_type = 0;
	}
	
}
