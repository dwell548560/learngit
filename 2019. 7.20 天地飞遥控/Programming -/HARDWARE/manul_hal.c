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
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC1 ��λ��ʼ
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //��λ����
	

	ADC1->CR2|=0x00000300;//����DMAģʽ��DMA��ֹѡ����CR2�� λ8 ��λ9��1
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
	//ADC_SoftwareStartConv(ADC1); //ʹ��ָ���� ADC1 �����ת����������  ������������󣬷���ɼ��������ݣ�����ǿ���ADC�ɼ����˹�ģʽ������Ŵ�

}


void Accelerate_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&ADC1->DR);	
  // �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı���	
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)acce_data.accelerate_re;  //�ڴ��ַ
  // ���ݴ��䷽��Ϊ���赽�洢��	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	// ��������СΪ��ָһ�δ����������
	DMA_InitStructure.DMA_BufferSize = ACCELERATE_ADC_NUM;//
	// ����Ĵ���ֻ��һ������ַ���õ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // �洢����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�洢����ַ����
  // // �������ݴ�СΪ���֣��������ֽ� 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  //	�洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	
	// ѭ������ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  // ��ֹDMA FIFO	��ʹ��ֱ��ģʽ
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
  // FIFO ��С��FIFOģʽ��ֹʱ�������������	
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	// ѡ�� DMA ͨ����ͨ������������
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
  //��ʼ��DMA�������൱��һ����Ĺܵ����ܵ������кܶ�ͨ��
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	// ʹ��DMA��
	DMA_Cmd(DMA2_Stream0, ENABLE);

}



//ʵ������ADC�ɼ��������������ʱΪ0.83V,���Ųȵ�����2.56V��
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


//�˹�����
void manual_hal(void)
{
	
	
	static u8 star_type = 0  ; 
	static u8 swap_type = 0;//����ӣ� ��ֹ���Ϸ���RPDOռ������
	static u8 last_speed_type = 0;//���ڱ����ϴ��ٶ�ֵ
	
//����Ӳ�����ţ����������ѯ�ķ�ʽ�ж��˹��л�״̬
	if(SWITCH_GPIO == MAN_BUTTON_TRI)//����
	{
		switch_button_flag = MAN_TRI_SUCCESS;//��1�������˹�����
		
	
		
		
	}
	else if(SWITCH_GPIO == UN_MAN_BUTTON_TRI)
	{
		switch_button_flag = MAN_TRI_FAIL;//
		
	}
		
		
	if(MAN_DIR_GPIO == DIR_MAN_TRI)//���� ����
	{
		manual_motor_dir_flag = MAN_DIR_BACK;//	
	}
	else if(MAN_DIR_GPIO == UN_DIR_MAN_TRI)
	{
		manual_motor_dir_flag = MAN_DIR_FOR;//��־��0���Զ�ģʽ
	}
		
		
	if(switch_button_flag == MAN_TRI_SUCCESS )//������˹�ģʽ
	{
			if(star_type == 0)
			{	

				CAN2_RPDO_Speed(0, 0x00, 5,1);//�����ֶ�ģʽ ����λ������һ��COB-IDΪ0x666�����ݣ����ݱ��������壬�������ֶ��е��˿�ģʽ
				delay_ms(2);

				ADC_Cmd(ADC1, ENABLE);
				ADC_SoftwareStartConv(ADC1);//�����˹�ģʽ�˲Ŵ�
				CAN2_RPDO_Control_Word(0x06, 0x02, 2, 2); //ǰ�ֽ���
				
			}
			ADC_average_handle();//���Ųɼ�����
			delay_us(10);

			if((acce_data.adc_voltage >= ADC_ZERO_VAL) && (acce_data.adc_voltage <= 3.3))//
			{
				speed_type = MAX_VEL_RPM * (float)((acce_data.adc_voltage - ADC_ZERO_VAL) / 3.3);//��Ϊ�����0.83V���� 
						
				if(speed_type <= MAX_VEL_RPM)
				{
					speed_type = (speed_type * 512 * KINCO_RATIO)/1875;//���Ƶ���ٶ�ת����ʽ
					if(manual_motor_dir_flag == 0 )	CAN2_RPDO_Speed(speed_type, 0x01, 1,4);//
					else if(manual_motor_dir_flag == 1)	CAN2_RPDO_Speed(-speed_type, 0x01, 1,4);//���ֵ����ת������4.28 .2019
				
				}
			}	
			else
			{
				CAN2_RPDO_Speed(0, 0x01, 1,4);
				
			}
			last_speed_type = speed_type;
					
			star_type = 1;
			swap_type = 1;
			delay_ms(2);//��Ҫ������ʱ�������͸����ߵ��ٶ�0x201 id̫��ռ�����ߣ��������������ϴ���TPDO1 TPDO2�޷��ϴ�
	}

	else if (switch_button_flag == MAN_TRI_FAIL)//�Զ�ģʽ�¾͹ر�AD�ɼ�
	{	
		ADC_Cmd(ADC1, DISABLE);
		
		if(swap_type != 0)//����ǰ�ֻ����޷�����ǰ�ֵ�Ħ�����⣬���Դ��ֶ��лص��Զ���Ҫ����ǰ�ָ�λ����
		{	
			//CAN2_RPDO_Speed(0, 0x00, 6,1);//�����Զ�ģʽ��COB-ID,���ݱ���������
			//delay_ms(2);
			CAN2_RPDO_Speed(0, 0x01, 1,4);//ֹͣ���ֵ��
		
			mcu_reset();//��λ����
		}
		
		acce_data.adc_voltage = 0;
		star_type = 0;	
		swap_type = 0;
	}
	
}
