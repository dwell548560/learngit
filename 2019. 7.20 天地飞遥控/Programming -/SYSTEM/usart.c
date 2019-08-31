#include "usart.h"

void ADC_conver_Usart(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;//9600
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode =  USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
  USART_ClearITPendingBit(USART1, USART_IT_TC);//??3y?D??TC??
  USART_Cmd(USART1, ENABLE);
}

void USART_ADC_Send_data(u8 length,u16 * address)//����??������?����y?Y��?PC
{
    u8 i;
	for(i=0;i<length;i++)
	{
		USART_SendData(USART1, *(address+i));
		/* �̨���y����?��������? */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	}
	
	USART_SendData(USART1, * address);
}


////�ض���c�⺯��printf������USART1���ض�������ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����?����???��??����y?Y��?��??��DEBUG_USART */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �̨���y����?��������? */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��printf������USART1���ض�������ʹ��scanf getchar����
int fgetc(FILE *f)
{
		/* �̨���y��??����?��?��y?Y */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
/*********************************************END OF FILE**********************/