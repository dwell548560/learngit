#ifndef _USART_H
#define _USART_H
#include "stdio.h"//头文件必须包含标准输入输出头文件
#include "sys.h"
void ADC_conver_Usart(void);
void USART_ADC_Send_data(u8 length,u16 * address);

#endif
