#ifndef _common_exit_h
#define _common_exit_h

#include "sys.h"
#include <string.h>
#include "reset_motor.h"
#include "manul_hal.h"
#include "reset_motor.h"
#include "hard_stop.h"



#define CRASH_CLK_GPIOE    			 	RCC_AHB1Periph_GPIOF
#define GPIOX_CRASH_PIN  			 	GPIO_Pin_14 	
#define GPIOX_CRASH     	        	GPIOF
#define GPORTX_GPIO_EXTI_CRASH			EXTI_PortSourceGPIOF
#define GPORTX_EXTIPINSOURCE_CRASH  	EXTI_PinSource14
#define GPIOX_EXTI_LINE_CRASH   		EXTI_Line14
#define GPIO_EXTI_IRn_CRASH   			EXTI15_10_IRQn
#define GPIO_EXTI_IRQHANDLER_CRASH   	EXTI15_10_IRQHandler

#define CRASH_SWTICH          PFin(14) //·À×²Ìõ¿ª¹Ø


void crash_GPIO_Init(void);


__packed typedef struct{



u8 crash1_flag;







}CRASH_STU;

extern CRASH_STU crash_sta_type;



#endif
