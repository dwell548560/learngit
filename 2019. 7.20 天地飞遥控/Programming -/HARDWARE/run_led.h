#ifndef _RUN_LED
#define _RUN_LED
#include "sys.h"
#include <string.h>





#include "run_led.h"

#define LED1_ON  GPIO_ResetBits(GPIOE,GPIO_Pin_8);
#define LED1_OFF GPIO_SetBits(GPIOE,GPIO_Pin_8);
#define  CNT_CONVER    20


void run_led_gpio_init(void);

void work_state_led1(void);

void run_led_tim11_init(void);

void systick_robot_init(void);

extern u8 led1_flag;

extern u8 cnt_led ;
#endif
