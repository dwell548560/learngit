#ifndef _manul_hal_h
#define _manul_hal_h
#include "sys.h"
#include <string.h>
#include "reset_motor.h"
#include "delay.h"
#include "my_reset.h"

#define KINCO_RATIO 10000       //步科电机编码器分辨率 一圈10000脉冲
#define ACCELERATE_ADC_NUM  10  //adc一次保存数据 进行平均值滤波

#define ADC_ZERO_VAL   0.83

#define MAX_VEL_RPM     6000  //最大转速


void Accelerate_ADC_Init(void);

void Accelerate_DMA(void);

//不可以加入 __packed  否则ADC的值数据不对
 typedef struct accele{

u16 accelerate_re[ACCELERATE_ADC_NUM];

float adc_result_type;

float adc_voltage;

}ACCELERATE_TYPE;

extern ACCELERATE_TYPE  acce_data;
void ADC_average_handle(void);

void switch_gpio_init(void);

#define SWITCH_GPIO   PFin(15)
#define MAN_DIR_GPIO  PFin(13) 

#define UN_MAN_BUTTON_TRI  1
#define MAN_BUTTON_TRI     0
#define DIR_MAN_TRI        0
#define UN_DIR_MAN_TRI     1






#define MAN_TRI_SUCCESS    1
#define MAN_TRI_FAIL       0

#define MAN_DIR_FOR        0
#define MAN_DIR_BACK       1



#define TRIGGER_MANUL     1
#define UN_TRIGGER_MANUL  0


#define TRIGGER_DIR       1
#define UN_TRIGGER_DIR    0


extern u8 manul_trigger_flag;
extern u8 switch_button_flag;
extern u8 manual_motor_dir_flag;//人工控制时，方向控制标志，0：正向  1：反向

void manual_hal(void);


void manul_gpio_init(void);






//ACCELERATE_TYPE  acce_data;
#endif
