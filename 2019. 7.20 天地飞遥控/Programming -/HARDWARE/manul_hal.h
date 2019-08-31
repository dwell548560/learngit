#ifndef _manul_hal_h
#define _manul_hal_h
#include "sys.h"
#include <string.h>
#include "reset_motor.h"
#include "delay.h"
#include "my_reset.h"

#define KINCO_RATIO 10000       //���Ƶ���������ֱ��� һȦ10000����
#define ACCELERATE_ADC_NUM  10  //adcһ�α������� ����ƽ��ֵ�˲�

#define ADC_ZERO_VAL   0.83

#define MAX_VEL_RPM     6000  //���ת��


void Accelerate_ADC_Init(void);

void Accelerate_DMA(void);

//�����Լ��� __packed  ����ADC��ֵ���ݲ���
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
extern u8 manual_motor_dir_flag;//�˹�����ʱ��������Ʊ�־��0������  1������

void manual_hal(void);


void manul_gpio_init(void);






//ACCELERATE_TYPE  acce_data;
#endif
