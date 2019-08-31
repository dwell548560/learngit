#ifndef _WFLY_H
#define _WFLY_H

#include "sys.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "manul_hal.h"
#include "reset_motor.h"


#define OPEN_RESET_WBUS          1

#define MAX_RX_SBUS   50
#define SBUS_DATA_LEN  25
#define SBUS_DATA_CH   16  //16��ͨ��
#define SBUS_CHECK_SUCCESS_FLAG  1
#define SBUS_CHECK_RESET         0

#define RX_SBUS_FINISH_FLAG      1
#define RX_SBUS_FINISH_RESET     0

#define CH3_THORRT 1
#define CH4_STEER  2

#define KINCO_RATIO    10000  //��ʹ�õĲ����������ı�����Ϊ10000ϸ��

#define MAX_ANGLE_PULSE   61111     //ɨ�س�ǰ�ֵ��������55�ȣ����ϼ�������һȦ����400000������55�ȵ�����Ϊ�˹�ʽ(400000/360) * 55  


//��ط�WFT09ң���� 2.4G 4096ģʽ��
#define DEAD_ZONE   5  //ҡ���޶�������뾶

#define MAX_THORRT_SBUS  1706  
#define MEDIUM_THORRT_SBUS 1024
#define MIN_THORRT_SBUS  333

#define MAX_STEER_SBUS  1681  
#define MEDIUM_STEER_SBUS 999
#define MIN_STEER_SBUS  316


#define MAX_MOTOT_DRIVER_RPM 2000


#define AUTO_STATE    1
#define MANUL_STATE   2


void Wfly_Usart_Init(void);

void Wfly_DMA_Init(void);

u8 Wfly_SBUS_Check(void);//У������

void Wfly_Data_Channel( const u8 *sBusData);//����ת��ͨ�����ֽ���3ͨ���ţ�4ͨ�����

void Get_Ch_Hal(void);

void SBUS_reopen_rx(void);

void SBUS_Motor_Hal(u16 thorrt_ch3, u16 sheer_ch4);//ң����ִ�жԵ���Ĳ���

int SBUS_Swap_Motor(u16 swap_data, u8 data_class);//��ң�����������ݽ���ת�����õ����ԶԵ�����п��Ƶ�ʵ����

__packed typedef struct Wfly_Type{


u8 rx_sbus_buf[MAX_RX_SBUS];

u8 check_success_flag;

u8 avail_buf[SBUS_DATA_LEN];

u16 ch_data_buf[SBUS_DATA_CH];

u8 rx_DMA_finish_flag;

u8 rx_DMA_Len;
	
u8 open_wfly_reset_flag;//��ң�ظ�λ�����־

int64_t motor_rate;
int64_t steer_angle_pulse;

}wfly_para;

extern wfly_para my_wfly;


#endif