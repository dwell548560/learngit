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
#define SBUS_DATA_CH   16  //16个通道
#define SBUS_CHECK_SUCCESS_FLAG  1
#define SBUS_CHECK_RESET         0

#define RX_SBUS_FINISH_FLAG      1
#define RX_SBUS_FINISH_RESET     0

#define CH3_THORRT 1
#define CH4_STEER  2

#define KINCO_RATIO    10000  //所使用的步科驱动器的编码器为10000细分

#define MAX_ANGLE_PULSE   61111     //扫地车前轮单方向最大55度，加上减速器，一圈脉冲400000，所以55度的脉冲为此公式(400000/360) * 55  


//天地飞WFT09遥控器 2.4G 4096模式下
#define DEAD_ZONE   5  //摇杆无动作区域半径

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

u8 Wfly_SBUS_Check(void);//校验数据

void Wfly_Data_Channel( const u8 *sBusData);//数据转成通道，现仅用3通油门，4通道舵机

void Get_Ch_Hal(void);

void SBUS_reopen_rx(void);

void SBUS_Motor_Hal(u16 thorrt_ch3, u16 sheer_ch4);//遥控器执行对电机的操作

int SBUS_Swap_Motor(u16 swap_data, u8 data_class);//将遥控器过的数据进行转换，得到可以对电机进行控制的实际量

__packed typedef struct Wfly_Type{


u8 rx_sbus_buf[MAX_RX_SBUS];

u8 check_success_flag;

u8 avail_buf[SBUS_DATA_LEN];

u16 ch_data_buf[SBUS_DATA_CH];

u8 rx_DMA_finish_flag;

u8 rx_DMA_Len;
	
u8 open_wfly_reset_flag;//打开遥控复位允许标志

int64_t motor_rate;
int64_t steer_angle_pulse;

}wfly_para;

extern wfly_para my_wfly;


#endif