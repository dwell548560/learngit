#ifndef _hard_stop_h
#define _hard_stop_h
#include "sys.h"

#define GPIO_HARD_STOP 		   PCin(11)
#define HARD_STOP_TRI         0  //按下触发
#define HARD_STOP_UNTRI       1

#define HARD_STOP_PUSH        1     //按下
#define HARD_STOP_UNPUSH      2     //弹起


extern u8 hard_stop_state;

void hard_stop_init(void);



#endif

