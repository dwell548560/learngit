#ifndef _hard_stop_h
#define _hard_stop_h
#include "sys.h"

#define GPIO_HARD_STOP 		   PCin(11)
#define HARD_STOP_TRI         0  //���´���
#define HARD_STOP_UNTRI       1

#define HARD_STOP_PUSH        1     //����
#define HARD_STOP_UNPUSH      2     //����


extern u8 hard_stop_state;

void hard_stop_init(void);



#endif

