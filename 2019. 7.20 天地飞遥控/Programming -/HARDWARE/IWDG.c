#include "IWDG.h"

void IWDG_Init(u8 prer,u16 rlr)//��ʼ�����Ź�
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //????????
	IWDG_SetPrescaler(prer); //?? IWDG ????
	IWDG_SetReload(rlr); //?? IWDG ???
	IWDG_ReloadCounter(); //reload
	IWDG_Enable(); //?????
}





void IWDG_Feed(void)//ι��
{

	IWDG_ReloadCounter();//reload

}

