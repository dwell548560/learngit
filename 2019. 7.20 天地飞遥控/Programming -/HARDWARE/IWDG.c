#include "IWDG.h"

void IWDG_Init(u8 prer,u16 rlr)//初始化看门狗
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //????????
	IWDG_SetPrescaler(prer); //?? IWDG ????
	IWDG_SetReload(rlr); //?? IWDG ???
	IWDG_ReloadCounter(); //reload
	IWDG_Enable(); //?????
}





void IWDG_Feed(void)//喂狗
{

	IWDG_ReloadCounter();//reload

}

