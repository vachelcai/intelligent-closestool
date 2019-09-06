#include "fmq.h"

#include "stm32f0xx_ll_tim.h"

bool _fmqBusy=false;

bool FMQ(void){
	if(_fmqBusy) return false;
	_fmqBusy=true;
	LL_GPIO_SetOutputPin(FMQ_GPIO_Port,FMQ_Pin);
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);
	return true;
}

