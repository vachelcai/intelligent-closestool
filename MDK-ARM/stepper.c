#include "stepper.h"

#include "user_config.h"
#include "stm32f0xx_ll_tim.h"
//#include "fmq.h"
/*
	暂定 pj实用tim6,FSF用tim14
*/

//暂定无负数pos

extern osMessageQId sCtrlQueHandle;

stepperStr _pj;
stepperStr _fsf;

uint16_t PJPos(void){
	return _pj.pos;
}

void PJStop(){
	LL_GPIO_ResetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
	LL_GPIO_ResetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
	LL_GPIO_ResetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
	LL_GPIO_ResetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
}

void FSFStop(){
	LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
	LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
	LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
	LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
}

bool PJStep(){
	_pj.clockwise?_pj.pos++:_pj.pos--;
	//每一步的位置.4步八步,双状态 等等
	if(_pj.clockwise){
		switch(_pj.pos &0x03){
			case 0:	//4,1	↓
				LL_GPIO_ResetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
				LL_GPIO_SetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
				break;
			case 1:	//3,4 ↓
				LL_GPIO_ResetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
				LL_GPIO_SetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
				break;
			case 2:	//2,3	↓
				LL_GPIO_ResetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
				LL_GPIO_SetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
				break;
			case 3:	//1,2 ↓
				LL_GPIO_ResetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
				LL_GPIO_SetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
				break;
		}
	}else{
		switch(_pj.pos &0x03){
			case 0:	//4,1 ↑
				LL_GPIO_ResetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
				LL_GPIO_SetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
			break;
			case 1:	//3,4	↑
				LL_GPIO_ResetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
				LL_GPIO_SetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
			break;
			case 2:	//2,3	↑
				LL_GPIO_ResetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
				LL_GPIO_SetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
			break;
			case 3:	//1,2 ↑
				LL_GPIO_ResetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
				LL_GPIO_SetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
			break;
		}
	}
	
	if(_pj.pos==_pj.to){
		_pj.busy=pdFAIL;
		osMessagePut(sCtrlQueHandle,0x02,00);
		return pdTRUE;
	}
	return pdFAIL;
}

void pjStart(){
	switch(_pj.pos & 0x03){
		case 0:
			LL_GPIO_SetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
		LL_GPIO_SetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
		break;
		case 1:
			LL_GPIO_SetOutputPin(BJJ_4_GPIO_Port,BJJ_4_Pin);
		LL_GPIO_SetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
		break;
		case 2:
			LL_GPIO_SetOutputPin(BJJ_3_GPIO_Port,BJJ_3_Pin);
		LL_GPIO_SetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
		break;
		case 3:
			LL_GPIO_SetOutputPin(BJJ_2_GPIO_Port,BJJ_2_Pin);
		LL_GPIO_SetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
		break;
	}
}

void fsfStart(){
	switch(_fsf.pos & 0x03){
		case 0:	//5,6
				LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 1:	//6,7
				LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 2:	//7,8
				LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
			case 3:	//8,5
				LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
	}
}

bool FSFStep(){
	_fsf.clockwise? _fsf.pos++: _fsf.pos--;
	//每一步的位置.4步八步,双状态 等等
	if(_fsf.clockwise){
		switch(_fsf.pos &0x03){
			case 0:	//5,6
				LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 1:	//6,7
				LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 2:	//7,8
				LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
			case 3:	//8,5
				LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
		}
		/*
		switch(_fsf.pos & 0x07){
			case 0:	//5
				LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
				//LL_GPIO_SetOutputPin(BJJ_1_GPIO_Port,BJJ_1_Pin);
			break;
			case 1: //5,6
				LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 2:	//6
				LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
			case 3:	//6,7
				LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 4:	//7
				LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 5: //7,8
				LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
			case 6: //8
				LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 7:	//8,5
				LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;			
		}
		*/
	}else{
		switch(_fsf.pos &0x03){
			case 0:	//5,6
				LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
			case 1:	//6,7
				LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 2:	//7,8
				LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 3:	//8,5
				LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
				
		}
		/*
		switch(_pj.pos &0x07){	//7->6->5.....->1->7
			case 0:	//5
				LL_GPIO_ResetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
				break;
			case 1:	//5,6
				LL_GPIO_SetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
			case 2:	//6
				LL_GPIO_ResetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 3:	//6,7
				LL_GPIO_SetOutputPin(BJJ_6_GPIO_Port,BJJ_6_Pin);
			break;
			case 4:	//7
				LL_GPIO_ResetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
			case 5:	//7,8
				LL_GPIO_SetOutputPin(BJJ_7_GPIO_Port,BJJ_7_Pin);
			break;
			case 6:	//8
				LL_GPIO_ResetOutputPin(BJJ_5_GPIO_Port,BJJ_5_Pin);
			break;
			case 7:	//8,5
				LL_GPIO_SetOutputPin(BJJ_8_GPIO_Port,BJJ_8_Pin);
			break;
		}
		*/
	}
	
	//
	if(_fsf.pos==_fsf.to){
		_fsf.busy=pdFAIL;
		osMessagePut(sCtrlQueHandle,0x03,0);
		return pdTRUE;
	}
	return pdFAIL;
}

void PJTo(uint16_t pos){
	if(!_pj.busy && ( pos+PJ_JXF== _pj.pos )){
		_pj.to=_pj.pos;
		_pj.busy=false;
		//停了计时器
		//LL_TIM_DisableIT_UPDATE(TIM6);
		LL_TIM_DisableCounter(TIM6);
		return;
	}
	_pj.clockwise=pos>_pj.pos;
	if(_pj.clockwise)
		_pj.to=pos+PJ_JXF;
	else _pj.to=pos;
	
	_pj.busy=true;
	LL_TIM_EnableIT_UPDATE(TIM6);
	LL_TIM_EnableCounter(TIM6);
}

void FSFTo(uint16_t pos){
	if(pos==_fsf.pos){
		_fsf.to=UINT16_MAX;
		_fsf.busy=pdFAIL;
		LL_TIM_DisableCounter(TIM14);
	}
	_fsf.to=pos;
	_fsf.busy=pdTRUE;
	_fsf.clockwise=pos>_fsf.pos;
	LL_TIM_EnableIT_UPDATE(TIM14);
	LL_TIM_EnableCounter(TIM14);
}

void stepperInit(){
	_pj.busy=pdFAIL;
	_pj.pos=PJ_STEP_MAX;
	fsfStart();
	FSFTo(FSF_STEP_MAX);
	pjStart();
	PJTo(0);
	osEvent oe;
	
		do{
			oe=osMessageGet(sCtrlQueHandle,osWaitForever);
			if(oe.status==osEventMessage && oe.value.v==0x03)
				break;
		}while(1);
		FSFTo(FSF_STEP_MAX-1000);
		do{
			oe=osMessageGet(sCtrlQueHandle,osWaitForever);
			if(oe.status==osEventMessage && oe.value.v==0x03)
				break;
			if(oe.status==osEventMessage && oe.value.v==0x02){
				PJStop();
			}
		}while(1);
		FSFTo(FSF_STEP_MAX-1000+100);
		do{
			oe=osMessageGet(sCtrlQueHandle,osWaitForever);
			if(oe.status==osEventMessage && oe.value.v==0x03)
				break;
		}while(1);
		FSFStop();
}
