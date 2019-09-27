#include "button.h"
#include "main.h"
#include "cmsis_os.h"
#include "fmq.h"

extern uint8_t clearState;
extern osTimerId hotwaterHandle;
extern _Bool isBottom;
extern int8_t _closestoolState;
extern _Bool isSit;
extern osMessageQId ctrlQueHandle;
extern osMessageQId sCtrlQueHandle;
extern osMessageQId clearQueHandle;
extern osMessageQId jkQueHandle;
extern _Bool message;

typedef struct _BUTTON_STRUCT{
	_Bool on:1;
	uint8_t lastState:7;	//因为设置为15ms电平不变就算稳定下来了,因为要涉及到长按功能
} buttonType;

uint16_t _buttonAj2Chang=0;

buttonType buttons[4]={
	{0,0},{0,0},{0,0},{0,0}
};

//1ms扫描一次
void buttonLoop(void const * argument){
	//aj1 按键1,轻按,没有长按功能
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[0].on){
		buttons[0].lastState=0;
	}else{
		buttons[0].lastState++;
		if(buttons[0].lastState>15){
			buttons[0].on=!buttons[0].on;
			if(buttons[0].on){
				//aj1 事件
				if(isSit){
					if(_closestoolState ==2 && isBottom==1){
						isBottom=!isBottom;
						if(message && _closestoolState ==2){
								LL_GPIO_SetOutputPin(QB_GPIO_Port,QB_Pin);
							}else{
								LL_GPIO_ResetOutputPin(QB_GPIO_Port,QB_Pin);
							}
					}else{
						isBottom=1;
						osMessagePut(ctrlQueHandle,0x32,osWaitForever);
						osMessagePut(jkQueHandle,3,osWaitForever);
					}
				}
				FMQ();
				//end
			}
		}
	}
	//aj2 有长按功能,如果电平相同转换时,如果持续时间小于15tick放弃,如果大于执行请按功能,如果已经
	if(LL_GPIO_IsInputPinSet(AJ_2_GPIO_Port,AJ_2_Pin)==buttons[1].on){
		buttons[1].lastState=0;
		if(buttons[1].on && _buttonAj2Chang <10000) {
			_buttonAj2Chang++;
			if(_buttonAj2Chang>5000){	//长按事件
				if(clearState!=56){
				xQueueReset(clearQueHandle);
				osMessagePut(clearQueHandle,0x56,0);
				}
			}
		}
	}else{
		buttons[1].lastState++;
		if(buttons[1].lastState>15){
			buttons[1].on=!buttons[1].on;
			_buttonAj2Chang=0;
			if(!buttons[1].on){
				//aj2 轻按事件
				xQueueReset(sCtrlQueHandle);									
				xQueueReset(ctrlQueHandle);
				osMessagePut(sCtrlQueHandle,0xff,0);
				//end
			}
		}
	}
	
	//aj1 按键3,轻按,没有长按功能
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[2].on){
		buttons[2].lastState=0;
	}else{
		buttons[2].lastState++;
		if(buttons[2].lastState>15){
			buttons[2].on=!buttons[2].on;
			if(buttons[2].on){
				//aj1 事件
				if(isSit){
					if(_closestoolState ==2 && isBottom==0){
						isBottom=!isBottom;
						if(message && _closestoolState ==2){
								LL_GPIO_SetOutputPin(QB_GPIO_Port,QB_Pin);
							}else{
								LL_GPIO_ResetOutputPin(QB_GPIO_Port,QB_Pin);
							}
					}else{
						isBottom=1;
						osMessagePut(ctrlQueHandle,0x32,osWaitForever);
						osMessagePut(jkQueHandle,3,osWaitForever);
					}
				}
				FMQ();
				
				//end
			}
		}
	}

	//aj1 按键4,轻按,没有长按功能
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[0].on){
		buttons[3].lastState=0;
	}else{
		buttons[3].lastState++;
		if(buttons[3].lastState>15){
			buttons[3].on=!buttons[3].on;
			if(buttons[3].on){
				//aj1 事件
								if(isSit){
				if(_closestoolState==0)
				osMessagePut(ctrlQueHandle,0x76,osWaitForever);
				else if(_closestoolState==1 || _closestoolState ==2 || _closestoolState==3){
					xQueueReset(sCtrlQueHandle);
					osMessagePut(sCtrlQueHandle,0xff,0);
					osMessagePut(ctrlQueHandle,0x76,0);
					osMessagePut(jkQueHandle,4,osWaitForever);
				}else if(_closestoolState==4){
					osMessagePut(sCtrlQueHandle,0xff,0);
				}
			}
			FMQ();
			
			//end
			}
		}
	}
	
	
}
