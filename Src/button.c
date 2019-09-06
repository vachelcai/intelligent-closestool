#include "button.h"
#include "main.h"
#include "cmsis_os.h"

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
				
			}
		}
	}else{
		buttons[1].lastState++;
		if(buttons[1].lastState>15){
			buttons[1].on=!buttons[1].on;
			_buttonAj2Chang=0;
			if(!buttons[1].on){
				//aj2 轻按事件
				
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
				
				//end
			}
		}
	}
	
	
}
