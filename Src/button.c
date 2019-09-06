#include "button.h"
#include "main.h"
#include "cmsis_os.h"

typedef struct _BUTTON_STRUCT{
	_Bool on:1;
	uint8_t lastState:7;	//��Ϊ����Ϊ15ms��ƽ��������ȶ�������,��ΪҪ�漰����������
} buttonType;

uint16_t _buttonAj2Chang=0;

buttonType buttons[4]={
	{0,0},{0,0},{0,0},{0,0}
};

//1msɨ��һ��
void buttonLoop(void const * argument){
	//aj1 ����1,�ᰴ,û�г�������
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[0].on){
		buttons[0].lastState=0;
	}else{
		buttons[0].lastState++;
		if(buttons[0].lastState>15){
			buttons[0].on=!buttons[0].on;
			if(buttons[0].on){
				//aj1 �¼�
				
				//end
			}
		}
	}
	//aj2 �г�������,�����ƽ��ͬת��ʱ,�������ʱ��С��15tick����,�������ִ���밴����,����Ѿ�
	if(LL_GPIO_IsInputPinSet(AJ_2_GPIO_Port,AJ_2_Pin)==buttons[1].on){
		buttons[1].lastState=0;
		if(buttons[1].on && _buttonAj2Chang <10000) {
			_buttonAj2Chang++;
			if(_buttonAj2Chang>5000){	//�����¼�
				
			}
		}
	}else{
		buttons[1].lastState++;
		if(buttons[1].lastState>15){
			buttons[1].on=!buttons[1].on;
			_buttonAj2Chang=0;
			if(!buttons[1].on){
				//aj2 �ᰴ�¼�
				
				//end
			}
		}
	}
	
	//aj1 ����3,�ᰴ,û�г�������
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[2].on){
		buttons[2].lastState=0;
	}else{
		buttons[2].lastState++;
		if(buttons[2].lastState>15){
			buttons[2].on=!buttons[2].on;
			if(buttons[2].on){
				//aj1 �¼�
				
				//end
			}
		}
	}

	//aj1 ����4,�ᰴ,û�г�������
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[0].on){
		buttons[3].lastState=0;
	}else{
		buttons[3].lastState++;
		if(buttons[3].lastState>15){
			buttons[3].on=!buttons[3].on;
			if(buttons[3].on){
				//aj1 �¼�
				
				//end
			}
		}
	}
	
	
}
