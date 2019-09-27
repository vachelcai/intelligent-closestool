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
	//aj2 �г�������,�����ƽ��ͬת��ʱ,�������ʱ��С��15tick����,�������ִ���밴����,����Ѿ�
	if(LL_GPIO_IsInputPinSet(AJ_2_GPIO_Port,AJ_2_Pin)==buttons[1].on){
		buttons[1].lastState=0;
		if(buttons[1].on && _buttonAj2Chang <10000) {
			_buttonAj2Chang++;
			if(_buttonAj2Chang>5000){	//�����¼�
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
				//aj2 �ᰴ�¼�
				xQueueReset(sCtrlQueHandle);									
				xQueueReset(ctrlQueHandle);
				osMessagePut(sCtrlQueHandle,0xff,0);
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

	//aj1 ����4,�ᰴ,û�г�������
	if(LL_GPIO_IsInputPinSet(AJ_1_GPIO_Port,AJ_1_Pin)==buttons[0].on){
		buttons[3].lastState=0;
	}else{
		buttons[3].lastState++;
		if(buttons[3].lastState>15){
			buttons[3].on=!buttons[3].on;
			if(buttons[3].on){
				//aj1 �¼�
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
