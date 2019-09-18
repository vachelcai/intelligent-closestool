#include "closestool.h"
#include "user_config.h"
#include "cmsis_os.h"
#include "stepper.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_tim.h"
#include "fmq.h"

#define DCF_ON()	LL_GPIO_SetOutputPin(DCF_GPIO_Port,DCF_Pin)
#define DCF_OFF()	LL_GPIO_ResetOutputPin(DCF_GPIO_Port,DCF_Pin)
#define QB_ON()		LL_GPIO_SetOutputPin(QB_GPIO_Port,QB_Pin)
#define QB_OFF()	LL_GPIO_ResetOutputPin(QB_GPIO_Port,QB_Pin)
#define HG_JRS_OFF()	LL_GPIO_SetOutputPin(HG_JRS_GPIO_Port,HG_JRS_Pin)
#define HG_JRS_ON()		LL_GPIO_ResetOutputPin(HG_JRS_GPIO_Port,HG_JRS_Pin)
#define CLOSE_HW()	(closeHotWater=true)
extern osMessageQId sCtrlQueHandle;
extern osMessageQId ctrlQueHandle;
extern osMessageQId clearQueHandle;
extern uint8_t waterTemperature;
extern uint8_t windWarm;	//����Ϊ��λ,��ʵ���¶�
extern uint8_t pjpos;	//���20��
extern uint8_t FSFpos;	//���10��
extern uint8_t setTemperature;	//��λ,��Ϊ��Ӧͷ�ڼ���˿����,�¶Ⱥͱ�����̫��,���춬��һ������
extern _Bool message;	//��Ħ
extern _Bool startHotWater;
extern osTimerId hotwaterHandle;
_Bool isBottom=0;
//_Bool isHGHot=0;
_Bool hasReceive=0;
uint8_t EIWHReceiveData[8];
uint8_t receiveWrongTime=0;


uint8_t EIWHsendData[8]={0x33,0x01,0x1e,0x00,0x00,0x00,0x00,0x1f};
_Bool closeHotWater=0;
uint8_t closeHotWaterCount=0;
void hotWaterLoop(void const * argument){
	if(hasReceive){
		if(closeHotWater && (++closeHotWaterCount>4)){
			osTimerStop(hotwaterHandle);
		}
		if(01 +	EIWHReceiveData[2]+EIWHReceiveData[3]+EIWHReceiveData[4]+EIWHReceiveData[5]+EIWHReceiveData[6] ==EIWHReceiveData[7]){
			receiveWrongTime=0;
		}else{
			receiveWrongTime++;
			if(receiveWrongTime>5){
				//���ճ���
			}
		}
	}
	if(closeHotWater){
		EIWHsendData[2]=0x00;
		EIWHsendData[7]=1;
	}else{
		EIWHsendData[2]=waterTemperature;
		EIWHsendData[7]=1+waterTemperature;
	}
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_2,8);
	DMA1->IFCR = LL_DMA_IFCR_CGIF2 | LL_DMA_IFCR_CTCIF2 | LL_DMA_IFCR_CHTIF2 | LL_DMA_IFCR_CTEIF2;
	LL_USART_ClearFlag_TC(USART1);
	LL_USART_EnableIT_TC(USART1);
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_2);
	LL_USART_EnableDMAReq_TX(USART1);
}

//��Ͱ״̬,8bit 
//��ϴ  	Ůϴ		���
// ��Ħ		
int8_t _closestoolState=0;

int8_t ClosestoolGetState(){
	return _closestoolState;
}
uint32_t endTick=0,nowTick=0;
void Startclosestool(void const * argument){
	osEvent toe;
	
	_Bool ish=0;
	//��ʼ������ʽ
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);
	//DMA1->IFCR = LL_DMA_IFCR_CGIF2 | LL_DMA_IFCR_CTCIF2 | LL_DMA_IFCR_CHTIF2 | LL_DMA_IFCR_CTEIF2; /* Clear previous flags */
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_2,(uint32_t)&USART1->TDR);
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_2,(uint32_t) EIWHsendData);
	//LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_2,8);
	FMQ();
	stepperInit();
	osMessagePut(clearQueHandle,0xff,0);
	FMQ();
	for(;;){
		osEvent oe= osMessageGet(ctrlQueHandle,osWaitForever);
		if(oe.status==osEventMessage){
			switch(oe.value.v){
				case 0x32:
					_closestoolState=1;
					DCF_ON();
				if(startHotWater)	osTimerStart(hotwaterHandle,300);
				nowTick=osKernelSysTick();
				endTick=nowTick+2000;
					
					while(nowTick<endTick || (endTick<2000 && nowTick> UINT32_MAX -2000 )){
						toe = osMessageGet(sCtrlQueHandle,endTick-nowTick);
						nowTick=osKernelSysTick();
						if(toe.value.v	==0xff) goto end;
					}
					pjStart();
					PJTo(PJ_LEVEL(pjpos));
					do{
					toe= osMessageGet(sCtrlQueHandle,osWaitForever);
					if(toe.value.v ==0xff) goto end;
					}while(toe.value.v!=0x02);	// sCtrlQueHandle �յ� pj ���������ź�
					fsfStart();
					FSFTo(FSF_LEVEL(FSFpos,isBottom));
					do{
						toe= osMessageGet(sCtrlQueHandle,osWaitForever);
						if(toe.value.v ==0xff) goto end;
					}while(toe.value.v!=0x03);	// sCtrlQueHandle �յ� FSF ���������ź�
					//������ϴ����
					_closestoolState=2;
					if(message)	{
						LL_GPIO_SetOutputPin(QB_GPIO_Port,QB_Pin);
						PJTo(PJ_LEVEL(pjpos)+PJ_MESSAGE);
					}
					nowTick=osKernelSysTick();
					endTick=nowTick+60*1000;
					
					while(nowTick<endTick || (endTick<60*1000 && nowTick> UINT32_MAX -60*1000 )){
						toe = osMessageGet(sCtrlQueHandle,endTick-nowTick);
						if(toe.status==osEventTimeout) break; 
						switch(toe.value.v){
							case 0xff:
								goto end;
							//break;
							case 0x02:	//pj������
								if(message){
									if(PJPos()<PJ_LEVEL(pjpos)) PJTo(PJ_LEVEL(pjpos)+PJ_MESSAGE);
									else PJTo(PJ_LEVEL(pjpos)-PJ_MESSAGE);
								}
								break;
							default:
								break;
						}
						nowTick=osKernelSysTick();
						//if(toe.value.v	==0xff) goto end;
					}
					
					//��������
					//����ˮ,�ذ�Ħ
					end:
					LL_GPIO_ResetOutputPin(QB_GPIO_Port,QB_Pin);
					_closestoolState=3;
					CLOSE_HW();
					FSFTo(FSF_ZEROP_POS);
					do{
						toe=osMessageGet(sCtrlQueHandle,osWaitForever);
					}while(toe.value.v!=0x03);
					FSFStop();
					osTimerStop(hotwaterHandle);
					PJTo(0);
					do{
						toe=osMessageGet(sCtrlQueHandle,osWaitForever);
					}while(toe.value.v!=0x02);
					PJStop();
					osDelay(3000);	//��ϴ��ͷ3s
					DCF_OFF();
					_closestoolState=0;
					break;
				case 0x76:// ���
					LL_GPIO_SetOutputPin(HG_FAN_GPIO_Port,HG_FAN_Pin);
						_closestoolState=4;
				
					nowTick=osKernelSysTick();
					endTick=nowTick+90*1000;
				
					LL_TIM_EnableIT_UPDATE(TIM16);
					LL_TIM_EnableCounter(TIM16);
						
						while(nowTick<endTick || (endTick<90*1000 && nowTick> UINT32_MAX -90*1000 )){
							toe = osMessageGet(sCtrlQueHandle,endTick-nowTick);
							if(toe.status==osEventTimeout) break;
							nowTick=osKernelSysTick();
							if(toe.value.v	==0xff) break;
							if(toe.value.v ==0x66){
								//TODO:��HG_JRS_pinΪPWM���
								if(ish){
									if(windWarm!=0)
									HG_JRS_ON();
									//LL_TIM_SetCounter(TIM16,2000*windWarm);
									LL_TIM_SetAutoReload(TIM16,2000*windWarm);
								}else{
									if(windWarm!=4)
									HG_JRS_OFF();
									//LL_TIM_SetCounter(TIM16,2000*(4-windWarm));
									LL_TIM_SetAutoReload(TIM16,2000*(4-windWarm));
								}
								ish=!ish;
							}
						}
					
					LL_GPIO_ResetOutputPin(HG_FAN_GPIO_Port,HG_FAN_Pin);
						HG_JRS_OFF();
						LL_TIM_DisableCounter(TIM16);
						_closestoolState=0;
					break;
			}
		}
	}
}
