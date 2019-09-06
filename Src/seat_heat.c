#include "seat_heat.h"

#include "cmsis_os.h"

extern osMessageQId sCtrlQueHandle;
extern osMessageQId jkQueHandle;

extern uint8_t setTemperature;
extern ADC_HandleTypeDef hadc;
_Bool isSit=0;

int16_t tempList[][2]={
	{-10,3353},
	{-5,3197},
	{0,3024},
	{5,2839},
	{10,2645},
	{15,2446},
	{20,2245},
	{25,2048},
	{30,1857},
	{35,1675},
	{40,1506},
	{45,1347},
	{50,1203},
	{55,1072},
	{60,955},
	{65,830},
	{70,755},
	{75,672},
	{80,598}
};

uint32_t sitData[12];
uint8_t _sitDataPos=0;
uint32_t tmin=0,tmax=0,tsum=0;
void sitLoop(void const * argument){
	HAL_ADC_Start(&hadc);
	if(HAL_ADC_PollForConversion(&hadc,400)!= HAL_TIMEOUT){
			uint32_t _tempSitData=HAL_ADC_GetValue(&hadc);
		 if(_sitDataPos==0){
				tmin=_tempSitData;
				tmax=_tempSitData;
				tsum=_tempSitData;
			 _sitDataPos=1;
		 }else if(_sitDataPos>11){
			_sitDataPos=0;
			tsum=tsum-tmin-tmax;
			tsum/=10;
			if(tsum>3500 || tsum<550){
				//����,û������,������������,��Ӧͷ��Χ��ƫ
				FMQ();
			}
			//���бȽ϶Ա�
			switch(setTemperature){
				case 0:
					LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
				break;
				case 1:	//�ڲ�15~25��
					if(tsum>2446) LL_GPIO_ResetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);	//����15
					if(tsum<2048)	LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);		//����20
				break;	
				case 2:	//25~33
					if(tsum>2048) LL_GPIO_ResetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					if(tsum<1740) LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					break;
				case 3:	//33~40
					if(tsum>1740) LL_GPIO_ResetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					if(tsum<1506) LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					break;
				case 4:	//40~45
					if(tsum>1506) LL_GPIO_ResetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					if(tsum<1347) LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					break;
				default:
					LL_GPIO_SetOutputPin(ZQ_JRS_GPIO_Port,ZQ_JRS_Pin);
					break;
					
			}			 
		 }else{
			 if(_tempSitData>tmax) tmax=_tempSitData;
			 else if(_tempSitData<tmin) tmin=_tempSitData;
			 tsum+=_tempSitData;
			 _sitDataPos++;
		 }

	}else{
		HAL_ADC_Stop(&hadc);
	}
}

uint16_t noSitCount=0;
void startIsSit(const void * arg){
	//���崫���� ����Ϊ�͵�ƽ
	if(!LL_GPIO_IsInputPinSet(RZCGQ_GPIO_Port,RZCGQ_Pin)){
		if(!isSit){
			isSit=true;
			LL_GPIO_SetOutputPin(CC_GPIO_Port,CC_Pin);
			//��������
			osMessagePut(jkQueHandle,0,osWaitForever);
			//end ��������
			FMQ();
		}else{
			noSitCount=0;
		}
	}else{
		//��������,��ʱ3s
		if(isSit){
			if(noSitCount>3000){
				isSit=false;
				FMQ();
				LL_GPIO_ResetOutputPin(CC_GPIO_Port,CC_Pin);
				//TODO: ���߶���
				xQueueReset(sCtrlQueHandle);
				osMessagePut(sCtrlQueHandle,0xff,0);
				osMessagePut(jkQueHandle,0xff,osWaitForever);
				//endTODO
			}else{
				
				noSitCount++;
			}
		}
	}
}
