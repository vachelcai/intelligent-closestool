/*
 * remote_control.c
 *
 *  Created on: 2019年7月24日
 *      Author: caiwe
 */
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stepper.h"
#include "led.h"
#include "eeprom.h"
#include "fmq.h"
#include "eeprom.h"

extern osSemaphoreId RCTigHandle;
extern osMessageQId jkQueHandle;
extern uint8_t remoteCtrlData[10];
extern osMessageQId ctrlQueHandle;
extern osMessageQId sCtrlQueHandle;
extern osMessageQId clearQueHandle;
extern uint32_t * timeStamp;
extern osTimerId sitTimeHandle;
extern osTimerId hotwaterHandle;

extern uint8_t clearState;
extern osTimerId hotwaterHandle;
extern _Bool isBottom;
extern int8_t _closestoolState;
uint8_t remoteCtrlAdd[2];
uint8_t remoteCtrlBackData[32]={0,13,15,16,14,15,12,14,15,12};

uint8_t waterTemperature=30;
uint8_t windWarm=0;	//风温为挡位,非实际温度
uint8_t pjpos=0;	//大概20档
uint8_t FSFpos=0;	//大概10档
uint8_t setTemperature=0;	//挡位,因为感应头在加热丝附近,温度和表面差距太大,夏天冬天一样差距大
_Bool message=0;	//按摩
_Bool startHotWater=0;
bool ledZM=true;
extern _Bool isSit;
//uint8_t Ctrl=0;
//bool Usart2Free;
	//remoteCtrlData format is 0x00 data[1]~data[8] 0x00;
	// and data[8]= data[1]^.....data[7]
	// 地址2位 data[1] data[2]
const uint32_t rtFlashDefualtValue[11] __attribute__((at (PRO_SET_ADD) )) ={
			35,0x03,10,5,2,1,1,0x3534,1};


uint32_t jkListRead=0;
uint32_t jkListWrite=0;

void StartRemoteControl(void const * argument){
	//读取默认值
	
	waterTemperature=(uint8_t)(*(__IO uint32_t *)PRO_SET_waterTemperature);
	windWarm=*(__IO uint32_t *)PRO_SET_windWarm;
	pjpos=(uint8_t)(*(__IO uint32_t *)PRO_SET_pjpos);
	FSFpos=(uint8_t)(*(__IO uint32_t *)PRO_SET_FSFpos);
	setTemperature=(uint8_t)(*(__IO uint32_t *)PRO_SET_setTemperature);
	message=*(__IO uint32_t *)PRO_SET_message==1;
	startHotWater= *(__IO uint32_t *)PRO_SET_startHotWater ==1;
	uint32_t tnds=*(__IO uint32_t *)PRO_SET_RC_ADD;
	remoteCtrlAdd[0]=tnds& 0xff;
	remoteCtrlAdd[1]=(tnds>>8) & 0xff;
	ledZM=*(__IO uint32_t *)PRO_SET_LED_ON ==1;
	if(ledZM) LL_GPIO_SetOutputPin(ZM_GPIO_Port,ZM_Pin);
	//RCTigHandle =osS
	osSemaphoreWait(RCTigHandle,osWaitForever);
	//setup DMA
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_4,(uint32_t) & USART2->TDR);
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_4,(uint32_t) remoteCtrlBackData);
	//LL_GPIO_SetOutputPin(D1_GPIO_Port,D1_Pin);
	//printf("haspe");
	for(;;){
		osSemaphoreWait(RCTigHandle,osWaitForever);
		//LL_GPIO_TogglePin(D1_GPIO_Port,D1_Pin);
			uint8_t indCode=remoteCtrlData[1]^remoteCtrlData[2]^remoteCtrlData[3]^remoteCtrlData[4]^remoteCtrlData[5]^remoteCtrlData[6]^remoteCtrlData[7];
			if(indCode== remoteCtrlData[8]){	//格式正确
				if(remoteCtrlData[1]==remoteCtrlAdd[0] && remoteCtrlData[2]==remoteCtrlAdd[1]){	//地址验证
					//解读代码,2部分,一部分动作,交给通道,一部分设置,立刻赋值给对应部分.
					if(remoteCtrlData[3]==0x00){
						//第三位为操作码.0x00为正常遥控功能
						uint8_t data=remoteCtrlData[4]>>4;
						if(data+30!=waterTemperature){
							waterTemperature=data+30;
							//if(waterTemperature<30) waterTemperature=30;
							if(waterTemperature>40) waterTemperature=40;
						}
						data=remoteCtrlData[4]&0x0f;
						if(data!=windWarm){
							windWarm=data;
						}
						data=remoteCtrlData[5]>>4;
						if(data!=pjpos){
							pjpos=data;
							if(_closestoolState==2 && !message){
								PJTo(PJ_LEVEL(pjpos));
							}
						}
						data=remoteCtrlData[5]&0x0f;
						if(data!=FSFpos){
							FSFpos=data;
							if(_closestoolState ==2){	//正在清洗阶段
								FSFTo(FSF_LEVEL(FSFpos,isBottom));
							}
						}
						data=remoteCtrlData[6]>>4;
						if(data!=setTemperature){	//座温
							setTemperature=data;
							if(data>0){
								ledSetOn(2,1);
								osTimerStart(sitTimeHandle,10);	//10ms 扫描一下温度
							}else{
								ledSetOn(2,0);
								osTimerStop(sitTimeHandle);
							}
						}
						data=(remoteCtrlData[6]&0x08) >0;
						if(data !=message){	//按摩
							message=data;
							if(message && _closestoolState ==2){
								LL_GPIO_SetOutputPin(QB_GPIO_Port,QB_Pin);
							}else{
								LL_GPIO_ResetOutputPin(QB_GPIO_Port,QB_Pin);
							}
						}
						data=(remoteCtrlData[6] &0x04)>0;
						if(data !=startHotWater){
							startHotWater=data;
							if(_closestoolState==2) osTimerStart(hotwaterHandle,3000);
						}
						data=(remoteCtrlData[6] & 0x02)>0;
						if(data !=ledZM){
							ledZM=data;
							if(ledZM) LL_GPIO_SetOutputPin(ZM_GPIO_Port,ZM_Pin);
							else LL_GPIO_ResetOutputPin(ZM_GPIO_Port,ZM_Pin);
						}
						if(remoteCtrlData[7]!=0x00){
							//osEvent oe;
							//uint32_t isCb;
							switch(remoteCtrlData[7]){
								case 0xff:	//停止
									xQueueReset(sCtrlQueHandle);									
									xQueueReset(ctrlQueHandle);
									osMessagePut(sCtrlQueHandle,0xff,0);
									break;
								case 0x45:	//小冲
									if(clearState==0 || clearState ==0x34)
										osMessagePut(clearQueHandle,0x45,0);
									break;
								case 0x56:	//大冲
									if(clearState==56) break;
								xQueueReset(clearQueHandle);
								osMessagePut(clearQueHandle,0x56,0);
								break;
								case 0x32:	//臀洗
									if(!isSit) break;
									isBottom=1;
									osMessagePut(ctrlQueHandle,0x32,osWaitForever);
								osMessagePut(jkQueHandle,3,osWaitForever);
									break;
								case 0x33:	//女洗
									if(!isSit) break;
									isBottom=0;
									osMessagePut(ctrlQueHandle,0x32,osWaitForever);
								osMessagePut(jkQueHandle,2,osWaitForever);
								break;
								
								case 0x39:	//烘干
									if(!isSit) break;
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
									
								break;
								default:
									break;
									//xxx
							}
							
						}
						remoteCtrlBackData[0]=0;
						remoteCtrlBackData[1]=remoteCtrlAdd[0];
						remoteCtrlBackData[2]=remoteCtrlAdd[1];
						remoteCtrlBackData[3]=_closestoolState;
						remoteCtrlBackData[4]=clearState;
						remoteCtrlBackData[5]=0;
						remoteCtrlBackData[6]=0;
						remoteCtrlBackData[7]=0;
						remoteCtrlBackData[8]=remoteCtrlBackData[1]^remoteCtrlBackData[2]^remoteCtrlBackData[3]^remoteCtrlBackData[4];
						remoteCtrlBackData[9]=0;
						//end 遥控收发部分
						LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);
						LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_4,10);
						FMQ();
					}else if(remoteCtrlData[3]==0x01){ //格式为 01 后面4个为int32 为秒数,当机器记一次读到秒数时,自动与系统那边结合
						//begin 健康数据传输部分
						if(*timeStamp ==0){
							*timeStamp+=remoteCtrlData[4]+(remoteCtrlData[5]<<8) +(remoteCtrlData[6]<<16)+(remoteCtrlData[7]<<24);
							*timeStamp-=osKernelSysTick()/1000;
						}
						if(jkListRead!=jkListWrite){
							uint32_t tbegAdd=JK_LIST_BEGIN_ADD+jkListRead*32;
							for(int i=0;i<32;i++){
								remoteCtrlBackData[i]=*((uint8_t *) (tbegAdd+i));
							}
							if(++jkListRead==JK_MAX_NUM) jkListRead=0;
						}else{
							for(int i=0;i<32;i++){
								remoteCtrlBackData[i]=0;
							}
						}
						LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);
						LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_4,32);
						//end 健康数据传输部分
					}
					
				}else if(remoteCtrlData[1]==0x56 && remoteCtrlData[2]==0x38) {
					//配对地址 暂定 0x56 0x38
				}
				
				//返回数据
				
				
			}else{
				remoteCtrlBackData[0]=0x00;
				remoteCtrlBackData[1]=0x56;
				remoteCtrlBackData[2]=0x56;
				remoteCtrlBackData[3]=0x00;
						LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);
				LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_4,4);
			}
				LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);
				DMA1->IFCR = LL_DMA_IFCR_CGIF4 | LL_DMA_IFCR_CTCIF4 | LL_DMA_IFCR_CHTIF4 | LL_DMA_IFCR_CTEIF4; /* Clear previous flags */
				LL_USART_ClearFlag_TC(USART2);
				LL_USART_EnableIT_TC(USART2);
				LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_4);
				LL_USART_EnableDMAReq_TX(USART2);

	}
	//vTaskDelete(NULL);
}
