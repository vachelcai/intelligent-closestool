#include "led.h"

#include "main.h"
#include "cmsis_os.h"

//针对u14接口,采用灌电流从3.3v pin输出负时灯亮

typedef struct _LED{
	_Bool on:1;
	_Bool blink:1;
} LEDS;

//_Bool LedBlink=pdTRUE;

LEDS Leds[]={
	{0,0},{0,0},{0,0}
};


uint32_t ledBlinkParMs =500;
uint32_t _ledBlinkLastDo=0;

void ledSetOn(uint8_t d,_Bool setOn){
	if(d>2) return;
	if(Leds[d].on == setOn) return;
	Leds[d].on=setOn;
	if(Leds[d].blink) return;
	switch(d){
		case 0:
			if(setOn) LL_GPIO_SetOutputPin(D1_GPIO_Port,D1_Pin);
			else LL_GPIO_ResetOutputPin(D1_GPIO_Port,D1_Pin);
		break;
			case 1:
				if(setOn) LL_GPIO_SetOutputPin(D2_GPIO_Port,D2_Pin);
			else LL_GPIO_ResetOutputPin(D2_GPIO_Port,D2_Pin);
			break;
			case 2:
				if(setOn) LL_GPIO_SetOutputPin(D3_GPIO_Port,D3_Pin);
					else LL_GPIO_ResetOutputPin(D3_GPIO_Port,D3_Pin);
			break;		
	}
}

void ledSetBlink(uint8_t d,_Bool setBlink){
	if(d>2) return;
	Leds[d].blink=setBlink;
	if(!setBlink){
		switch(d){
			case 0:
				if(Leds[0].on) 
					LL_GPIO_SetOutputPin(D1_GPIO_Port,D1_Pin);
				else LL_GPIO_ResetOutputPin(D1_GPIO_Port,D1_Pin);
				//LL_GPIO_SetPinMode(D1_GPIO_Port,D1_Pin,LL_gpio_
			break;
			case 1:
				if(Leds[1].on) LL_GPIO_SetOutputPin(D2_GPIO_Port,D2_Pin);
			else LL_GPIO_ResetOutputPin(D2_GPIO_Port,D2_Pin);
				//LL_GPIO_TogglePin(D2_GPIO_Port,D2_Pin);
			break;
			case 2:
				if(Leds[2].on) LL_GPIO_SetOutputPin(D3_GPIO_Port,D3_Pin);
					else LL_GPIO_ResetOutputPin(D3_GPIO_Port,D3_Pin);
				//LL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
			break;
		}
	}
}

void ledLoop(void const * argument){
	if(!Leds[0].blink && !Leds[1].blink && !Leds[2].blink) return;
	if(HAL_GetTick()-_ledBlinkLastDo >ledBlinkParMs){
		if(Leds[0].blink){
			LL_GPIO_TogglePin(D1_GPIO_Port,D1_Pin);
		}
		if(Leds[1].blink){
			LL_GPIO_TogglePin(D2_GPIO_Port,D2_Pin);
		}
		if(Leds[2].blink){
			LL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
		}
		_ledBlinkLastDo =HAL_GetTick();
	}
}

