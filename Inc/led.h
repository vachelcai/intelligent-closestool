#ifndef LED_H
#define LED_H

#include "stdint.h"

void ledLoop(void const * argument);
void ledSetBlink(uint8_t d,_Bool setBlink);
void ledSetOn(uint8_t d,_Bool setOn);

#endif
