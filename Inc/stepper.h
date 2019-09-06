#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"

typedef struct _steep_stru{
	uint16_t pos;
	_Bool clockwise:1;
	_Bool busy:1;
	uint16_t to;
} stepperStr;

void PJTo(uint16_t pos);
bool PJStep(void);
void PJInit(void);
void FSFTo(uint16_t pos);
bool FSFStep(void);
void stepperInit(void);
uint16_t PJPos(void);
void pjStart(void);
void PJStop(void);
void fsfStart(void);
void FSFStop(void);
#endif
