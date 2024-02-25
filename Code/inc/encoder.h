#pragma once

#include "main.h"

#ifdef USE_ENCODER

#define EncTimer  	TIM3
#define EncTimerIRQ	TIM3_IRQn

void EncoderOn(void);
void EncoderValue(void);

struct encValue{
	uint32_t prevValue;
	uint32_t prevCntMainTick;
	uint32_t prevMainTick;
};

#endif
