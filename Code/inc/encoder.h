#pragma once

#include "main.h"

#define EncTimer  	TIM3
#define EncTimerIRQ	TIM3_IRQn
struct encValue{
	uint32_t prevValue;
	uint32_t prevCntMainTick;
	uint32_t prevMainTick;
};

#ifdef USE_ENCODER

void EncoderOn(void);
void EncoderValue(void);

#endif
