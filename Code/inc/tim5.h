#pragma once
/*
 * Author: Shamil Gusseynov
*/
void TIM5On(void);

#define Timer5 			TIM5
#define Timer5Enable	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN
#define TimerFreqMeterIRQ	TIM5_IRQn

