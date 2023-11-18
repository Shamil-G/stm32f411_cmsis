#include "main.h"

#ifdef USE_FREERTOS

unsigned long mainTick;
unsigned long cntMainTick;
unsigned long durationMs;

void InitMainTick(void){
	mainTick=0;
	cntMainTick=0;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->PSC = 10-1;  // 10 MHz
	TIM4->ARR = 100-1;  // каждую 0.01 us, или 100kHz
	TIM4->DIER |= TIM_DIER_UIE; // Выставляем флаг для прерывания по обновлению счетчика
	TIM4->CR1 |= TIM_CR1_CEN;   // Начали считать

	NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void){
    TIM4->SR &= ~TIM_SR_UIF;
    if(mainTick==0)
	cntMainTick++;
    mainTick++;
    durationMs++;
}

#endif
