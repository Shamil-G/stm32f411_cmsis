/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"

#include "tim5.h"

extern volatile uint32_t freq_meter_ticks;
extern volatile uint32_t freqMeter;

//void TIM5_IRQHandler(void){
//	TIM5->SR &= ~TIM_SR_UIF;
//
//	freqMeter=freq_meter_ticks;
//    freq_meter_ticks=0;
//
//    LED1_TOGGLE
//}

// TIM5 as 1 Hz timer
void TIM5On(void){

//  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	// !!! Включаем тактирование таймера
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

  TIM5->PSC = 1-1;  // 1 MHz
  TIM5->ARR = SystemCoreClock-1;  // частота 1 Hz
  // Обнуляем счетчик
  TIM5->CNT=0;
  // Разрешаем прерывания
  // UIE: Update interrupt enable
  TIM5->DIER |= TIM_DIER_UIE;
  // Включаем таймер
  TIM5->CR1 |= TIM_CR1_CEN;

  //NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  NVIC_EnableIRQ(TIM5_IRQn);
};
