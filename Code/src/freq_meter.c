#include "main.h"
#include "led.h"
#include "freq_meter.h"

// from pwm_single.c
volatile uint32_t freq_meter_ticks = 0; // Для EXTI1_IRQHandler() - считает по входящим на порт
volatile uint32_t freqMeter = 0;

// Прерывание по фронтам входного сигнала
// Идет через GPIO - те есть через EXTI класс прерываний
void EXTI1_IRQHandler() {
  // Очистка прерывания
	EXTI->PR |= EXTI_PR_PR1;
	freq_meter_ticks++;
}

// Прерывание по переполнению счетчика
// Т.е. идет от Таймера через класс прерываний IRQHandler
void TIM5_IRQHandler(void){
//    if(TimerFreqMeter->SR & TIM_SR_TIF){
//	    TimerFreqMeter->SR &= ~TIM_SR_TIF;
//    }
//    if(TimerFreqMeter->SR & TIM_SR_UIF){
//	    TimerFreqMeter->SR &= ~TIM_SR_UIF;
//    }
	TIM5->SR &= ~TIM_SR_UIF;

	freqMeter=freq_meter_ticks;
    freq_meter_ticks=0;

    LED1_TOGGLE
}


inline uint32_t getFreqPWM(void){
  uint32_t result;
  result = (selected_timer==TIMER2?SystemCoreClock/(TIM2->ARR+1):SystemCoreClock/(2*(TIM1->ARR+1)));
  return result;
}

float getFreqDuty(void){
  float result=0;

  if(selected_timer==TIMER1){
      return currDutyTim1/(2*10);
  }
  if(selected_timer==TIMER2){
      result=(1000-currDutyTim)/10;
  }
  return (100-result);
}

// Для измерения частоты будем использовать PA1, который будет входом для TIM5
// TIM5 по восходящему фронту импульса на PA1 будет вызыать прерывание EXTI1_IRQHandler, в котором будем считать импульсы
// Кроме того сам таймер настроим на прерывание на 1 сек, чтобы подсчитанное кол-во импульсов сохранить как частоту
// Это дает безошибочное определение частоты до 500KHz, на 1MHz ошибка составляет около 12%
void FreqMeterOn(void){
	  InitGPio( FreqMeterGPIO,
			  	FreqMeterPin,
				alternateF,   	//MODER:10
				veryHigh, 		//SPEEDR
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Подтяжка pull-up внешняя, изначально порт на земле
				FreqMeterAF); //AF2 for TIM5_CH2 on PA1

	// !!! Включаем тактирование таймера
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

	// From HAL
	/* Disable the Channel 2: Reset the CC2E Bit */
	TIM5->CCER &= ~TIM_CCER_CC2E;

	// Считать будем столько раз, сколько сконфигурим
	TIM5->PSC = 1-1;
	//Reset Value for ARR
	TIM5->ARR=SystemCoreClock-1;
	// Обнуляем счетчик
	TIM5->CNT=0;

	TIM5->CR1 = 0;
	// OnePulse Mode must be 0 for continue counter
	//TimerFreqMeter->CR1 &= ~TIM_CR1_OPM;
	// CMS: Center-aligned mode selection - Counting Down
	//TimerFreqMeter->CR1 &= TIM_CR1_CMS_1;

	/*---------------- Согласно примеру из Cookbook ----------------*/


	/* Clear Capture/compare 1 interrupt flag 2 */
	TIM5->SR = 0;

	// Не будем сравнивать CCR2 с ARR и вызывать переполнение
	TIM5->SR |= TIM_SR_CC2IF;
	// Заодно очистим флаг прерываний
	TIM5->SR &= ~TIM_SR_UIF;

	/*----------------------- End Section ---------------------------*/

	/*----------- TIMx capture/compare mode register 1 (TIMx_CCMR1) -----*/

	TIM5->CCMR1 = 0;
	// Select Input
	// CC2 01: CC2 channel is configured as input
	TIM5->CCMR1 |= TIM_CCMR1_CC2S_0;

	/*----------------------- End Section ---------------------------*/

	/*-------------- TIMx slave mode control register (TIMx_SMCR)----------*/

	// Выбираем триггер TS - trigger selection
	// Обнуляем
	TIM5->SMCR = 0;
	// 110: Filtered Timer Input 2 (TI2FP2)
	TIM5->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_1;
	// Считать вверх/вниз по краю TI2FP1 в зависимости от TI2FP2
	//	CLEAR_BIT(TimerFreqMeter->SMCR, TIM_SMCR_SMS_Msk);

	/*----------------End Section-----------------------------------------*/

	/*-------- TIMx capture/compare enable register (TIMx_CCER) -----*/

	// Учитывать по передним фронтам: TI1FP1 noninverted, TI2FP2 noninverted
	// Ненвертирующий вход - это когда 1 считается как 1
	TIM5->CCER |= TIM_CCER_CC2P;
//	TimerFreqMeter->CCER &= ~TIM_CCER_CC2P;
	// CC1E: Capture/Compare 1 output enable
	TIM5->CCER |= TIM_CCER_CC2E;

	/*----------------------- End Section ---------------------------*/

	//	TimerFreqMeter->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
	// Включаем таймер
	TIM5->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM5_IRQn);
	/* ------------- TIMx DMA/Interrupt enable register (TIMx_DIER) ----------*/
	// Разрешаем прерывания
	//	TimerFreqMeter->DIER |= TIM_DIER_TIE;
	// UIE: Update interrupt enable
	TIM5->DIER |= TIM_DIER_UIE;

	/*--------------------- Настройка EXTI прерывания для PA1--------------------*/
	// Так как считаем входные сигналы приходящие от GPIO, то и прерывание
	// должно быть класса EXTI
	// В первом регистре SYSCFG из 4, находится группа 1 портов для прерывания
	// В группе первых портов выбираем для прерывания порт PA1
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	// Ловить входящий сигнал будем восходящему фронту
	// Rising trigger selection register
	EXTI->RTSR |= EXTI_RTSR_TR1;
	// Ловить сигнал будем нисходящему фронту не будем!
	// Falling trigger selection register
	EXTI->FTSR &= ~EXTI_FTSR_TR1;
	/*--------------------------------------------------------------------------*/

	// Очистка прерывания
	EXTI->PR |= EXTI_PR_PR1;
	//Регистр IMR отвечает за включение/отключение прерывания
	EXTI->IMR |= EXTI_IMR_MR1;
	NVIC_EnableIRQ(EXTI1_IRQn);
	/*-------------------------- End Section ----------------------------------*/
};
