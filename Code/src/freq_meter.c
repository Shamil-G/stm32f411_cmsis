#include "main.h"

//#define freqTick CPU_CLOCK/100

#define IDLE 0
#define DONE 1

struct  freqMeter structFreqMeter;

extern uint8_t  Tim1_posFreqPWM;
extern uint32_t Tim1_listFreqPWMPSC[];
volatile uint32_t freqState = IDLE;
volatile uint32_t overCalc=0;
volatile uint32_t T1=0, T2=0, Ticks=0;

// Прерывание по фронтам входного сигнала
// Идет через GPIO - те есть через EXTI класс прерываний
void EXTI1_IRQHandler() {
  // Очистка прерывания
	EXTI->PR |= EXTI_PR_PR1;

	if(freqState == IDLE)
	{
		structFreqMeter.prevTicks = TimerFreqMeter->CCR2;
		freqState = DONE;
	}
	else if(freqState == DONE)
	{
		structFreqMeter.curTicks = TimerFreqMeter->CCR2;
		freqState = IDLE;
	}
}

// Прерывание по переполнению счетчика
// Т.е. идет от Таймера через класс прерываний IRQHandler
void TIM5_IRQHandler(void){
    if(TimerFreqMeter->SR & TIM_SR_TIF){
	    TimerFreqMeter->SR &= ~TIM_SR_TIF;
    }
    if(TimerFreqMeter->SR & TIM_SR_UIF){
	    TimerFreqMeter->SR &= ~TIM_SR_UIF;
    }
}


inline uint32_t getFreqPWM(void){
  uint32_t result;
  result = (selected_timer==TIMER2?CPU_CLOCK/(TIM2->ARR+1):CPU_CLOCK/(TIM1->ARR+1));
  return result;
}

float getFreqDuty(void){
  float result;
  if(selected_timer==TIMER1){
      result=TIM1->CCR1*100*2;
      result/=TIM1->ARR;
  }
  if(selected_timer==TIMER2){
      result=PWMSingleTimerCCR*100/PWMSingleTimer->ARR;
  }
  return (100-result);
}

inline uint32_t getFreqMeter(void){
  if(structFreqMeter.curTicks > structFreqMeter.prevTicks){
      Ticks = structFreqMeter.curTicks - structFreqMeter.prevTicks;
//      structFreqMeter.freq = (uint32_t)(CPU_CLOCK*Ticks/100)/structFreqMeter.psc;
      structFreqMeter.freq = (uint32_t)(CPU_CLOCK/Ticks)/structFreqMeter.psc;
  }
  return structFreqMeter.freq;
//			(CPU_CLOCK/listFreqPWMPSC[posFreqPWM]);
}

inline void setFreqMeterPSK(uint16_t psk){
	structFreqMeter.psc=psk>0?psk:1;
	TimerFreqMeter->PSC = structFreqMeter.psc-1;
}
void FreqMeterOn(void){
	  structFreqMeter.psc=1;
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
	TimerFreqMeterEnable;

	// From HAL
	/* Disable the Channel 2: Reset the CC2E Bit */
	TimerFreqMeter->CCER &= ~TIM_CCER_CC2E;

	//Reset Value for ARR
	TimerFreqMeter->ARR=CPU_CLOCK-1;
	// Обнуляем счетчик
	TimerFreqMeter->CNT=0;
	// Считать будем столько раз, сколько сконфигурим
	TimerFreqMeter->PSC = structFreqMeter.psc-1;

	TimerFreqMeter->CR1 = 0;
	// OnePulse Mode must be 0 for continue counter
	//TimerFreqMeter->CR1 &= ~TIM_CR1_OPM;
	// CMS: Center-aligned mode selection - Counting Down
	//TimerFreqMeter->CR1 &= TIM_CR1_CMS_1;

	/*---------------- Согласно примеру из Cookbook ----------------*/


	/* Clear Capture/compare 1 interrupt flag 2 */
	TimerFreqMeter->SR = 0;

	// Не будем сравнивать CCR2 с ARR и вызывать переполнение
	TimerFreqMeter->SR |= TIM_SR_CC2IF;
	// Заодно очистим флаг прерываний
	TimerFreqMeter->SR &= ~TIM_SR_UIF;

	/*----------------------- End Section ---------------------------*/

	/*----------- TIMx capture/compare mode register 1 (TIMx_CCMR1) -----*/

	TimerFreqMeter->CCMR1 = 0;
	// Select Input
	// CC2 01: CC2 channel is configured as input
	TimerFreqMeter->CCMR1 |= TIM_CCMR1_CC2S_0;

	/*----------------------- End Section ---------------------------*/

	/*-------------- TIMx slave mode control register (TIMx_SMCR)----------*/

	// Выбираем триггер TS - trigger selection
	// Обнуляем
	TimerFreqMeter->SMCR = 0;
	// 110: Filtered Timer Input 2 (TI2FP2)
	TimerFreqMeter->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_1;
	// Считать вверх/вниз по краю TI2FP1 в зависимости от TI2FP2
	//	CLEAR_BIT(TimerFreqMeter->SMCR, TIM_SMCR_SMS_Msk);

	/*----------------End Section-----------------------------------------*/

	/*-------- TIMx capture/compare enable register (TIMx_CCER) -----*/

	// Учитывать по передним фронтам: TI1FP1 noninverted, TI2FP2 noninverted
	// Ненвертирующий вход - это когда 1 считается как 1
	TimerFreqMeter->CCER |= TIM_CCER_CC2P;
//	TimerFreqMeter->CCER &= ~TIM_CCER_CC2P;
	// CC1E: Capture/Compare 1 output enable
	TimerFreqMeter->CCER |= TIM_CCER_CC2E;

	/*----------------------- End Section ---------------------------*/

	//	TimerFreqMeter->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
	// Включаем таймер
	TimerFreqMeter->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TimerFreqMeterIRQ);
	/* ------------- TIMx DMA/Interrupt enable register (TIMx_DIER) ----------*/
	// Разрешаем прерывания
	//	TimerFreqMeter->DIER |= TIM_DIER_TIE;
	// UIE: Update interrupt enable
	TimerFreqMeter->DIER |= TIM_DIER_UIE;

	/*-------------------------- End Section ----------------------------------*/

	/*--------------------- Настройка EXTI прерывания -------------------------*/

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

	// Очистка прерывания
	EXTI->PR |= EXTI_PR_PR1;
	//Регистр IMR отвечает за включение/отключение прерывания
	EXTI->IMR |= EXTI_IMR_MR1;
	NVIC_EnableIRQ(EXTI1_IRQn);
	/*-------------------------- End Section ----------------------------------*/
};
