/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"

#include "tim2.h"
#include "encoder.h"

#define DEAD_TIME 1
// PWM out port: GPIOB 3
uint8_t lockTim;
uint8_t  posFreqPWM;
// В процентах заполнение меандра положительным сигналом
uint16_t  currDutyTim;

uint32_t listFreqPWMPSC[]=
   { 5000000,	// 20Hz
     2000000,	// 50Hz
     1000000,	// 100Hz
     500000,	// 200Hz
     200000,	// 500Hz
     100000, 	// 1 KHz
     50000,  	// 2 KHz
     20000,  	// 5 kHz
     10000,  	// 10 kHz
     5000,	// 20 KHz
     4000,	// 25 KHz
     2000,	// 50 KHz
     1250,	// 80 KHz
     1000,	// 100 KHz
     500,	// 200 KHz
     400,	// 250 KHz
     250,	// 400 KHz
     200	// 500 KHz
     ,125	// 800 KHz
     ,100	// 1 MHz
     ,80	// 1,25 MHz
//     ,50	// 2 MHz
//     ,25	// 4 MHz
//     ,20	 	// 5 MHz
    };

void pwm_tune(){
	if(lockTim==0){
		lockTim=1;
		PWMSingleTimerCCR = (PWMSingleTimer->ARR  * (1000-currDutyTim) / 1000) + DEAD_TIME;
		lockTim=0;
	}
}

void freq_tune(){
	if(lockTim==0){
		lockTim=1;
  	    PWMSingleTimer->ARR=listFreqPWMPSC[posFreqPWM]-1;
		lockTim=0;
		pwm_tune();
	}
}


void pwm_tim2_up(){
	if(currDutyTim<1000){
		if(currDutyTim>=100 && currDutyTim<900)
			currDutyTim+=100;
		else
			currDutyTim=(currDutyTim/10+1)*10;
		if(currDutyTim>1000)
			currDutyTim=1000;
		pwm_tune();
	}
}

void pwm_tim2_down(){
	if(currDutyTim>0){
		if(currDutyTim>100 && currDutyTim<=900)
			currDutyTim-=100;
		else
			currDutyTim=(currDutyTim/10-1)*10;
		pwm_tune();
	}
}


void tim2_freqUp(){
  if(posFreqPWM<sizeof(listFreqPWMPSC)/sizeof(uint32_t)-1){
  	    posFreqPWM++;
	    freq_tune();
      }
}

void tim2_freqDown(){
  if(posFreqPWM>0){
	    posFreqPWM--;
	    freq_tune();
  }
}


void init_pwm(void)
{
	// Выход генератора PWM: TIM2_CH2 - PB3, TIM2_CH3 - PB10, TIM2_CH4 - PB11
	InitGPio(GPIOB, 10, alternateF, push_pull, veryHigh, noPull, af1);
//	InitGPio(GPIOA, 3, alternateF, push_pull, veryHigh, noPull, af1);
//	InitGPio(GPIOB, 11, alternateF, push_pull, veryHigh, noPull, af1);
	if(PWMSingleTimer==TIM1)
		RCC->APB1ENR |= RCC_APB2ENR_TIM1EN;
	if(PWMSingleTimer==TIM2)
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	if(PWMSingleTimer==TIM3)
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// SMS: Slave mode selection -> disable
	// ECE: External clock enable -> disable
	CLEAR_BIT(PWMSingleTimer->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE);
	WRITE_REG(PWMSingleTimer->PSC, 1 - 1);

	lockTim = 0;
	posFreqPWM=18;
	currDutyTim = 200; // 20%
	freq_tune();
//	PWMSingleTimerCCR = PWMSingleTimer->ARR/currDuty;		// 10% - Коэффициент заполнения
//	PWMSingleTimer->CCR3 = PWMSingleTimer->ARR/10;		// 10% - Коэффициент заполнения

	/* Set the Preload enable bit for channel3 */
	PWMSingleTimer->CCMR2 |= TIM_CCMR2_OC3PE;

	/* Configure the Output Fast mode */
	PWMSingleTimer->CCMR2 &= ~TIM_CCMR2_OC3FE;

	// Channel 3, PWM mode 1
	PWMSingleTimer->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

	// Сhannel 1, PWM mode 1
	//	Timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	// 00:	CC3S - selection channel 3 is configured as output
//	PWMSingleTimer->CCMR2 &= ~TIM_CCMR2_CC3S;
	// 00:	CC4S - selection channel 4 is configured as output
	PWMSingleTimer->CCMR2 &= ~TIM_CCMR2_CC3S;

	// CC1E: Capture/Compare 3 output enable
	PWMSingleTimer->CCER |= TIM_CCER_CC3E;
	// CC1P: Capture/Compare output Polarity
	PWMSingleTimer->CCER &= ~TIM_CCER_CC3P;

	//	Timer->CR1 &= ~TIM_CR1_CKD;  // Делитель для цифрового фильтра
	//	Timer->CR1 &= ~TIM_CR1_OPM;  // Счетчик не останавливается при прерывании
	//	Timer->CR1 &= ~TIM_CR1_CMS;  // Выравнивание сигнала будет определяться битом DIR
	//	Timer->CR1 &= ~TIM_CR1_DIR;  // счет вверх
	CLEAR_BIT(PWMSingleTimer->CR1, TIM_CR1_CKD | TIM_CR1_OPM | TIM_CR1_CMS | TIM_CR1_DIR);
	// Auto reload-preload enable! must be for driving PWM
	PWMSingleTimer->CR1 |= TIM_CR1_ARPE;

	// Для тестирования выставляем возможность прерывания
	PWMSingleTimer->DIER |= TIM_DIER_UIE; // Выставляем флаг для прерывания по обновлению счетчика

	// включаем таймер
	if(selected_timer==TIMER2)
	    TIM2->CR1 |= TIM_CR1_CEN;

	// Для тестирования выставляем возможность прерывания
//	NVIC_EnableIRQ(TIM2_IRQn);
}


void TIM2_IRQHandler(void){
  TIM2->SR &= ~TIM_SR_UIF;
}


