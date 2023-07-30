#include "tim1.h"

ModePWM  curr_mode_pwm=freeMode;

// Для блокирования изменения регистров TIM1
uint8_t lockTim1;
//
// В процентах заполнение меандра положительным сигналом
// % = currDutyTim1/10
uint16_t  currDutyTim1;
// Номер выбранной частоты в списке Tim1_listFreqPWMPSC
uint8_t  Tim1_posFreqPWM;
// Список формируемых частот
uint32_t Tim1_listFreqPWMPSC[]=
   { 2000000,	// 50Hz
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
     400	// 250 KHz
//     ,250	// 400 KHz
//     ,200	// 500 KHz
//     ,125	// 800 KHz
     };

//
struct str_regs_pwm{
  unsigned int tim1_arr;
  unsigned int tim1_ccr1;
  unsigned int tim2_arr;
  unsigned int tim2_ccr1;

} regs_pwm;


void tim1_gpio_init(){
  InitGPio( TIM1_CH1_Port, TIM1_CH1_Pin, alternateF, push_pull, veryHigh, noPull,  TIM1_CH1_AF);
  InitGPio( TIM1_CH1N_Port, TIM1_CH1N_Pin, alternateF, push_pull, veryHigh, noPull,  TIM1_CH1N_AF);

  InitGPio( TIM1_CH2_Port, TIM1_CH2_Pin, alternateF, push_pull, veryHigh, noPull,  TIM1_CH2_AF);
  InitGPio( TIM1_CH2N_Port, TIM1_CH2N_Pin, alternateF, push_pull, veryHigh, noPull,  TIM1_CH2N_AF);
}

void change_pwm_mode(ModePWM mode_pwm){
  if(curr_mode_pwm==sinusFifty && mode_pwm==freeMode){
      TIM1->PSC=1-1;
      TIM1->ARR = regs_pwm.tim1_arr;
      TIM1->CCR1 = regs_pwm.tim1_ccr1;
      PWMSingleTimer->ARR = regs_pwm.tim2_arr;
      PWMSingleTimer->CCR1 = regs_pwm.tim2_ccr1;

      curr_mode_pwm=mode_pwm;
  }
  if(curr_mode_pwm==freeMode && mode_pwm==sinusFifty){
      regs_pwm.tim1_arr=TIM1->ARR;
      regs_pwm.tim1_ccr1=TIM1->CCR1;
      regs_pwm.tim2_arr=PWMSingleTimer->ARR;
      regs_pwm.tim2_ccr1=PWMSingleTimer->CCR1;

      TIM1->PSC=2000-1;
      TIM1->ARR = 1000-1;
      TIM1->CCR1 = (TIM1->ARR/2)-1;

      PWMSingleTimer->PSC = 2 - 1;
      PWMSingleTimer->ARR  = 1000-1; 	// 50 kHz
      PWMSingleTimerCCR = 1000/2-1;		// 50% - Коэффициент заполнения

      curr_mode_pwm=mode_pwm;
  }
}

// Ниже все протестировано 23.07.2023
//
// Второй канал, который в моде=1, счетчик считает обычно по возрастанию
// Первый канал считает в обратную сторону

void tim1_pwm_tune(){
	volatile uint16_t first_counter_value, second_counter_value;
	if (lockTim1==0){
		lockTim1=1;
		first_counter_value = (TIM1->ARR * currDutyTim1 / (1000 * 2)) - T1_DEAD_Time;
		second_counter_value = TIM1->ARR+T1_DEAD_Time-first_counter_value;
		T1_FIRST_COUNTER = first_counter_value;
		T1_SECOND_COUNTER = (second_counter_value<TIM1->ARR)?second_counter_value:TIM1->ARR;
		lockTim1=0;
	}
}

void tim1_freq_tune(){
  if(lockTim1==0){
	  lockTim1=1;
      TIM1->ARR=Tim1_listFreqPWMPSC[Tim1_posFreqPWM]-1;
	  lockTim1=0;
	  // Пересчитаем и установим PWM
	  tim1_pwm_tune();
  }
}

void pwm2_tim1_up(){
	if(currDutyTim1<1000){
		if(currDutyTim1>=100 && currDutyTim1<900)
			currDutyTim1+=100;
		else
			currDutyTim1=(currDutyTim1/10+1)*10;
		if(currDutyTim1>1000)
			currDutyTim1=1000;
		tim1_pwm_tune();
	}
};

void pwm2_tim1_down(){
	if(currDutyTim1>0){
		if(currDutyTim1>100 && currDutyTim1<=900)
			currDutyTim1-=100;
		else
			currDutyTim1=(currDutyTim1/10-1)*10;
		tim1_pwm_tune();
	}
};

void tim1_freqUp(void){
  if(Tim1_posFreqPWM<sizeof(Tim1_listFreqPWMPSC)/sizeof(uint32_t)-1){
      Tim1_posFreqPWM++;
      tim1_freq_tune();
  }
}

void tim1_freqDown(void){
  if(Tim1_posFreqPWM>0){
	  Tim1_posFreqPWM--;
      tim1_freq_tune();
  }
}

void tim1_init(){
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  TIM1->PSC=PWM_PSC;
  lockTim1 = 0;
  Tim1_posFreqPWM=12;
  currDutyTim1 = 100;
  curr_mode_pwm=freeMode;
  tim1_freq_tune();
  // Счетчик регистра захвате/сравнения первого канала
//  T1_FIRST_COUNTER=(MEANDR_TIMER_TICKS/INIT_PART)-(T1_DEAD_Time/2)-PWM_VALUE;
//  // Счетчик регистра захвате/сравнения второго канала
//  T1_SECOND_COUNTER=(MEANDR_TIMER_TICKS*(INIT_PART-1)/INIT_PART)+(T1_DEAD_Time/2)+PWM_VALUE;

//  TIM1->CCR1 = (TIM1->ARR/4)-1;

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->CCER = 0;
  TIM1->CCMR1 = 0;

  // Установим делитель частоты для DEAD TIME на 25МГц
  TIM1->CR1 |= TIM_CR1_CKD_1;
  // Центруем фронт импульса - center-aligned mode selection
  // TIM1->CR1 |= TIM_CR_CMS;
  // Считаем вниз
  // TIM1->CR1 |= TIM_CR_DIR;

  // Включаем PWM_mode=1 для 1 канала, output compare fast enable
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1FE;
  //Output Compare 1 preload enable
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
  //  Включим CCR1
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;

  // Включаем PWM_mode=2 для 2 канала
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2FE;
  //Output Compare 1 preload enable
  TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
  //  Включим CCR1
  TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;

  // Main output enable - почему MOE в этом регистре?
  TIM1->BDTR=TIM_BDTR_MOE | T1_DEAD_Time;
  // Буферизуем загрузку регистров
  TIM1->CR1 |= TIM_CR1_ARPE;
  // выравнивание сигналов центру,
  // флаг прерывания при счете вверх и вниз: 5 и 6 биты = 11
  __TIMER->CR1|= TIM_CR1_CMS_1 | TIM_CR1_CMS_0;

  // Включим таймер
  if(selected_timer==TIMER1)
    TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->EGR = TIM_EGR_UG;
}


void soft_start(){
    for( currDutyTim1=0; currDutyTim1<1000; currDutyTim1++){
    	tim1_pwm_tune();
	    Delay(2);
    }
}

void pwm2_test(void){
    soft_start();
}
