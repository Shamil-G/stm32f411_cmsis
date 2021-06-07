#include "main.h"
#include "tim1.h"

struct str_regs_pwm{
  unsigned int tim1_arr;
  unsigned int tim1_ccr1;
  unsigned int tim2_arr;
  unsigned int tim2_ccr1;

} regs_pwm;

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
     200,	// 500 KHz
     125	// 800 KHz
     };

uint8_t  Tim1_posFreqPWM=12;
ModePWM curr_mode_pwm=freeMode;


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


void tim1_pwm_up(void){
  TIM1->CCR1=(TIM1->CCR1+Tim1_listFreqPWMPSC[Tim1_posFreqPWM]/10<TIM1->ARR-1)?
      TIM1->CCR1+Tim1_listFreqPWMPSC[Tim1_posFreqPWM]/10:Tim1_listFreqPWMPSC[Tim1_posFreqPWM]-2;
}

void tim1_pwm_down(void){
  TIM1->CCR2=(TIM1->CCR1>Tim1_listFreqPWMPSC[Tim1_posFreqPWM]/10)?
			(TIM1->CCR1-Tim1_listFreqPWMPSC[Tim1_posFreqPWM]/10)
			:0;
}

void tim1_freqUp(void){
  if(Tim1_posFreqPWM<sizeof(Tim1_listFreqPWMPSC)/4-1){
      uint16_t part;
      part = TIM1->ARR / TIM1->CCR1;
      Tim1_posFreqPWM++;
      TIM1->ARR=Tim1_listFreqPWMPSC[Tim1_posFreqPWM]-1;
      // Счетчик регистра захвате/сравнения первого канала
      T1_FIRST_COUNTER=(TIM1->ARR/part)-1;
      // Счетчик регистра захвате/сравнения второго канала
      T1_SECOND_COUNTER=TIM1->ARR - T1_FIRST_COUNTER;
  }
}

void tim1_freqDown(void){
  if(Tim1_posFreqPWM>0){
      uint16_t part;
      part = TIM1->ARR / TIM1->CCR1;
      Tim1_posFreqPWM--;
      TIM1->ARR=listFreqPWMPSC[Tim1_posFreqPWM]-1;
      // Счетчик регистра захвате/сравнения первого канала
      T1_FIRST_COUNTER=(TIM1->ARR/part)-1;
      // Счетчик регистра захвате/сравнения второго канала
      T1_SECOND_COUNTER=TIM1->ARR - T1_FIRST_COUNTER;
  }
}

// Второй канал, который в моде=1,  счетчик считает обычно по возрастанию
// Первый канал считает в обратную сторону

uint8_t pwm2_tim1_down(uint16_t pwm_value){
	volatile uint8_t add_value, minus_value = 0;
	minus_value= (T1_FIRST_COUNTER<pwm_value)?T1_FIRST_COUNTER:pwm_value;
	add_value = (T1_SECOND_COUNTER+pwm_value>=TIM1->ARR)?TIM1->ARR - T1_SECOND_COUNTER:pwm_value;
	if (add_value>0 || minus_value>0){
	    T1_FIRST_COUNTER -= minus_value ;
	    T1_SECOND_COUNTER += add_value;
	    return 1;
	}
	return 0;
};

uint8_t pwm2_tim1_up(uint16_t pwm_value){
	volatile uint8_t add_value, minus_value = 0;
	add_value = (T1_FIRST_COUNTER+pwm_value>=(TIM1->ARR-T1_DEAD_Time)/2)?(TIM1->ARR - T1_DEAD_Time)/2-T1_FIRST_COUNTER:pwm_value;
	minus_value = (T1_SECOND_COUNTER-pwm_value<=(TIM1->ARR+T1_DEAD_Time)/2)?T1_SECOND_COUNTER-(TIM1->ARR + T1_DEAD_Time)/2:pwm_value;
	if (minus_value>0 || add_value>0){
	    T1_FIRST_COUNTER += add_value;
	    T1_SECOND_COUNTER -= minus_value;
	    return 1;
	}
	return 0;
};

void tim1_init(){
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  TIM1->PSC=PWM_PSC;
  Tim1_posFreqPWM=12;
  TIM1->ARR = Tim1_listFreqPWMPSC[Tim1_posFreqPWM]-1;

  // Счетчик регистра захвате/сравнения первого канала
  T1_FIRST_COUNTER=(MEANDR_TIMER_TICKS/INIT_PART)-(T1_DEAD_Time/2)-PWM_VALUE;
  // Счетчик регистра захвате/сравнения второго канала
  T1_SECOND_COUNTER=(MEANDR_TIMER_TICKS*(INIT_PART-1)/INIT_PART)+(T1_DEAD_Time/2)+PWM_VALUE;

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
    volatile uint8_t add_value, minus_value = 0;

//    init_adc_struct();
    for( uint8_t i=0; i<(MEANDR_TIMER_TICKS-T1_DEAD_Time)/5; i++){
	    add_value = (T1_FIRST_COUNTER+PWM_VALUE>=(MEANDR_TIMER_TICKS-T1_DEAD_Time)/2)?(MEANDR_TIMER_TICKS-T1_DEAD_Time)/2-T1_FIRST_COUNTER:PWM_VALUE;
	    minus_value = (T1_SECOND_COUNTER-PWM_VALUE<=(MEANDR_TIMER_TICKS+T1_DEAD_Time)/2)?T1_SECOND_COUNTER-(MEANDR_TIMER_TICKS+T1_DEAD_Time)/2:PWM_VALUE;
	    if (minus_value==0 || add_value==0){
		    return;
	    }
	    T1_FIRST_COUNTER += add_value;
	    T1_SECOND_COUNTER -= minus_value;
	    Delay(2);
    }
}

void pwm2_test(void){
    for(int j=0; j<1; j++){
	    for(int i=0; i<500 && pwm2_tim1_up(PWM_VALUE); i++){
		Delay(1);
	    }
	    for(int i=0; i<500 && pwm2_tim1_down(PWM_VALUE); i++)
	      Delay(1);
    }
//    soft_start();
}
