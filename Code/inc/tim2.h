#pragma once
/*
 * Author: Shamil Gusseynov
*/
void init_pwm(void);
void tim2_freqDown();
void tim2_freqUp();
void pwm_tim2_down();
void pwm_tim2_up();
void freq_tune();
void pwm_tune();

//  PWM Single Section
#define PWMSingleTimer TIM2
// Выбираем канал СС на который выйдет PWM
#define PWMSingleTimerCCR PWMSingleTimer->CCR3


