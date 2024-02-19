#pragma once

#include "main.h"

#define __TIMER TIM1
#define T1_FIRST_COUNTER  TIM1->CCR1
#define T1_SECOND_COUNTER TIM1->CCR2
//#define T1_DEADTIME_TICKS      	1

#define T1_DEAD_Time		1

#define INIT_PART		15
#define PWM_VALUE           	1
#define PWM_PSC			0
#define MEANDR_TIMER_TICKS  	TIM1->ARR
#define INIT_PWM_VALUE1		(MEANDR_TIMER_TICKS-DEADTIME_TICKS)/4-PWM_VALUE
#define INIT_PWM_VALUE2		(MEANDR_TIMER_TICKS-DEADTIME_TICKS)*3/4+PWM_VALUE

#define TIM1_CH1_Port  GPIOA
#define TIM1_CH1_Pin   8
#define TIM1_CH1_AF   af1
#define TIM1_CH2_Port  GPIOA
#define TIM1_CH2_Pin   9
#define TIM1_CH2_AF   af1
#define TIM1_CH3_Port  GPIOA
#define TIM1_CH3_Pin   10
#define TIM1_CH3_AF   af1

#define TIM1_BKIN_Port GPIOA
#define TIM1_BKIN_Pin  6
#define TIM1_BKIN_AF   af1

#define TIM1_ETR_Port  GPIOA
#define TIM1_ETR_Pin   12
#define TIM1_CH3_AF    af1

#define TIM1_CH1N_Port GPIOA
#define TIM1_CH1N_Pin  7
#define TIM1_CH1N_AF   af1

#define TIM1_CH2N_Port GPIOB
#define TIM1_CH2N_Pin  0
#define TIM1_CH2N_AF   af1

#define TIM1_CH3N_Port GPIOB
#define TIM1_CH3N_Pin  1
#define TIM1_CH2N_AF   af1

#define SET_CH1_DUTY(value) (TIM1->CCR1=(int)value)
