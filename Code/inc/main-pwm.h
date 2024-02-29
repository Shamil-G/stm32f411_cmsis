/*
 * pwm-2.h
 *
 *  Created on: Feb 21, 2021
 *  Author: Shamil Gusseynov
 */

// PP_MODE - Push Pull Mode - выравнивание ШИМ по середине с одинаковой длиной
// COMPL_MODE - если один сигнал уменьшается то другой увеличивается за его счет

#pragma once


//  Разгон до 1/INIT_PART
#define INIT_PART			9
#define PWM_VALUE           1
#define PWM_PSC				0

void soft_start(void);
void start_pwm_2(void);

uint8_t pwm_up(uint16_t pwm_value);
uint8_t pwm_down(uint16_t pwm_value);
void 	pwm_lock();

void PWM_init(void);
void pwm_test(void);
