/*
 * hrtim-stm32f334.h
 *
 *  Created on: 20 апр. 2021 г.
 *      Author: Shamil Gusseynov
 */

#pragma once

#ifndef INC_HRTIM_STM32F4xx
#define INC_HRTIM_STM32F4xx

typedef enum
{
	Timer_A=0,
	Timer_B,
	Timer_C,
	Timer_D,
	Timer_E
} HRTimer;


void HRTim_GPioOn(HRTimer timer);
#define HRTIMPeriod (uint16_t)30000
#define CountChannel	2
#define ChannelPeriodHalf HRTIMPeriod/(2*CountChannel)


#endif
