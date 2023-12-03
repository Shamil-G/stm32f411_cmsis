#pragma once

#ifndef __FREQ_METER__
#define __FREQ_METER__

volatile uint32_t freq_meter_ticks = 0; // Для EXTI1_IRQHandler() - считает по входящим на порт
volatile uint32_t freqMeter = 0;


#endif
