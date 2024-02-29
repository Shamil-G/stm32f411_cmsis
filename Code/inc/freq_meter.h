#pragma once

#ifndef __FREQ_METER__
#define __FREQ_METER__

extern uint16_t currDutyTim;

extern volatile uint32_t  freq_meter_ticks; // Для EXTI1_IRQHandler() - считает по входящим на порт
extern volatile uint32_t  freqMeter;
extern volatile uint16_t  currDutyTim1;

uint32_t getFreqPWM(void);
void  FreqMeterOn(void);
float getFreqDuty(void);
inline uint32_t getFreqMeter(void){
  return freqMeter;
//			(CPU_CLOCK/listFreqPWMPSC[posFreqPWM]);
};

// Freq Meter Section
#define FreqMeterGPIO GPIOA
#define FreqMeterPin  1
#define FreqMeterAF   af2

#endif
