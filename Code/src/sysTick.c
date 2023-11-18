#include "main.h"


#ifdef USE_SYSTICK

volatile uint32_t ticks_delay = 0;
volatile uint32_t encoder_ticks = 0;
volatile uint32_t freq_ticks = 0;


extern uint32_t SystemCoreClock;

void init_SysTick(){
  SysTick_Config(SystemCoreClock/1000);
}

#ifndef USE_FREERTOS

void SysTick_Handler(void) {
  ticks_delay++;
  encoder_ticks++;
  freq_ticks++;
}

#ifndef __delay
#define __delay

void Delay(uint32_t milliseconds) {
  ticks_delay=0;
  while(ticks_delay < milliseconds);
}
#endif

#endif
#endif


