#include "main.h"

volatile uint32_t ticks_delay = 0;

extern uint32_t SystemCoreClock;

void init_SysTick(){
  SystemCoreClock = CPU_CLOCK;
  SysTick_Config(SystemCoreClock/1000);
}

#ifndef USE_FREERTOS

void SysTick_Handler(void) {
  ticks_delay++;
}

void Delay(uint32_t milliseconds) {
  ticks_delay=0;
  while(ticks_delay < milliseconds);
}

#endif


