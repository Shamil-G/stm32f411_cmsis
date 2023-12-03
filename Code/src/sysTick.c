#include "main.h"
#include "led.h"

#ifdef USE_SYSTICK

extern uint32_t SystemCoreClock;

volatile uint32_t ticks_delay = 0;
volatile uint32_t encoder_ticks = 0;
// Every second set to 0
volatile uint16_t s1_ticks = 0;
extern uint32_t freqMeter;
extern uint32_t meter_ticks;
extern uint32_t adc_ticks;
extern uint32_t screen_ticks;
extern uint32_t tim5_freq_meter;

void led_upd(void);
void freqMeter_upd(void);

void init_SysTick(){
  SysTick_Config(SystemCoreClock/1000);
}

#ifndef USE_FREERTOS

void SysTick_Handler(void) {
//  freq_ticks++;
  adc_ticks++;
  screen_ticks++;
  ticks_delay++;
  encoder_ticks++;
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


