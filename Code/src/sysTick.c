/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "led.h"

#ifdef USE_SYSTICK

extern uint32_t SystemCoreClock;

volatile uint32_t ticks_delay = 0;
// Every second set to 0
volatile uint16_t s1_ticks = 0;
extern volatile uint32_t freqMeter;
extern uint32_t meter_ticks;
extern uint32_t tim5_freq_meter;

#ifdef USE_ENCODER
extern uint16_t encoder_ticks;
#endif

#ifdef USE_ADC
extern uint16_t adc_ticks;
#endif

#ifdef USE_SCREEN
extern uint16_t screen_ticks;
#endif

#ifdef USE_SPI
extern uint16_t spi_ticks;
#endif

#ifdef USE_USART
extern uint16_t usart_ticks;
#endif

#ifdef USE_I2C
extern uint16_t i2c_ticks;
#endif

//void led_upd(void);
//void freqMeter_upd(void);
//
void init_SysTick(){
  SysTick_Config(SystemCoreClock/1000);
}

#ifndef USE_FREERTOS

void SysTick_Handler(void) {
//  freq_ticks++;
  ticks_delay++;
#ifdef USE_SCREEN
  screen_ticks++;
#endif
#ifdef USE_ADC
  adc_ticks++;
#endif
#ifdef USE_ENCODER
  encoder_ticks++;
#endif
#ifdef USE_SPI
  spi_ticks++;
#endif
#ifdef USE_USART
  usart_ticks++;
#endif
#ifdef USE_I2C
  i2c_ticks++;
#endif
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


