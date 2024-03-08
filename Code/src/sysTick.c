/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "led.h"

extern uint32_t SystemCoreClock;

volatile uint32_t ticks_delay = 0;

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

#ifdef USE_RTC
extern volatile uint8_t rtc_ticks;
#endif

//void led_upd(void);
//void freqMeter_upd(void);
//
void init_SysTick(){
  SysTick_Config(SystemCoreClock/1000);
}

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
#ifdef USE_RTC
  rtc_ticks++;
#endif
}

void Delay(uint32_t milliseconds) {
  ticks_delay=0;
  while(ticks_delay < milliseconds);
}



