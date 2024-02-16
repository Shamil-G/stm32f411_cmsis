#include "main.h"

int monitorStarted=0;
SELECT_TIMER selected_timer=TIMER1;
void toggle_led1(void);
void enable_led1(void);
//void meterPortOn();
void FreqMeterOn(void);

int main(void){
      SystemUp();
//      initRCC_F411();
//      InitMainTick();
      init_SysTick();

      init_pwm();
      EncoderOn();

//      change_pwm_mode(sinusFifty);

      spi2_gpio_init();
//      ili9341_gpio_init();
      spi_init(SPI2);
      dma_spi2_init();

	InitADC();
//	meterPortOn();
	FreqMeterOn();
//	Phase3_InitHrpwm();

	enable_led1();
	while(1){
	  show_ili9341_monitor();
	}
}

void pwm_up(void){
  if(selected_timer==TIMER2)
    pwm_tim2_up();
  if(selected_timer==TIMER1)
    pwm2_tim1_up(TIM1->ARR/20);
}

void pwm_down(void){
  if(selected_timer==TIMER2)
    pwm_tim2_down();
  if(selected_timer==TIMER1)
    pwm2_tim1_down(TIM1->ARR/20);
}

void freqUp(void){
  if(selected_timer==TIMER2)
    tim2_freqUp();
  if(selected_timer==TIMER1)
    tim1_freqUp();
}

void freqDown(void){
  if(selected_timer==TIMER2)
    tim2_freqDown();
  if(selected_timer==TIMER1)
    tim1_freqDown();
}
