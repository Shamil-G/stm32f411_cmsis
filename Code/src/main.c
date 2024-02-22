#include "main.h"

int monitorStarted=0;
SELECT_TIMER selected_timer=TIMER2;
void toggle_led1(void);
void enable_led1(void);
//void meterPortOn();
void FreqMeterOn(void);
float SHT31_GetHumidity();
float SHT31_GetTemperature();
uint8_t sht31_request(I2C_TypeDef* p_i2c, uint8_t addr_device, uint32_t timeout_ms);

volatile uint8_t addr_device = 0x44; // Address sht31
uint8_t status;
float humidity;
float temper;

int main(void){
      SystemUp();
//      initRCC_F411();
//      InitMainTick();
      init_SysTick();

//      init_pwm(); 		// 17.02.2024
      EncoderOn();

//      change_pwm_mode(sinusFifty);

//      spi2_gpio_init();
//      spi_init(SPI2);
//      dma_spi2_init();

//      ili9341_gpio_init();
//      ili9341_init(240,320);
//      ili9341_primary_tune();

//	InitADC(); 			// 17.02.2024
//	meterPortOn();
//	FreqMeterOn(); 		// 17.02.2024
//	Phase3_InitHrpwm();
  	init_i2c(I2C1);
  	dma_i2c1_init();


	enable_led1();
	while(1){
		status = sht31_request(I2C1, addr_device, 500);
		humidity=SHT31_GetHumidity();
		temper=SHT31_GetTemperature();
//	  show_ili9341_monitor();
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
