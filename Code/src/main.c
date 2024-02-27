#include "main.h"

#include "string.h"

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
extern uint8_t tx_ready;

int main(void){
      SystemUp();
//      initRCC_F411();
//      InitMainTick();
#ifdef USE_SYSTICK
      init_SysTick();
#endif

//      init_pwm(); 		// 17.02.2024
#ifdef USE_ENCODER
      EncoderOn();
#endif

//      change_pwm_mode(sinusFifty);

#ifdef USE_SPI
	spi2_gpio_init();
	spi_init(SPI2);
	dma_spi2_init();
#endif

//      ili9341_gpio_init();
//      ili9341_init(240,320);
//      ili9341_primary_tune();

//	InitADC(); 			// 17.02.2024
//	meterPortOn();
//	FreqMeterOn(); 		// 17.02.2024
//	Phase3_InitHrpwm();
#ifdef USE_I2C
  	init_i2c(I2C1);
#endif

#ifdef USE_USART
  	uint8_t buff_rx[16]={'a','b','c','d','e','f','g','h','i','g','k'};
  	uint8_t buff_tx[16]= {8,7,6,5,4,3,2,1,0,1,2,3,4,5,6,7};
  	usart_init(USART1);
#endif
	enable_led1();

	while(1){

#ifdef USE_I2C
		status = sht31_request(I2C1, addr_device, 500);
		humidity=SHT31_GetHumidity();
		temper=SHT31_GetTemperature();
#endif
#ifdef USE_USART
//		usart1_tx(buff_tx, sizeof(buff_tx), 100);
		usart1_dma_tx(buff_tx, sizeof(buff_tx), 100);
//	  	memset(buff_rx, 0, sizeof(buff_rx));
		usart1_dma_rx(buff_rx, sizeof(buff_tx), 100);
//		usart1_rx(buff_rx, sizeof(buff_tx), 100);
#endif
		Delay(100);
		toggle_led1();
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
