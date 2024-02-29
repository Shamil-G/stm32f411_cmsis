#ifndef __MAIN_H__
#define __MAIN_H__

#define STM32F411

#define USE_SYSTICK
#define USE_USART
#define USE_USART_DMA

//#define USE_SPI
//#define USE_SPI_DMA

//#define USE_I2C
//#define USE_SHT31
//#define USE_I2C_DMA


#define USE_ADC

//#define USE_ENCODER

//#define USE_FREQ_METER

#include "stm32f4xx.h"
#include "adc-inject.h"
#include "gpio.h"
#include "dma.h"
#include "tim1.h"

#ifdef USE_SYSTICK
#include "SysTick.h"
#endif

#ifdef USE_SPI
#define USE_SPI_ILI9341
#include "spi.h"
#include "spi_ili9341.h"
#include "driver_ili9341.h"
#endif

#ifdef USE_USART
#include "usart.h"
#endif

#ifdef USE_ENCODER
#include "encoder.h"
#endif

#ifdef USE_I2C
#include "i2c.h"

#ifdef USE_SHT31
#include "sht31.h"
#endif

#endif

#ifdef USE_FREQ_METER
#include "freq_meter.h"
#endif


#define _PLUG_NEWLIB

void Delay(uint32_t ms);

//This Project for TSM32F411 from WeAct
//Main feature:
//1. EncoderButton on EXTI3		-> PB_3
//2. TIM3_Encoder on TIM3_IRQHandler	-> PB_4, PB_5
//3. TIM2_CH3 PWM			-> PB_10
//4. TIM4 MainTick on TIM4_IRQHandler	-> no Port
//5. TIM5 frequency measuring on EXTI1_IRQHandler 	-> PA_1
//6. TIM1
//7. USART1				-> PB_6 TX, PB_7 RX
//8. SPI1				-> PB_12 NSS, PB_13 NSCK, PB14_MISO, PB_15 MOSI

//#define CPU_CLOCK 	100000000

//#define USE_SYSTICK

// SPI Section

void init_i2c(I2C_TypeDef * p_i2c);
void dma_i2c1_init();

// Encoder Section



//  PWM Single Section
#define PWMSingleTimer TIM2
// Выбираем канал СС на который выйдет PWM
#define PWMSingleTimerCCR PWMSingleTimer->CCR3

#define Timer5 			TIM5
#define Timer5Enable	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN
#define TimerFreqMeterIRQ	TIM5_IRQn

// 50Hz Timer for sinusoidal
#define TIMER_50Hz	TIM11
void tim11_50Hz_init(void);

extern uint32_t mainTick;
extern uint8_t  posFreqPWM;
extern uint16_t  currDutyTim1;
extern uint32_t listFreqPWMPSC[];

typedef  enum{
  TIMER1 = 1,
  TIMER2 = 2
} SELECT_TIMER;
extern SELECT_TIMER selected_timer;

typedef  enum{
  Common = 0,
  PWM_FREQ,
  PWM_DUTY,
  CH_TIMER,
  Sinus
} Menu;
extern Menu active_menu_item;

typedef  enum{
  Select = 0,
  Edit
} MenuEdit;
extern MenuEdit item_menu_status;

uint32_t getFreqPWM(void);
//void spi_master_init(void);

void tim1_gpio_init(void);
void tim1_init(void);
typedef enum {
  freeMode = 0,
  sinusFifty
} ModePWM;

void change_pwm_mode(ModePWM mode);

void SystemUp();
void led1_on(void);
void led1_off(void);
void init_pwm_tim3(void);
void Phase3_InitHrpwm(void);
void Phase3_HRTim_OFF(void);
void checkEncButton(void);
void checkEncValue(void);
void showSOS(void);
void showUp(void);
void showDown(void);


void pwm_tim2_up();
void pwm_tim2_down();
void tim2_freqUp();
void tim2_freqDown();

void pwm2_tim1_up();
void pwm2_tim1_down();
void tim1_freqUp();
void tim1_freqDown();

void freqUp(void);
void freqDown(void);
void pwm_up(void);
void pwm_down(void);
void init_pwm(void);
void pwm2_test(void);


#ifdef USE_ADC
#include "adc-inject.h"
#endif

void InitMainTick(void);
void showBip(void);

#endif
