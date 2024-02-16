#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "adc-inject.h"
#include "gpio.h"
#include "spi.h"
#include "dma.h"
#include "tim1.h"

#include "spi_ili9341.h"
#include "driver_ili9341.h"

#define USE_SYSTICK
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
#define USE_SPI_ILI9341

#define SPI_MOSI_Port	GPIOB
#define SPI_MOSI_Pin	15
#define SPI_SCK_Port	GPIOB
#define SPI_SCK_Pin	13
#define SPI_MISO_Port	GPIOB
#define SPI_MISO_Pin	14

#ifndef USE_SPI_ILI9341

#define SPI_NSS_Port	GPIOB
#define SPI_NSS_Pin	12

#endif

// Encoder Section
#define EncTimer  	TIM3
#define EncTimerIRQ	TIM3_IRQn

// ADC Section
#define ADCTimer TIM4

//  PWM Single Section
#define PWMSingleTimer TIM2
// Выбираем канал СС на который выйдет PWM
#define PWMSingleTimerCCR PWMSingleTimer->CCR3

// Freq Meter Section
#define FreqMeterGPIO GPIOA
#define FreqMeterPin  1
#define FreqMeterAF   af2

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

struct encValue{
	uint32_t prevValue;
	uint32_t prevCntMainTick;
	uint32_t prevMainTick;
};

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

void init_SysTick(void);

uint32_t getFreqPWM(void);
//void spi_master_init(void);

void tim1_gpio_init(void);
void tim1_init(void);
typedef enum {
  freeMode = 0,
  sinusFifty
} ModePWM;

void change_pwm_mode(ModePWM mode);


void spi_init(SPI_TypeDef *spi);
void spi2_gpio_init(void);
void ili9341_gpio_init(void);
void spi_ili9341_init(void);
int  spi2_dma_ready();
void spi2_dma_enable();

void SystemUp();
void EncoderOn(void);
void EncoderValue(void);
void led1_on(void);
void led1_off(void);
void init_pwm_tim3(void);
void Phase3_InitHrpwm(void);
void Phase3_HRTim_OFF(void);
void checkEncButton(void);
void checkEncValue(void);
void InitADC(void);
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

void FreqMeterOn(void);
uint32_t getFreqMeter(void);
float getFreqDuty(void);
void setFreqMeterPSK(uint16_t psk);

float getInputVoltage(void);
float getOutputVoltage(void);
float getInputCurrent(void);
float getOutputCurrent(void);

void vTaskLed1(void *parameter);
void vTaskMeasureFreq(void *parameter);
void vTaskMeasureADC(void *parameter);
void vTaskMonitor(void *parameter);

void InitMainTick(void);
void showBip(void);

#endif
