#pragma once
/*
 * Author: Shamil Gusseynov
*/
#define STM32F411

#define _PLUG_NEWLIB

//This Project for TSM32F411 from WeAct
//Main feature:
//1. EncoderButton on EXTI3		-> PB_3
//2. TIM3_Encoder on TIM3_IRQHandler	-> PB_4, PB_5
//3. TIM2_CH3 PWM			-> PB_10
//4. TIM4 MainTick on TIM4_IRQHandler	-> no Port
//5. TIM5 frequency measuring on EXTI1_IRQHandler 	-> PA_1
//7. USART1				-> PB_6 TX, PB_7 RX
//8. SPI1				-> PB_12 NSS, PB_13 NSCK, PB14_MISO, PB_15 MOSI

void SystemUp();

#define USE_SYSTICK
#define USE_SCREEN

#define USE_TIM1
//// Single PWM
#define USE_TIM2

//#define USE_USART
//#define USE_USART_DMA

#define USE_RTC

#define USE_SPI
#define USE_SPI_DMA
//
//#define USE_I2C
//#define USE_SHT31
//#define USE_I2C_DMA
//
//
#define USE_ADC
//
#define USE_ENCODER

#define USE_FREQ_METER

#include "stm32f4xx.h"
#include "led.h"
#include "gpio.h"
#include "dma.h"

#ifdef USE_TIM1
#include "tim1.h"
#endif

#ifdef USE_TIM2
#include "tim2.h"
#endif

#ifdef USE_SYSTICK
#include "SysTick.h"
#endif

#ifdef USE_SPI
#define USE_SPI_ILI9341
#include "spi.h"
#endif

#ifdef USE_SCREEN
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

#ifdef USE_RTC
#include "rtc.h"
#endif

#ifdef USE_FREQ_METER
#include "freq_meter.h"
#endif

#ifdef USE_ADC
#include "adc-inject.h"
#endif



