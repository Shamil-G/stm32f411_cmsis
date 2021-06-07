/********************************************************************************
 * class        Settings port I/O                                               *
 *                                                                              *
 * file         Gpio.h                                                          *
 * author       Shamil Gusseynov                                                 *
 * date         11.05.2020                                                      *
*                                                                               *
 ********************************************************************************/

#pragma once

#define __IO volatile

#define STM32F4xx
//#define STM32F334

#ifdef STM32F4xx

#include "stm32f4xx.h"
#define ENABLE_GPORT_A RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#define ENABLE_GPORT_B RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#define ENABLE_GPORT_C RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#define ENABLE_GPORT_D RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN
#define ENABLE_GPORT_E RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#endif

#ifdef STM32F334
#include "stm32f3xx.h"
#define ENABLE_GPORT_A RCC->AHBENR  |= RCC_AHBENR_GPIOAEN
#define ENABLE_GPORT_B RCC->AHBENR  |= RCC_AHBENR_GPIOBEN
#define ENABLE_GPORT_C RCC->AHBENR  |= RCC_AHBENR_GPIOCEN
#define ENABLE_GPORT_D RCC->AHBENR  |= RCC_AHBENR_GPIODEN
#define ENABLE_GPORT_E RCC->AHBENR  |= RCC_AHBENR_GPIOEEN
#define ENABLE_GPORT_F RCC->AHBENR  |= RCC_AHBENR_GPIOFEN
#endif



typedef enum
{
    push_pull = 0b0,    // Output push-pull (reset state)
    openDrain = 0b1,    // Output open-drain
    t_shift = 1, // Сдвиг на 1 бит
    t_full = 0b1 // Маска
} OTyper;

typedef enum
{
    low = 0b00,
    medium = 0b01,
    high = 0b10,
    veryHigh = 0b11,
    s_shift = 2,
    s_full = 0b11
} Speed;

/* Активация подтягивающих регистров*/
typedef enum
{
    noPull = 0b00,
    pullUp = 0b01,
    pullDown = 0b10,
    p_shift = 2,
    p_full = 0b11
} Pupdr;

typedef enum
{
    af0,
    af1,
    af2,
    af3,
    af4,
    af5,
    af6,
    af7,
    af8,
    af9,
    af10,
    af11,
    af12,
    af13,
    af14,
    af15,
    af_shift = 4,
    af_full = 15
} AF;

typedef enum
{
    input = 0b00,
    output = 0b01,
    alternateF = 0b10,
    analog = 0b11,
    m_shift = 2,
    m_full = 0b11
} Moder;

void InitGPio(
    GPIO_TypeDef *port,
    unsigned int NumPin,
    Moder v_moder,
    OTyper v_type,
    Speed v_speed,
    Pupdr v_pupdr, // Pull-up
    AF v_af);

void PortReset (GPIO_TypeDef* port, uint8_t NumPin);
void PortSet (GPIO_TypeDef* port, uint8_t NumPin);
void PortToggle (GPIO_TypeDef* port, uint8_t NumPin);

