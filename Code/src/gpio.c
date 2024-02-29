/*
 * gpio.c
 *
 *  Created on: 5 апр. 2021 г.
 *  Author: Shamil Gusseynov
 */

#include "gpio.h"
#include "main.h"

unsigned long set_speedr(__IO uint32_t* dest, int NumPin, Speed speed)
{
    volatile uint32_t new_value = *(dest);
    new_value &= ~(((unsigned long)s_full) << (NumPin * s_shift));
    new_value |= ((unsigned long)speed) << (NumPin * s_shift);
    *dest = new_value;
    return new_value;
};

unsigned long set_otyper(__IO uint32_t* dest, int NumPin, OTyper typer)
{
    volatile uint32_t new_value = *(dest);
    new_value &= ~(((unsigned long)t_full) << (NumPin * t_shift));
    new_value |= ((unsigned long)typer) << (NumPin * t_shift);
    *dest = new_value;
    return new_value;
};

unsigned long set_pupdr(__IO uint32_t *dest, int NumPin, Pupdr pull)
{
    volatile uint32_t new_value = *(dest);
    new_value &= ~(((unsigned long)p_full) << (NumPin * p_shift));
    new_value |= ((unsigned long)pull) << (NumPin * p_shift);
    *dest = new_value;
    return new_value;
};

unsigned long set_af(__IO uint32_t* dest, int NumPin, AF af)
{
    volatile uint32_t new_value = *(dest);
    unsigned int newNumPin = NumPin < 8 ? NumPin : NumPin - 8;
    new_value &= ~(((unsigned long)af_full) << (newNumPin * af_shift));
    new_value |= ((unsigned long)af) << (newNumPin * af_shift);
    *dest = new_value;
    return new_value;
};

unsigned long set_moder(__IO uint32_t* dest, int NumPin, Moder mod)
{
    volatile uint32_t new_value = *(dest);
    new_value &= ~(((unsigned long)m_full) << (NumPin * m_shift));
    new_value |= ((unsigned long)mod) << (NumPin * m_shift);
    *dest = new_value;
    return new_value;
};

void InitGPio(
    GPIO_TypeDef *port,
    unsigned int NumPin,
    Moder v_moder,
    OTyper v_type,
    Speed v_speed,
    Pupdr v_pupdr, // Определяет подтягивающие регистры
    AF v_af)
{
    if (port == GPIOA)
    	ENABLE_GPORT_A;
    if (port == GPIOB)
    	ENABLE_GPORT_B;
    if (port == GPIOC)
    	ENABLE_GPORT_C;
    if (port == GPIOD)
    	ENABLE_GPORT_D;
    if (port == GPIOE)
    	ENABLE_GPORT_E;

    set_speedr(&port->OSPEEDR, NumPin, v_speed);
    set_otyper(&port->OTYPER, NumPin, v_type);
    set_pupdr(&port->PUPDR, NumPin, v_pupdr);
    if(v_af!=af0)
    	set_af(NumPin<8?&port->AFR[0]:&port->AFR[1], NumPin, v_af);
    set_moder(&port->MODER, NumPin, v_moder);
}

void PortSet (GPIO_TypeDef* port, uint8_t NumPin) {
  port->BSRR = 1 << NumPin;
}

void PortReset (GPIO_TypeDef* port, uint8_t NumPin) {
  port->BSRR = 1 << (NumPin + 16);
}

void PortToggle (GPIO_TypeDef* port, uint8_t NumPin) {
    port->ODR ^= (0x1UL<<NumPin);
}
