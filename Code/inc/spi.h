#pragma once

#include "stm32f411xe.h"

#define USE_DMA
//#define SPI_CLEAR_OVRFLAG

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define spi2_enable() 	SET_BIT(SPI2->CR1, SPI_CR1_SPE)
#define spi2_ready()	READ_BIT(SPI2->CR1, SPI_CR1_SPE)
#define spi2_end_operation()	while(SPI2->SR & SPI_SR_BSY)


#define ERROR_TIMEOUT 100


#define spi2_clear_ovrflag()		\
  do{					\
    __IO uint32_t tmpreg_ovr = 0x00U;	\
    tmpreg_ovr = SPI2->DR;		\
    tmpreg_ovr = SPI2->SR;		\
    UNUSED(tmpreg_ovr);			\
  }while(0U)

uint8_t SPI2_WriteData(uint8_t* pData, uint32_t Size, uint32_t Timeout);
