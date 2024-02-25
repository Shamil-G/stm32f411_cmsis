#pragma once

#include "main.h"

#define SPI_MOSI_Port	GPIOB
#define SPI_MOSI_Pin	15
#define SPI_SCK_Port	GPIOB
#define SPI_SCK_Pin	13
#define SPI_MISO_Port	GPIOB
#define SPI_MISO_Pin	14

#ifndef USE_SPI_ILI9341

#define SPI_NSS_Port	GPIOB
#define SPI_NSS_Pin		12

#endif

#ifdef USE_SPI_DMA
	// STM32F411, Page 170 of RM0383:
	// SPI1: DMA2
	// Channel_3
	// Rx SPI1: DMA2_Stream0_IRQn or DMA2_Stream2_IRQn interrupt
	// Tx SPI1: DMA2_Stream3_IRQn or DMA2_Stream5_IRQn interrupt
	// Channel_2
	// Tx SPI1: DMA2_Stream2_IRQn

	// SPI2: DMA1
	// Channel_0
	// Rx SPI2: DMA1_Stream3_IRQn interrupt
	// Tx SPI2: DMA1_Stream4_IRQn interrupt

	// SPI3: DMA1
	// Channel_0
	// Rx SPI3: DMA1_Stream0_IRQn interrupt
	// Tx SPI3: DMA1_Stream5_IRQn interrupt

	// SPI4: DMA2
	// Channel_4
	// Rx SPI4: DMA2_Stream0_IRQn interrupt
	// Tx SPI4: DMA2_Stream1_IRQn interrupt
	// Channel_5
	// Tx SPI4: DMA2_Stream4_IRQn interrupt

	// SPI5: DMA2
	// Channel_2
	// Rx SPI5: DMA2_Stream3_IRQn interrupt
	// Tx SPI5: DMA2_Stream4_IRQn interrupt
	// Channel_7
	// Rx SPI5: DMA2_Stream5_IRQn interrupt
	// Tx SPI5: DMA2_Stream6_IRQn interrupt
	// Channel_5
	// Tx SPI5: DMA2_Stream5_IRQn interrupt
void dma_spi2_init();
int  spi2_dma_ready();
void spi2_dma_enable();

#endif

//#define SPI_CLEAR_OVRFLAG

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define spi2_enable() 	SET_BIT(SPI2->CR1, SPI_CR1_SPE)
#define spi2_ready()	READ_BIT(SPI2->CR1, SPI_CR1_SPE)
#define spi2_end_operation()	while(SPI2->SR & SPI_SR_BSY)
#define ERROR_TIMEOUT 100

void spi_init(SPI_TypeDef *spi);
void spi2_gpio_init(void);
void ili9341_gpio_init(void);
void spi_ili9341_init(void);
uint8_t SPI2_WriteData(uint8_t* pData, uint16_t Size, uint32_t Timeout);


