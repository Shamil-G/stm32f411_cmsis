#pragma once

//#include "stm32f4xx.h"
#ifdef USE_SPI

#include "spi_ili9341.h"

void spi_change_mode_transfer(SPI_TypeDef *spi);
void spi_dma_enable(void);
void ili9341_display_init(uint16_t w_size, uint16_t h_size);
void ili9341_primary_tune(void);
void show_ili9341_monitor(void);
void spi_end_send(SPI_TypeDef *spi);


#define spi_master_init()	spi_init(MasterSPI)
#define spi_ili9341_ready()	spi_ready(MasterSPI)
#define spi_master_enable()	spi_enable(MasterSPI)
#define spi_master_disable()	spi_off(MasterSPI)
#define spi_master_wait_tx()	spi_wait_tx(MasterSPI)
#define spi_master_wait_bsy()	spi_wait_bsy(MasterSPI)

#define spi_send(pData, Size) spi_dma_send(MstSpiDmaStreamTX, pData, Size);
#define spi_tx_ready spi_tx_empty(MasterSPI)

#define getRandomNumber	(mainTick && 0x0000ffff)

#endif
