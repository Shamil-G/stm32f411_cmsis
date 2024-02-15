#pragma once

#ifndef DMA_H_
#define DMA_H_

void dma_spi_init(SPI_TypeDef *spi);

void dma_init(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_channel);

#define dma1_enable()   RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN

#define dma1_streamEnable(dmaStream)  	((DMA_Stream_TypeDef *)dmaStream)->CR |= DMA_SxCR_EN
#define dma_ready(dmaStream) 		((((DMA_Stream_TypeDef *)dmaStream)->CR&DMA_SxCR_EN)==0?0:1)

void dma1_clear_error(void);

#endif
