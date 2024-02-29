#pragma once
/*
 * Author: Shamil Gusseynov
*/
void dma_init(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_channel);

#define dma1_streamEnable(dmaStream)  	((DMA_Stream_TypeDef *)dmaStream)->CR |= DMA_SxCR_EN
#define dma_ready(dmaStream) 		((((DMA_Stream_TypeDef *)dmaStream)->CR&DMA_SxCR_EN)==0?0:1)
