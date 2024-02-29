#include "main.h"


void dma_init(DMA_TypeDef * dma, DMA_Stream_TypeDef *dma_channel){
  //DMA controller clock enable
	if(dma==DMA1)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	if(dma==DMA2)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  // Выставим передачу из памяти в порт SPI
  // dma_channel->CR &= ~((uint32_t)DMA_SxCR_EN);
  dma_channel->CR = 0;
  dma_channel->NDTR = 0;
  dma_channel->PAR = 0;
  dma_channel->M0AR = 0;
  dma_channel->M1AR = 0;

  // Прерывание, на которые будем реагировать
  dma_channel->CR |=
		  	  	  	DMA_SxCR_TCIE | // Прерывание: Передача данных завершена
//		  	  	  	DMA_SxCR_HTIE | // Прерывание: Разрешение прерывания половинной передачи
		      	  	DMA_SxCR_TEIE | // Прерывание: Передача c ошибкой
					DMA_SxCR_DMEIE	// Direct mode error interrupt enable
					;
}

