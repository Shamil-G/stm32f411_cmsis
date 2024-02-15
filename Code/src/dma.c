#include "main.h"


/*
void dma_reinit(SPI_TypeDef *spi){
  // Подождем окончания передачи данных
  if ( MstSpiDmaStreamTX->CR & DMA_SxCR_EN ){
      MstSpiDmaStreamTX->CR  &= ~DMA_SxCR_EN; //Отключаем DMA, если оно включено
      while(MstSpiDmaStreamTX->CR & DMA_SxCR_EN);
  }

  // Загружаем адрес периферийного регистра SPI - куда мы будем писать
  // Это потом выставим в битах DIR
  MstSpiDmaStreamTX->PAR = (uint32_t)(&spi->DR);

  //Настройка канала DMA
  //приоритет низкий
  MstSpiDmaStreamTX->CR &= ~DMA_SxCR_PL;
  //разрядность данных в памяти 8 бит
  MstSpiDmaStreamTX->CR &= ~DMA_SxCR_MSIZE;
  //разрядность регистра данных SPI 16 бит
  MstSpiDmaStreamTX->CR &= ~DMA_SxCR_PSIZE;
  MstSpiDmaStreamTX->CR |=  DMA_SxCR_PSIZE_0;
  //Включить инкремент адреса памяти
  MstSpiDmaStreamTX->CR |= DMA_SxCR_MINC;
  //Инкремент адреса периферии отключен
  MstSpiDmaStreamTX->CR &= ~DMA_SxCR_PINC;
  //кольцевой режим отключен
  MstSpiDmaStreamTX->CR &= ~DMA_SxCR_CIRC;

  //Передача данных из памяти в периферию
  MstSpiDmaStreamTX->CR &=  ~DMA_SxCR_DIR;
  MstSpiDmaStreamTX->CR |=  DMA_SxCR_DIR_0;
}
//*/

void dma_init(DMA_TypeDef * dma, DMA_Stream_TypeDef *dma_channel){
  //DMA controller clock enable
	if(dma==DMA1)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	if(dma==DMA2)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  // Выставим передачу из памяти в порт SPI
  // dma_channel->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC;

  // Прерывание, кажется есть лишние, может все
  dma_channel->CR = 0UL |
		  	  	  	DMA_SxCR_TCIE | // Прерывание: Передача данных завершена
		  	  	  	DMA_SxCR_HTIE | // Прерывание: Разрешение прерывания половинной передачи
		      	  	DMA_SxCR_TEIE | // Прерывание: Передача c ошибкой
					DMA_SxCR_DMEIE; // Direct mode error interrupt enable
}

