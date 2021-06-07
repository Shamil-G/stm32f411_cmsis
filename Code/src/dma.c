#include "main.h"

void dma1_streamDisable(){
//  DMA1_Stream3->CR &= ~DMA_SxCR_EN;
  DMA1_Stream4->CR &= ~DMA_SxCR_EN;
}

void dma1_clear_error(){
  WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CTCIF4);
  WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CTEIF4);
  WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CFEIF4);
  WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CHTIF4);
}

// SPI2_TX -> DMA1, Channel 0, Stream 4
void DMA1_Stream4_IRQHandler(void)
{
  if( READ_BIT(DMA1->HISR, DMA_HISR_TCIF4) != 0 )
  {
    //Clear Channel 4 interrupt flag
    WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CTCIF4);
    __NOP();  }
  else{
	dma1_clear_error();
      __NOP();
  }
}

// Подключаем DMA1 к SPI2
void spi2_dma_enable(){
#ifdef USE_DMA
  dma1_clear_error();
  // SET_BIT(SPI2->CR2, SPI_CR2_RXDMAEN);
  SET_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);

  //Enable Transfer complete interrupt Channel4
  SET_BIT(DMA1_Stream4->CR, DMA_SxCR_TCIE);
  //Enable Transfer error interrupt Channel4
  SET_BIT(DMA1_Stream4->CR, DMA_SxCR_TEIE);

  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
#endif
}

void dma_init(SPI_TypeDef *spi){
  //DMA controller clock enable
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  // Установим адрес порта SPI куда DMA будет перекладывать данные
  DMA1_Stream4->PAR = (uint32_t)&spi->DR;
  //Выставим передачу из памяти в порт SPI
  DMA1_Stream4->CR = 0UL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC;

  // Прерывание, кажется есть лишние, может все
  DMA1_Stream4->CR |= DMA_SxCR_TCIE | //Transfer complete interrupt enable
		      DMA_SxCR_TEIE | //TEIE: Transfer error interrupt enable
		      DMA_SxCR_DMEIE; //Direct mode error interrupt enable

  // Очищаем потенциальные прерывания
  DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
}

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
*/
