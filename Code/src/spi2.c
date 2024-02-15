#include "main.h"

//#define SPI_MOSI_Port	GPIOB
//#define SPI_MOSI_Pin	15
//#define SPI_SCK_Port	GPIOB
//#define SPI_SCK_Pin	13
//#define SPI_MISO_Port	GPIOB
//#define SPI_MISO_Pin	14

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#define dmaStreamTx DMA1_Stream4
#define dma4spi	    DMA1
#define READY_DATA_REGISTR 	(SPI2->SR & SPI_SR_TXE)

extern unsigned long durationMs;

void dma_spi2_init(SPI_TypeDef *spi){
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
	
//  DMA2_Stream2->CR = 0x0UL;

  dma_init(DMA1, DMA1_Stream4);  // SPI DMA Tx
  SET_BIT(DMA1_Stream4->CR, DMA_SxCR_CHSEL_2); // Выбор 4 канала

  NVIC_EnableIRQ(DMA1_Stream4_IRQn);	
}

void spi_init(SPI_TypeDef *spi){
  // SCK
  InitGPio( SPI_SCK_Port, SPI_SCK_Pin, alternateF, push_pull, high, noPull, af5);
  // MOSI TX DMA1 Stream4
  InitGPio( SPI_MOSI_Port, SPI_MOSI_Pin, alternateF, push_pull, high, noPull, af5);
  // MISO-PB14 RX DMA1 Stream4
  InitGPio( SPI_MISO_Port, SPI_MISO_Pin, alternateF, push_pull, high, noPull, af5);

#ifndef USE_SPI_ILI9341
  // NSS-PB12
  InitGPio( SPI_NSS_Port, SPI_NSS_Pin, alternateF, push_pull, high, noPull, af5);
#endif

  // Включаем SPI
  if(spi==SPI1)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  if(spi==SPI2)
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
  // Выключаем все биты прерываний для работы с DMA
  spi->CR1 = 0;
  spi->CR2 = 0;
//  SPI1->CR1 = SPI_CR1_BIDIMODE| //1: 1-line bidirectional data mode selected
//	      SPI_CR1_BIDIOE; 	  //1: Output enabled (transmit-only mode)
  spi->CR1 |=
//	SPI_CR1_DFF 	| //1: 16-bit data frame format is selected for transmission/reception
//	SPI_CR1_CPOL_Pos| //0: SCK=1 when idle, else SCK=0
//	SPI_CR1_CPHA	| //1: Second edge is  is the MSBit capture strobe
	SPI_CR1_SSM 	| // 1: NSS Software slave management enabled
	SPI_CR1_SSI 	| // SSI: Internal slave select
	// 0: SPI_CR1_BR_х - означает деление на 2 для f411 серии
	// При использовании SPI2 на APB1ENR это означает 50MHz
	// Baud rate (SPI_CR1_BR_2 | SPI_CR1_BR_1) = Fpclk/128 -> 400KHz
	// Baud rate (SPI_CR1_BR_2 | SPI_CR1_BR_0) = Fpclk/64  -> 800KHz
	// Baud rate (SPI_CR1_BR_2) = Fpclk/32  -> 1.6MHz
//	SPI_CR1_BR_2 |
//	SPI_CR1_BR_1 |
	SPI_CR1_MSTR; // Master Configuration

  // Включаем SPI
  spi->CR1 |= SPI_CR1_SPE;
}

void spi2_dma_tx(uint8_t *data, uint32_t len){
  // Очищаем потенциальные прерывания, иначе не запустится
  // DMA1_Stream4->CR &= ~DMA_SxCR_EN;
  // while(DMA1_Stream4->CR & DMA_SxCR_EN);

  DMA1_Stream4->CR |= DMA_SxCR_DIR_0 |   // Направление данных из памяти в периферию
			DMA_SxCR_MINC;     // Инкремент памяти включен
  // Установим адрес порта SPI куда DMA будет перекладывать данные
  DMA1_Stream4->PAR = (uint32_t)&SPI2->DR;
  // Заносим адрес памяти откуда мы будем передавать данные
  DMA1_Stream4->M0AR = (uint32_t)data;
  // Количество передаваемых данных
  DMA1_Stream4->NDTR = len;
  // По умолчанию стоит 1 байт
  // dmaStreamTx->CR &= ~DMA_SxCR_MSIZE;

  //Сообщаем SPI что надо использовать DMA при передаче
  spi->CR2 |= SPI_CR2_TXDMAEN;

  DMA1_Stream4->CR |= DMA_SxCR_EN;
}

uint8_t SPI2_WriteData(uint8_t* pData, uint32_t Size, uint32_t Timeout)
{
//	#if (USE_SPI_CRC != 0U)
//	  /* Reset CRC Calculation */
//	  if (HSPI.Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//	  {
//	    SPI_RESET_CRC(hspi);
//	  }
//	#endif /* USE_SPI_CRC */

  uint32_t ticks=0;
  if( (SPI2->CR2 & SPI_CR2_TXDMAEN) == 0 ){
	  while (Size > 0U)
	  {
	    if (READY_DATA_REGISTR)  // Ожидаем флага TXE - что регистр DR готов для приема данных
	    {
	      *((__IO uint8_t *)&SPI2->DR) = (*pData);
	      pData += sizeof(uint8_t);
	      Size--;
	      ticks=0;
	    }
	    else
	    {
		Delay(1);

		if(ticks++ > Timeout)
	      // Timeout management
//	      if ( durationMs >=  Timeout )
	      {
	        return ERROR_TIMEOUT;
	      }
	    }
	  }
  }
  else spi2_dma_send(pData, Size);

//	#if (USE_SPI_CRC != 0U)
//	  /* Enable CRC Transmission */
//	  if (HSPI.Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//	  {
//	    SET_BIT(HSPI.Instance->CR1, SPI_CR1_CRCNEXT);
//	  }
//	#endif /* USE_SPI_CRC */
//
	  /* Clear overrun flag in 2 Lines communication mode because received is not read */
	  //*
//	  if (HSPI.Init.Direction == SPI_DIRECTION_2LINES)
//	  {
#ifdef  SPI_CLEAR_OVRFLAG
  spi2_clear_ovrflag();
//	__HAL_SPI_CLEAR_OVRFLAG();
#endif
//	  }
	//*/
  return 0;
}

// SPI2_TX -> DMA1, Channel 0, Stream 4
void DMA1_Stream4_IRQHandler(void)
{
  if( READ_BIT(DMA1->HISR, DMA_HISR_TCIF4) != 0 )
  {
    //Clear Channel 4 interrupt flag
    DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
    __NOP();  
  }
  else{
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	DMA1->HIFCR |= 	DMA_HIFCR_CHTIF4 | // Stream x clear half transfer interrupt flag
			DMA_HIFCR_CTEIF4 | // Stream x clear transfer error interrupt flag
			DMA_HIFCR_CDMEIF4| // Stream x clear direct mode error interrupt flag
			DMA_HIFCR_CFEIF4;  // Stream x clear FIFO error interrupt flag
	  
      __NOP();
  }
}
