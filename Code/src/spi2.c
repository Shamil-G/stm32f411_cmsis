/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "spi.h"

//#define SPI_MOSI_Port	GPIOB
//#define SPI_MOSI_Pin	15
//#define SPI_SCK_Port	GPIOB
//#define SPI_SCK_Pin	13
//#define SPI_MISO_Port	GPIOB
//#define SPI_MISO_Pin	14

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#define READY_DATA_REGISTR 	(SPI2->SR & SPI_SR_TXE)

uint32_t rx_buf=0UL;
uint16_t spi_ticks=0;
uint8_t	 spi_status;

void spi2_clear_rx(){
	if(	SPI2->SR & SPI_SR_OVR ||
		SPI2->SR & SPI_SR_RXNE
	) {
		rx_buf = 0;
		rx_buf = SPI2->DR;
		rx_buf = SPI2->SR;
	}
}

void spi2_gpio_init(){
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
}
void spi2_clear_flags(){
  // For SPI2 Tx
  DMA1->HIFCR |=  DMA_HIFCR_CTCIF4 |
		  	  	  DMA_HIFCR_CHTIF4 | // Stream x clear half transfer interrupt flag
				  DMA_HIFCR_CTEIF4 | // Stream x clear transfer error interrupt flag
				  DMA_HIFCR_CDMEIF4| // Stream x clear direct mode error interrupt flag
				  DMA_HIFCR_CFEIF4;  // Stream x clear FIFO error interrupt flag
}

void spi_init(SPI_TypeDef *spi){
  spi2_gpio_init();
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
//	SPI_CR1_BR		| // 50/2=25 MHz - default
	//	SPI_CR1_BR_1 | // // 50/4=12,5 MHz - default
	SPI_CR1_MSTR; // Master Configuration

// Включаем SPI
//Port SPI2 have to use DMA
#ifdef USE_SPI_DMA
  spi->CR2 |= SPI_CR2_TXDMAEN;
#endif
  spi->CR1 |= SPI_CR1_SPE;
#ifndef USE_SPI_DMA
  NVIC_EnableIRQ(SPI2_IRQn);
#endif
}

void dma_spi2_init(){
  dma_init(DMA1, DMA1_Stream4);  // SPI DMA Tx

  DMA1_Stream4->CR &= ~DMA_SxCR_CHSEL_Msk; // Выбор 0 канала

  DMA1_Stream4->CR &= ~DMA_SxCR_PINC |   // Peripheral address pointer is fixed
	  		~DMA_SxCR_CIRC |   // Circular mode disabled
	  		~DMA_SxCR_MSIZE |  // Memory data size = 8bit 
	  		~DMA_SxCR_PSIZE;   // Peripheral data size = 8bit

//  DMA1_Stream4->CR |= DMA_SxCR_PSIZE_0; // Peripheral data size

  // Установим адрес порта SPI2 куда DMA будет перекладывать данные
  DMA1_Stream4->PAR = (uint32_t)&SPI2->DR;
//  DMA1_Stream4->FCR = 0x21L;

//  NVIC_EnableIRQ(DMA1_Stream3_IRQn);  // Rx
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);  // Tx
}

uint8_t spi2_dma_tx(uint8_t *data, uint16_t len, uint32_t Timeout){
  spi_ticks=0;
  spi_status=1;
  // Отключаем DMA канал
  DMA1_Stream4->CR &= ~DMA_SxCR_EN;
  while( (DMA1_Stream4->CR & DMA_SxCR_EN) && (spi_ticks<Timeout) );
  // While Tx buffer not empty
  while(!(SPI2->SR & SPI_SR_TXE) && (spi_ticks<Timeout) );
  // while SPI not busy
  while((SPI2->SR & SPI_SR_BSY) && (spi_ticks<Timeout));

//  spi2_clear_flags();	// Clear in HISR registr

  // Заносим адрес памяти откуда мы будем передавать данные
  DMA1_Stream4->M0AR = (uint32_t)data;

  // Start Tx
  DMA1_Stream4->CR |= DMA_SxCR_DIR_0 |   // Направление данных из памяти в периферию
					  DMA_SxCR_MINC;     // Инкремент памяти включен

  DMA1_Stream4->NDTR = len;

  // Количество передаваемых данных
  DMA1_Stream4->CR |= DMA_SxCR_EN;
  if(spi_ticks<Timeout)
		spi_status=0;
  return spi_status;
}


uint8_t spi2_tx(uint8_t* pData, uint16_t Size, uint32_t Timeout){

	// While Tx buffer not empty
    spi_ticks=0;
    spi_status=1;
	while(!(SPI2->SR & SPI_SR_TXE) && spi_ticks<Timeout);

    // Clear in HISR registr
	while (Size > 0U && spi_ticks<Timeout)
	{
		if(	SPI2->SR & SPI_SR_OVR ||
			SPI2->SR & SPI_SR_RXNE
		) {
			rx_buf = SPI2->DR;
			rx_buf = SPI2->SR;
		}
		if(SPI2->SR & SPI_SR_TXE)  // Ожидаем флага TXE - что регистр DR готов для приема данных
		{
		  *((__IO uint8_t *)&SPI2->DR) = (*pData);
		  pData += sizeof(uint8_t);
		  Size--;
		}
	}
	if(spi_ticks<Timeout)
		spi_status=0;
	return spi_status;
}

uint8_t SPI2_WriteData(uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
//	#if (USE_SPI_CRC != 0U)
//	  /* Reset CRC Calculation */
//	  if (HSPI.Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//	  {
//	    SPI_RESET_CRC(hspi);
//	  }
//	#endif /* USE_SPI_CRC */

#ifdef USE_SPI_DMA
	spi_status = spi2_dma_tx(pData, Size, Timeout);
#endif
#ifndef	USE_SPI_DMA
	spi_status = spi2_tx(pData, Size, Timeout);
#endif
//	spi_status = spi2_dma_tx_2(pData, Size);
//	status = spi2_dma_tx_2(&bb, 16);
//	spi_status = spi2_dma_tx_3(pData, Size, Timeout);

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


//	  }
	//*/
//	  spi2_clear_rx();
  return spi_status;
}

// SPI2_TX -> DMA1, Channel 0, Stream 4
void DMA1_Stream4_IRQHandler(void)
{
  // Необходимо прочитать флаг SPI2->SR &  SPI2_SR_OVR
  if( READ_BIT(DMA1->HISR, DMA_HISR_TCIF4) != 0 )
  {
    //Clear Channel 4 interrupt flag
    DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
    // Clear SPI_SR_OVR
    if(SPI2->SR & SPI_SR_OVR){
		rx_buf = 0;
		rx_buf = SPI2->DR;
		rx_buf = SPI2->SR;    }
  }
  if(DMA1->HISR & (DMA_HISR_HTIF4 | // Stream x clear half transfer interrupt flag
			DMA_HISR_TEIF4 | // Stream x clear transfer error interrupt flag
			DMA_HISR_DMEIF4| // Stream x clear direct mode error interrupt flag
			DMA_HISR_FEIF4) )
  {
	DMA1->HIFCR |= 	DMA_HIFCR_CHTIF4 | // Stream x clear half transfer interrupt flag
			DMA_HIFCR_CTEIF4 | // Stream x clear transfer error interrupt flag
			DMA_HIFCR_CDMEIF4| // Stream x clear direct mode error interrupt flag
			DMA_HIFCR_CFEIF4;  // Stream x clear FIFO error interrupt flag
  }
}


void SPI2_IRQHandler(void){
    __NOP();
}

