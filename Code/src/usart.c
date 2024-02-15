/*
 * usart.c
 *
 *  Created on: Feb 11, 2024
 *      Author: sguss
 */

#define STM32F411
#include "usart.h"
#include "gpio.h"
#include "main.h"
#include "dma.h"

uint8_t fl_tx, fl_rx;
volatile uint16_t usart_counter=0;
volatile uint32_t usart_buffer[USART_SIZE_BUFFER];

void init_usart1(){
	  InitGPio( USART1_TX_PORT,
			  	USART1_TX_PIN,
				alternateF,   	//MODER:10
				veryHigh, 		//SPEEDR
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Подтяжка pull-up внешняя, изначально порт на земле
				af7); //AF8 for USART6
	  InitGPio( USART1_RX_PORT,
			  	USART1_RX_PIN,
				alternateF,   	//MODER:10
				veryHigh, 		//SPEEDR
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Подтяжка pull-up внешняя, изначально порт на земле
				USART1_af); //AF8 for USART6
	  // USART2 enabled from APB1ENR for STM32F411
	  // USART1 and USART6 enabled from APB2ENR for STM32F411
	  SET_BIT(RCC->APB2ENR,RCC_APB2ENR_USART1EN);

	  // INIT SPEED:
	  // https://www.youtube.com/watch?v=-icZ4Zv_qB4&list=PLu9BJ8Y5m4bSrp6WGYSAlRBp6NowvChqn&index=6
	  // 115kbit
	  // baud = 115200 = Fclk1/ ( 16 * USARTDIV ) => USARTDIV=Fpclk2/(115000*16) = 54,0
	  // Then DIV_MANTISSA = 54
	  // 	  DIV_FRACTIOIN = 0.0 * 16 = 0  - nearest real number
	  MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction_Msk, 0 << USART_BRR_DIV_Fraction_Pos);
	  MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa_Msk, 54 << USART_BRR_DIV_Mantissa_Pos);
	  // CLEAN CR1
	  USART1->CR1=0x00L;
	  USART1->GTPR=0L;
	  USART1->CR2=0L;
	  USART1->CR3=0L;
	  // 0: 1 Start bit, 8 Data bits, n Stop bit
	  // CLEAR_BIT(USART1->CR1, USART1_CR1_M);
	  // Enable DMA transmitter, receiver
	  USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	  // ENABLED USART1
	  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	  // Прерывание по приему данных
	  // USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_IDLEIE;
	  // NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void){
	if(READ_BIT(USART1->SR, USART_SR_RXNE)){
		usart_buffer[usart_counter] = USART1->DR;
		usart_counter++;
	}
	if(READ_BIT(USART1->SR, USART_SR_IDLE)){
		USART1->DR;
		usart_counter=0;
	}

}

//----------------------------------------------------------
// For Rx
void DMA2_Stream2_IRQHandler(void)
{
  // Stream x transfer complete interrupt flag - TCIF
  if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF2) == (DMA_LISR_TCIF2))
  {
	// Stream x clear transfer complete interrupt flag
	// Writing 1 to this bit clears the corresponding
	// TCIFx flag in the DMA_LISR register
    WRITE_REG(DMA2->LIFCR, DMA_LIFCR_CTCIF2);
    fl_tx = 1;
  }
  else
	  // Stream x transfer error interrupt flag - TEIF
	  if(READ_BIT(DMA2->LISR, DMA_LISR_TEIF2) == (DMA_LISR_TEIF2))
	  {
		//Disable DMA channels
		CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
		DMA2->LIFCR |=
						DMA_LIFCR_CHTIF2 | // Stream x clear half transfer interrupt flag
						DMA_LIFCR_CTEIF2 | // Stream x clear transfer error interrupt flag
						DMA_LIFCR_CDMEIF2| // Stream x clear direct mode error interrupt flag
						DMA_LIFCR_CFEIF2;  // Stream x clear FIFO error interrupt flag
	  }
}
//----------------------------------------------------------
// For Tx
void DMA2_Stream7_IRQHandler(void)
{
  if(READ_BIT(DMA2->HISR, DMA_HISR_TCIF7) == (DMA_HISR_TCIF7))
  {
    WRITE_REG(DMA2->HIFCR, DMA_HIFCR_CTCIF7);
    fl_rx = 1;
  }
  else
	  if(READ_BIT(DMA2->HISR, DMA_HISR_TEIF7) == (DMA_HISR_TEIF7))
	  {
		// Disable DMA channels
		CLEAR_BIT(DMA2_Stream7->CR, DMA_SxCR_EN);
		// Очищаем потенциальные прерывания
		DMA2->HIFCR |= 	DMA_HIFCR_CHTIF7 | // Stream x clear half transfer interrupt flag
						DMA_HIFCR_CTEIF7 | // Stream x clear transfer error interrupt flag
						DMA_HIFCR_CDMEIF7| // Stream x clear direct mode error interrupt flag
						DMA_HIFCR_CFEIF7;  // Stream x clear FIFO error interrupt flag
	  }
}

void dma_usart1_init_(){
  //DMA controller clock enable

//  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
//  uint32_t tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
//  (void)tmpreg;
  // Page 170 RM0383:
  // USART1: Channel_4
  // Rx USART1: DMA2_Stream2_IRQn or DMA2_Stream5_IRQn interrupt
  // Tx USART1: DMA2_Stream7_IRQn interrupt
  // USART6: Channel_5
  // Rx USART6: DMA2_Stream1_IRQn or DMA2_Stream2_IRQn interrupt
  // Tx USART6: DMA2_Stream6_IRQn or DMA2_Stream7_IRQn interrupt
  // USART2: Channel_6 for Stream7 and Channel_4 for Stream5 and Stream6
  // Rx USART2: DMA1_Stream5_IRQn or DMA2_Stream7_IRQn interrupt
  // Tx USART2: DMA2_Stream6_IRQn
//  DMA2_Stream2->CR = 0x0UL;
/*
  DMA2_Stream2->CR =
		     (DMA_SxCR_CHSEL_2) << DMA_SxCR_CHSEL_Pos // Выбор 4 канала
		    |0      << DMA_SxCR_CT_Pos          // Текущий буфер
		    |0      << DMA_SxCR_DBM_Pos         // Двойной буфер
		    |(0x03) << DMA_SxCR_PL_Pos          // Приоритет
		    |0      << DMA_SxCR_PINCOS_Pos      // Инкремент адреса периферии
		    |(0x00) << DMA_SxCR_MSIZE_Pos       // Размер данных в памяти 8 бит
		    |(0x00) << DMA_SxCR_PSIZE_Pos       // Размер данных в периферии 8 бит
		    |1      << DMA_SxCR_MINC_Pos        // Инкремент адреса памяти
		    |0      << DMA_SxCR_PINC_Pos        // Инкремент адреса периферии
		    |0      << DMA_SxCR_CIRC_Pos        // Циркуляционный режим
		    |(0x01) << DMA_SxCR_DIR_Pos         // Направление данных из памяти в периферию
		    |0      << DMA_SxCR_PFCTRL_Pos      // DMA управляет процессом
		    |0      << DMA_SxCR_TCIE_Pos        // Прерывание по окончанию передачи
		    |0      << DMA_SxCR_HTIE_Pos        // Прерывание по половине передачи
		    |0      << DMA_SxCR_TEIE_Pos        // Прерывание по ошибке передачи
		    |0      << DMA_SxCR_DMEIE_Pos       // Прерывание по
		    |1      << DMA_SxCR_EN_Pos;         // Активация DMA
*/
  dma_init(DMA2, DMA2_Stream2);  // Rx
  dma_init(DMA2, DMA2_Stream7);  // Tx

  SET_BIT(DMA2_Stream2->CR, DMA_SxCR_CHSEL_2); // Выбор 4 канала
  SET_BIT(DMA2_Stream7->CR, DMA_SxCR_CHSEL_2); // Выбор 4 канала

  // SET_BIT(DMA2_Stream2->CR, DMA_SxCR_MSIZE); // Размер данных в памяти 8 бит. По умолчанию 8 бит
  // SET_BIT(DMA2_Stream2->CR, DMA_SxCR_PSIZE); // Размер данных в периферии. По умолчанию 8 бит

  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void usart1_dma_tx(uint8_t * txData, uint16_t buff_size){
    DMA2_Stream2->CR |= DMA_SxCR_DIR_0 |   // Направление данных из памяти в периферию
						DMA_SxCR_MINC;     // Инкремент памяти включен

    DMA2_Stream2->PAR  = (uint32_t)&(USART1->DR);                                 // Set address periphery
	DMA2_Stream2->M0AR = (uint32_t)txData;                                        // Set address buf
	DMA2_Stream2->NDTR = buff_size;                                                // Set len

	DMA2_Stream2->CR  |= DMA_SxCR_EN;                                             // Enable DMA
}

void usart1_dma_rx(uint8_t * rxData, uint16_t buff_size){
	CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_DIR_0);

    DMA2_Stream2->CR  |= DMA_SxCR_MINC;    // Инкремент памяти включен

    DMA2_Stream2->PAR  = (uint32_t)&(USART1->DR);                                 // Set address periphery
	DMA2_Stream2->M0AR = (uint32_t)rxData;                                        // Set address buf
	DMA2_Stream2->NDTR = buff_size;                                                // Set len

	DMA2_Stream2->CR  |= DMA_SxCR_EN;                                             // Enable DMA
}