/*
 * usart.c
 *
 *  Created on: Feb 11, 2024
 *      Author: sguss
 */

#define STM32F411

#include "string.h"
#include "usart.h"

volatile uint16_t usart_ticks;
volatile uint16_t usart_counter=0;
volatile uint8_t  usart_status;
volatile uint8_t  usart_rx_ovr;
volatile uint8_t  usart_buff_pos;

volatile uint8_t  usart_buffer_rx[USART_SIZE_BUFFER];

void usart1_gpio_init() {
	InitGPio(USART1_TX_PORT,
		USART1_TX_PIN,
		alternateF,   	//MODER:10
		veryHigh, 		//SPEEDR
		0,
		// Pull-UP, если питание идет с STM32
		// Pull-down, если питание идет с енкодера
		noPull, // Подтяжка pull-up внешняя, изначально порт на земле
		af7); //AF8 for USART6
	InitGPio(USART1_RX_PORT,
		USART1_RX_PIN,
		alternateF,   	//MODER:10
		veryHigh, 		//SPEEDR
		0,
		// Pull-UP, если питание идет с STM32
		// Pull-down, если питание идет с енкодера
		noPull, // Подтяжка pull-up внешняя, изначально порт на земле
		USART1_af); //AF8 for USART6
}

void dma_usart1_init_(){
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
	//DMA controller clock enable

	dma_init(DMA2, DMA2_Stream2);  // Rx
	dma_init(DMA2, DMA2_Stream7);  // Tx

	// Rx
	SET_BIT(DMA2_Stream2->CR, DMA_SxCR_CHSEL_2);// Выбор 4 канала
	DMA2_Stream2->PAR = (uint32_t) & (USART1->DR);	// Set address periphery
	DMA2_Stream2->CR &= ~DMA_SxCR_DIR_0;		// Направление данных из периферии в память
	DMA2_Stream2->CR |= DMA_SxCR_MINC;			// Инкремент памяти включен
	// Tx
	SET_BIT(DMA2_Stream7->CR, DMA_SxCR_CHSEL_2); // Выбор 4 канала
	DMA2_Stream7->PAR = (uint32_t) & (USART1->DR);	// Set address periphery
	DMA2_Stream7->CR |=	DMA_SxCR_DIR_0	|	// Направление данных из памяти в периферию
						DMA_SxCR_MINC;		// Инкремент памяти включен
	// Enable DMA transmitter for USART

	// SET_BIT(DMA2_Stream2->CR, DMA_SxCR_MSIZE); // Размер данных в памяти 8 бит. По умолчанию 8 бит
	// SET_BIT(DMA2_Stream2->CR, DMA_SxCR_PSIZE); // Размер данных в периферии. По умолчанию 8 бит
#ifdef USE_USART_DMA
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
#endif
}

void usart_init(USART_TypeDef* usart){
	usart1_gpio_init();

	// Включаем тактирование
	if(usart==USART1 || usart == USART6)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	if (usart == USART2)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// INIT SPEED:
	// https://www.youtube.com/watch?v=-icZ4Zv_qB4&list=PLu9BJ8Y5m4bSrp6WGYSAlRBp6NowvChqn&index=6
	// 115kbit
	// baud = 115200 = Fclk1/ ( 16 * USARTDIV ) => USARTDIV=Fpclk2/(115000*16) = 54,0
	// Then DIV_MANTISSA = 54
	// 	  DIV_FRACTIOIN = 0.0 * 16 = 0  - nearest real number
	MODIFY_REG(usart->BRR, USART_BRR_DIV_Fraction_Msk, 0 << USART_BRR_DIV_Fraction_Pos);
	MODIFY_REG(usart->BRR, USART_BRR_DIV_Mantissa_Msk, 54 << USART_BRR_DIV_Mantissa_Pos);
	// CLEAN CR1
	usart->CR1=0x00L;
	usart->GTPR=0L;
	usart->CR2=0L;
	usart->CR3=0L;
	// 0: 1 Start bit, 8 Data bits, n Stop bit
	// CLEAR_BIT(USART1->CR1, USART1_CR1_M);

#ifdef USE_USART_DMA
	dma_usart1_init_();
	usart->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
#endif

//#ifndef USE_USART_DMA
	// Прерывание по приему данных
	usart->CR1 = USART_CR1_RXNEIE | USART_CR1_IDLEIE;
//#endif
	NVIC_EnableIRQ(USART1_IRQn);

	usart->CR1 |= USART_CR1_TE | USART_CR1_RE;
	// ENABLED USART1
	usart->CR1 |= USART_CR1_UE;
}
//---------------------------------------------------------------------------
void usart1_callback_rx(){
	if(usart_buff_pos>=USART_SIZE_BUFFER){
		usart_rx_ovr++;
		usart_buff_pos=0;
	}
	usart_buffer_rx[usart_buff_pos]=USART1->DR;
	usart_counter++;
	usart_buff_pos++;
}
uint8_t usart1_dma_rx(uint8_t* rxData, uint16_t buff_size, uint32_t timeout){
	usart_ticks = 0;
	usart_status = 0;
	// While Rx buffer not empty
	while (!(USART1->SR & USART_SR_RXNE) && (usart_ticks < timeout));

	// При внесении адреса в DMA2_Stream2->M0AR 
	// Или при изменении DMA2_Stream2->NDTR
	// - USART сразу переходит в DMA DISABLE
	DMA2_Stream2->M0AR = (uint32_t)rxData;			// Set address buf
	DMA2_Stream2->NDTR = buff_size;					// Set len

	DMA2_Stream2->CR |= DMA_SxCR_EN;				// Enable DMA
	// Wait end of Receive or timeout
    	// With disabled DMA IRQ
	while(!(DMA2->LISR & DMA_LISR_TCIF2) && (usart_ticks < timeout));

	DMA2->LIFCR |= 	DMA_LIFCR_CTCIF2 | // clear transfer complete interrupt flag
					DMA_LIFCR_CHTIF2 | // clear half transfer interrupt flag
					DMA_LIFCR_CTEIF2 | // clear transfer error interrupt flag
				 	DMA_LIFCR_CDMEIF2 | // clear direct mode error interrupt flag
					DMA_LIFCR_CFEIF2;   // clear FIFO error interrupt flag

	if (usart_ticks >= timeout)
		usart_status = usart_ticks;
	return usart_status;
}
//----------------------------------------------------------------------------
uint8_t usart1_rx(uint8_t * rxData, uint16_t buff_size, uint32_t timeout){
	usart_ticks = 0;
	usart_status = 0;
	usart_rx_ovr = 0;
	usart_counter = 0;
	usart_buff_pos = 0;

	while( (usart_ticks < timeout) && (USART1->SR & USART_SR_RXNE)) {
		if(usart_buff_pos>=buff_size){
			usart_rx_ovr++;
			usart_buff_pos=0;
		}
		*(rxData+usart_buff_pos)=USART1->DR;
		usart_counter++;
		usart_buff_pos++;
	}

	if (usart_ticks >= timeout)
		usart_status = usart_ticks;
	return usart_status;
}
uint8_t usart1_dma_tx(uint8_t * txData, uint16_t buff_size, uint32_t timeout){
	usart_ticks = 0;
	usart_status = 0;
	// While Tx buffer not empty
	while (!(USART1->SR & USART_SR_TXE) && (usart_ticks < timeout));

    DMA2_Stream7->M0AR = (uint32_t)txData;			// Set address buf
    DMA2_Stream7->NDTR = buff_size;					// Set len
    // После передачи данных в канал, DMA автоматически становится DISABLE
    DMA2_Stream7->CR  |= DMA_SxCR_EN;				// Enable DMA

	while ( (usart_ticks < timeout) && !READ_BIT(DMA2->HISR, DMA_HISR_TCIF7) );
	while ( (usart_ticks < timeout) && !(USART1->SR & USART_SR_TC) );        //Wait transmit all data
	// Flag interrupt clear register
	DMA2->HIFCR |= 	DMA_HIFCR_CTCIF7 | // clear transfer complete interrupt flag
					DMA_HIFCR_CHTIF7 | // clear half transfer interrupt flag
					DMA_HIFCR_CTEIF7 | // clear transfer error interrupt flag
				 	DMA_HIFCR_CDMEIF7 |// clear direct mode error interrupt flag
					DMA_HIFCR_CFEIF7;  // clear FIFO error interrupt flag
	if (usart_ticks >= timeout)
		usart_status = usart_ticks;
	return usart_status;
}
uint8_t usart1_tx(uint8_t * txData, uint16_t buff_size, uint32_t timeout){
	usart_ticks = 0;
	usart_status = 0;
	// While Tx buffer not empty
	while (!(USART1->SR & USART_SR_TXE) && (usart_ticks < timeout));

	for(uint8_t i=0; i<buff_size && usart_ticks < timeout;){
		if(USART1->SR & USART_SR_TXE){
			USART1->DR=*(txData+i);
			i++;
		}
	}
	// Have to check it - needed this or no?
	// USART_SR_TC - frame transfer complete. Size frame data may different from BYTE
	while ( (usart_ticks < timeout) && !(USART1->SR & USART_SR_TC) );
	// Clear flag "transfer complete"
	USART1->SR &= ~USART_SR_TC;

	if (usart_ticks >= timeout)
		usart_status = usart_ticks;
	return usart_status;
}
//----------------------------------------------------------
// For Rx
void DMA2_Stream2_IRQHandler(void)
{
	// Stream x RECEIVE complete interrupt flag - TCIF
	if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF2) == (DMA_LISR_TCIF2))
	{
//		USART1->CR3 &= USART_CR3_DMAR;
		// Stream x clear transfer complete interrupt flag
		// Writing 1 to this bit clears the corresponding
		// TCIFx flag in the DMA_LISR register
		WRITE_REG(DMA2->LIFCR, DMA_LIFCR_CTCIF2);
//		USART1->CR3 |= USART_CR3_DMAR;
		usart1_callback_rx();
	}
	// Stream x transfer error interrupt flag - TEIF
	if(READ_BIT(DMA2->LISR, DMA_LISR_TEIF2) == (DMA_LISR_TEIF2))
	{
		//Disable DMA channels
		CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
		DMA2->LIFCR |=	DMA_LIFCR_CHTIF2 | // Stream x clear half transfer interrupt flag
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
	}
	if( DMA2->HISR & (DMA_HISR_TEIF7 | DMA_HISR_HTIF7 |
		DMA_HISR_DMEIF7 | DMA_HISR_FEIF7) )
	{
		// Очищаем потенциальные прерывания
		DMA2->HIFCR |= 	DMA_HIFCR_CHTIF7 | // Stream x clear half transfer interrupt flag
						DMA_HIFCR_CTEIF7 | // Stream x clear transfer error interrupt flag
						DMA_HIFCR_CDMEIF7| // Stream x clear direct mode error interrupt flag
						DMA_HIFCR_CFEIF7;  // Stream x clear FIFO error interrupt flag
	}
}
//----------------------------------------------------------
// Without DMA
void USART1_IRQHandler(void) {
	if (READ_BIT(USART1->SR, USART_SR_RXNE)) {
		usart_status = usart1_rx((uint8_t*)&usart_buffer_rx, sizeof(usart_buffer_rx), 10);
	}
	if (READ_BIT(USART1->SR, USART_SR_IDLE)) {
		// Reset flag IDLE
		USART1->DR;
		usart_status = usart_counter;
		usart_counter = 0;
	}
}

