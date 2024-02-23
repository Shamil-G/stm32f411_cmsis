/*
 * I2C.cpp
 *
 *  Created on: Aug 30, 2023
 *      Author: sguss
 */
#include "i2c.h"

uint16_t i2c_ticks;
uint16_t sr_status;
uint8_t	 i2c_status;
uint8_t	 tx_ready;

void i2c_force_reset(){
	RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST);
	i2c_ticks=0;
	while(i2c_ticks<3);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);
}
void i2c_soft_reset(I2C_TypeDef * p_i2c){
	p_i2c->CR1 |= I2C_CR1_SWRST_Msk;
	i2c_ticks=0;
	while((p_i2c->CR1 & I2C_CR1_SWRST_Msk) == 0 && (i2c_ticks < 1000) );
	p_i2c->CR1 &= ~I2C_CR1_SWRST_Msk;
	while((p_i2c->CR1 & I2C_CR1_SWRST_Msk) && (i2c_ticks < 1000));
}
void i2c_stop(I2C_TypeDef * p_i2c){
	p_i2c->CR1 |= I2C_CR1_STOP;

	//	This bit can be used to reinitialize the peripheral after an error or a locked state. As an
	//	example, if the BUSY bit is set and remains locked due to a glitch on the bus, the
	//	SWRST bit can be used to exit from this state
	if(p_i2c->SR2 & I2C_SR2_BUSY)
		i2c_soft_reset(p_i2c);

	CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
	Delay(2);
	SET_BIT(p_i2c->CR1, I2C_CR1_PE);
}

void I2C_gpio_init() {
	//	PortSet (PORT_I2C1_CLK, PIN_I2C1_DATA);
	InitGPio(PORT_I2C1_CLK, PIN_I2C1_CLK, alternateF, openDrain, medium, noPull, AF_I2C1_CLK);
	//	PortSet (PORT_I2C1_DATA, PIN_I2C1_DATA);
	InitGPio(PORT_I2C1_DATA, PIN_I2C1_DATA, alternateF, openDrain, medium, noPull, AF_I2C1_DATA);
}

void init_i2c(I2C_TypeDef * p_i2c){
	I2C_gpio_init();

	//RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	RCC->APB1ENR = RCC_APB1ENR_I2C1EN;

	p_i2c->CR1 = 0;
	p_i2c->CR2 = 0;
	p_i2c->CCR = 0;

	p_i2c->SR1 = 0;
	I2C1->SR2 = 0;
	p_i2c->TRISE = 0;

	p_i2c->OAR1 = 0;
	p_i2c->OAR2 = 0;

	//	I2C1->CR1 |= I2C_CR1_NOSTRETCH_Msk;

	//	50 MHZ APB1
	//I2C_CR2_FREQ_1 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_5;
	p_i2c->CR2 |= I2C_CR2_FREQ_Msk & (50 << I2C_CR2_FREQ_Pos);

	//	Default mode Sm = 100KHZ
	//	Fast Mode 400 KHz
	//	I2C1->CCR = I2C_CCR_FS;
	//
	//  When set bit I2C_CCR_FS
	//  and Tlow/Thigh = 16/19
	//	I2C1->CCR |= I2C_CCR_DUTY_Msk;
	//

	//  Для частоты, заданной в CCR->FS как 100 KHz и APB1 = 50 MHz
	//  (1 / 100 000) = Thigh + TLow = CCR*Tpclk1 + CCR*Tpclk1 = 2CCR*Tpclk1 =>
	//  CCR = Tpclk1 / (100 000 * 2) = 50 000 000 / ( 2 * 100000 ) = 500/2
	//	CCR = 250
	p_i2c->CCR |= 0xFA & I2C_CCR_CCR_Msk;

	//  I2C1->TRISE = (SDA and SCL maxtime / I2C1->I2C_CR2_FREQ) + 1
	//  According to Datasheet on Sm=100 KHz, SDA and SCL maxtime = 1000 ns
	//  I2C1->TRISE = 1000 / 50 + 1 = 21
	//	I2C1->TRISE = (1000 / (I2C_CR2_FREQ_1 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_5)) + 1;
	p_i2c->TRISE = (1000 / (I2C1->CR2 & I2C_CR2_FREQ_Msk)) + 1;

#ifdef USE_I2C_DMA
	// Enable I2C
	dma_i2c1_init();
	p_i2c->CR2 |= I2C_CR2_DMAEN;
#endif

#ifndef USE_I2C_DMA
	NVIC_EnableIRQ(I2C1_IRQn);
#endif

	// Enable I2C
	p_i2c->CR1 |= I2C_CR1_PE;
};

void dma_i2c1_init() {
	dma_init(DMA1, DMA1_Stream0);  // I2C1 DMA Rx
	dma_init(DMA1, DMA1_Stream6);  // I2C1 DMA Tx
	//
	DMA1_Stream0->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC;   // Выбор 1 канала
	// Rx
	//DMA1_Stream0->CR &= ~DMA_SxCR_PINC |   // Peripheral address pointer is fixed
	//					~DMA_SxCR_CIRC |   // Circular mode disabled
	//					~DMA_SxCR_MSIZE|  // Memory data size = 8bit 
	//					~DMA_SxCR_PSIZE;   // Peripheral data size = 8bit
	// Tx
	//DMA1_Stream6->CR &= ~DMA_SxCR_PINC |   // Peripheral address pointer is fixed
	//					~DMA_SxCR_CIRC |   // Circular mode disabled
	//					~DMA_SxCR_MSIZE |  // Memory data size = 8bit 
	//					~DMA_SxCR_PSIZE;   // Peripheral data size = 8bit
	DMA1_Stream6->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_DIR_0 |	// Направление данных из памяти в периферию
						DMA_SxCR_MINC;		// Инкремент памяти включен

	// Установим адрес порта I2C1 откуда DMA будет перекладывать данные в память
	DMA1_Stream0->PAR = (uint32_t)&(I2C1->DR);
	// Установим адрес порта I2C1 куда DMA будет перекладывать данные из памяти
	DMA1_Stream6->PAR = (uint32_t)&(I2C1->DR);
#ifdef USE_DMA_IRQ
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);  // Rx
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);  // Tx
#endif
};
void DMA1_Stream6_IRQHandler(void){
  // Необходимо прочитать флаг SPI2->SR &  SPI2_SR_OVR
  if( READ_BIT(DMA1->HISR, DMA_HISR_TCIF6) != 0 )
  {
	//Clear Channel 4 interrupt flag
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	// Clear SPI_SR_OVR
//	while( READ_BIT(I2C1->SR1, I2C_SR1_BTF) == 0 && (i2c_ticks < 500))

//	I2C1->CR1 |= I2C_CR1_STOP;
//	CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
	tx_ready=tx_ready+1;
  }
  if(DMA1->HISR & (DMA_HISR_HTIF6 | // Stream x clear half transfer interrupt flag
			DMA_HISR_TEIF6 | // Stream x clear transfer error interrupt flag
			DMA_HISR_DMEIF6| // Stream x clear direct mode error interrupt flag
			DMA_HISR_FEIF6) )
  {
	DMA1->HIFCR |= 	DMA_HIFCR_CHTIF6 | // Stream x clear half transfer interrupt flag
			DMA_HIFCR_CTEIF6 | // Stream x clear transfer error interrupt flag
			DMA_HIFCR_CDMEIF6| // Stream x clear direct mode error interrupt flag
			DMA_HIFCR_CFEIF6;  // Stream x clear FIFO error interrupt flag
  }
}

uint8_t i2c_call_device(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t mode, uint32_t timeout_ms){
	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	i2c_ticks = 0;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && (i2c_ticks < timeout_ms) );
	if( i2c_ticks > timeout_ms ) return 0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;
	p_i2c->DR = (addr_device << 1) | mode;
	// Здесь SB сбросится

	//	Wait AF or ADDR
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0) &&	// Address not sent yet
			(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) &&		// Acknowledge Fail
			(i2c_ticks < timeout_ms) );
	if( i2c_ticks >= timeout_ms ) return 0;
//		clearTimer(num_timer);
//			rem_time=remain_time(num_timer);
//	success = READ_BIT(p_i2c->SR1, I2C_SR1_ADDR);
	// СБрос бита ADDR производится чтением SR1, then SR2

	if( p_i2c->SR1 & I2C_SR1_ADDR ){ // Устройство отозвалось
		// Сброс бита ADDR производится чтением SR1, then SR2
		sr_status = p_i2c->SR1 | p_i2c->SR2;
		i2c_ticks = 0;
		while( 	READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 &&
				(i2c_ticks < timeout_ms) );
		if( (i2c_ticks >= timeout_ms) || READ_BIT(p_i2c->SR1, I2C_SR1_AF) )
				return 0;
		return 1;
	}
//	Вызвать функцию дороже чем две команды
	p_i2c->CR1 |= I2C_CR1_STOP;
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return 0;
}

int8_t i2c_Device_Scan(I2C_TypeDef * p_i2c, int8_t addr_device, uint32_t timeout_ms){
	i2c_status = i2c_call_device (p_i2c, addr_device, 0, timeout_ms);
	i2c_stop(p_i2c);
	return i2c_status;
}
//---------------------------------------------------------------------------
uint8_t i2c1_dma_rx(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
	i2c_status=1;
	i2c_ticks = 0;

	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && (i2c_ticks < timeout_ms) );
	if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0; // Acknowledge Failure (AF)
	I2C1->CR1 |= I2C_CR1_ACK; //Enable acknowledge
	//--------------------------------------------------------------
	// Передаем Адрес устройства
	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
//	(void)p_i2c->SR1;
	p_i2c->DR = (addr_device << 1) | 1;
	// Здесь SB сбросится
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0) &&
			(i2c_ticks < timeout_ms) && i2c_status );
	if ( i2c_ticks >= timeout_ms ) i2c_status=0;

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && i2c_status ){ // Устройство отозвалось
		// Сброс бита ADDR производится чтением SR1, then SR2
		sr_status = p_i2c->SR1 | p_i2c->SR2;
	}

	//---- Проверить необходимость
	DMA1_Stream0->M0AR = (uint32_t)data;        //Set address buf
	DMA1_Stream0->NDTR = len;                 	//Set len
	DMA1_Stream0->CR |= DMA_SxCR_EN;            //Enable DMA
	//Wait recive all data

//	while( READ_BIT(p_i2c->SR1,I2C_SR1_RXNE) && (i2c_ticks < timeout_ms) );
	while(!(DMA1->LISR & DMA_LISR_TCIF0) && (i2c_ticks < timeout_ms));
	DMA1->LIFCR |= 	DMA_LIFCR_CTCIF0 | // clear transfer complete interrupt flag
					DMA_LIFCR_CHTIF0 | // clear half transfer interrupt flag
					DMA_LIFCR_CTEIF0 | // clear transfer error interrupt flag
				 	DMA_LIFCR_CDMEIF0 | // clear direct mode error interrupt flag
					DMA_LIFCR_CFEIF0;   // clear FIFO error interrupt flag
	p_i2c->CR1 |= I2C_CR1_STOP;                	//Stop generation
	p_i2c->CR1 &= ~I2C_CR1_ACK;                	//Disable acknowledge
	return 1;
}
uint8_t i2c_read(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	i2c_status=1;
	i2c_ticks = 0;

	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && (i2c_ticks < timeout_ms) );
	if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0; // Acknowledge Failure (AF)

	//--------------------------------------------------------------
	// Передаем Адрес устройства
	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;
	p_i2c->DR = (addr_device << 1) | 1;
	// Здесь SB сбросится
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0) &&
			(i2c_ticks < timeout_ms) && i2c_status );
	if ( i2c_ticks >= timeout_ms ) i2c_status=0;

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && i2c_status ){ // Устройство отозвалось
		// Сброс бита ADDR производится чтением SR1, then SR2
		sr_status = p_i2c->SR1 | p_i2c->SR2;
	}
	//--------------------------------------------------------------
	// Принимаем данные
	for(uint16_t i=0; i<Count && i2c_status; i++){
		if (i < Count - 1) {
			// Для продолжения приема после каждого принятого байта будем отправлять ACK
			SET_BIT(p_i2c->CR1, I2C_CR1_ACK);
		}
		else
			CLEAR_BIT(p_i2c->CR1, I2C_CR1_ACK);

		while( 	READ_BIT(p_i2c->SR1,I2C_SR1_RXNE) == 0 && (i2c_ticks < timeout_ms) );
		if ( i2c_ticks >= timeout_ms ) i2c_status=0;
		*(data+i) = p_i2c->DR;
	}
	//--------------------------------------------------------------
	// Включаем NACK
	SET_BIT(p_i2c->CR1, I2C_CR1_STOP);
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return i2c_status;
}
//-------------------------------------------------------------------------------
uint8_t i2c_write(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	i2c_status=1;
	i2c_ticks = 0;
	if(p_i2c->SR2 & I2C_SR2_BUSY ){
		i2c_stop(p_i2c);
		return 0;
	}
	//CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && (i2c_ticks < timeout_ms) );
	if( i2c_ticks >= timeout_ms ) i2c_status=0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;

	p_i2c->DR = (addr_device << 1) | 0;
	// Здесь SB сбросится
	//	Wait AF or ADDR
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR)) == 0 &&
			(i2c_ticks < timeout_ms) );
	if( (i2c_ticks >= timeout_ms) || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0;

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && i2c_status ){ // Устройство отозвалось
		// Сброс бита ADDR производится чтением SR1, then SR2
		sr_status = (p_i2c->SR1 |	p_i2c->SR2);
		while(	READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && (i2c_ticks < timeout_ms));
	}

//	Write DATA
	if (i2c_status){
		for(uint16_t i=0; i<Count && i2c_status >0; i++){
			p_i2c->DR = *(data+i);
			while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && i2c_status && (i2c_ticks < timeout_ms));
			if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0;
		}
		if(i2c_status){
			while( READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 0 && (i2c_ticks < timeout_ms))
			if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status = 0;
		}
	}
	p_i2c->CR1 |= I2C_CR1_STOP;
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return i2c_status;
}
uint8_t i2c1_dma_tx(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t len,   uint32_t timeout_ms)
{
	i2c_ticks = 0;
	i2c_status = 1;
	tx_ready=0;
	//Wait if bus busy
	if(p_i2c->SR2 & I2C_SR2_BUSY || p_i2c->SR1 & I2C_SR1_AF){
		i2c_stop(p_i2c);
		return 0;
	}

	//CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && (i2c_ticks < timeout_ms) );
	if( i2c_ticks >= timeout_ms )  return 0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;
	p_i2c->DR = (addr_device << 1) | 0;
	// Здесь SB сбросится
	// Сброс бита ADDR производится чтением SR1, then SR2
	sr_status = (p_i2c->SR1 |	p_i2c->SR2);
	i2c_ticks = 0;
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR)) == 0 &&
			(i2c_ticks < timeout_ms) );
	if( (i2c_ticks >= timeout_ms) || READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
		i2c_stop(p_i2c);
		return 0;
	}

	// Сброс бита ADDR производится чтением SR1, then SR2
	sr_status = (p_i2c->SR1 |	p_i2c->SR2);
//		i2c_ticks = 0;
	while(	READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 &&
				(i2c_ticks < timeout_ms));

//	DMA1_Stream6->CR &= ~DMA_SxCR_EN;		//Enable DMA
	DMA1_Stream6->M0AR = (uint32_t)data;	//Set address buf
	DMA1_Stream6->NDTR = len;				//Set len
	DMA1_Stream6->CR |= DMA_SxCR_EN;		//Enable DMA

#ifdef USE_DMA_IRQ
	while(tx_ready==0 && i2c_ticks < timeout_ms);
	if( i2c_ticks >= timeout_ms ) return 0;
#endif

	while ( (i2c_ticks < timeout_ms) && !READ_BIT(DMA1->HISR, DMA_HISR_TCIF6) );
	while ( (i2c_ticks < timeout_ms) && !(p_i2c->SR1 & I2C_SR1_BTF) );        //Wait transmit all data

	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;		//Clear DMA event
//
	p_i2c->CR1 |= I2C_CR1_STOP;				//Stop generation
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);

	if(i2c_ticks >= timeout_ms) return 0;

	return 1;
}
//---------------------------------------------------------------------------
