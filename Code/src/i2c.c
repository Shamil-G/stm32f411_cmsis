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

void i2c_soft_reset(I2C_TypeDef * p_i2c){
	p_i2c->CR1 |= I2C_CR1_SWRST_Msk;
	i2c_ticks=0;
	while((p_i2c->CR1 & I2C_CR1_SWRST_Msk) == 0 && (i2c_ticks < 1000) );
	p_i2c->CR1 &= ~I2C_CR1_SWRST_Msk;
	while((p_i2c->CR1 & I2C_CR1_SWRST_Msk) && (i2c_ticks < 1000));
}

void i2c_force_reset(){
	RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST);
	i2c_ticks=0;
	while(i2c_ticks<3);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);
}
uint8_t successWaitBTF(I2C_TypeDef * p_i2c, uint32_t timeout_ms){
	i2c_ticks = 0;
	while (READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 0 && (i2c_ticks < timeout_ms));
	if (READ_BIT(p_i2c->SR1, I2C_SR1_AF) || (i2c_ticks >= timeout_ms)) {
		CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
		p_i2c->CR1 |= I2C_CR1_STOP;
		return 0;
	}
	return 1;
}

void i2c_restart(I2C_TypeDef * p_i2c){
	CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
	Delay(2);
	SET_BIT(p_i2c->CR1, I2C_CR1_PE);
	//Delay(2);
}

void i2c_stop(I2C_TypeDef * p_i2c){
	p_i2c->CR1 |= I2C_CR1_STOP;

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

	// Enable I2C
	p_i2c->CR1 |= I2C_CR1_PE_Msk;

#ifdef USE_I2C_DMA
	p_i2c->CR2 |= I2C_CR2_DMAEN;
#endif

#ifndef USE_I2C_DMA
	NVIC_EnableIRQ(I2C1_IRQn);
#endif
};

void dma_i2c1_init() {
	dma_init(DMA1, DMA1_Stream0);  // I2C1 DMA Rx
	dma_init(DMA1, DMA1_Stream6);  // I2C1 DMA Tx
	//
	DMA1_Stream0->CR &= ~DMA_SxCR_CHSEL_0;   // Выбор 1 канала
	DMA1_Stream6->CR &= ~DMA_SxCR_CHSEL_0;   // Выбор 1 канала
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
	DMA2_Stream6->CR |= DMA_SxCR_DIR_0 |	// Направление данных из памяти в периферию
						DMA_SxCR_MINC;		// Инкремент памяти включен

	// Установим адрес порта I2C1 откуда DMA будет перекладывать данные в память
	DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;
	// Установим адрес порта I2C1 куда DMA будет перекладывать данные из памяти
	DMA1_Stream6->PAR = (uint32_t)&I2C1->DR;
#ifdef USE_I2C_DMA
//	NVIC_EnableIRQ(DMA1_Stream0_IRQn);  // Rx
//	NVIC_EnableIRQ(DMA1_Stream6_IRQn);  // Tx
#endif
};
//---------------------------------------------------------------------------
uint8_t i2c1_dma_rx(uint8_t* rxData, uint16_t buff_size, uint32_t timeout) {
	i2c_ticks = 0;
	i2c_status = 1;
	// While Rx buffer not empty
	while (!(I2C1->SR1 & I2C_SR1_RXNE) && (i2c_ticks < timeout));
	if(i2c_ticks >= timeout){
		i2c_stop(I2C1);
		return 0;
	}

	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream0->CR & DMA_SxCR_EN) && (i2c_ticks < timeout));

	DMA1_Stream0->M0AR = (uint32_t)rxData;			// Set address buf
	DMA1_Stream0->NDTR = buff_size;					// Set len

	DMA1_Stream0->CR |= DMA_SxCR_EN;				// Enable DMA

	if (i2c_ticks >= timeout)
		i2c_status = i2c_ticks;
	return i2c_status;
}
uint8_t i2c1_dma_rx_2(I2C_TypeDef* p_i2c, uint8_t addr, uint8_t* data, uint8_t len, uint32_t timeout_ms)
{
	i2c_ticks = 0;
	while (p_i2c->SR2 & I2C_SR2_BUSY && (i2c_ticks < timeout_ms));
	if (p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	p_i2c->CR1 |= I2C_CR1_START;               //Start genera
	while (!(p_i2c->SR1 & I2C_SR1_SB) );         //Wait start condition generated
	p_i2c->CR1 |= I2C_CR1_ACK;                 //Enable acknowledge
	p_i2c->DR = addr;                          //Write slave address
	//Wait send addsess
	while (!(p_i2c->SR1 & I2C_SR1_ADDR) && (i2c_ticks < timeout_ms))
	{
		if (p_i2c->SR1 & I2C_SR1_AF)          //Acknowledge failure
		{
			p_i2c->CR1 |= I2C_CR1_STOP;       //Stop generation
			return 0;
		}
	}
	// Сброс бита ADDR производится чтением SR1, then SR2
	sr_status = p_i2c->SR1 | p_i2c->SR2;
	//---- Прверить необходимость
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;            //Enable DMA
	//----
	DMA1_Stream0->M0AR = (uint32_t)data;        //Set address buf
	DMA1_Stream0->NDTR = len;                 	//Set len
	DMA1_Stream0->CR |= DMA_SxCR_EN;            //Enable DMA
	//Wait recive all data
	while (!(DMA1->LISR & DMA_LISR_TCIF0) && (i2c_ticks < timeout_ms));
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0;          	//Clear DMA event
	p_i2c->CR1 |= I2C_CR1_STOP;                	//Stop generation
	p_i2c->CR1 &= ~I2C_CR1_ACK;                	//Disable acknowledge
	return 1;
}

//---------------------------------------------------------------------------
uint8_t i2c1_dma_tx(uint8_t* data, uint16_t len, uint32_t Timeout) {
	i2c_ticks = 0;
	i2c_status = 1;
	// While Tx buffer not empty
	while (!(I2C1->SR1 & I2C_SR1_TXE) && (i2c_ticks < Timeout));
	// while SPI not busy
	while ((I2C1->SR1 & SPI_SR_BSY) && (i2c_ticks < Timeout));
	// Отключаем DMA канал
	DMA1_Stream6->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream6->CR & DMA_SxCR_EN) && (i2c_ticks < Timeout));

	// Заносим адрес памяти откуда мы будем передавать данные
	DMA1_Stream6->M0AR = (uint32_t)data;
	DMA1_Stream6->NDTR = len;				// Количество передаваемых данных
	// Start Tx, Set in dma_i2c1_init
	//DMA1_Stream6->CR |= DMA_SxCR_DIR_0 |	// Направление данных из памяти в периферию
	//					DMA_SxCR_MINC;		// Инкремент памяти включен

	DMA1_Stream6->CR |= DMA_SxCR_EN;
	if (i2c_ticks < Timeout)
		i2c_status = 0;
	return i2c_status;
}
uint8_t i2c1_dma_tx_2(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
	i2c_ticks = 0;
	i2c_status = 1;
	//Wait if bus busy
	while (p_i2c->SR2 & I2C_SR2_BUSY && (i2c_ticks < timeout_ms));
	if(p_i2c->SR2 & I2C_SR2_BUSY || p_i2c->SR1 & I2C_SR1_AF){
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
	i2c_ticks = 0;
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR)) == 0 &&
			(i2c_ticks < timeout_ms) );
	if ( i2c_ticks >= timeout_ms ) i2c_status=0;

	//Wait send addsess
//	while (!(p_i2c->SR1 & I2C_SR1_ADDR) && (i2c_ticks < timeout_ms))
	while (!(p_i2c->SR1 & I2C_SR1_ADDR) && (i2c_ticks < timeout_ms))
	{
		if (p_i2c->SR1 & I2C_SR1_AF)          //Acknowledge failure
		{
			p_i2c->CR1 |= I2C_CR1_STOP;       //Stop generation
			return 0;
		}
	}
	// Сброс бита ADDR производится чтением SR1, then SR2
	sr_status = (p_i2c->SR1 |	p_i2c->SR2);
	i2c_ticks = 0;
	while(	READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 &&
			(i2c_ticks < timeout_ms));
	if( (i2c_ticks >= timeout_ms) || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0;

	DMA1_Stream6->CR &= ~DMA_SxCR_EN;		//Enable DMA
	DMA1_Stream6->M0AR = (uint32_t)data;	//Set address buf
	DMA1_Stream6->NDTR = len;				//Set len
	DMA1_Stream6->CR |= DMA_SxCR_EN;		//Enable DMA

	while (!(p_i2c->SR1 & I2C_SR1_BTF) && (i2c_ticks < timeout_ms));        //Wait transmit all data
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;		//Clear DMA event

	p_i2c->CR1 |= I2C_CR1_STOP;				//Stop generation
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);


//	if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) return 0;
//	return i2c_status;
	return 1;
}
//---------------------------------------------------------------------------
uint8_t successWaitTXE(I2C_TypeDef * p_i2c, uint32_t timeout_ms){
	i2c_ticks = 0;
	while (READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && (i2c_ticks < timeout_ms));
	if( READ_BIT(p_i2c->SR1, I2C_SR1_AF) || (i2c_ticks>= timeout_ms) ){
		CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
		p_i2c->CR1 |= I2C_CR1_STOP;
		return 0;
	}
	return 1;
}

uint8_t i2c_call_device(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t mode, uint32_t timeout_ms){
	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
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
	// СБрос бита ADDR производится чтением SR1, then SR2
//	CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
	Delay(2);
//	SET_BIT(p_i2c->CR1, I2C_CR1_PE);
	return 0;
}

int8_t i2c_Device_Scan(I2C_TypeDef * p_i2c, int8_t addr_device, uint32_t timeout_ms){
	i2c_status = i2c_call_device (p_i2c, addr_device, 0, timeout_ms);
	i2c_stop(p_i2c);
	return i2c_status;
}

uint8_t i2c_write_2(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	i2c_status = i2c_call_device (p_i2c, addr_device, 0, timeout_ms);
	if (i2c_status){
		uint16_t Write_size = Count;
		uint8_t* pBuffPtr = data;
		while (Write_size > 0U)
		{
			if(!successWaitTXE(p_i2c, timeout_ms))
				return 0;
		      /* Write data to DR */
		      p_i2c->DR = *pBuffPtr;
	
		      /* Increment Buffer pointer */
		      pBuffPtr++;
		      Write_size--;
	
		      if ((READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 1) && (Write_size != 0U))
		      {
			/* Write data to DR */
			      p_i2c->DR = *pBuffPtr;
			      /* Increment Buffer pointer */
			      pBuffPtr++;
			      Write_size--;
		      }
	
		      /* Wait until BTF flag is set */
		      if (!successWaitBTF(p_i2c, timeout_ms) )
			  return 0;
		}
		p_i2c->CR1 |= I2C_CR1_STOP;
		CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
		return 1;
	}
	return i2c_status;
}

uint8_t i2c_write(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	i2c_status=1;
	i2c_ticks = 0;
	if(p_i2c->SR2 & I2C_SR2_BUSY){
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
	i2c_ticks = 0;
	while(	(READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
			(READ_BIT(p_i2c->SR1, I2C_SR1_ADDR)) == 0 &&
			(i2c_ticks < timeout_ms) );
	if ( i2c_ticks >= timeout_ms ) i2c_status=0;

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && i2c_status ){ // Устройство отозвалось
		// СБрос бита ADDR производится чтением SR1, then SR2
		sr_status = (p_i2c->SR1 |	p_i2c->SR2);
		i2c_ticks = 0;
		while(	READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && 
				(i2c_ticks < timeout_ms));
		if( (i2c_ticks >= timeout_ms) || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0;
	}

//	Write DATA
	if (i2c_status){
		for(uint16_t i=0; i<Count && i2c_status >0; i++){
			p_i2c->DR = *(data+i);
			i2c_ticks = 0;
			while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && i2c_status && (i2c_ticks < timeout_ms));
			if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status=0;
		}
		if(i2c_status){
			i2c_ticks = 0;
			while( READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 0 && (i2c_ticks < timeout_ms))
			if(i2c_ticks >= timeout_ms || READ_BIT(p_i2c->SR1, I2C_SR1_AF)) i2c_status = 0;
		}
	}
	p_i2c->CR1 |= I2C_CR1_STOP;
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return i2c_status;
}

uint8_t i2c_read(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	i2c_status=1;
	i2c_ticks = 0;
	// Clear Buffer - проверить надо ли
	for(int i = 0; i< Count; i++)
		*(data+i) = 0;

	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}
	// Используется только в 2 байтной конфигурации в мастер моде
	// Может быть надо удалить/закомментарить 
	CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта

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


