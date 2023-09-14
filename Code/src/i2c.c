/*
 * I2C.cpp
 *
 *  Created on: Aug 30, 2023
 *      Author: sguss
 */
#include "i2c.h"
#include "gpio.h"
#include "SysTick.h"

extern volatile uint32_t Delay_i2c;

void i2c_soft_reset(I2C_TypeDef * p_i2c){
	p_i2c->CR1 |= I2C_CR1_SWRST_Msk;
	while((p_i2c->CR1 & I2C_CR1_SWRST_Msk) == 0);
	p_i2c->CR1 &= ~I2C_CR1_SWRST_Msk;
	while(p_i2c->CR1 & I2C_CR1_SWRST_Msk);
}

void i2c_force_reset(){
	RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST);
	Delay(2);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);
}

void init_i2c(I2C_TypeDef * p_i2c){
	InitGPio(PORT_I2C1_CLK, PIN_I2C1_CLK, alternateF, openDrain, medium, noPull, AF_I2C1_CLK);
//	PortSet (PORT_I2C1_CLK, PIN_I2C1_DATA);

	InitGPio(PORT_I2C1_DATA, PIN_I2C1_DATA, alternateF, openDrain, medium, noPull, AF_I2C1_DATA);
//	PortSet (PORT_I2C1_DATA, PIN_I2C1_DATA);

	Delay(2);
//	PORT_I2C1_CLK->BSRR |= GPIO_BSRR_BS_7;
//	PORT_I2C1_DATA->BSRR |= GPIO_BSRR_BS_6;
//

	RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	RCC->APB1ENR = RCC_APB1ENR_I2C1EN;

	p_i2c->CR1 &= ~I2C_CR1_PE;

	p_i2c->CCR = 0;
	p_i2c->CR1 = 0;
	p_i2c->CR2 = 0;

	p_i2c->SR1 = 0;
//	I2C1->SR2 = 0;
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
	//
	//  I2C1->TRISE = (SDA and SCL maxtime / I2C1->I2C_CR2_FREQ) + 1
	//  According to Datasheet on Sm=100 KHz, SDA and SCL maxtime = 1000 ns
	//  I2C1->TRISE = 1000 / 50 + 1 = 21
//	I2C1->TRISE = (1000 / (I2C_CR2_FREQ_1 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_5)) + 1;
	p_i2c->TRISE = (1000 / (I2C1->CR2 & I2C_CR2_FREQ_Msk)) + 1;

	// Enable I2C
	p_i2c->CR1 |= I2C_CR1_PE_Msk;

	//	Разрешим генерацию условия ACK после приёма байта
	//	p_i2c->CR1 |= I2C_CR1_ACK;
};

uint8_t successWaitTXE(I2C_TypeDef * p_i2c, uint32_t timeout_ms){
	Delay_i2c = timeout_ms;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 ){
		if(READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
			CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
			p_i2c->CR1 |= I2C_CR1_STOP;
			return 0;
		}
		if(!Delay_i2c){
			p_i2c->CR1 |= I2C_CR1_STOP;
			return 0;
		}
	}
	return 1;
}

uint8_t successWaitBTF(I2C_TypeDef * p_i2c, uint32_t timeout_ms){
	Delay_i2c = timeout_ms;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 0 ){
		if(READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
			CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
			p_i2c->CR1 |= I2C_CR1_STOP;
			return 0;
		}
		if(!Delay_i2c){
			p_i2c->CR1 |= I2C_CR1_STOP;
			return 0;
		}
	}
	return 1;
}

void i2c_restart(I2C_TypeDef * p_i2c){
	CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
	Delay(2);
	SET_BIT(p_i2c->CR1, I2C_CR1_PE);
	Delay(2);
}


void i2c_stop(I2C_TypeDef * p_i2c){
	//	Вызвать функцию дороже чем две команды
		p_i2c->CR1 |= I2C_CR1_STOP;

		CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
		// СБрос бита ADDR производится чтением SR1, then SR2
		CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
		Delay(2);
		SET_BIT(p_i2c->CR1, I2C_CR1_PE);
}


uint8_t i2c_call_device(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t mode, uint32_t timeout_ms){
	if(p_i2c->SR2 & I2C_SR2_BUSY){
//		i2c_force_reset(p_i2c);
//		init_i2c(p_i2c);
//		Delay(100);
//		PORT_I2C1_CLK->BSRR |= GPIO_BSRR_BS_7;
//		PORT_I2C1_DATA->BSRR |= GPIO_BSRR_BS_6;
//		Delay(2);
//		PORT_I2C1_CLK->BSRR |= GPIO_BSRR_BS_7;
//		PORT_I2C1_DATA->BSRR |= GPIO_BSRR_BS_6;
		i2c_stop(p_i2c);
		return 0;
	}

	CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	Delay_i2c = timeout_ms;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 )
		if( !Delay_i2c )
			return 0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;

	p_i2c->DR = (addr_device << 1) | mode;
	// Здесь SB сбросится

	//	Wait AF or ADDR
	Delay_i2c = timeout_ms;
	while( (READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
		   (READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0 )
		 )
		if ( !Delay_i2c )
			return 0;
//		clearTimer(num_timer);
//			rem_time=remain_time(num_timer);
//	success = READ_BIT(p_i2c->SR1, I2C_SR1_ADDR);
	// СБрос бита ADDR производится чтением SR1, then SR2

	if( p_i2c->SR1 & I2C_SR1_ADDR ){ // Устройство отозвалось
		// СБрос бита ADDR производится чтением SR1, then SR2
		p_i2c->SR1;
		p_i2c->SR2;
		Delay_i2c = timeout_ms;
		while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 ){
			if(!Delay_i2c || READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
				return 0;
			}
		}
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
	uint8_t status = i2c_call_device (p_i2c, addr_device, 0, timeout_ms);
	i2c_stop(p_i2c);
	return status;
}

#define __HAL_I2C_GET_FLAG(__HANDLE__, __FLAG__) ((((uint8_t)((__FLAG__) >> 16U)) == 0x01U) ? \
                                                  (((((__HANDLE__)->Instance->SR1) & ((__FLAG__) & I2C_FLAG_MASK)) == ((__FLAG__) & I2C_FLAG_MASK)) ? SET : RESET) : \
                                                  (((((__HANDLE__)->Instance->SR2) & ((__FLAG__) & I2C_FLAG_MASK)) == ((__FLAG__) & I2C_FLAG_MASK)) ? SET : RESET))


uint8_t i2c_write_2(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	uint8_t status = i2c_call_device (p_i2c, addr_device, 0, timeout_ms);

	if (status){
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
	return status;
}


uint8_t i2c_write(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	uint8_t status=1;
	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		return 0;
	}

	CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	Delay_i2c = timeout_ms;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 )
		if( !Delay_i2c )
			status=0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;

	p_i2c->DR = (addr_device << 1) | 0;
	// Здесь SB сбросится

	//	Wait AF or ADDR
	Delay_i2c = timeout_ms;
	while( (READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
		   (READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0 &&
				   status )
		 )
		if ( !Delay_i2c )
			status=0;
//		clearTimer(num_timer);
//			rem_time=remain_time(num_timer);

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && status ){ // Устройство отозвалось
		// СБрос бита ADDR производится чтением SR1, then SR2
		uint8_t status = (p_i2c->SR1 |	p_i2c->SR2);
		Delay_i2c = timeout_ms;
		while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && status){
			if(!Delay_i2c || READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
				status=0;
			}
		}
	}

//	Write DATA
	if (status){
		for(uint16_t i=0; i<Count && status >0; i++){
			p_i2c->DR = *(data+i);
			Delay_i2c = timeout_ms;
			while( READ_BIT(p_i2c->SR1, I2C_SR1_TXE) == 0 && status){
				if(!Delay_i2c || READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
					status=0;
				}
			}
		}
		if(status){
			Delay_i2c = timeout_ms;
			while( READ_BIT(p_i2c->SR1, I2C_SR1_BTF) == 0 && status){
				if(!Delay_i2c || READ_BIT(p_i2c->SR1, I2C_SR1_AF)){
					status = 0;
				}
			}
		}
	}
	p_i2c->CR1 |= I2C_CR1_STOP;
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return status;
}

uint8_t i2c_read(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t* data, uint16_t Count, uint32_t timeout_ms){
	uint8_t status=1;
	//		Clear Buffer
	for(int i = 0; i< Count; i++)
		*(data+i) = 0;

	if(p_i2c->SR2 & I2C_SR2_BUSY){
		i2c_stop(p_i2c);
		status=0;
	}

	CLEAR_BIT(p_i2c->CR1, I2C_CR1_POS); // Управляет (N)ACK текущего байта
	SET_BIT(p_i2c->CR1, I2C_CR1_START); // Отправляеме сигнал START, чтобы стать мастером

	Delay_i2c = timeout_ms;
	while( READ_BIT(p_i2c->SR1, I2C_SR1_SB) == 0 && status )
		if( !Delay_i2c )
			status=0;

	//	Сброс статуса SB через чтение SR1 с последующей записью в DR
	p_i2c->SR1;

	p_i2c->DR = (addr_device << 1) | 1;
	// Здесь SB сбросится

	//	Wait AF or ADDR
	Delay_i2c = timeout_ms;
	while( (READ_BIT(p_i2c->SR1, I2C_SR1_AF) == 0) && // При получении NACK ???
		   (READ_BIT(p_i2c->SR1, I2C_SR1_ADDR) == 0 &&
				   status )
		 )
		if ( !Delay_i2c )
			status=0;

	if( (p_i2c->SR1 & I2C_SR1_ADDR) && status ){ // Устройство отозвалось
		// СБрос бита ADDR производится чтением SR1, then SR2
		status = (p_i2c->SR1 |	p_i2c->SR2);
		Delay_i2c = timeout_ms;
	}



	for(uint16_t i=0; i<Count && status; i++){
		if(i<Count-1){
			// Для продолжения приема после каждого принятого байта выставляем ACK
			SET_BIT(p_i2c->CR1, I2C_CR1_ACK);

			Delay_i2c=timeout_ms;
			while( READ_BIT(p_i2c->SR1,I2C_SR1_RXNE) == 0 && status)
				if(!Delay_i2c) status=0;
			*(data+i) = p_i2c->DR;
		}
		else{
			// Отключаем ACK, перед приемом последнего байта
			CLEAR_BIT(p_i2c->CR1, I2C_CR1_ACK);
			// Останавливаем I2C (тактирование)
			Delay_i2c = timeout_ms;
			while( READ_BIT(p_i2c->SR1,I2C_SR1_RXNE) == 0 && status)
				if(!Delay_i2c) status=0;
			*(data+i) = p_i2c->DR;
		}
	}
	// Включаем NACK
	SET_BIT(p_i2c->CR1, I2C_CR1_STOP);
	CLEAR_BIT(p_i2c->SR1, I2C_SR1_AF);
	return status;
}


