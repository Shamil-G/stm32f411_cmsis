#pragma once

#include "main.h"

#define USE_I2C_DMA

#ifdef USE_I2C_DMA
	// STM32F411, Page 170 of RM0383:
	// I2C1: DMA1
	// Channel_0
	// Tx: DMA1_Stream1_IRQn
	// Channel_1
	// Rx: DMA1_Stream0_IRQn, DMA1_Stream5_IRQn
	// Tx: DMA1_Stream6_IRQn, DMA1_Stream7_IRQn

	// I2C2: DMA1
	// Channel_7
	// Rx: DMA1_Stream2_IRQn, DMA1_Stream3_IRQn
	// Tx: DMA1_Stream7_IRQn

	// I2C3: DMA1
	// Channel_1
	// Rx: DMA1_Stream1_IRQn
	// Channel_3
	// Rx: DMA1_Stream2_IRQn
	// Tx: DMA1_Stream4_IRQn
	// Channel_6
	// Tx: DMA1_Stream5_IRQn

	void dma_i2c_init();
	uint8_t i2c1_dma_tx(I2C_TypeDef* p_i2c, uint8_t addr, uint8_t* data, uint16_t len, uint32_t timeout_ms);
	uint8_t i2c1_dma_rx(I2C_TypeDef* p_i2c, uint8_t addr, uint8_t* data, uint16_t len, uint32_t timeout_ms);

#endif

uint8_t i2c_call_device(I2C_TypeDef * p_i2c, int8_t addr_device, uint8_t mode, uint32_t timeout_ms);

#define START_I2C1_TRANSMIT I2C1->CR1 |= I2C_CR1_START
#define STOP_I2C1_TRANSMIT  I2C1->CR1 &= ~I2C_CR1_STOP

#define SHT31_ADDR 0x44
#define I2C_READ  1
#define I2C_WRITE 0

#ifdef  STM32F4xx
#define PORT_I2C1_CLK  GPIOB
#define PORT_I2C1_DATA GPIOB
#define PIN_I2C1_CLK  6
#define PIN_I2C1_DATA 7
#define AF_I2C1_CLK	 af4
#define AF_I2C1_DATA af4
#define IDR_I2C1_CLK  GPIO_IDR_IDR_6
#define IDR_I2C1_DATA GPIO_IDR_IDR_7
#endif


enum i2c_command{
	hi_stretching_clk = 0x2c06,
	medium_stretching_clk = 0x2c0d,
	low_stretching_clk = 0x2c10,
	hi_clk = 0x2400,
	medium_clk = 0x240b,
	low_clk = 0x2416
};

//void InitI2C(I2C_TypeDef*);
//int i2c_recv_data(I2C_TypeDef *i2c, uint8_t Address_device, uint8_t* data, uint16_t count, uint32_t timeout);

enum eCR1{
	CR1_PE      = 0x0001,   // Peripheral enable
	CR1_SMBBUS  = 0x0002,   // SMBus mode. 0: I2C mode. 1: SMBus mode
	CR1_SMBTYPE = 0x0008,   // SMBus type. 0: SMBus Device,	1: SMBus Host
	CR1_ENARP	= 0x0010,   // ARP enable
	CR1_ENPEC	= 0x0020,   // PEC enable
	CR1_ENGC	= 0x0040,   //  General call enable
	CR1_NOSTRETCH = 0x0080, // Clock stretching disable (Slave mode)
	CR1_START     = 0x0100, // Start generation
	CR1_STOP      = 0x0200, // STOP: Stop generation
	CR1_ACK		  = 0x0400, // Acknowledge enable. 0: No acknowledge returned, 1: Acknowledge returned after a byte is received (matched address or data)
	CR1_POS		  = 0x0800, // Acknowledge/PEC Position (for data reception)
	CR1_PEC		  = 0x1000, // Packet error checking
	CR1_ALERT	  = 0x2000, // SMBus alert
	CR1_SWRST	  = 0x8000  // SWRST: Software reset
};


enum CR2{
	CR2_FREQ = 0x003F, // Peripheral clock frequency (APB), 0b110010: 50 MHz
	CR2_ITERREN = 0x0100, // Error interrupt enable
	CR2_ITEVTEN = 0x0200, // Event interrupt enable
	CR2_ITBUFEN = 0x0400, // Buffer interrupt enable
	CR2_DMAEN   = 0x0800, // DMA requests enable. 1: DMA request enabled when TxE=1 or RxNE =1
	CR2_LAST    = 0x1000  // DMA last transfer. 1: Next DMA EOT is the last transfer
};

enum OAR1{         // Own address register
	OAR1_ADD0 = 0x0001,  // Dual addressing mode enable
	OAR1_ADD  = 0x00fe,  // Interface address
	OAR1_ADDR10   = 0x03ff,  // Interface 10 bit address
	OAR1_ADDR7    = 0x4000,  // Must be 1
	OAR1_ADDMODE  = 0x8000  // Addressing mode. 0: 7 bit, 1: 10 bit address
};

enum eOAR2{
	OAR2_ENDUAL = 0x0001, // Dual addressing mode enable
	OAR2_ADD2   = 0x00fe  // Interface address
};

enum eDR{
	DR_DR = 0x00FF // 8-bit Data register
};

enum eSR1{       // Status register
	SR1_SB = 0x0001, // Start Bit generated? Master mode
	SR1_ADDR  = 0x0002, // Address sent (master mode)/matched (slave mode)
	SR1_BTF   = 0x0004, //  Byte transfer finished
	SR1_ADD10 = 0x0008, //
	SR1_STOPF = 0x0010, // Stop detection (slave mode)
	SR1_RXNE = 0x0040,  // Data register not empty (receivers)	TXE  = 0x0080,
	SR1_TXE  = 0x0080,  // Data register empty (transmitters)
	SR1_BERR = 0x0100,  // Bus error
	SR1_ARLO = 0x0200,  //  Arbitration lost (master mode)
	SR1_AF   = 0x0400,  //  Acknowledge failure
	SR1_OVR  = 0x0800,  // Overrun/Underrun. New received byte is lost/
	SR1_PECERR  = 0x1000,  // : PEC Error in reception
	SR1_TIMEOUT = 0x4000,  // Timeout or Tlow error
};

enum eSR2{
	SR2_MSL  = 0x0001, // Master/slave. 0: Slave mode, 1: Master mode
	SR2_BUSY = 0x0002, // Bus busy
	SR2_TRA  = 0x0004, // ransmitter/receiver. 0: Data bytes received, 1: Data bytes transmitted
	SR2_GENCALL = 0x0010,
	SR2_DUALF   = 0x0080,
	SR2_PEC	= 0xFF00  // Packet error checking register
};

enum eCCR{        // Clock control register
	CCR_FS  = 0x8000, // Master mode selection. 0: Sm mode, 1:Fm mode I2c
	CCR_DUTY = 0x4000, // Fm mode duty cicle. 0: Fm mode Tlow/Thigh = 2, 1: Fm mode Tlow/Thigh = 16/9
	CCR_CCR  = 0x0fff //
};


