#include "main.h"

void flash_init(){
  // allow access to the Flash control register and so, to allow program and erase operations
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}
//pageAddress - любой адрес, принадлежащий стираемой странице
void flash_erase(unsigned int pageAddress) {
	while (FLASH->SR & FLASH_SR_BSY);
  // Ждем конца всех операций
	while( !(FLASH->SR & FLASH_SR_EOP) ); // EOP - End of operation
  // СБрасываем состояние запиьсю туда же единицы
	FLASH->SR = FLASH_SR_EOP;
	

	FLASH->CR |= FLASH_CR_SER;

  // Номер страницы для зачистки
	// FLASH->AR = pageAddress;
  FLASH->CR |= pageAddress << FLASH_CR_SNB_Pos;
  // This bit triggers an erase operation when set. It is set only by software and 
  // cleared when the BSY bit is cleared
	FLASH->CR |= FLASH_CR_STRT;
  
	while (!(FLASH->SR & FLASH_SR_EOP));
	FLASH->SR = FLASH_SR_EOP;

  // Sector Erase activated.
	FLASH->CR &= ~FLASH_CR_SER;
}

//data - указатель на записываемые данные
//address - адрес во flash
//count - количество записываемых байт, должно быть кратно 2
void flash_write(unsigned char* data, unsigned int address, unsigned int count) {
	unsigned int i;

	while (FLASH->SR & FLASH_SR_BSY);
  // Ждем конца всех операций
	while( !(FLASH->SR & FLASH_SR_EOP) ); // EOP - End of operation
  // СБрасываем состояние запиьсю туда же единицы
	FLASH->SR = FLASH_SR_EOP;

  // used to allow program and erase operations in the user configuration sector
  // FLASH->OPTKEYR = 0x08192A3B;
  // FLASH->OPTKEYR = 0x4C5D6E7F;
  
  // Flash programming activated.
	FLASH->CR |= FLASH_CR_PG;

	for (i = 0; i < count; i += 2) {
		*(volatile unsigned short*)(address + i) = (((unsigned short)data[i + 1]) << 8) + data[i];
		while (!(FLASH->SR & FLASH_SR_EOP));
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR &= ~(FLASH_CR_PG);
	FLASH->CR |= FLASH_CR_LOCK;
}
