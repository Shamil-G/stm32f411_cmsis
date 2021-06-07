#include "main.h"

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */


// Here System go to Running Stage
void SystemUp(void){
	uint32_t tick=0, HSEReady;
	// Turn on HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	// Wait Ready HSE
	tick=0;
	do{
		HSEReady = READ_BIT(RCC->CR,RCC_CR_HSERDY);
	}while(++tick<255 && !HSEReady);

	if(HSEReady){

		/*--------------------- FLASH code must be here ------------------*/

		// FLASH->ACR |= FLASH_ACR_PRFTBE;
		CLEAR_BIT(FLASH->ACR, FLASH_ACR_LATENCY_Msk);
		// So HCLK will be 100MHz - then Latency must be 3 for stm32f411
		SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_3WS);

		/*--------------------- End Flash Code ---------------------------*/

		/*--------------------- Tune PLL --------------------------------*/
		// PLL must be OFF before tuning
		// Turn off PLL
		CLEAR_BIT(RCC->CR,RCC_CR_PLLON_Msk);

		//
		/*--------------------- Tune PLL end ------------------------------*/

		// Turn on PLL
		SET_BIT(RCC->CR,RCC_CR_PLLON);
		// Wait for PLL ready
		tick=0;
		while(tick<255 && !READ_BIT(RCC->CR,RCC_CR_PLLRDY)){
			tick++;
		};
		// Set PLL as Source SystemCLock
		CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW_Msk);
		SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);
		// Wait for PLL ready for SysClock
		tick=0;
		while(tick<255 && !READ_BIT(RCC->CFGR,RCC_CFGR_SWS_PLL)){
			tick++;
		};

		SET_BIT(RCC->CR, RCC_CR_CSSON);
	}
	else{
	    // showSOS();
	    // HSE don't working
	}

}

void PLL_SRC_64(void){

}
void PLL_SRC_25(void){

}
// Here System going to Zero Stage
void SystemInit(){
	  /* FPU settings ------------------------------------------------------------*/
	  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	  #endif

	#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
	  SystemInit_ExtMemCtl();
	#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

	  /* Configure the Vector Table location -------------------------------------*/
	#if defined(USER_VECT_TAB_ADDRESS)
	  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
	#endif /* USER_VECT_TAB_ADDRESS */

	// 1. Включаем HSI

	// 2. Выбираем HSI как источник тактирования
	// 2. Сбрасываем все HCLK, APB предделители

	// 3. Сбрасываем HSE PLL CSS
	//
	// 4. Сбрасываем все предделители и умножители

	// 5. Устанавливаем в начальное состояние источники сигналов для  USART, TIM, HRTIM ...

	// 6. Блокируем все прерывания


	// 1.
	SET_BIT(RCC->CR, RCC_CR_HSION);

	// 2.
	// 0000 0000 0000 0000 0001 0000 0000 1100
	// [1:0]:	SW - System clock switch from HSI
//	RCC->CFGR &= ~RCC_CFGR_SW;
	// [7:4]:	HPRE 	- HLCK prescaler = 1 (старший бит = 0)
//	RCC->CFGR &= ~RCC_CFGR_HPRE_3;
	// [10:8]	PPRE1 	- APB Low-speed prescaler (APB1) = 2
//	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
//	RCC->CFGR |= ~RCC_CFGR_PPRE1_2;
	// [13:11]	PPRE2 	- APB high-speed prescaler (APB2) = 1
//	RCC->CFGR &= ~RCC_CFGR_PPRE2_2;
	// [26:24]	MCO		- Microcontroller clock output disabled
	RCC->CFGR &= ~( RCC_CFGR_SW_Msk |  RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk | RCC_CFGR_MCO1_Msk );
	RCC->CFGR |= RCC_CFGR_PPRE1_2;
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_HSI));

	// 3.
	/* Reset PLLON,  CSSON and HSEON bits */
	// 1111 1110 1111 0110 1111 1111 1111 1111
	RCC->CR &= ~(RCC_CR_PLLON_Msk | RCC_CR_CSSON_Msk | RCC_CR_HSEON_Msk);

	/* Reset HSEBYP bit */
	// 0: HSE crystal oscillator not bypassed
	RCC->CR &= ~RCC_CR_HSEBYP;

	// 4.
	/* Reset PLLSRC, PLLXTPRE, PLLMUL, MCO */
	// 1111 0100 0000 0001 1001 1000 0000 1100
	// [27:24]	PLLQ = 4
	// [22]		PLLSRC: 	Clock from HSI
	// [17:16]	PLLP = 2
	// [14:6]	PLLN = 96
	// [5:0] 	PLLM = 12
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLP_Msk |
						RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLM_Msk );
	// 25 MHz SRC
//	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLN_5  | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_3;

	// 16 MHz SRC
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 |
					RCC_PLLCFGR_PLLN_2  |RCC_PLLCFGR_PLLN_5  | RCC_PLLCFGR_PLLN_6 |
					RCC_PLLCFGR_PLLM_3;

	// 5.
	/* Reset USARTSW[1:0], I2CSW and TIMs bits */
	// 1111 1111 0000 0000 1110 1100 1100 1100
	// [1:0]	USART1SW:	PCLK selected as USART1 clock source (default)
	// [4]		I2C1SW:		HSI clock selected as I2C1 clock source (default)
	// [8]		TIM1SW: 	Timer1 clock source selection: PCLK2 clock
	// [12]		HRTIM1SW:	High Resolution Timer1 clock source selection: PLL vco(144 MHz)
//	RCC->CFGR3 &= 0xFF00FCCCU;

	// 6.
	/* Disable all interrupts */
	RCC->CIR = 0x00000000U;
}
