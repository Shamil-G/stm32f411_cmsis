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

void PllOn_stm32f411(){
	uint32_t tick=0, Ready;

//	SET_BIT(RCC->CR, RCC_CR_CSSON);

	RCC->CR |= (uint32_t)RCC_CR_HSEON;
	// Wait Ready HSE
	tick=0;
	do{
		Ready = READ_BIT(RCC->CR,RCC_CR_HSERDY);
	}while(++tick<255 && !Ready);

	if (Ready){
	//	AHB
		MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
	//  APB1 Low Speed prescaler
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
	//  APB2 High Speed prescaler
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);

		/*--------------------- FLASH code must be here ------------------*/
		// So HCLK will be 100MHz - then Latency must be 3 for stm32f411
		MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS);
		SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
		SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
		SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
		/*--------------------- End Flash Code ---------------------------*/

		// 25 MHz SRC
	//	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLN_5  | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_3;

		// 4. CLEAN n SystemInit function
		/* RES    PLLQ   RES PLLSRC  RES   PLLP   RES              PLLN         PLLM */
		// 1111   0100    0    1     0000   00     0            001 1000 00    00 1100
		// [27:24]	PLLQ = 4 = 0x0100
		// [22]		PLLSRC: 	0: Clock from HSI, 1: HSE
		// [17:16]	PLLP = 2  0x00
		// [14:6]	PLLN = 200 0x0 1100 1000
		// [5:0] 	PLLM = 25 0x11001
		//		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLP_Msk |
		//							RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLM_Msk );
		// Second: PLLM=16, PLLN=128, PLLP=2, AHB=1, APB1=2
		// (25000000/16)*128/2=1562500*128/2=200000000/2=100MHz

		// 16 MHz SRC
		MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLM_0);
		MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_3);
		MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ, RCC_PLLCFGR_PLLQ_2);
		//	REGISTER_MODIFY(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, RCC_PLLCFGR_PLLQ_1); -


		// 5.
		/* Reset USARTSW[1:0], I2CSW and TIMs bits */
		// 1111 1111 0000 0000 1110 1100 1100 1100
		// [1:0]	USART1SW:	PCLK selected as USART1 clock source (default)
		// [4]		I2C1SW:		HSI clock selected as I2C1 clock source (default)
		// [8]		TIM1SW: 	Timer1 clock source selection: PCLK2 clock
		// [12]		HRTIM1SW:	High Resolution Timer1 clock source selection: PLL vco(144 MHz)
		//	RCC->CFGR3 &= 0xFF00FCCCU;


		// bit[22]: SET as source PLL
		SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);
		//
		// Turn on PLL
		SET_BIT(RCC->CR,RCC_CR_PLLON);
		// Wait for PLL ready
		tick=0;
		while(tick<255 && !READ_BIT(RCC->CR,RCC_CR_PLLRDY)){
			tick++;
		};
		/* Select PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL){}

	}
	SystemCoreClockUpdate();
}

// Это аналог  function SystemInit()
void initRCC_F411(void){
	uint32_t tick=0, Ready;
	// -------------- SystemInit ----------------*/
	RCC->CR |= RCC_CR_HSION;
	// Wait Ready HSE
	tick=0;
	do{
		Ready = READ_BIT(RCC->CR,RCC_CR_HSIRDY);
	}while(++tick<255 && !Ready);

	/* Reset CFGR register */
//	  RCC->CFGR &= 0xF87FC00CU;
	// RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_MCO1 | RCC_CFGR_MCO2);
	RCC->CFGR = 0x00000000U;

	/* Reset PLLON,  CSSON and HSEON bits */
	// 1111 1110 1111 0110 1111 1111 1111 1111
	RCC->CR &= ~(RCC_CR_PLLON_Msk | RCC_CR_CSSON_Msk | RCC_CR_HSEON_Msk);


	/* Reset PLLCFGR register */
  	// 0010 0100 : 0000 0000 : 0011 0000 : 0001 0000
  	RCC->PLLCFGR = 0x24003010;

	/* Reset HPRE, PPRE1 and PPRE2 bits */
	// 3.

	/* Reset HSEBYP bit */
	RCC->CR &= ~RCC_CR_HSEBYP;

	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
	//	RCC->CFGR2 = RCC_CFGR_
	// Запрет всех прерываний

	RCC->CIR = 0x00000000UL;
	// -------------- SystemInit ----------------*/


	/*--------------------- Tune PLL end ------------------------------*/
	PllOn_stm32f411();
	/*--------------------- Tune PLL --------------------------------*/
	SystemCoreClockUpdate();
}
