#include "main.h"

// Original function SystemInit(void) is in file system__stm32f4xx.c
// This function using for replace original function SystemInit(void) in file system__stm32fxx.c

void SystemInit_F411(void)
{
  /* FPU settings ------------------------------------------------------------*/
//  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
//    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
//  #endif

    /* FPU settings ------------------------------------------------------------*/
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  /* After the HSION bit is cleared, HSIRDY goes low after 6 HSI clock cycles. */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  // 0010 0100 : 0000 0000 : 0011 0000 : 0001 0000
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

/*
#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl();
#endif
*/
/* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/* Configure the Vector Table location -------------------------------------*/
/*
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; // Vector Table Relocation in Internal SRAM
#endif
*/
/* USER_VECT_TAB_ADDRESS */
	SystemCoreClockUpdate();

}



