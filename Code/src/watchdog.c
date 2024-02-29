/*
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "watchdog.h"

uint8_t init_watchdog(uint16_t counter){
	uint32_t status=0UL;

	// IWDG_SR_RVU>0 - IWDG busy, update counter
	// IWDG_SR_PVU>0 - IWDG busy, update prescaler
	// Status must be 0
	status = status & IWDG_SR_PVU & IWDG_SR_RVU;

	if(counter < 4096 && status==0){
		// In example first start_watchdog
//		start_watchdog();

		// FOR Access to IWDG_PR and IWDG_PLR registry WRITE 0x5555
		IWDG->KR=(0x5555 << IWDG_KR_KEY_Pos);
		// DIvide FREQ
		IWDG->PR=(PRESCALE_256 << IWDG_PR_PR_Pos);

		// LOAD counter
		IWDG->RLR=(counter << IWDG_RLR_RL_Pos);

		// Close Access to IWDG->PR and IWDG->PLR registry WRITE 0x0000
		IWDG->KR=(0x0000 << IWDG_KR_KEY_Pos);
		return 1;
	}
	return 0;
}


inline void reload_watchdog(){
	// RELOAD WATCHDOG
	IWDG->KR=(0xAAAA << IWDG_KR_KEY_Pos);
}

void start_watchdog(){
	// Start WATCHDOG
	IWDG->KR=(0xCCCC << IWDG_KR_KEY_Pos);
}

uint16_t watchdog_value(){
	return IWDG->RLR;
}
