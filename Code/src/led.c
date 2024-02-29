/*
 * manage_pin.c
 *
 *  Created on: 22 дек. 2019 г.
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "led.h"


uint8_t init_led1, status_led1 = 0;

void enable_led1(){
	ENABLE_LED_BUS;
	ENABLE_LED_PIN_1;
}
void toggle_led1(void){
	GPIOC->ODR ^= GPIO_ODR_OD13;
}


void led1_on(){
	LED_1_LIGTH;
	status_led1=1;
}

void led1_off(){
	LED_1_DARK;
	status_led1=0;
}

void led2_on(){
#ifdef TWO_LED
	uint8_t init_led2, status_led2 = 0;
	void toggle_led2(void){
		if(status_led2){
			LED_2_DARK;
			status_led2=0;
			return;
		}
		LED_2_LIGTH;
		status_led2=1;
	}
	void enable_led2(){
		ENABLE_LED_BUS;
		ENABLE_LED_PIN_2;
		init_led2=1;
	}
	if(!init_led2){
		ENABLE_LED_BUS;
		ENABLE_LED_PIN_2;
		init_led2=1;
		ligth2=0;
	}
	if(!ligth2){
		LED_2_LIGTH;
		ligth2=1;
	}
#endif
}

void led2_off(){
#ifdef TWO_LED
if(ligth2){
	LED_2_DARK;
	ligth2=0;
}
#endif
}
