#include "main.h"

void showSOS(){
	for(uint8_t i=0; i<3; i++){
		led1_on();
		Delay(150);
		led1_off();
		Delay(100);
	}
	Delay(250);
	for(uint8_t i=0; i<3; i++){
		led1_on();
		Delay(300);
		led1_off();
		Delay(100);
	}
	Delay(300);
	for(uint8_t i=0; i<3; i++){
		led1_on();
		Delay(150);
		led1_off();
		Delay(100);
	}
}

void showBip(){
	led1_on();
	led1_off();
}

void showUp(){
	led1_on();
	Delay(300);
	led1_off();
	Delay(100);
	led1_on();
	Delay(300);
	led1_off();

}

void showDown(){
	led1_on();
	Delay(150);
	led1_off();
	Delay(100);
	led1_on();
	Delay(150);
	led1_off();
}
