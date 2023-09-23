/*
 * ads115.c
 *
 *  Created on: Sep 18, 2023
 *      Author: sguss
 */

#include "main.h"
#include "i2c.h"
#include "string.h"

#define USE_GAIN_2

#include "adc1115.h"

#define DELAY TIMEOUT_SPS_860

extern uint32_t Delay_i2c;

uint8_t i2c_write(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* command, uint16_t, uint32_t);
uint8_t i2c_read(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t count, uint32_t timeout_ms);
void led1_on(void);
void Delay(uint32_t);


uint8_t result_adc1115[2];

float ADC1115_GetValue() {
	uint32_t result=0;
	float  return_value;
	for(int i=0; i<2; i++){
		result = (result << 8) | result_adc1115[i];
	}
	return_value = result;

//	if(ads1115[0] & SIGN_BIT)
//		return_value -= return_value;
	return return_value * LSB;
}


uint8_t ADC1115_request(I2C_TypeDef* p_i2c, uint8_t addr_device){
	uint8_t status = 0;
	uint8_t cr_command[3];
	uint8_t pr_command = CONVERSION_REG;
	cr_command[0] = CONFIG_REG;
	cr_command[1] = 0x84; // = ONE_SHOT_START | FSR_2
	cr_command[2] = 0x83; // | SPS_128 | DISABLE_COMPARATOR

	//	Отправим команду конфигурации в Config Register
	//  Address Pointer Register select "Config Register" (second transmit byte or first byte in cr_command)
	status = i2c_write(p_i2c, addr_device, (uint8_t*)&cr_command, sizeof(cr_command), DELAY);
	//	Delay(TIMEOUT_SPS_860);
	// Without Delay will be mistake status==0
	Delay(1);
	if(status){
		// Теперь выберем "Conversation Register" - для последующего получения резульата
		//  Address Pointer Register select "Conversation Register"
		status = i2c_write(p_i2c, addr_device, (uint8_t*)&pr_command, sizeof(pr_command), DELAY);
		if(status){
			status=0;
			for(int i=0; i<4 && !status; i++){
				//	Задержка для измерения
				Delay(DELAY);
				// Read "Conversation Register"
				status = i2c_read(p_i2c, addr_device, result_adc1115, sizeof(result_adc1115), DELAY);
			}

		}

	}
	return status;
}

float test_adc1115(){
	uint8_t status;
	status = ADC1115_request(I2C1, ADDR_ADS1115_GND);
	if(status)
		return ADC1115_GetValue();
	return 0.0;
}
