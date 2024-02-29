/*
 * sht31.c
 *
 *  Created on: Sep 2, 2023
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "sht31.h"

//uint8_t i2c_write(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* command, uint16_t, uint32_t);
//uint8_t i2c_read(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t count, uint32_t timeout_ms);


struct SHT31_INFO{
	uint16_t temperature_raw;
	uint16_t humidity_raw;
} sht31;


uint8_t SHT31_CRC_8(uint8_t* data, int len) {
	uint8_t crc = 0xFF;
	const uint8_t poly = 0x31;

	for(uint8_t byte = len; byte; byte--) {
		crc ^= *(data++);
		for(uint8_t i = 8; i; i--) {
			crc = (crc & 0x80)? (crc<<1)^poly : (crc<<1);
		}
	}
	return crc;
}

float SHT31_GetHumidity() {
	float res = 100.0*(sht31.humidity_raw)/65535;
	return res;
}
float SHT31_GetTemperature() {
	float res = 175.0*(sht31.temperature_raw)/65535 - 45;
	return res;
}

uint16_t SHT31_GetHumidity_raw() {
	return sht31.humidity_raw;
}
uint16_t SHT31_GetTemperature_raw() {
	return sht31.temperature_raw;
}

uint8_t sht31_request(I2C_TypeDef* p_i2c, uint8_t addr_device, uint32_t timeout_ms){
	uint8_t status = 0;
	uint8_t command[2] = { 0x24, 0x0b};
	uint8_t buffer[6];

	//	Отправим команду на чтение
//	status = i2c_write(p_i2c, addr_device, command, sizeof(command), timeout_ms);
	status = i2c1_dma_tx(p_i2c, addr_device, command, sizeof(command), timeout_ms);
	if(status)
	{
//		Delay(timeout_ms);
	//		Делаем 4 попытки прочитать результат
		status=0;
		for(int i=0; i<4 && !status; i++){
		//	Send Command
			//	Задержка для измерения
			Delay(20);
			// Read Result
//			status = i2c_read(p_i2c, addr_device, buffer, 6, timeout_ms);
			status = i2c1_dma_rx(p_i2c, addr_device, buffer, 6, timeout_ms);

			if(status){
				if(buffer[2] != SHT31_CRC_8(&buffer[0], 2)) {
					status = 0;
				}
				else {
					sht31.temperature_raw = (buffer[0] << 8) + buffer[1];
					status = 1;
				}

				if(buffer[5] == SHT31_CRC_8(&buffer[3], 2)) {
					sht31.humidity_raw = (buffer[3] << 8) + buffer[4];
					status++;
				}
			}
		}
	}
	return status;
}
