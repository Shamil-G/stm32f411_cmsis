/*
 * sht31.c
 *
 *  Created on: Sep 2, 2023
 *      Author: sguss
 */
#include "main.h"
#include "i2c.h"
#include "mcp3421.h"
#include "string.h"

extern uint32_t Delay_i2c;

uint8_t mcp3421[MCP3421_DATA_LEN];

void Delay(uint32_t);
uint8_t i2c_write(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* command, uint16_t, uint32_t);
uint8_t i2c_read(I2C_TypeDef* p_i2c, uint8_t addr_device, uint8_t* data, uint16_t count, uint32_t timeout_ms);


float MCP3421_GetValue() {
	uint32_t result=0;
	float  return_value;
	for(int i=0; i<MCP3421_DATA_LEN; i++){
		result = (result << 8) | mcp3421[i];
	}
	return_value = result;

	if(mcp3421[0] & SIGN_BIT)
		return_value -= return_value;
	return return_value * LSB;
}


uint8_t MCP3421_request(I2C_TypeDef* p_i2c, uint8_t addr_device){
	uint8_t status = 0;
	uint8_t command =  ONE_SHOT_START | MCP3421_PRECISION;
	uint8_t buffer[MCP3421_DATA_LEN+1]; // +1, так как читается и конфиг байт

	//	Отправим команду на чтение
	status = i2c_write(p_i2c, addr_device, &command, sizeof(command), MCP3421_TIMEOUT);
	if(status)
	{
//		Delay(timeout_ms);
	//		Делаем 4 попытки прочитать результат
		status=0;
		for(int i=0; i<4 && !status; i++){
		//	Send Command
			//	Задержка для измерения
			Delay(MCP3421_TIMEOUT);
			// Read Result
			status = i2c_read(p_i2c, addr_device, buffer, MCP3421_DATA_LEN+1, MCP3421_TIMEOUT);

			if(status){
				memcpy(mcp3421, buffer, sizeof(MCP3421_DATA_LEN));
			}
		}
	}
	return status;
}
