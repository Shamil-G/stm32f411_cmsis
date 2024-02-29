#pragma once
/*
 * Author: Shamil Gusseynov
*/
#include "i2c.h"

uint8_t sht31_request(I2C_TypeDef* p_i2c, uint8_t addr_device, uint32_t timeout_ms);
float SHT31_GetTemperature();
float SHT31_GetHumidity();

#define SHT31_ADDR 0x44
#define SHT31_command 0x240b



