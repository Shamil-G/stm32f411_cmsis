/*
 * adc-inject.h
 *
 *  Created on: 26 февр. 2021 г.
 *      Author: sguss
 */
#pragma once

#include "main.h"

#ifndef INC_DEV_INCLUDE_ADC_INJECT_H_
#define INC_DEV_INCLUDE_ADC_INJECT_H_

#define ADC_LEN_BUF 8

typedef enum {
	simple_calibration = 0,
	diff_calibration = 1
} TypeCalibration;

extern struct adc_result_buf {
	uint32_t i_voltage[ADC_LEN_BUF];
	uint32_t i_current[ADC_LEN_BUF];
	uint32_t o_voltage[ADC_LEN_BUF];
	uint32_t o_current[ADC_LEN_BUF];
	uint8_t pos;		  // sample number
	uint32_t count;
	uint32_t adc_max_calc;
} adc_result_buf;

void init_adc_struct(void);

#endif /* INC_DEV_INCLUDE_ADC_INJECT_H_ */
