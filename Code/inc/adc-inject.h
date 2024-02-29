#pragma once
/*
 * adc-inject.h
 *
 * Created on: 26 февр. 2021 г.
 * Author: Shamil Gusseynov
*/

#define ADCTimer TIM4

void InitADC(void);

void init_adc_struct(void);
float getInputVoltage(void);
float getInputCurrent(void);
float getOutputVoltage(void);
float getOutputCurrent(void);

#define ADC_LEN_BUF 4
//#define ADC_LEN_BUF 8

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

//#define ADC_16Bit

#ifdef ADC_16Bit
#define ADC_COEFF 65536
#else
#define ADC_COEFF 4096
#endif

#define reference_voltage 	3.31
#define reference_shift 	0
#define TARGET_VALUE 		1.5
#define PRECISION_VALUE 	0.005
#define TARGET_VALUE_MAX 	TARGET_VALUE + TARGET_VALUE * PRECISION_VALUE
#define TARGET_VALUE_MIN 	TARGET_VALUE + TARGET_VALUE - PRECISION_VALUE
#define FAULT_MAX_VALUE 	3
#define FAULT_MIN_VALUE 	0.06
#define MIN_INPUT_VOLTAGE 	0.5
#define MAX_INPUT_VOLTAGE 	2.5
#define MAX_CURRENT 		2.5
#define FAULT_LOW 		getInputVoltage()<FAULT_MIN_VALUE
#define FAULT_HIGHT 		result.output_fault>FAULT_MAX_VALUE
#define CNT_FAULT 			20
