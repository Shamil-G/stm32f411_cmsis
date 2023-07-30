/*
 * adc_inject.c
 *
 *  Created on: Feb 18, 2021
 *      Author: Shamil Gusseynov
 */

#include "main.h"
#include "string.h"

uint8_t lockGetValue;
//#define ADC_16Bit

#ifdef ADC_16Bit
#define ADC_COEFF (65536*4)
#else
#define ADC_COEFF (4096*4)
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

struct adc_result_buf adc_result_buf;

void ADCDown(ADC_TypeDef * Adc){
	CLEAR_BIT(Adc->CR2, ADC_CR2_ADON);
};

void ADCUp(ADC_TypeDef * Adc){
	CLEAR_BIT(Adc->CR2, ADC_CR2_CONT);
	Adc->CR2 |= ADC_CR2_ADON;
//	while(!(Adc->ISR & ADC_ISR_ADRDY));
};


uint32_t getValue(uint32_t *p){
	volatile uint32_t minVal=0;
	volatile uint32_t maxVal=0;
	volatile uint32_t sumAVG=0;
	if(lockGetValue==0){
		lockGetValue=1;
		//  volatile uint32_t minVal=ADC_COEFF/4;
		//  volatile uint32_t maxVal=ADC_COEFF/4;
		uint32_t lbuf[ADC_LEN_BUF];
		memcpy(lbuf,p,sizeof(lbuf));

		sumAVG=lbuf[0];
		minVal=lbuf[0];
		maxVal=lbuf[0];

		for(int i=1;i<ADC_LEN_BUF;i++){
		sumAVG+=lbuf[i];
		if(minVal>lbuf[i]){
			minVal=lbuf[i];
		}
		}
		for(int i=1;i<ADC_LEN_BUF;i++){
		if(maxVal<lbuf[i]){
			maxVal=lbuf[i];
		}
		}
		sumAVG-=(minVal+maxVal);
		sumAVG/=2;
		lockGetValue=0;
	}
	return sumAVG;
}

float getInputVoltage(){
  return getValue(adc_result_buf.i_voltage)*reference_voltage/ADC_COEFF+0.0005;
//	return ((float) (result.i_voltage[0]+result.i_voltage[1]+result.i_voltage[2]+result.i_voltage[3]))*reference_voltage/ADC_COEFF+reference_shift;
}
float getInputCurrent(){
  return getValue(adc_result_buf.i_current)*reference_voltage/ADC_COEFF+0.0036;
//	return ((float) (result.i_current[0]+result.i_current[1]+result.i_current[2]+result.i_current[3]))*reference_voltage/ADC_COEFF+reference_shift;
}

float getOutputVoltage(){
  return getValue(adc_result_buf.o_voltage)*reference_voltage/ADC_COEFF+0.0036;
//	return ((float) (result.o_voltage[0]+result.o_voltage[1]+result.o_voltage[2]+result.o_voltage[3]))*reference_voltage/ADC_COEFF+reference_shift;
}
float getOutputCurrent(){
  return getValue(adc_result_buf.o_current)*reference_voltage/ADC_COEFF+0.0036;
//	return ((float) (result.o_current[0]+result.o_current[1]+result.o_current[2]+result.o_current[3]))*reference_voltage/ADC_COEFF+reference_shift;
}

void manageAdcResult(){
	if(FAULT_LOW){
			showSOS();
	}
	float adc_output_voltage =getOutputVoltage();
	float adc_output_current=getOutputCurrent();
	if( adc_output_voltage<TARGET_VALUE_MIN ){
		showUp();
//		pwm_up(PWM_VALUE * (TARGET_VALUE_MIN - result.output_voltage) );
		return;
	}
	if( adc_output_voltage>TARGET_VALUE_MAX ){
		showDown();
//		pwm_down(PWM_VALUE * (result.output_voltage - TARGET_VALUE_MIN));
		return;
	}
	if( adc_output_current>MAX_CURRENT ){
		showDown();
//		pwm_down(PWM_VALUE*(result.output_current - MAX_CURRENT));
		return;
	}
}

void ADC_IRQHandler()
{
  // Clear flag
  if(ADC1->SR & ADC_SR_JEOC){
      ADC1->SR &= ~ADC_SR_JEOC;
      if(++adc_result_buf.pos>ADC_LEN_BUF-1) adc_result_buf.pos = 0;
      adc_result_buf.i_voltage[adc_result_buf.pos]=ADC1->JDR1; // Input Voltage
      adc_result_buf.i_current[adc_result_buf.pos]=ADC1->JDR2; // Input Current
      adc_result_buf.o_voltage[adc_result_buf.pos]=ADC1->JDR3; // Output Voltage
      adc_result_buf.o_current[adc_result_buf.pos]=ADC1->JDR4; // Output Current
      adc_result_buf.count++;
      ADC1->CR2 |= ADC_CR2_JSWSTART;
  }
}


void InitADC(void){
    adc_result_buf.pos=3;
	lockGetValue=0;

      // Тактируем ADC
      RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

//*
// * Порт занят ADC, нам важно только прерывание
	// ADC1: Init GPio PB6
/*
	InitGPio( GPIOA,
				0,
				analog,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Порт на 05.05.2023 не используется
//				noPull,
				0); //AF:2
*/
	InitGPio( GPIOA,
				2,
				analog,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull,
				0); //AF:2
	// Check output voltage ADC1 CH4
	InitGPio( GPIOA,
				4,
				analog,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull,
				0); //AF:2
	// Check output current ADC1 CH5
	InitGPio( GPIOA,
				5,
				analog,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull,
				0); //AF:2
//*/

	/************************* Start TimerADC Init ****************************************/
	// Тактируем TIM4
//*
//	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
//	//	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
//	ADCTimer->PSC = 1-1;
//	ADCTimer->ARR = 100000-1;
//*/
	//Enable - the Counter enable signal, CNT_EN, is used as a trigger output (TRGO).
	ADCTimer->CR2 |= TIM_CR2_MMS_2;
	ADCTimer->CR1 |= TIM_CR1_CEN;

	/************************* End TimerADC Init ****************************************/

	// отключим АЦП
	ADC1->CR2 &= ~ADC_CR2_ADON;

	// На всякий случай всем каналам одинаковое кол-во тактов сэмплирования - 15 тактов
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP5_1;
	/*---------------- Разрешаем запуск преобразования по триггеру-----------------------*/
	//  JEXTEN=10 выбираем спадающий фронт
//	ADC1->CR2 &= ~ADC_CR2_JEXTEN_Msk;
//	ADC1->CR2 |= ADC_CR2_JEXTEN_1;

	/*--------------------------- End Section  -----------------------------------------*/


	/*-------------------- Выбираем триггер для запуска преобразования ----------------*/

	// Сбрасываем в default: TIM1->CC4
	ADC1->CR2 &= ~ADC_CR2_JEXTSEL_Msk;

	// JEXTSEL=0001: Timer 1 TRGO event
	// ADC1->CR2 |= ADC_CR2_JEXTSEL_0;

	// JEXTSEL=1001: TIM4_TRGO event
	// ADC1->CR2 |= ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTSEL_3;

	// JEXTSEL=1011: TIM5_TRGO event
	// ADC1->CR2 |= ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTSEL_1 | ADC_CR2_JEXTSEL_3;

	/*--------------------------- End Section  -----------------------------------------*/
	 ADC1->JOFR2=2;
//	 ADC1->JOFR4=32;

	// Одно преобразование последовательности, возможно до 4
	ADC1->JSQR &= ~( ADC_JSQR_JL_Msk );

	//ADC setting to the injection channel
	// По умолчанию, когда  ADC_JSQR_JL=0 только один канал работает из 4 и это JSQR4
	// 20: Работают 4 канала
	ADC1->JSQR |= ADC_JSQR_JL;
	//Работают 3 канала, кроме JSQ1
//	ADC1->JSQR |= ADC_JSQR_JL_1;

	ADC1->JSQR &= ~(ADC_JSQR_JSQ1_Msk | ADC_JSQR_JSQ2_Msk | ADC_JSQR_JSQ3_Msk | ADC_JSQR_JSQ4_Msk);
//	ADC1->JSQR |= ADC_JSQR_JSQ1_0 | ADC_JSQR_JSQ2_0 | ADC_JSQR_JSQ3_0 | ADC_JSQR_JSQ4_0;
	// В JSQ1 -> 0, т.е. третий канал  PA0 - уже установлено в 0 командой выше
	// В JSQ2 -> 2, т.е. третий канал  PA2
	// В JSQ3 -> 4, т.е. второй канал  PA4
	// В JSQ4 -> 5, т.е. нулевой канал PA5
	ADC1->JSQR |= ADC_JSQR_JSQ2_1| ADC_JSQR_JSQ3_2| ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_2;
	//
//	ADC1->CR1 &=~ ADC_CR1_JDISCEN;
	ADC1->CR1 |= ADC_CR1_SCAN;
	// Enable interrupt
	ADCUp(ADC1);
	// ADC1->CR2 |= ADC_CR2_JSWSTART;
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	// Enable injector conversion
	ADC1->CR1 |= ADC_CR1_JEOCIE;
//	//Enable Interrupt ADC1
	NVIC_EnableIRQ(ADC_IRQn);
	//Enable ADC1
}

