#include "main.h"
#include "encoder.h"

// Button IRQ for Encoder -> PB_3
SELECT_TIMER selected_timer=TIMER2;

uint8_t lockEncoder;

uint16_t bounce_encoder;
uint16_t encoder_ticks;

extern void pwm_tune();

#define bounceButton 1000
#define bounceTurn 90


struct encValue EncValue;
//*
int is_bounce(uint16_t bounce){
  uint8_t ret = 0;
  if(encoder_ticks<bounce){
	  ret = 1;
	  bounce_encoder=encoder_ticks;
  }
  encoder_ticks=0;
  return ret;
}

void TIM3_IRQHandler(void){
  uint32_t tim3_irq = EncTimer->SR & TIM_SR_TIF;
  // Сбрасываем флаг прерывания
  EncTimer->SR &= ~TIM_SR_TIF;
  if(lockEncoder==0){
	  lockEncoder = 1;
	  if( tim3_irq && !is_bounce(bounceTurn)){
		  // DIR=0 go to to right, DIR>0 - go to left
		  uint8_t direction =  EncTimer->CR1 & TIM_CR1_DIR;

		  if(item_menu_status==Select){
			  if(direction==0 && active_menu_item!=Sinus && active_menu_item!=CH_TIMER){
				 active_menu_item+=1;
			  }
			  else if(direction>0 && active_menu_item!=Common){
				active_menu_item-=1;
			  }
		  }
		  else{
		  switch (active_menu_item){
			case Common:
			break;
			case PWM_FREQ:
			  if(direction==0) freqUp();
			  else
			  if(direction>0) freqDown();
			  // После установления новой частоты, надо обновить счетчик PWM - pwm_tune()
			break;
			case PWM_DUTY:
			  if(direction==0) pwm_up();
			  else
			  if(direction>0) pwm_down();
			break;
			case CH_TIMER:
				if(selected_timer==TIMER1){
					TIM1->CR1 &= ~TIM_CR1_CEN;
					TIM2->CR1 |= TIM_CR1_CEN;
					selected_timer=TIMER2;
				//Сбрасываем в default: TIM1->CC4
				ADC1->CR2 &= ~ADC_CR2_JEXTSEL_Msk;
				// JEXTSEL=0110: Timer 2 TRGO event
				ADC1->CR2 |= ADC_CR2_JEXTSEL_0;
				}
				else{
					TIM2->CR1 &= ~TIM_CR1_CEN;
					TIM1->CR1 |= TIM_CR1_CEN;
					selected_timer=TIMER1;
				//Сбрасываем в default: TIM1->CC4
				ADC1->CR2 &= ~ADC_CR2_JEXTSEL_Msk;
				// JEXTSEL=0001: Timer 1 TRGO event
				ADC1->CR2 |= ADC_CR2_JEXTSEL_0;
				}
			break;
			default: break;
		  }
		  }
	  }
	  lockEncoder = 0;
  }
}

// Button IRQ
void EXTI3_IRQHandler() {
  EXTI->PR |= EXTI_PR_PR3;
  if(lockEncoder==0){
	  lockEncoder=1;
//	  if( active_menu_item!=Common && !is_bounce() )
	  if( active_menu_item!=Common && !is_bounce(bounceButton))
		  item_menu_status=(item_menu_status==Select)?Edit:Select;
	  // Очистка прерывания
//	  EXTI->PR |= EXTI_PR_PR3;
	  lockEncoder=0;
  }
}
// Button IRQ
void EXTI15_10_IRQHandler() {
  EXTI->PR |= EXTI_PR_PR15;
  if(lockEncoder==0){
	  lockEncoder=1;
//	  if( active_menu_item!=Common && !is_bounce() )
	  if( active_menu_item!=Common && !is_bounce(bounceButton))
		  item_menu_status=(item_menu_status==Select)?Edit:Select;
	  // Очистка прерывания
//	  EXTI->PR |= EXTI_PR_PR3;
	  lockEncoder=0;
  }
}

void ButtonIRQOn(void){
//	EncValue.buttonStatus=1;
	InitGPio(
//				GPIOA,
				GPIOB, // Работало! когда реальное соединение было на PA3
				3,
				input,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Подтяжка pull-up внешняя, изначально порт на земле
				0); //AF:2
	InitGPio(
				GPIOA,
//				GPIOB, // Работало! когда реальное соединение было на PA3
				15,
				input,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull, // Подтяжка pull-up внешняя, изначально порт на земле
				0); //AF:2

	// Обяззательно тактирование!
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Если PA15, то
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA;;

	// Надо выбрать что то одно, либо РВ3 либо РА15
	EXTI->RTSR |= EXTI_RTSR_TR15;
	EXTI->FTSR &= ~EXTI_FTSR_TR15;
	EXTI->PR |= EXTI_PR_PR15;
	EXTI->IMR |= EXTI_IMR_MR15;
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	//Interrupt mask register
	//Последующие два регистра отвечают за прерывание по переднему фронту и заднему соответственно.
	//Мы будем реагировать на передний, поэтому следует настроить RTSR
	//Rising trigger selection register

	//	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA | SYSCFG_EXTICR1_EXTI3_PB;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
	EXTI->RTSR |= EXTI_RTSR_TR3;
	//Falling trigger selection register
	EXTI->FTSR &= ~EXTI_FTSR_TR3;

	// Очистка прерывания
	EXTI->PR |= EXTI_PR_PR3;
	//Регистр IMR отвечает за включение/отключение прерывания
	EXTI->IMR |= EXTI_IMR_MR3;
	NVIC_EnableIRQ(EXTI3_IRQn);
};

void EncoderOn(void){
//	EncValue.maxBounceUp=10;
//	EncValue.maxBounceDown=10;
	ButtonIRQOn();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// TIM3_CH1
	InitGPio( GPIOB,
				4,
				alternateF,    //MODER:00
				0,
				0,
				// Pull-UP, если питание идет с STM32
				// Pull-down, если питание идет с енкодера
				noPull,
				af2); //AF:2
	// TIM3_CH2
	InitGPio( GPIOB,
				5,
				alternateF,    //MODER:00
				0,
				0,
				noPull, // Pull-down
				af2); //AF:2
	lockEncoder = 0;
	// Включаем тактирование таймера
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Маппируем два входа Таймера для функции Encoder
	// CC1 channel is configured as input, IC1 is mapped on TI1
	// CC2 channel is configured as input, IC2 is mapped on TI2
	EncTimer->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	// Учитывать по передним фронтам: TI1FP1 noninverted, TI2FP2 noninverted
	// Ненвертирующий вход - это когда 1 считается как 1
	//	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	// Инвертирующий вход - это когда 1 считается как 0
	EncTimer->CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P;

	// Encoder mode 1: Порядок учета сигналов от ножек
	// Считать вверх/вниз по краю TI1FP1 в зависимости от TI2FP2
	CLEAR_BIT(EncTimer->SMCR, TIM_SMCR_SMS_Msk);
	EncTimer->SMCR |= TIM_SMCR_SMS_0;

	// Настраиваем дребезг - filter sampling
	TIM3->PSC = 100-1;
	// Загружаем счетчик
	EncTimer->ARR = 65000;
	// Очищаем счетчик
	EncTimer->CNT = 32000;
	// Увеличиваем период сэмплирования в 4 раза
	// [9:8] Clock division Tdts = Tck_int * 4
	//	TIM3->CR1 &= TIM_CR1_CKD_Msk;
		TIM3->CR1 |= TIM_CR1_CKD_1;

	// [3:2]	11: capture is done once every 8 events
//	EncTimer->CCMR1 |= TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC;
	// [7:4]	1111: fSAMPLING=fDTS/32, N=8 - IC1F: Input capture 1 filter
	EncTimer->CCMR1 |= TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk;

	// Выбираем триггер TS - trigger selection
	// 100: TI1 Edge Detector TI1F_ED
	EncTimer->SMCR &= ~TIM_SMCR_TS;
	EncTimer->SMCR |= TIM_SMCR_TS_2;

	// Разрешаем прерывания от таймера
	EncTimer->DIER |= TIM_DIER_TIE;

	// Auto reload-preload enable! must be for driving PWM
	EncTimer->CR1 |= TIM_CR1_ARPE;
	// Включаем таймер
	EncTimer->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(EncTimerIRQ);
}


//*/
