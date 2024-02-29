#pragma once
/*
 * Author: Shamil Gusseynov
*/
void checkEncButton(void);
void checkEncValue(void);

#define EncTimer  	TIM3
#define EncTimerIRQ	TIM3_IRQn

typedef  enum{
  TIMER1 = 1,
  TIMER2 = 2
} SELECT_TIMER;

typedef  enum{
  Common = 0,
  PWM_FREQ,
  PWM_DUTY,
  CH_TIMER,
  Sinus
} Menu;

struct encValue{
	uint32_t prevValue;
	uint32_t prevCntMainTick;
	uint32_t prevMainTick;
};

typedef  enum{
  Select = 0,
  Edit
} MenuEdit;

extern MenuEdit item_menu_status;
extern SELECT_TIMER selected_timer;
extern Menu active_menu_item;

void EncoderOn(void);
void EncoderValue(void);
void freqUp(void);
void freqDown(void);
void pwm_up(void);
void pwm_down(void);
