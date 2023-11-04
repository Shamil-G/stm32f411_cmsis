#include "main.h"

int monitorStarted=0;
SELECT_TIMER selected_timer=TIMER1;
void vTaskTim1(void *parameter);

int main(void){
      SystemUp();
//      initRCC_F411();
      InitMainTick();
//      init_SysTick();

      init_pwm();
      EncoderOn();

//      tim11_50Hz_init();
      tim1_gpio_init();
      tim1_init();
//      change_pwm_mode(sinusFifty);

      spi_gpio_init();
      ili9341_gpio_init();
      spi_master_init();

      dma_init(MasterSPI);
      spi2_dma_enable();
#ifndef USE_FREERTOS
      ili9341_init(240,320);
      ili9341_primary_tune();
#endif

	InitADC();
	FreqMeterOn();
//	Phase3_InitHrpwm();

#ifdef USE_FREERTOS
	xTaskCreate(vTaskMonitor, "ShowMonitor", 512, NULL, 2, NULL);
//	xTaskCreate(vTaskTim1, "SetTim1", 128, NULL, 1, NULL);
	xTaskCreate( vTaskLed1,
	"LED1", // Task Name
	16, 	// Stack Size, при 8 подвисало!
	NULL, 	// void* parameter
	1, 	// Priority freeRTOS растет вместе с номером, в NVIC все наоборот
	NULL	// TaskHandler_t *pxCreateTask
	);
	vTaskStartScheduler();
	while(1){
	    showSOS();
	}
#endif

#ifndef USE_FREERTOS
	while(1){
	  Delay(50);
	  show_ili9341_monitor();
	  showBip();
//	  pwm2_test();
  //	    SET_CH1_DUTY(incr);
	}
#endif
}

void vTaskLed1 (void *parameter){
  while(1)
  {
    showBip();
  }
}

void vTaskTim1(void *parameter){
  while(1){
//    pwm2_test();
#ifdef USE_FREERTOS
    vTaskDelay(10);
#endif
#ifndef USE_FREERTOS
	  Delay(100);
#endif
  }
}


void vTaskMonitor(void *parameter){
  while(1){
    if(!monitorStarted){
      monitorStarted=1;
      ili9341_init(240,320);
#ifdef  USE_FREERTOS
      vTaskDelay(100);
#endif
#ifndef USE_FREERTOS
	  Delay(100);
#endif
      ili9341_primary_tune();
#ifdef USE_FREERTOS
      vTaskDelay(100);
#endif
#ifndef USE_FREERTOS
	  Delay(100);
#endif
    }
    show_ili9341_monitor();
#ifdef USE_FREERTOS
    vTaskDelay(150);
#endif
#ifndef USE_FREERTOS
	  Delay(150);
#endif
  }
}

void pwm_up(void){
  if(selected_timer==TIMER2)
    pwm_tim2_up();
  if(selected_timer==TIMER1)
    pwm2_tim1_up(TIM1->ARR/20);
}

void pwm_down(void){
  if(selected_timer==TIMER2)
    pwm_tim2_down();
  if(selected_timer==TIMER1)
    pwm2_tim1_down(TIM1->ARR/20);
}

void freqUp(void){
  if(selected_timer==TIMER2)
    tim2_freqUp();
  if(selected_timer==TIMER1)
    tim1_freqUp();
}

void freqDown(void){
  if(selected_timer==TIMER2)
    tim2_freqDown();
  if(selected_timer==TIMER1)
    tim1_freqDown();
}
