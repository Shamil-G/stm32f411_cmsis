#include "main.h"
#include "stdio.h"
#include "fonts.h"

ST7735_Button button;
ST7735_ProgressBar pbar;
float    i_voltage, i_current;
float    o_voltage, o_current;
extern volatile uint32_t freqMeter;
extern uint32_t Bounce;
uint32_t prevTicks=0, curTicks;

char i_voltage_buf[48];
char i_current_buf[48];

char o_voltage_buf[48];
char o_current_buf[48];

char o_freq_buf[48];
char o_duty[48];
char o_status[200];

//typedef  enum{
//  Common = 0,
//  PWM_FREQ,
//  PWM_DUTY,
//  Sinus
//} Menu;
//Menu active_menu_item=Common;
//
//typedef  enum{
//  SelectMenu = 0,
//  SelectEdit
//} MenuEdit;
//MenuEdit item_menu_edit=SelectMenu;

MenuEdit prev_item_menu_status=Select;
MenuEdit item_menu_status=Select;
Menu active_menu_item=Common;
Menu prev_active_menu_item;
//uint8_t active_menu_item=0;

char 	*titleMenu[]={"Info", "Freq", "Duty", "Timer", "S"};
uint16_t pos_MenuTitle[]={5, 75, 150, 215, 300};

void ili9341_gpio_init(void){
  // RESET
  InitGPio( SPI_RESET_Port, SPI_RESET_Pin, output, push_pull, high, noPull, af0);
  PortReset(SPI_RESET_Port, SPI_RESET_Pin);
  // CS
  InitGPio( SPI_CS_Port, SPI_CS_Pin, output, push_pull, high, noPull, af0);
  PortSet(SPI_CS_Port, SPI_CS_Pin);
  // DC_Port -> data command
  InitGPio( SPI_DC_Port, SPI_DC_Pin, output, push_pull, high, noPull, af0);
  PortSet(SPI_DC_Port, SPI_DC_Pin);
}

void showTitle(){
  ili9341_SetRotation(1); //1 - albom, 0 portret
  Delay(100);
  ili9341_FillScreen(COLOR565_BLUE);
  ili9341_SetFont(&Font20);

  for(uint8_t i=0;i<4;i++){
      if(i==active_menu_item){
	  ili9341_SetTextColor(COLOR565_BLACK);
	  ili9341_SetBackColor(COLOR565_DARK_SEA_GREEN);
	  ili9341_String(pos_MenuTitle[i],(1-0.3)*lcdprop.pFont->Height,titleMenu[i]);
	  ili9341_SetTextColor(COLOR565_GOLD);
	  ili9341_SetBackColor(COLOR565_BLUE);
      }
      else{
	  ili9341_String(pos_MenuTitle[i],(1-0.3)*lcdprop.pFont->Height,titleMenu[i]);

      }
  }
  ili9341_SetFont(&Font16);
  ili9341_String(pos_MenuTitle[4],(1+0.2)*lcdprop.pFont->Height, (item_menu_status==Select?"S":"E"));
  prev_active_menu_item=active_menu_item;
}

void showTitleCommon(){
  ili9341_SetTextColor(COLOR565_GOLD);
  ili9341_SetBackColor(COLOR565_BLUE);
  ili9341_String(5,(3+0.3)*lcdprop.pFont->Height,"Voltage input:");
  ili9341_String(5,(4+0.3)*lcdprop.pFont->Height,"Current input:");
  ili9341_String(5,(6+0.3)*lcdprop.pFont->Height,"Voltage output:");
  ili9341_String(5,(7+0.3)*lcdprop.pFont->Height,"Current output:");
  ili9341_String(5,(9+0.3)*lcdprop.pFont->Height,"Frequency:");
//  ili9341_SetTextColor(COLOR565_GOLD);
//  ili9341_SetBackColor(COLOR565_BLACK);
}

void showCommon(){
  if(prev_active_menu_item!=active_menu_item){
//      showTitle();
      ili9341_primary_tune();
      prev_active_menu_item=active_menu_item;
  }
  i_voltage = getInputVoltage();
  i_current = getInputCurrent();
  o_voltage = getOutputVoltage();
  o_current = getOutputCurrent();

  sprintf(i_voltage_buf,"%f V ", i_voltage);
  sprintf(i_current_buf,"%f A ", i_current);
  sprintf(o_voltage_buf,"%f V ", o_voltage);
  sprintf(o_current_buf,"%f A ", o_current);

  sprintf(o_freq_buf," %ld Hz          ", getFreqMeter());

  ili9341_String(175,(3+0.3)*lcdprop.pFont->Height,i_voltage_buf);
  ili9341_String(175,(4+0.3)*lcdprop.pFont->Height,i_current_buf);
  ili9341_String(175,(6+0.3)*lcdprop.pFont->Height,o_voltage_buf);
  ili9341_String(175,(7+0.3)*lcdprop.pFont->Height,o_current_buf);
  ili9341_String(165,(9+0.3)*lcdprop.pFont->Height,o_freq_buf);
}

void showTitlePWM_FREQ(){
  ili9341_String(5,(4+0.3)*lcdprop.pFont->Height,"Freq:");
  ili9341_String(5,(6+0.3)*lcdprop.pFont->Height,"Duty:");
}
void showTitlePWM_DUTY(){
  ili9341_String(5,(4+0.3)*lcdprop.pFont->Height,"Freq:");
  ili9341_String(5,(6+0.3)*lcdprop.pFont->Height,"Duty:");
}
void showTitleTIMER(){
  ili9341_String(5,(4+0.3)*lcdprop.pFont->Height,"Timer:");
}
void showTIMER(){
  if(prev_active_menu_item!=active_menu_item){
//      showTitle();
      ili9341_primary_tune();
      prev_active_menu_item=active_menu_item;
  }
  else
      if( prev_item_menu_status!=item_menu_status){
	  ili9341_String(pos_MenuTitle[4],(1+0.2)*lcdprop.pFont->Height, (item_menu_status==Select?"S":"E"));
//	  ili9341_SetFont(&Font16);
	  prev_item_menu_status=item_menu_status;
  }
  sprintf(o_status,"%s ", (selected_timer==TIMER1)?"TIM1":"TIM2");
  ili9341_String(75,(4+0.3)*lcdprop.pFont->Height,o_status);
}

void showPWM_FREQ(){
  if(prev_active_menu_item!=active_menu_item){
//      showTitle();
      ili9341_primary_tune();
      prev_active_menu_item=active_menu_item;
  }
  else
      if( prev_item_menu_status!=item_menu_status){
	  ili9341_String(pos_MenuTitle[4],(1+0.2)*lcdprop.pFont->Height, (item_menu_status==Select?"S":"E"));
//	  ili9341_SetFont(&Font16);
	  prev_item_menu_status=item_menu_status;
  }
  sprintf(o_freq_buf,"%8ld Hz ", getFreqPWM());
  sprintf(o_duty," %0.2f%% ", getFreqDuty());
  ili9341_String(80,(4+0.3)*lcdprop.pFont->Height,o_freq_buf);
  ili9341_String(90,(6+0.3)*lcdprop.pFont->Height,o_duty);
}
void showPWM_DUTY(){
  if(prev_active_menu_item!=active_menu_item){
//      showTitle();
      ili9341_primary_tune();
      prev_active_menu_item=active_menu_item;
  }
  else
      if( prev_item_menu_status!=item_menu_status){
	  ili9341_String(pos_MenuTitle[4],(1-0.3)*lcdprop.pFont->Height, (item_menu_status==Select?"S":"E"));
//	  ili9341_SetFont(&Font16);
	  prev_item_menu_status=item_menu_status;
  }
  sprintf(o_freq_buf,"%8ld Hz ", getFreqPWM());
  sprintf(o_duty," %0.2f%% ", getFreqDuty());
  ili9341_String(80,(4+0.3)*lcdprop.pFont->Height,o_freq_buf);
  ili9341_String(90,(6+0.3)*lcdprop.pFont->Height,o_duty);
}

void showTitleSinus(){

}

void showSinus(){

}

void ili9341_primary_tune(){
  showTitle();
  switch (active_menu_item){
    case Common: showTitleCommon();
      break;
    case PWM_FREQ: showTitlePWM_FREQ();
      break;
    case PWM_DUTY: showTitlePWM_DUTY();
      break;
    case CH_TIMER: showTitleTIMER();
      break;
    case Sinus: showTitleSinus();
      break;
    default: showTitleCommon();
  }
  Delay(50);
}

void show_ili9341_monitor(){
  switch (active_menu_item){
    case Common: showCommon();
      break;
    case PWM_FREQ: showPWM_FREQ();
      break;
    case PWM_DUTY: showPWM_DUTY();
      break;
    case CH_TIMER: showTIMER();
      break;
    case Sinus: showSinus();
      break;
    default: showCommon();
  }
  // ?????????????? ???????????? ???????????? ??????????????
  curTicks=mainTick;
  uint32_t ticks=(mainTick-prevTicks)/100;
  sprintf(o_status,"%4ldms  %s %4ldkHz %4ld  ", ticks, (selected_timer==TIMER1)?"TIM1":"TIM2", adc_result_buf.adc_max_calc/ticks, Bounce );
  ili9341_String(7,210,o_status);
  adc_result_buf.adc_max_calc=0;
  prevTicks=curTicks;
}

