#include "main.h"

#include "rtc.h"

volatile RTC_struct rtc;

volatile uint8_t rtc_ticks;
volatile sRTC_DateType rtc_date;
volatile sRTC_TimeType rtc_time;

inline uint32_t active_rtc(){
	return RTC->ISR & RTC_ISR_INITS;
}


void RTC_Update(sRTC_DateType *date_ptr, sRTC_TimeType *time_ptr)
{
	uint32_t TR, DR;
	// Выключаем защиту от записи
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// INIT: Initialization mode.
	//	Входим в режим редактирования
	RTC->ISR |= RTC_ISR_INIT;

	// Ждем подтверждения входа в режим редактирования
	// Не более 1 мс
	rtc_ticks = 0;
	while( !(RTC->ISR & RTC_ISR_INITF) && rtc_ticks<1 );
	if(rtc_ticks>0) return;

	// Устанавливаем асинхронный предделитель на 100(99+1).
	RTC->PRER = (uint32_t)(99 << 16);
	// Установим синхронный предделитель на 400(399+1).
	RTC->PRER |= (uint32_t)399;

	// Устанавливаем время
	TR = (uint32_t)(time_ptr->h/10*16 + time_ptr->h%10) << 16 |
			(uint32_t)(time_ptr->m/10*16 + time_ptr->m%10) << 8 |
			(uint32_t)(time_ptr->s/10*16 + time_ptr->s%10);
	RTC->TR = TR;
	// Устанавливаем дату
	DR = (uint32_t)(date_ptr->y/10*16 + date_ptr->y%10) << 16  |
			(uint32_t)(date_ptr->w/10*16 + date_ptr->w%10) << 13 |
			(uint32_t)(date_ptr->m/10*16 + date_ptr->m%10) << 8 |
			(uint32_t)(date_ptr->d/10*16 + date_ptr->d%10);
	RTC->DR = DR;

	// Set 12 Hour, Default 24 Hour
	// RTC->CR |= RTC_CR_FMT;

	// Выходим из режима редактирования
	RTC->ISR &= ~(RTC_ISR_INIT);
	// Включаем защиту от записи
	// Путем записи неверного значения
	RTC->WPR = 0xFF;
}

void RTC_Init_LSI(sRTC_DateType *date_ptr, sRTC_TimeType *time_ptr)
{
	// Включаем LSI
	RCC->CSR |= RCC_CSR_LSION;
	// Ждем его готовности
	rtc_ticks = 0;
	while ( !(RCC->CSR & RCC_CSR_LSIRDY) && rtc_ticks<1);
	if(rtc_ticks>0) return;

	// Включаем тактирование модуля PWR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// После сброса регистры RTC находятся в защищенном домене
	// Для разрешения записи необходимо поставить флаг в PWR->CR
	PWR->CR |= PWR_CR_DBP;

	//	01: LSE oscillator clock used as the RTC clock
	//	10: LSI oscillator clock used as the RTC clock
	//	11: HSE oscillator clock divided by a programmable prescaler (selection through the
	//	RTCPRE[4:0] bits in the RCC clock configuration register (RCC_CFGR)) used as the RTC
	//	clock

	// Выбераем источник тактирования RTC
	// от низкоскоростного внутреннего источника LSI(40 kHz)
	RCC->BDCR |= RCC_BDCR_RTCSEL_1; // = LSI
	// Включаем тактиование RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	RTC_Update(date_ptr, time_ptr);
}

void RTC_Init_LSE(sRTC_DateType *date_ptr, sRTC_TimeType *time_ptr)
{
	//Проверяем тикают ли часики
	if(RTC->ISR & RTC_ISR_INITS) return;

    //Включаем LSE
	RCC->BDCR |= RCC_BDCR_LSEON;
	// Ждем его готовности
	rtc_ticks = 0;
	while (!(RCC->BDCR & RCC_BDCR_LSEON) && rtc_ticks<1);
	if(rtc_ticks>0) return;

	//	01: LSE oscillator clock used as the RTC clock (кварц 32768)
	//	10: LSI oscillator clock used as the RTC clock
	//	11: HSE oscillator clock divided by a programmable prescaler (selection through the
	//	RTCPRE[4:0] bits in the RCC clock configuration register (RCC_CFGR)) used as the RTC
	//	clock

	// Выбераем источник тактирования RTC
	// от низкоскоростного внутреннего источника LSI(40 kHz)
	RCC->BDCR |= RCC_BDCR_RTCSEL_0; // = LSE
	// Включаем тактиование RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	RTC_Update(date_ptr, time_ptr);
}

void RTC_Init_HSE(sRTC_DateType *date_ptr, sRTC_TimeType *time_ptr)
{
	//Проверяем тикают ли часики
	if(active_rtc()) return;

	// Включаем тактирование модуля PWR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// После сброса регистры RTC находятся в защищенном домене
	// Для разрешения записи необходимо поставить флаг в PWR->CR
	PWR->CR |= PWR_CR_DBP;


	// Выбераем источник тактирования RTC
	// от скоростного источника HSE
	RCC->BDCR |= RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCSEL_1; // = HSE
	// Включаем тактиование RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	RTC_Update(date_ptr, time_ptr);
}

void rtc_get_data(){
	uint8_t v;
	// Date
	rtc_date.d = ( (RTC->DR & 0x3F) >> 4 ) * 10 + (RTC->TR & 0x0F);
	v = (RTC->DR & 0x1F00) >> 8;
	rtc_date.m = ( (v & 0x1F) >> 4 ) * 10 + (v & 0x0F);
	v = (RTC->DR & 0xfF0000) >> 16;
	rtc_date.y = ( (v & 0xF0) >> 4 ) * 10 + (v & 0x0F);
	rtc_date.w = ( RTC->DR & 0xF000 ) >> 12;
	// Time
	rtc_time.s = ( (RTC->TR & 0x7F) >> 4 ) * 10 + (RTC->TR & 0x0F);
	v = (RTC->TR & 0x7F00) >> 8;
	rtc_time.m = ( (v & 0x7F) >> 4 ) * 10 + (v & 0x0F);
	v = (RTC->TR & 0x7F0000) >> 16;
	rtc_time.h = ( (v & 0x7F) >> 4 ) * 10 + (v & 0x0F);
}
