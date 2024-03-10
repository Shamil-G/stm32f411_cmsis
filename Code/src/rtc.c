#include "main.h"

#include "rtc.h"

volatile uint8_t rtc_ticks;
volatile sRTC_DateType rtc_date;
volatile sRTC_TimeType rtc_time;

volatile sRTC_DateType set_rtc_date;
volatile sRTC_TimeType set_rtc_time;

inline uint32_t active_rtc(){
	return RTC->ISR & RTC_ISR_INITS;
}


void RTC_Update()
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
	TR = ((set_rtc_time.h/10) << RTC_TR_HT_Pos) | ((set_rtc_time.h%10) << RTC_TR_HU_Pos) |
			((set_rtc_time.m/10) << RTC_TR_MNT_Pos) | ((set_rtc_time.m%10) << RTC_TR_MNU_Pos) |
			((set_rtc_time.s/10) << RTC_TR_ST_Pos) | ((set_rtc_time.s%10) << RTC_TR_SU_Pos);

	RTC->TR = TR;
	// Устанавливаем дату
	DR  = set_rtc_date.w << RTC_DR_WDU_Pos |
			((set_rtc_date.y/10) << RTC_DR_YT_Pos) | ((set_rtc_date.y%10) << RTC_DR_YU_Pos) |
			((set_rtc_date.m/10) << RTC_DR_MT_Pos) | ((set_rtc_date.m%10) << RTC_DR_MU_Pos) |
			((set_rtc_date.d/10) << RTC_DR_DT_Pos) | ((set_rtc_date.d%10) << RTC_DR_DU_Pos);

	RTC->DR = DR;

	// Set 12 Hour, Default 24 Hour
	// RTC->CR |= RTC_CR_FMT;

	// Выходим из режима редактирования
	RTC->ISR &= ~(RTC_ISR_INIT);
	// Включаем защиту от записи
	// Путем записи неверного значения
	RTC->WPR = 0xFF;
}

void RTC_Init_LSI()
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

	RTC_Update();
}

void RTC_Init_LSE()
{
//	if(active_rtc()) return;

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

	RTC_Update(&rtc_date, &rtc_time);
}

void RTC_Init_HSE()
{
	//Проверяем тикают ли часики
//	if(active_rtc()) return;

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

	RTC_Update(&rtc_date, &rtc_time);
}

void rtc_init_date(){
	set_rtc_time.h=23;
	set_rtc_time.m=58;
	set_rtc_time.s=56;

	set_rtc_date.y=2024 - 2000;
	set_rtc_date.m=3;
	set_rtc_date.d=9;
	set_rtc_date.w=6;
}

void rtc_get_data(){
	// Чтение текущего времени RTC блокирует значения в теневых регистрах календаря до тех пор,
	// пока не будет прочитана текущая дата

	// Time
	rtc_time.s = ( (RTC->TR & RTC_TR_ST) >> RTC_TR_ST_Pos ) * 10 + ((RTC->TR & RTC_TR_SU) >> RTC_TR_SU_Pos);
	rtc_time.m = ( (RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos ) * 10 + ((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos);
	rtc_time.h = ( (RTC->TR & RTC_TR_HT) >> RTC_TR_HT_Pos ) * 10 + ((RTC->TR & RTC_TR_HU) >> RTC_TR_HU_Pos);
	// Date
	rtc_date.y = ( (RTC->DR & RTC_DR_YT) >> RTC_DR_YT_Pos ) * 10 + ((RTC->DR & RTC_DR_YU) >> RTC_DR_YU_Pos) + 2000;
	rtc_date.m = ( (RTC->DR & RTC_DR_MT) >> RTC_DR_MT_Pos ) * 10 + ((RTC->DR & RTC_DR_MU) >> RTC_DR_MU_Pos);
	rtc_date.d = ( (RTC->DR & RTC_DR_DT) >> RTC_DR_DT_Pos ) * 10 + ((RTC->DR & RTC_DR_DU) >> RTC_DR_DU_Pos);
	rtc_date.w = ( RTC->DR & RTC_DR_WDU ) >> RTC_DR_WDU_Pos;
}
