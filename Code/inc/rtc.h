#pragma once

typedef struct{
	uint8_t h; // hour
	uint8_t m; // minute
	uint8_t s; // seconds
}
sRTC_TimeType;

typedef struct{
	uint16_t y; // year
	uint8_t m; // month
	uint8_t d; // day
	uint8_t w; // week
}
sRTC_DateType;

extern volatile sRTC_DateType rtc_date;
extern volatile sRTC_TimeType rtc_time;

void rtc_init_date();
void rtc_get_data();
void RTC_Init_LSI();

