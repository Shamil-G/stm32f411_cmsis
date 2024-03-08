#pragma once

typedef struct{
	uint8_t h; // hour
	uint8_t m; // minute
	uint8_t s; // seconds
}
sRTC_TimeType;

typedef struct{
	uint8_t y; // year
	uint8_t m; // month
	uint8_t d; // day
	uint8_t w; // week
}
sRTC_DateType;

typedef struct sRTC_struct{
		 unsigned year : 6;
		 unsigned month : 4;
		 unsigned week : 3;
		 unsigned date : 5;
		 unsigned hour : 5;
		 unsigned minute :6;
		 unsigned sec : 6;
} RTC_struct;



extern volatile RTC_struct rtc;
extern volatile sRTC_DateType rtc_date;
extern volatile sRTC_TimeType rtc_time;
