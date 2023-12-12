/*
 * spi_ili9341.h
 *
*/

//https://www.youtube.com/watch?v=F9iZjc_r6k0&list=PLu9BJ8Y5m4bSrp6WGYSAlRBp6NowvChqn&index=1&t=973s
#include "main.h"
#include "fonts.h"
#include "color565.h"

#ifndef INC_SPI_ILI9341_H_
#define INC_SPI_ILI9341_H_
// Size of LCD
// here type lcd is ili9341 (st7735)
extern uint16_t ili9341_WIDTH;
extern uint16_t ili9341_HEIGHT;

// The best - to name this pin as show below
//----------------------------------------------------------------------
#define SPI_RESET_Port 	GPIOA
#define SPI_RESET_Pin 	11
#define SPI_DC_Port	GPIOB
#define SPI_DC_Pin	12
#define SPI_CS_Port GPIOB
#define SPI_CS_Pin	9
// Command for manage LCD
#define RST_RESET()	PortReset(SPI_RESET_Port, SPI_RESET_Pin)
#define RST_SET() 	PortSet(SPI_RESET_Port, SPI_RESET_Pin)
#define CS_RESET() 	PortReset(SPI_CS_Port, SPI_CS_Pin)
#define CS_SET()	PortSet(SPI_CS_Port, SPI_CS_Pin)
//#define DC_COMMAND() 	PortReset(SPI_DC_Port, SPI_DC_Pin)
//#define DC_DATA()	PortSet(SPI_DC_Port, SPI_DC_Pin)

#define RESET_ACTIVE()	(SPI_RESET_Port->BSRR = 1 << (SPI_RESET_Pin+16) )
#define RESET_IDLE() 	(SPI_RESET_Port->BSRR = 1 << SPI_RESET_Pin )
#define CS_ACTIVE() 	(SPI_CS_Port->BSRR = 1 << (SPI_CS_Pin+16) )
#define CS_IDLE()	(SPI_CS_Port->BSRR = 1 << SPI_CS_Pin )
#define DC_COMMAND() 	(SPI_DC_Port->BSRR = 1 << (SPI_DC_Pin+16) )
#define DC_DATA()	(SPI_DC_Port->BSRR = 1 << SPI_DC_Pin )
//-------------------------------------------------------------------
#define GetRandomNumber	getRandomNumber
//-------------------------------------------------------------------
#define TFT9341_MADCTL_MY  0x80
#define TFT9341_MADCTL_MX  0x40
#define TFT9341_MADCTL_MV  0x20
#define TFT9341_MADCTL_ML  0x10
#define TFT9341_MADCTL_RGB 0x00
#define TFT9341_MADCTL_BGR 0x08
#define TFT9341_MADCTL_MH  0x04
#define TFT9341_ROTATION (TFT9341_MADCTL_MX | TFT9341_MADCTL_BGR)
//----------------------------------------------------------------
// Command for select area for insert pixel
#define ST7735_CASET   0x2A
#define ST7735_RASET   ((uint8_t)0x2B)
#define ST7735_RAMWR   ((uint8_t)0x2C)
#define ST7735_RAMRD   ((uint8_t)0x2E)
//-------------------------------------------------------------------
#define swap(a,b) {int16_t t=a;a=b;b=t;}
//-------------------------------------------------------------------
// Base function
void ili9341_init(uint16_t width, uint16_t hegth);
void ili9341_DrawPixel(int x, int y, uint16_t color);
void ili9341_ini(uint16_t w_size, uint16_t h_size);
void ili9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ili9341_FillScreen(uint16_t color);
void ili9341_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color);
void ili9341_SetTextColor(uint16_t color);
void ili9341_SetBackColor(uint16_t color);
void ili9341_SetFont(sFONT *pFonts);
void ili9341_DrawChar(uint16_t x, uint16_t y, uint8_t c);
void ili9341_String(uint16_t x,uint16_t y, char *str);
void ili9341_DrawRect(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
uint16_t ili9341_RandColor(void);
void ili9341_SetRotation(uint8_t r);
// I think here place have mistake
__STATIC_INLINE void DelayMicro(uint32_t __IO micros)
{
  micros *=(SystemCoreClock/1000000)/5;
  while(micros--);
};
//----------------------------------------------------------------------
typedef enum
{
	ST7735_TextAlign_Left = 0,
	ST7735_TextAlign_Center = 1,
	ST7735_TextAlign_Right = 2
} ST7735_TextAlign;
//---------------------------------------------------------------------
/* Choose color */
typedef struct
{
	uint16_t TextColor;
	uint16_t BackColor;
	sFONT *pFont;
}LCD_DrawPropTypeDef;
extern LCD_DrawPropTypeDef lcdprop;
//---------------------------------------------------------------------
typedef struct
{
	uint8_t x,y;//X,Y Coords of btn
	uint8_t h,w;//H,W of btn

	uint16_t BorderColor,OldBackGroundColor;//BorderColor
	uint8_t* txt;//Text on Btn
	uint8_t txt_length;//Length of string
	ST7735_TextAlign txt_Align;//Align of text
	LCD_DrawPropTypeDef lcdprop;
} ST7735_Button;
//---------------------------------------------------------------------
typedef struct
{
	uint16_t x,y;//X,Y Coords o
	uint16_t h,w;//Height,width

	uint16_t BorderColor,BackgroundColor,ProgressColor;//BorderColor
	uint16_t progress;
} ST7735_ProgressBar;
//---------------------------------------------------------------------
// Button Function
void ili7735_Button_StructInit(ST7735_Button* btn,uint8_t x,uint8_t y,char* txt,uint8_t length, sFONT* font, uint16_t BackColor, uint16_t TextColor, uint16_t BorderColor, uint16_t OldBackgrounColor);
void ili7735_Button_Draw(ST7735_Button* btn);
void ili7735_Button_Destroy(ST7735_Button* btn);
//---------------------------------------------------------------------
// ProgressBar Function
void ili7735_ProgressBar_Destroy(ST7735_ProgressBar* pb);
void ili7735_ProgressBar_Init(ST7735_ProgressBar* pb,uint16_t x,uint16_t y, uint16_t w, uint16_t h, uint16_t ProgressColor, uint16_t BorderColor, uint16_t BackgroundColor);
void ili7735_ProgressBar_SetProgress(ST7735_ProgressBar* pb, uint16_t progress);

#endif
