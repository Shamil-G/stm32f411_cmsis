/*
 * spi_ili9341.c
 *
 *  Created on: 26 окт. 2020 г.
 *  Author: Shamil Gusseynov
 */

#include "main.h"
#include "spi.h"
#include "spi_ili9341.h"
#include "fonts.h"
#include "string.h"
#include "stdlib.h"

/*
#include "font24.c"
#include "font20.c"
#include "font16.c"
#include "font12.c"
#include "font8.c"
*/
//extern SPI_HandleTypeDef hspi2;
//extern RNG_HandleTypeDef hrng;

//#define HSPI hspi2
//#define hspi hspi2
//-------------------------------
uint16_t ili9341_WIDTH;
uint16_t ili9341_HEIGHT;

#define SPI_ENABLE()	    spi2_enable()
#define SPI_READY()	    	spi2_ready()
#define END_OPERATION()		spi2_end_operation()
#define SPI_WriteData		SPI2_WriteData


#define TRANSMIT_CMD  0
#define TRANSMIT_DATA 1
#define TRANSMIT_NONE -1

uint8_t type_send=TRANSMIT_NONE;
LCD_DrawPropTypeDef lcdprop;
extern uint32_t spi_ticks;
void spi2_clear_rx(void);

/*-------------------------------------------------------*/
void ili9341_SendCommand(uint8_t cmd){
  if(type_send!=TRANSMIT_CMD){
		if (SPI_READY()==0)
		{
		  SPI_ENABLE();
		}
		spi_ticks=0;
		while(SPI2->SR & SPI_SR_BSY) // WAIT END_OPERATION
			if(spi_ticks>=5000) return;
		type_send=TRANSMIT_CMD;
		SPI_DC_Port->BSRR = 1 << (SPI_DC_Pin+16); // DC_COMMAND(); GO Down
  }
  SPI_WriteData(&cmd, 1, 5000);
  spi2_clear_rx();
}

static void ili9341_SendData(uint8_t* buff, uint16_t buff_size){
    if(type_send!=TRANSMIT_DATA){
		spi_ticks=0;
		while(SPI2->SR & SPI_SR_BSY) // WAIT END_OPERATION
			if(spi_ticks>=5000) return;

		type_send=TRANSMIT_DATA;
		//#define SPI_DC_Port	GPIOB
		//#define SPI_DC_Pin	12
		SPI_DC_Port->BSRR = 1 << SPI_DC_Pin; // DC_DATA(); GO Up
    }
    SPI_WriteData(buff, buff_size, 10000);
}
//-------------------------------------------------------------------
static void ili9341_SendDataByte(uint8_t buff){
    if(type_send!=TRANSMIT_DATA){
		spi_ticks=0;
		while(SPI2->SR & SPI_SR_BSY) // WAIT END_OPERATION
			if(spi_ticks>=5000) return;

		type_send=TRANSMIT_DATA;
		SPI_DC_Port->BSRR = 1 << SPI_DC_Pin; // DC_DATA(); GO Up
    }
    SPI_WriteData(&buff, 1, 10000);
}
//--------------------------------------------------------------------
void ili9341_reset(void){
	//#define SPI_RESET_Port 	GPIOA
	//#define SPI_RESET_Pin 	11
	SPI_RESET_Port->BSRR = 1 << (SPI_RESET_Pin+16); // Go Down
	Delay(5);
	SPI_RESET_Port->BSRR = 1 << SPI_RESET_Pin; 		// Go Up
}
//-------------------------------------------------------------------
static void ili9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  // column address set
  ili9341_SendCommand(0x2A); // CASET
  {
    uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
    ili9341_SendData(data, sizeof(data));
  }

  // row address set
  ili9341_SendCommand(0x2B); // RASET
  {
    uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
    ili9341_SendData(data, sizeof(data));
  }

  // write to RAM
  ili9341_SendCommand(0x2C); // RAMWR
}//-------------------------------------------------------------------
void ili7735_AddrSet(uint16_t XS, uint16_t YS, uint16_t XE, uint16_t YE)
{
  ili9341_SendCommand(ST7735_CASET); // Column address set
//  DC_DATA();
  ili9341_SendDataByte(0x00);
  ili9341_SendDataByte(XS);
  ili9341_SendDataByte(0x00);
  ili9341_SendDataByte(XE);

  ili9341_SendCommand(ST7735_RASET); // Row address set
//  DC_DATA();
  ili9341_SendDataByte(0x00);
  ili9341_SendDataByte(YS);
  ili9341_SendDataByte(0x00);
  ili9341_SendDataByte(YE);
  ili9341_SendCommand(ST7735_RAMWR); // Memory write
}

void ili9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  if((x1 >= ili9341_WIDTH) || (y1 >= ili9341_HEIGHT) || (x2 >= ili9341_WIDTH) || (y2 >= ili9341_HEIGHT)) return;
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
  ili9341_SetAddrWindow(x1, y1, x2, y2);
  //ili7735_AddrSet(x1, y1, x2, y2);

  uint8_t data[] = { color >> 8, color & 0xFF };
  //DC_DATA();
  for(uint32_t i = 0; i < (x2-x1+1)*(y2-y1+1); i++)
  {
	  ili9341_SendDataByte(data[0]);
	  ili9341_SendDataByte(data[1]);
      //HAL_SPI_Transmit(&hspi2, &data[0], 2, HAL_MAX_DELAY);
  }
}
//-------------------------------------------------------------------
void ili9341_FillScreen(uint16_t color)
{
  ili9341_FillRect(0, 0, ili9341_WIDTH-1, ili9341_HEIGHT-1, color);
}
uint16_t ili9341_RandColor(void)
{
  return (mainTick & 0x0000FFFF);
//	return HAL_GetTick() & 0x0000FFFF;
}
//-----------------------------------------------------
void ili9341_SetTextColor(uint16_t color)
{
  lcdprop.TextColor=color;
}
//------------------------------------
void ili9341_SetBackColor(uint16_t color)
{
  lcdprop.BackColor=color;
}
//------------------------------------
void ili9341_SetFont(sFONT *pFonts)
{
  lcdprop.pFont=pFonts;
}
//-------------------------------------------------------------------
void ili9341_DrawChar(uint16_t x, uint16_t y, uint8_t c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t offset;
  uint8_t *c_t;
  uint8_t *pchar;
  uint32_t line=0;
  height = lcdprop.pFont->Height;
  width  = lcdprop.pFont->Width;
  offset = 8 *((width + 7)/8) -  width ;
  c_t = (uint8_t*) &(lcdprop.pFont->table[(c-' ') * lcdprop.pFont->Height * ((lcdprop.pFont->Width + 7) / 8)]);
  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c_t + (width + 7)/8 * i);
    switch(((width + 7)/8))
    {
      case 1:
          line =  pchar[0];
          break;
      case 2:
          line =  (pchar[0]<< 8) | pchar[1];
          break;
      case 3:
      default:
        line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
        break;
    }
    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1)))
      {
        ili9341_DrawPixel((x + j), y, lcdprop.TextColor);
      }
      else
      {
        ili9341_DrawPixel((x + j), y, lcdprop.BackColor);
      }
    }
    y++;
  }
}
//-------------------------------------------------------------------
void ili9341_String(uint16_t x,uint16_t y, char *str)
{
  while(*str)
  {
    ili9341_DrawChar(x,y,str[0]);
    x+=lcdprop.pFont->Width;
    (void)*str++;
  }
}
//-------------------------------------------------------------------
void ili9341_SetRotation(uint8_t r)
{
  ili9341_SendCommand(0x36);
  switch(r)
  {
    case 0:
      ili9341_SendDataByte(0x48);
      ili9341_WIDTH = 240;
      ili9341_HEIGHT = 320;
      break;
    case 1:
      ili9341_SendDataByte(0x28);
      ili9341_WIDTH = 320;
      ili9341_HEIGHT = 240;
      break;
    case 2:
      ili9341_SendDataByte(0x88);
      ili9341_WIDTH = 240;
      ili9341_HEIGHT = 320;
      break;
    case 3:
      ili9341_SendDataByte(0xE8);
      ili9341_WIDTH = 320;
      ili9341_HEIGHT = 240;
      break;
  }
}
//-------------------------------------------------------------------
void ili9341_DrawPixel(int x, int y, uint16_t color)
{
	if((x<0)||(y<0)||(x>=ili9341_WIDTH)||(y>=ili9341_HEIGHT)) return;
	ili9341_SetAddrWindow(x,y,x,y);
	ili9341_SendCommand(0x2C);
	ili9341_SendDataByte(color>>8);
	ili9341_SendDataByte(color & 0xFF);
}
//------------------------------------------------------
void ili9341_DrawLine(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int steep = abs(y2-y1)>abs(x2-x1);
  if(steep)
  {
    swap(x1,y1);
    swap(x2,y2);
  }
  if(x1>x2)
  {
    swap(x1,x2);
    swap(y1,y2);
  }
  int dx,dy;
  dx=x2-x1;
  dy=abs(y2-y1);
  int err=dx/2;
  int ystep;
  if(y1<y2) ystep=1;
  else ystep=-1;
  for(;x1<=x2;x1++)
  {
    if(steep) ili9341_DrawPixel(y1,x1,color);
    else ili9341_DrawPixel(x1,y1,color);
    err-=dy;
    if(err<0)
    {
      y1 += ystep;
      err+=dx;
    }
  }
}
//-----------------------------------------------
void ili9341_DrawRect(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	ili9341_DrawLine(color,x1,y1,x2,y1);
	ili9341_DrawLine(color,x2,y1,x2,y2);
	ili9341_DrawLine(color,x1,y1,x1,y2);
	ili9341_DrawLine(color,x1,y2,x2,y2);
}
//---------------------------------------------------------------------
//-------------------------------------------------------------------
void ili9341_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color)
{
	int f = 1-r;
	int ddF_x=1;
	int ddF_y=-2*r;
	int x = 0;
	int y = r;
	ili9341_DrawPixel(x0,y0+r,color);
	ili9341_DrawPixel(x0,y0-r,color);
	ili9341_DrawPixel(x0+r,y0,color);
	ili9341_DrawPixel(x0-r,y0,color);
	while (x<y)
	{
		if (f>=0)
		{
			y--;
			ddF_y+=2;
			f+=ddF_y;
		}
		x++;
		ddF_x+=2;
		f+=ddF_x;
		ili9341_DrawPixel(x0+x,y0+y,color);
		ili9341_DrawPixel(x0-x,y0+y,color);
		ili9341_DrawPixel(x0+x,y0-y,color);
		ili9341_DrawPixel(x0-x,y0-y,color);
		ili9341_DrawPixel(x0+y,y0+x,color);
		ili9341_DrawPixel(x0-y,y0+x,color);
		ili9341_DrawPixel(x0+y,y0-x,color);
		ili9341_DrawPixel(x0-y,y0-x,color);
	}
}
//--------------------------------------------------------------------
void ili9341_FontsIni(void)
{
  Font8.Height = 8;
  Font8.Width = 5;
  Font12.Height = 12;
  Font12.Width = 7;
  Font16.Height = 16;
  Font16.Width = 11;
  Font20.Height = 20;
  Font20.Width = 14;
  Font24.Height = 24;
  Font24.Width = 17;
  lcdprop.BackColor=COLOR565_BLACK;
  lcdprop.TextColor=COLOR565_GREEN;
  lcdprop.pFont=&Font16;
}
//https://narodstream.ru/stm-urok-179-displej-tft-240x320-spi-chast-1/
void ili9341_init(uint16_t w_size, uint16_t h_size){
    uint8_t data[16];
    SPI2->CR1 |= SPI_CR1_SPE; 					// SPI2 Enable
    SPI_CS_Port->BSRR = 1 << (SPI_CS_Pin+16); 	// CS_ACTIVE()

    // HardReset	// SPI_RESET_Port->BSRR = 1 << (SPI_RESET_Pin+16)
    ili9341_reset();	// HAL_Delay(5);
			// RESET_IDLE();
    // SoftReset
    ili9341_SendCommand(0x01);
//    data[0] = 0x00;
//    ili9341_SendData(data, 1);
    // -----------------------
    Delay(1000);
    //Power Control A
    data[0] = 0x39;
    data[1] = 0x2C;
    data[2] = 0x00;
    data[3] = 0x34;
    data[4] = 0x02;
    ili9341_SendCommand(0xCB);
    ili9341_SendData(data, 5);
    //------------------------
    //Power Control B
    data[0] = 0x00;
    data[1] = 0xC1;
    data[2] = 0x30;
    ili9341_SendCommand(0xCF);
    ili9341_SendData(data, 3);
    //------------------------
    //Driver timing control A
    data[0] = 0x85;
    data[1] = 0x00;
    data[2] = 0x78;
    ili9341_SendCommand(0xE8);
    ili9341_SendData(data, 3);
    //------------------------
    //Driver timing control B
    data[0] = 0x00;
    data[1] = 0x00;
    ili9341_SendCommand(0xEA);
    ili9341_SendData(data, 2);
    //------------------------
    //Power on Sequence control
    data[0] = 0x64;
    data[1] = 0x03;
    data[2] = 0x12;
    data[3] = 0x81;
    ili9341_SendCommand(0xED);
    ili9341_SendData(data, 4);
    //------------------------
    //Pump ratio control
    data[0] = 0x20;
    ili9341_SendCommand(0xF7);
    ili9341_SendData(data, 1);
    //------------------------
    //Power Control,VRH[5:0]
    // data[0] = 0x10;			!!!!!!!!   17.02.2024
    data[0] = 0x23;
    ili9341_SendCommand(0xC0);
    ili9341_SendData(data, 1);
    //------------------------
    //Power Control,SAP[2:0];BT[3:0]
    data[0] = 0x10;
    ili9341_SendCommand(0xC1);
    ili9341_SendData(data, 1);
    //------------------------
    //VCM Control 1
    data[0] = 0x3E;
    data[1] = 0x28;
    ili9341_SendCommand(0xC5);
    ili9341_SendData(data, 2);
    //------------------------
    //VCM Control 2
    data[0] = 0x86;
    ili9341_SendCommand(0xC7);
    ili9341_SendData(data, 1);
    //------------------------
    //Memory Acsess Control
    data[0] = 0x48;				// Ориентация дисплея - вертикальная
    ili9341_SendCommand(0x36);
    ili9341_SendData(data, 1);
    //------------------------
    //Pixel Format Set
    data[0] = 0x55;				// 16 бит входной, 16 бит выходной
    ili9341_SendCommand(0x3A);
    ili9341_SendData(data, 1);
    //------------------------
    //Frame Rratio Control, Standard RGB Color
    data[0] = 0x00;
    data[1] = 0x18;
    ili9341_SendCommand(0xB1);
    ili9341_SendData(data, 2);
    //------------------------
    //Display Function Control
    data[0] = 0x08;
    data[1] = 0x82;
    data[2] = 0x27;//320 строк
    ili9341_SendCommand(0xB6);
    ili9341_SendData(data, 3);
    //------------------------
    // Отключаем гамма-коррекцию
    data[0] = 0x00;				//не включаем
    ili9341_SendCommand(0xF2);
    ili9341_SendData(data, 1);
    //--------------------------
    // Выбор кривой гамма-коррекции
    data[0] = 0x01;			//Gamma Curve (G2.2) (Кривая цветовой гаммы)
    ili9341_SendCommand(0x26);
    ili9341_SendData(data, 1);
    // Positive Gamma  Correction
    // Коррекция плавности изменения яркости в зависимости от прилагаемого напряжения
    data[0] = 0x0F;
    data[1] = 0x31;
    data[2] = 0x2B;
    data[3] = 0x0C;
    data[4] = 0x0E;
    data[5] = 0x08;
    data[6] = 0x4E;
    data[7] = 0xF1;
    data[8] = 0x37;
    data[9] = 0x07;
    data[10] = 0x10;
    data[11] = 0x03;
    data[12] = 0x0E;
    data[13] = 0x09;
    data[14] = 0x00;
    ili9341_SendCommand(0xE0);
    ili9341_SendData(data, 15);
    //-------------------------
    //Negative Gamma  Correction
    data[0] = 0x00;
    data[1] = 0x0E;
    data[2] = 0x14;
    data[3] = 0x03;
    data[4] = 0x11;
    data[5] = 0x07;
    data[6] = 0x31;
    data[7] = 0xC1;
    data[8] = 0x48;
    data[9] = 0x08;
    data[10] = 0x0F;
    data[11] = 0x0C;
    data[12] = 0x31;
    data[13] = 0x36;
    data[14] = 0x0F;
    ili9341_SendCommand(0xE1);
    ili9341_SendData(data, 15);
    //-------------------------
    ili9341_SendCommand(0x11); //Выйдем из спящего режима
    //-------------------------
    Delay(120);
    // включим дисплей, настроив нужную ориентацию экрана
    data[0] = TFT9341_ROTATION;
    ili9341_SendCommand(0x29);
    ili9341_SendData(data, 1);
    ili9341_WIDTH = w_size;
    ili9341_HEIGHT = h_size;
    // Инициализируем шрифты
    ili9341_FontsIni();
}

//-------------------------------------------------------------------------------------------
//BUTTON
void ili7735_Button_Draw(ST7735_Button* btn)
{
	uint16_t tmp,i;
	uint16_t text_pos=0;
	uint8_t BCH,BCL;

	ili9341_DrawRect(btn->BorderColor,btn->x,btn->y,btn->x+btn->w,btn->y+btn->h);
	ili9341_SetAddrWindow(btn->x+1,btn->y+1,btn->x+(btn->w-1),btn->y+(btn->h-1));
	tmp = btn->w * btn->h;

	BCL = btn->lcdprop.BackColor & 0xFF;
	BCH = (btn->lcdprop.BackColor>>8) & 0xFF;
	for (i=0;i<tmp;i++)
	{
		ili9341_SendDataByte(BCH);
		ili9341_SendDataByte(BCL);
	}

	ili9341_SetTextColor(btn->lcdprop.TextColor);
    ili9341_SetBackColor(btn->lcdprop.BackColor);
	if(btn->txt_Align == ST7735_TextAlign_Center)
	{
		text_pos = 1 + (((btn->txt_length-strlen((char*)btn->txt)) * lcdprop.pFont->Width)>>1);
	}
	else
		if(btn->txt_Align == ST7735_TextAlign_Left)
		{
			text_pos=2;
		}
		else
			if(btn->txt_Align == ST7735_TextAlign_Right)
			{
				text_pos = ((btn->txt_length-strlen((char*)btn->txt)) * lcdprop.pFont->Width)-2;
			}
	ili9341_String(btn->x+text_pos,btn->y+2,(char*)btn->txt);
}
//------
void ili7735_Button_StructInit(ST7735_Button* btn,uint8_t x,uint8_t y,char* txt,uint8_t length, sFONT* font,uint16_t BackColor, uint16_t TextColor, uint16_t BorderColor, uint16_t OldBackGroundColor)
{
	btn->BorderColor = BorderColor;
	btn->lcdprop.BackColor = BackColor;
	btn->lcdprop.TextColor = TextColor;
	btn->OldBackGroundColor = OldBackGroundColor;
	btn->lcdprop.pFont = font;

	btn->txt_Align = ST7735_TextAlign_Center;
	btn->x = x;
	btn->y = y;
	btn->w = length*(font->Width) + 3;
	btn->h = font->Height+2;
	btn->txt =(uint8_t *) txt;
	btn->txt_length = length;
}
//-------
void ili7735_Button_Destroy(ST7735_Button* btn)
{
	uint8_t BCL,BCH;
	uint16_t tmp,i;

	//ST7735_AddrSet(btn->x,btn->y,btn->x+btn->w,btn->y+btn->h+2);
	ili9341_SetAddrWindow(btn->x,btn->y,btn->x+btn->w+2,btn->y+btn->h+2);

	tmp = (btn->w+2) * (btn->h+2);

	BCL = btn->OldBackGroundColor & 0xFF;
	BCH = (btn->OldBackGroundColor>>8) & 0xFF;
	for (i=0;i<tmp;i++)
	{
		ili9341_SendDataByte(BCH);
		ili9341_SendDataByte(BCL);
	}
}
//ProgressBar
void ili7735_ProgressBar_Draw(ST7735_ProgressBar* pb)
{
	uint16_t tmp,i;
	uint8_t CH,CL;
	uint8_t BCH,BCL;
	ili9341_DrawRect(pb->BorderColor, pb->x, pb->y, pb->x + pb->w+2, pb->y+pb->h+2);
	ili9341_SetAddrWindow(pb->x+1,pb->y+1,pb->x+pb->w,pb->y+pb->h);

	//ST7735_DrawRectangle(pb->x,pb->y,pb->w,pb->h,pb->BorderColor);
	//ST7735_AddrSet(pb->x+1,pb->y+1,pb->x+pb->w-2,pb->y+pb->h-2);

	BCL = pb->BackgroundColor & 0xFF;
	BCH = (pb->BackgroundColor>>8) & 0xFF;

	tmp = (pb->w) * (pb->h);
	for (i=0;i<tmp;i++)
	{
		ili9341_SendDataByte(BCH);
		ili9341_SendDataByte(BCL);
	}

	//ST7735_AddrSet(pb->x+1,pb->y+1,pb->x+tmp-1,pb->y+pb->h-1);
	ili9341_SetAddrWindow(pb->x+2,pb->y+2,pb->x+pb->progress,pb->y+pb->h);

	tmp = pb->h*pb->progress;
	CL = pb->ProgressColor & 0xFF;
	CH = (pb->ProgressColor>>8) & 0xFF;
	for (i=0;i<tmp;i++)
	{
		ili9341_SendDataByte(CH);
		ili9341_SendDataByte(CL);
	}
}
void ili7735_ProgressBar_SetProgress(ST7735_ProgressBar* pb,uint16_t progress)
{
	pb->progress = progress>pb->w?pb->w:progress;
	ili7735_ProgressBar_Draw(pb);
}

void ili7735_ProgressBar_Init(ST7735_ProgressBar* pb, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t ProgressColor, uint16_t BorderColor, uint16_t BackgroundColor)
{
	pb->x = x;
	pb->y = y;
	pb->w = w-2;
	pb->h = h-2;
	pb->progress = 0;
	pb->BorderColor = BorderColor;
	pb->ProgressColor = ProgressColor;
	pb->BackgroundColor = BackgroundColor;
}

void ili7735_ProgressBar_Destroy(ST7735_ProgressBar* pb)
{
	uint8_t BCL,BCH;
	uint16_t tmp,i;

	//ST7735_AddrSet(pb->x,pb->y,pb->x+pb->w,pb->y+pb->h+2);
	ili9341_SetAddrWindow(pb->x,pb->y,pb->x+pb->w+2,pb->y+pb->h+4);

	BCL = pb->BackgroundColor & 0xFF;
	BCH = (pb->BackgroundColor>>8) & 0xFF;
	//A0_HIGH;
	tmp = (pb->w+2) * (pb->h+4);
	for (i=0;i<tmp;i++)
	{
		ili9341_SendDataByte(BCH);
		ili9341_SendDataByte(BCL);
	}
}
