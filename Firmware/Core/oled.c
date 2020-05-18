
// SSD1306 (0.96") or SSD1106 (1.3")

// GND power ground
// VCC is connected to 5V or 3.3v power supply
// Connect D0 to PB6 (SCL)
// Connect D1 to PB7 (SDA)
// RES connects to PB8
// DC connects to PB9

#include <stdlib.h>
#include <stdint.h>

#include "oled.h"
#include "oledfont.h"

#define OLED_RST_Clr()	HAL_GPIO_WritePin(OLED_RST_GPIO_Port,  OLED_RST_Pin,  GPIO_PIN_RESET)
#define OLED_RST_Set()	HAL_GPIO_WritePin(OLED_RST_GPIO_Port,  OLED_RST_Pin,  GPIO_PIN_SET)

#define OLED_DC_Clr()	HAL_GPIO_WritePin(OLED_DC_GPIO_Port,   OLED_DC_Pin,  GPIO_PIN_RESET)
#define OLED_DC_Set()	HAL_GPIO_WritePin(OLED_DC_GPIO_Port,   OLED_DC_Pin,  GPIO_PIN_SET)

#define OLED_CLK_Clr()	HAL_GPIO_WritePin(OLED_CLK_GPIO_Port,  OLED_CLK_Pin,  GPIO_PIN_RESET)
#define OLED_CLK_Set()	HAL_GPIO_WritePin(OLED_CLK_GPIO_Port,  OLED_CLK_Pin,  GPIO_PIN_SET)

#define OLED_SDA_Clr()	HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,  OLED_SDA_Pin,  GPIO_PIN_RESET)
#define OLED_SDA_Set()	HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,  OLED_SDA_Pin,  GPIO_PIN_SET)

#define OLED_CMD 		0	// Write command
#define OLED_DATA		1	// Write data

// SSD1306
#define SIZE			16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define Brightness	0xFF
#define X_WIDTH 		128
#define Y_WIDTH 		64

const uint8_t init_array[] =
{
	0xAE,		// display off
	0x00,		// ---set low column address
	0x10,		// ---set high column address
	0x40,		// --set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	0x81,		// --set contrast control register
	0xCF,		//  Set SEG Output Current Brightness
	0xA1,		// --Set SEG/Column Mapping     0xa0???? 0xa1??
	0xC8,		// Set COM/Row Scan Direction   0xc0???? 0xc8??
	0xA6,		// --set normal display
	0xA8,		// --set multiplex ratio(1 to 64)
	0x3f,		// --1/64 duty
	0xD3,		// -set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	0x00,		// -not offset
	0xd5,		// --set display clock divide ratio/oscillator frequency
	0x80,		// --set divide ratio, Set Clock as 100 Frames/Sec
	0xD9,		// --set pre-charge period
	0xF1,		// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	0xDA,		// --set com pins hardware configuration
	0x12,		//
	0xDB,		// --set vcomh
	0x40,		// Set VCOM Deselect Level
	0x20,		// -Set Page Addressing Mode (0x00/0x01/0x02)
	0x02,		//
	0x8D,		// --set Charge Pump enable/disable
	0x14,		// --set(0x10) disable
	0xA4,		// Disable Entire Display On (0xa4/0xa5)
	0xA6,		// Disable Inverse Display On (0xa6/a7)
	0xAF		// display on
};

// OLED memory
// The storage format is as follows.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

char Whole_map[8][128];

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
	int i;

	delay_ns(100);
	if (cmd)
		OLED_DC_Set();	// command
	else
		OLED_DC_Clr();	// data

	// SPI CS
	//delay_ns(100);
	//OLED_CS_Clr();

	for (i = 0; i < 8; i++)
	{
		delay_ns(100);
		OLED_CLK_Clr();

		delay_ns(100);
		if (dat & 0x80)
			OLED_SDA_Set();
		else
			OLED_SDA_Clr();

		delay_ns(100);
		OLED_CLK_Set();

		dat <<= 1;
	}

	// SPI CS
	//delay_ns(100);
	//OLED_CS_Set();

	delay_ns(100);
	OLED_DC_Set();
}

void OLED_SetPos(uint8_t x, uint8_t y)
{
	OLED_WR_Byte(0xb0 + y, OLED_CMD);
	OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
	OLED_WR_Byte(((x & 0x0f) >> 0) | 0x01, OLED_CMD);
}

void OLED_Display_On(void)
{
	OLED_WR_Byte(0x8d, OLED_CMD);  // SET DCDC
	OLED_WR_Byte(0x14, OLED_CMD);  // DCDC ON
	OLED_WR_Byte(0xaf, OLED_CMD);  // DISPLAY ON
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0x8d, OLED_CMD);  // SET DCDC
	OLED_WR_Byte(0x10, OLED_CMD);  // DCDC OFF
	OLED_WR_Byte(0xae, OLED_CMD);  // DISPLAY OFF
}

void OLED_Clear(void)
{
	int i;
	for (i = 0; i < 8; i++)
	{
		int n;
		OLED_WR_Byte(0xb0 + i, OLED_CMD);
		OLED_WR_Byte(0x00,     OLED_CMD);
		OLED_WR_Byte(0x10,     OLED_CMD);
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(0, OLED_DATA);
	}
}

//    x: 0~127
//    y: 0~63
// mode: 0,????;1,????
// size: ???? 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
	int i;
	uint8_t c = chr - ' ';

	if (x >= Max_Column)
	{
		x  = 0;
		y += 2;
	}

	if ((c + ' ') < 128)
	{
		if (SIZE == 16)
		{
			const int c16 = c * 16;
			OLED_SetPos(x, y);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(F8X16[c16 + i + 0], OLED_DATA);
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(F8X16[c16 + i + 8], OLED_DATA);
		}
		else
		{
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 6; i++)
				OLED_WR_Byte(F6x8[c][i], OLED_DATA);
		}
	}
	else
	{
		c -= 128;
		if (SIZE == 16)
		{
			const int c16 = c * 16;
			OLED_SetPos(x, y);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(~F8X16[c16 + i + 0], OLED_DATA);
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(~F8X16[c16 + i + 8], OLED_DATA);
		}
		else
		{
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 6; i++)
				OLED_WR_Byte(~F6x8[c][i], OLED_DATA);
		}
	}
}

//mode = 0
void OLED_ShowModeChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t show_mode)
{
	int i;
	uint8_t c = chr - ' ';

	if (x >= Max_Column)
	{
		x  = 0;
		y += 2;
	}

	if (SIZE == 16)
	{
		const int c16 = c * 16;
		if (show_mode != 0)
		{
			OLED_SetPos(x, y + 0);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte( F8X16[c16 + i + 0], OLED_DATA);
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte( F8X16[c16 + i + 8], OLED_DATA);
		}
		else
		{
			OLED_SetPos(x, y + 0);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(~F8X16[c16 + i + 0], OLED_DATA);
			OLED_SetPos(x, y + 1);
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(~F8X16[c16 + i + 8], OLED_DATA);
		}
	}
	else
	{
		OLED_SetPos(x, y + 1);
		for (i = 0; i < 6; i++)
			OLED_WR_Byte(F6x8[c][i], OLED_DATA);
	}
}

// m^n
uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

//x, y:
//len :
//size:
//mode: 0,????;1,????
// num: (0 ~ 4294967295);
void OLED_ShowModeNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2, uint8_t show_mode)
{
	uint8_t t;
	uint8_t enshow = 0;
	const uint8_t s2 = size2 / 2;
	for (t = 0; t < len; t++)
	{
		const int s2t = s2 * t;
		const uint8_t temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowModeChar(x + s2t, y, ' ', show_mode);
				continue;
			}
			enshow = 1;
		}
		OLED_ShowModeChar(x + s2t, y, temp + '0', show_mode);
	}
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2)
{
	uint8_t t;
	uint8_t enshow = 0;
	const uint8_t s2 = size2 / 2;
	for (t = 0; t < len; t++)
	{
		const int s2t = s2 * t;
		const uint8_t temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if( temp == 0)
			{
				OLED_ShowChar(x + s2t, y, ' ');
				continue;
			}
			enshow = 1;
		}
	 	OLED_ShowChar(x + s2t, y, temp + '0');
	}
}

void OLED_ShowString(uint8_t x, uint8_t y, char *chr)
{
	int j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j]);
		x += 8;
		if (x > 120)
		{
			x  = 0;
			y += 2;
		}
		j++;
	}
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no)
{
	int i;
	uint8_t adder;
	no *= 2;
	for (i = 0, adder = 0; i < 2; i++, y++, no++)
	{
		int t;
		OLED_SetPos(x, y);
		for (t = 0; t < 16; t++)
		{
			OLED_WR_Byte(Hzk[no][t], OLED_DATA);
			adder++;
		}
	}
}

void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
	uint8_t y;
	for (y = y0; y < y1; y++)
	{
		uint8_t x;
		OLED_SetPos(x0, y);
		for (x = x0; x < x1; x++)
			OLED_WR_Byte(*BMP++, OLED_DATA);
	}
}

// SSD1306
void OLED_Init(void)
{
	int i;
	int j;

	OLED_DC_Set();
	OLED_CLK_Set();
	OLED_SDA_Set();

	OLED_RST_Set();
	delay_ms(100);
	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set();

	for (i = 0; i < (int)sizeof(init_array); i++)
		OLED_WR_Byte(init_array[i], OLED_CMD);

	OLED_Clear();
	OLED_SetPos(0, 0);

	OLED_Clear();
	OLED_SetPos(0, 0);

	for (i = 0; i < 8; i++)
		for (j = 0; j < 128; j++)
			Whole_map[i][j] = 0;
}

void OLED_DrawPoint(uint8_t x, uint8_t y)
{
	const uint8_t x_index  =  x % 128;
	const uint8_t y_index  = (y / 8) % 8;
	const uint8_t col_move =  y % 8;
	Whole_map[y_index][x_index] |= 0x01 << col_move;
}

void clear_wholeMap(void)
{
	int i;
	int j;
	for (i = 0; i < 8; i++)
		for (j = 0; j < 128; j++)
			Whole_map[i][j] = 0;
}

void plot_wholeMap(void)
{
	int i;
	int j;
	for (i = 0; i < 8; i++)
	{
		OLED_SetPos(0, i);
		for (j = 0; j < 128; j++)
			OLED_WR_Byte(Whole_map[i][j], OLED_DATA);
	}
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t;
	int delta_x;
	int delta_y;
	int distance;
	int incx;
	int incy;
	int uRow;
	int uCol;
	int xerr = 0;
	int yerr = 0;

	delta_x = x2 - x1;
	delta_y = y2 - y1;

	uRow = x1;
	uCol = y1;

	if (delta_x > 0)
		incx = 1;
	else
	if (delta_x == 0)
		incx = 0;
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}

	if (delta_y > 0)
		incy = 1;
	else
	if (delta_y == 0)
		incy = 0;
	else
	{
		incy = -1;
		delta_y = -delta_y;
	}

	distance = (delta_x > delta_y) ? delta_x : delta_y;

	for (t = 0; t <= distance + 1; t++)
	{
		OLED_DrawPoint(uRow, uCol);
		//LCD_DrawPoint(uRow, uCol);
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

void OLED_ShowTemp(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 0);
	OLED_ShowChinese(x + 16, y, 1);
	OLED_ShowChinese(x + 32, y, 4);
	OLED_ShowChinese(x + 48, y, 5);
}

void OLED_ShowPreDam(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 8);
	OLED_ShowChinese(x + 16, y, 6);
}

void OLED_ShowNowDam(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 0);
	OLED_ShowChinese(x + 16, y, 1);
}

void OLED_ShowSettemp(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 6);
	OLED_ShowChinese(x + 16, y, 7);
	OLED_ShowChinese(x + 32, y, 2);
	OLED_ShowChinese(x + 48, y, 3);
}

void OLED_ShowDamp(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 0);
	OLED_ShowChinese(x + 16, y, 1);
	OLED_ShowChinese(x + 32, y, 2);
	OLED_ShowChinese(x + 48, y, 3);
}

void OLED_ShowExpectDamp(uint8_t x, uint8_t y)
{
	OLED_ShowChinese(x +  0, y, 6);
	OLED_ShowChinese(x + 16, y, 7);
	OLED_ShowChinese(x + 32, y, 2);
	OLED_ShowChinese(x + 48, y, 3);
}

void LCD_Show_CEStr(uint16_t x0, uint16_t y0, const char *pp)
{
	const char *p = pp;

	while (*p != 0)
	{
		int j;
		int index;

		if (*p < 128)
		{
			OLED_ShowChar(x0, y0, *p++);
			x0 += 8;
			continue;
		}

		for (index = 0; index < NUM_GB16; index++)
			if (*(p + 0) == codeGB_16[index].Index[0] && *(p + 1) == codeGB_16[index].Index[1])
					break;

//		OLED_ShowNum(0, 0, index, 3, 16);
//		delay_ms(200);

		if (index >= NUM_GB16)
			index = 0;

		for (j = 0; j < 2; j++)
		{
			const int j16 = j * 16;
			int i;
			OLED_SetPos(x0, y0 + j);
			for (i = 0; i < 16; i++)
				OLED_WR_Byte(codeGB_16[index].Msk[j16 + i], OLED_DATA);
		}

		x0 += 16;
		p  += 2;
	}
}

void LCD_Show_ModeCEStr(uint16_t x0, uint16_t y0, const char *pp, uint8_t show_mode)
{
	const char *p = pp;

	while (*p != 0)
	{
		int j;
		int index;

		if (*p < 128)
		{
			OLED_ShowModeChar(x0, y0, *p++, show_mode);
			x0 += 8;
			continue;
		}

		for (index = 0; index < NUM_GB16; index++)
			if (*(p + 0) == codeGB_16[index].Index[0] && *(p + 1) == codeGB_16[index].Index[1])
				break;
		if (index >= NUM_GB16)
			index = 0;

		for (j = 0; j < 2; j++)
		{
			const int j16 = j * 16;
			int i;
			OLED_SetPos(x0, y0 + j);
			for (i = 0; i < 16; i++)
			{
				const uint8_t temp = codeGB_16[index].Msk[j16 + i];
				if (show_mode != 0)
					OLED_WR_Byte( temp, OLED_DATA);
				else
					OLED_WR_Byte(~temp, OLED_DATA);
			}
		}

		x0 += 16;
		p  += 2;
	}
}

void draw_cell(uint8_t x, uint8_t y, uint8_t remain_len, uint8_t all_len)
{
	uint8_t i;
	uint8_t x_start;

	OLED_SetPos(x, y);

	for (i = 0; i < 4; i++)
		OLED_WR_Byte(0XFC, OLED_DATA);

	OLED_WR_Byte(0XFF, OLED_DATA);

	for(i = 0; i < all_len; i++)
		OLED_WR_Byte(0X03, OLED_DATA);

	OLED_WR_Byte(0XFF, OLED_DATA);

	x_start = x + 5 + all_len - remain_len;

	OLED_SetPos(x_start, y);

	for(i = 0; i < remain_len + 1; i++)
		OLED_WR_Byte(0XFF, OLED_DATA);

	OLED_SetPos(x, y + 1);

	for (i = 0; i < 4; i++)
		OLED_WR_Byte(0X3F, OLED_DATA);

	OLED_WR_Byte(0XFF, OLED_DATA);

	for (i = 0; i < all_len; i++)
		OLED_WR_Byte(0XC0, OLED_DATA);

	OLED_WR_Byte(0XFF, OLED_DATA);

	x_start = x + 5 + all_len - remain_len;

	OLED_SetPos(x_start, y + 1);

	for (i = 0; i < remain_len + 1; i++)
		OLED_WR_Byte(0XFF, OLED_DATA);
}
