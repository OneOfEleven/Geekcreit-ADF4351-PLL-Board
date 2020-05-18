
#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "common.h"

void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void clear_wholeMap(void);
void plot_wholeMap(void);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2);
void OLED_ShowString(uint8_t x, uint8_t y, char *p);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no);
void OLED_ShowTemp(uint8_t x, uint8_t y);
void OLED_ShowDamp(uint8_t x, uint8_t y);
void OLED_ShowSettemp(uint8_t x, uint8_t y);
void OLED_ShowPreDam(uint8_t x, uint8_t y);
void OLED_ShowNowDam(uint8_t x, uint8_t y);
void OLED_ShowExpectDamp(uint8_t x, uint8_t y);
void OLED_ShowModeNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2, uint8_t show_mode);
void OLED_ShowModeChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t show_mode);
void LCD_Show_CEStr(uint16_t x0, uint16_t y0, const char *pp);
void LCD_Show_ModeCEStr(uint16_t x0, uint16_t y0, const char *pp, uint8_t show_mode);
void draw_cell(uint8_t x, uint8_t y, uint8_t remain_len, uint8_t all_len);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]);

#ifdef __cplusplus
	}
#endif

#endif
