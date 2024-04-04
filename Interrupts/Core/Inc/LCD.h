/*
 * LCDW.h
 *
 *  Created on: Apr 4, 2024
 *      Author: badyl
 */
/*
#ifndef INC_LCD_H_
#define INC_LCD_H_



#endif /* INC_LCD_H_ */

#pragma once
#include <stdint.h>
#define BLACK     0x0000
#define RED       0xf800
#define GREEN     0x07e0
#define BLUE      0x001f
#define YELLOW    0xffe0
#define MAGENTA   0xf81f
#define CYAN      0x07ff
#define WHITE     0xffff

void lcd_init(void);
void lcd_fill_box(int x, int y, int width, int height, uint16_t color);
void lcd_put_pixel(int x, int y, uint16_t color);
void lcd_draw_image(int x, int y, int width, int height, const uint8_t* data);
