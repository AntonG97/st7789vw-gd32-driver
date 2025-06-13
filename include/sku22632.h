#ifndef SKU22632_H
#define SKU22632_H

#include <stdint.h>
#include "lcd_font.h"

/**
 * x and y coordinates MAX
 */
#define LCD_X_MAX 240
#define LCD_Y_MAX 240

/**
 * Bit patterns for colors
 */
typedef enum{
	RED = 0xF800,
	BLUE = 0x001F,
	GREEN = 0x07E0,
	WHITE = 0xFFFF,
	BLACK = 0x0000
}color;

/**
 * @brief initialises LCD. OBS
 * \param[in]: spi_periph: SPIx(x=0,1,2)
 * \param[in]: gpio_periph: GPIOx(x = A,B,C,D,E). OBS! Once chosen, base all GPIO pins on input param!
 * \param[in]: clk: SPI clock pin
 * \param[in]: din: SPI data out (MOSI)
 * \param[in]: rst: Reset => GPIO_PIN_x(x=0..15) 
 * \param[in]: cs: Chip select => GPIO_PIN_x(x=0..15)
 * \param[in]: dc: Data or cmd => GPIO_PIN_x(x=0..15)
 */
void lcd_init(uint32_t _spi_periph, uint32_t _gpio_perpih, uint32_t clk, uint32_t din, uint32_t _rst, uint32_t _cs, uint32_t _dc);
/**
 * @brief Clears LCD screen
 * @param[in]: color: Specifies the color of the screen to be cleared to
 */
void lcd_clear(color color);
/**
 * \brief Queue for LCD. Call first in superloop
 */
void lcd_queue_flush(void);
/**
 * @brief Draw single point
 * @param[in] x: col start coordinate [0 <= xs <= 240]
 * @param[in] y: row start coordinate [0 <= ys <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawPixel(const uint16_t x, const uint16_t y, color color);
/**
 * @brief Draw single point big size
 * @param[in] x: col start coordinate [0 <= xs <= 240]
 * @param[in] y: row start coordinate [0 <= ys <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawPixel_big(const uint16_t x, const uint16_t y, color color);
/**
 * @brief Sets a new window
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 * @param[in] color: Color to be filled
 */
void lcd_drawLine(uint16_t xs, const uint16_t xe, uint16_t ys, const uint16_t ye, color color);
/**
 * @brief Draw rectangle
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 */
void lcd_drawRec(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color);
/**
 * @brief Draw filled rectangle
 * @param[in] xs: col start coordinate 	[0 <= xs <= xe]
 * @param[in] xe: col end coordinate	[xs <= xe <= 240]
 * @param[in] ys: row start coordinate 	[0 <= ys <= ye]
 * @param[in] ye: row end coordinate 	[ys <= ye <= 240]
 */
void lcd_drawRec_filled(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color);
/**
 * @brief Draw circle
 * @param[in]: xs: center x coordinate 
 * @param[in]: ys: center y coordinate 
 * @param[in]: r: radie
 */
void lcd_drawCircle(uint16_t xs, uint16_t ys, uint16_t r, color color);
/**
 * @brief Draw filled circle
 * @param[in]: xs: center x coordinate 
 * @param[in]: ys: center y coordinate 
 * @param[in]: r: radie
 */
void lcd_drawCircle_filled(uint16_t xs, uint16_t ys, uint16_t r, color color);
/**
 * @brief Writes a character on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: ch: Character that is to be written
 * @param[in]: _color: Color of Character
 * @param[in]: size: Size of Character
 */
void lcd_ShowCh(const uint16_t xs, const uint16_t ys, const uint8_t ch, color color, font_size size);
/**
 * @brief Writes a string on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: str: String that is to be written
 * @param[in]: _color: Color of Characters
 * @param[in]: size: Size of Characters
 */
void lcd_showStr(const uint16_t xs, const uint16_t ys, const uint8_t *str, const color _color, const font_size size);
/**
 * @brief Writes a number on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: val: val to be written
 * @param[in]: _color: Color of val
 * @param[in]: size: Size of val
 */
void lcd_showNum(const uint16_t xs, const uint16_t ys, const int val, const color _color, const font_size size);
/**
 * @brief Writes a float number on LCD
 * @param[in]: xs: Center x coordinate
 * @param[in]: ys: Center y coordinate
 * @param[in]: val: float val to be written
 * @param[in]: decim_pt: Amount of decimals
 * @param[in]: _color: Color of val
 * @param[in]: size: Size of val
 */
void lcd_showNum_float(const uint16_t xs, const uint16_t ys, const float val, const uint8_t decim_pt, const color _color, const font_size size);
/**
 * @brief Draws picture on LCD.
 * @param[in]: bitMap: Pointer to bitmap of 240x240 / 8 bytes
 */
void lcd_showPicture(uint8_t *bitMap);
//void lcd_showLogo();

#endif