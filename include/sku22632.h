#ifndef SKU22632_H
#define SKU22632_H

#include <stdint.h>

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
 * \brief Queue for LCD. Call first in superloop
 */
void lcd_queue_flush(void);
/**
 * @brief Draw single point
 * @param[in] x: col start coordinate [0 <= xs <= 239]
 * @param[in] y: row start coordinate [0 <= ys <= 319]
 */
void lcd_drawPoint(const uint16_t x, const uint16_t y, color color);
/**
 * @brief Draw rectangle
 * @param[in] xs: col start coordinate [0 <= xs <= xe]
 * @param[in] xe: col end coordinate [xs <= xe <= 239]
 * @param[in] ys: row start coordinate [0 <= ys <= ye]
 * @param[in] ye: row end coordinate [ys <= ye <= 319]
 */
void lcd_drawRec(const uint16_t xs, const uint16_t xe, const uint16_t ys, const uint16_t ye, color color);


#endif