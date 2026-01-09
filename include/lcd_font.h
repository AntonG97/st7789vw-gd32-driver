#ifndef LCD_FONT_H
#define LCD_FONT_H

#include <stdint.h>

/**
 * Normal size of font. 16x24 pixels
 */
#define STD_FONT_X_SIZE 16
#define STD_FONT_Y_SIZE 24

/**
 * Fonts
 */
extern const uint8_t arial[4564];

/*
 * KUB image
 */

extern const uint8_t kub_map_v4[7200];
#endif
