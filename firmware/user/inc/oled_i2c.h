/*********************************************************************************************************************
* File              oled_i2c
* Brief             SSD1306 OLED display driver (0.96 inch, 128x64, I2C interface)
*                   Soft I2C: SCL=B6 SDA=B5, 7-bit address 0x3C (race timing screen, rule: <=2 inch)
*                   Fonts from zf_common_font (ascii_font_6x8 / ascii_font_8x16, index ch-32)
*                   Note: soft I2C byte writes block; full-screen refresh takes ~100ms,
*                   so only the timing line (partial pages) is updated while running.
*********************************************************************************************************************/

#ifndef _oled_i2c_h_
#define _oled_i2c_h_

#include "zf_common_headfile.h"

void oled_i2c_init (void);                                  // Init at power-up (safe when no screen attached)
void oled_i2c_clear (void);                                 // Clear whole screen
void oled_i2c_str6 (uint8 col, uint8 page, const char *s);  // 6x8 font: page 0~7, col 0~127 (6 px per char)
void oled_i2c_str16 (uint8 col, uint8 page, const char *s); // 8x16 font: even page (uses page and page+1)
void oled_i2c_clear_page (uint8 page);                      // Clear one page (used before overwriting a line)

#endif
