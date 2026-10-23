/*********************************************************************************************************************
* File              oled_i2c
* Brief             SSD1306 OLED display driver (0.96 inch, 128x64, I2C interface)
*                   Soft I2C: SCL=B6 SDA=B5, 7-bit address 0x3C
*********************************************************************************************************************/

#include "oled_i2c.h"

#define OLED_I2C_ADDR   ( 0x3C )                // SSD1306 7-bit address (common 0x3C; change to 0x3D if needed)
#define OLED_I2C_DELAY  ( 10   )                // Soft I2C clock delay
                                                // 【2026-10-16】50 -> 10：实测一行 16px 要 ~97ms，
                                                // 跑动时计时值每 10ms 变一次 → 主循环被刷屏拖到 9.7Hz
                                                // （球控 PID 只有 4.5~9.7Hz 带宽，是球摆 ±11cm 的根因之一）。
                                                // 10 ≈ 100kHz SCL，SSD1306 支持 400kHz，余量充足。
#define OLED_SCL_PIN    ( B6   )
#define OLED_SDA_PIN    ( B5   )
#define OLED_CTRL_CMD   ( 0x00 )                // Control byte: command
#define OLED_CTRL_DATA  ( 0x40 )                // Control byte: data

static soft_iic_info_struct oled_iic;

//-------------------------------------------------------------------------------------------------------------------
// Write command / write data (control byte goes through the register_name channel of soft_iic)
//-------------------------------------------------------------------------------------------------------------------
static void oled_cmd (uint8 cmd)
{
    soft_iic_write_8bit_register(&oled_iic, OLED_CTRL_CMD, cmd);
}

static void oled_data (const uint8 *dat, uint32 len)
{
    soft_iic_write_8bit_registers(&oled_iic, OLED_CTRL_DATA, dat, len);
}

//-------------------------------------------------------------------------------------------------------------------
// Set cursor: page 0~7, col 0~127
//-------------------------------------------------------------------------------------------------------------------
static void oled_set_pos (uint8 col, uint8 page)
{
    oled_cmd((uint8)(0xB0 | (page & 0x07)));
    oled_cmd((uint8)(0x00 | (col & 0x0F)));
    oled_cmd((uint8)(0x10 | ((col >> 4) & 0x0F)));
}

//-------------------------------------------------------------------------------------------------------------------
// Init (standard SSD1306 128x64 sequence)
//-------------------------------------------------------------------------------------------------------------------
void oled_i2c_init (void)
{
    static const uint8 init_seq[] =
    {
        0xAE,       // Display off
        0xD5, 0x80, // Clock divide
        0xA8, 0x3F, // Multiplex ratio 1/64
        0xD3, 0x00, // Display offset 0
        0x40,       // Start line 0
        0x8D, 0x14, // Charge pump on
        0x20, 0x00, // Horizontal addressing mode
        0xA1,       // Segment remap (flip horizontal, normal mounting)
        0xC8,       // COM scan decreasing (flip vertical, normal mounting)
        0xDA, 0x12, // COM pins hardware config
        0x81, 0xCF, // Contrast
        0xD9, 0xF1, // Pre-charge
        0xDB, 0x40, // VCOMH level
        0xA4,       // Display follows RAM
        0xA6,       // Normal display (not inverted)
        0xAF,       // Display on
    };
    uint8 i = 0;

    soft_iic_init(&oled_iic, OLED_I2C_ADDR, OLED_I2C_DELAY, OLED_SCL_PIN, OLED_SDA_PIN);
    system_delay_ms(100);                           // Wait for the screen to power up

    for(i = 0; i < sizeof(init_seq); i++)
    {
        oled_cmd(init_seq[i]);
    }
    oled_i2c_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// Clear screen / clear one page
//-------------------------------------------------------------------------------------------------------------------
void oled_i2c_clear (void)
{
    uint8 page = 0;
    uint8 blank[16];

    for(page = 0; page < 16; page++) { blank[page] = 0; }

    for(page = 0; page < 8; page++)
    {
        uint8 col = 0;
        oled_set_pos(0, page);
        for(col = 0; col < 8; col++)                // 128 = 8 x 16B
        {
            oled_data(blank, 16);
        }
    }
}

void oled_i2c_clear_page (uint8 page)
{
    uint8 blank[16];
    uint8 i = 0, col = 0;

    for(i = 0; i < 16; i++) { blank[i] = 0; }

    oled_set_pos(0, page & 0x07);
    for(col = 0; col < 8; col++)
    {
        oled_data(blank, 16);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// Strings: 6x8 (single page) / 8x16 (two pages, page must be even)
//-------------------------------------------------------------------------------------------------------------------
void oled_i2c_str6 (uint8 col, uint8 page, const char *s)
{
    uint8 buf[6];

    while(('\0' != *s) && (121 >= col))
    {
        uint8 c = (uint8)(*s - 32);
        uint8 i = 0;
        for(i = 0; i < 6; i++)
        {
            buf[i] = ascii_font_6x8[c][i];
        }
        oled_set_pos(col, page);
        oled_data(buf, 6);
        col = (uint8)(col + 6);
        s++;
    }
}

void oled_i2c_str16 (uint8 col, uint8 page, const char *s)
{
    while(('\0' != *s) && (120 >= col))
    {
        uint8 c = (uint8)(*s - 32);
        uint8 buf[8];
        uint8 i = 0;

        for(i = 0; i < 8; i++) { buf[i] = ascii_font_8x16[c][i]; }
        oled_set_pos(col, page);
        oled_data(buf, 8);
        for(i = 0; i < 8; i++) { buf[i] = ascii_font_8x16[c][i + 8]; }
        oled_set_pos(col, (uint8)(page + 1));
        oled_data(buf, 8);

        col = (uint8)(col + 8);
        s++;
    }
}
