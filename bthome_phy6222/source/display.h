#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <driver/gpio/gpio.h>

#define USE_GC9107

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_SWRESET 0x01
#define ST77XX_SLPOUT 0x11
#define ST77XX_NORON 0x13
#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6
#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5
#define ST7735_PWCTR6 0xFC
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// MADCTL bit definitions
#define ST77XX_MADCTL_MX 0x40  // Row address order
#define ST77XX_MADCTL_MY 0x80  // Column address order
#define ST77XX_MADCTL_MV 0x20  // Row/Column exchange
#define ST77XX_MADCTL_RGB 0x00
#define ST7735_MADCTL_RGB 0x00 // RGB order
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

// Display size definitions
#define ST7735_TFTWIDTH_128 128
#define ST7735_TFTHEIGHT_128 128

#define ST7735_TFTHEIGHT_160 160
#define ST7735_TFTWIDTH_80 80

// some flags for initR() :(
#define INITR_GREENTAB 0x00
#define INITR_REDTAB 0x01
#define INITR_BLACKTAB 0x02
#define INITR_18GREENTAB INITR_GREENTAB
#define INITR_18REDTAB INITR_REDTAB
#define INITR_18BLACKTAB INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80 0x04
#define INITR_HALLOWING 0x05
#define INITR_MINI160x80_PLUGIN 0x06

#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0x07E0
#define ST77XX_GREEN 0x001F
#define ST77XX_BLUE 0xF800

/* GC9106 stuff */

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define GC9106_TFTWIDTH 80  ///< GC9106 max TFT width
#define GC9106_TFTHEIGHT 160 ///< GC9106 max TFT height

#define GC9107_TFTWIDTH 128  ///< GC9107 max TFT width
#define GC9107_TFTHEIGHT 128 ///< GC9107 max TFT height

#define GC9106_NOP 0x00     ///< No-op register
#define GC9106_SWRESET 0x01 ///< Software reset register
#define GC9106_RDDID 0x04   ///< Read display identification information
#define GC9106_RDDST 0x09   ///< Read Display Status

#define GC9106_SLPIN 0x10  ///< Enter Sleep Mode
#define GC9106_SLPOUT 0x11 ///< Sleep Out
#define GC9106_PTLON 0x12  ///< Partial Mode ON
#define GC9106_NORON 0x13  ///< Normal Display Mode ON

#define GC9106_RDMODE 0x0A     ///< Read Display Power Mode
#define GC9106_RDMADCTL 0x0B   ///< Read Display MADCTL
#define GC9106_RDPIXFMT 0x0C   ///< Read Display Pixel Format
#define GC9106_RDIMGFMT 0x0D   ///< Read Display Image Format
#define GC9106_RDSELFDIAG 0x0F ///< Read Display Self-Diagnostic Result

#define GC9106_INVOFF 0x20   ///< Display Inversion OFF
#define GC9106_INVON 0x21    ///< Display Inversion ON
#define GC9106_GAMMASET 0x26 ///< Gamma Set
#define GC9106_DISPOFF 0x28  ///< Display OFF
#define GC9106_DISPON 0x29   ///< Display ON

#define GC9106_CASET 0x2A ///< Column Address Set
#define GC9106_PASET 0x2B ///< Page Address Set
#define GC9106_RAMWR 0x2C ///< Memory Write
#define GC9106_RAMRD 0x2E ///< Memory Read

#define GC9106_PTLAR 0x30    ///< Partial Area
#define GC9106_VSCRDEF 0x33  ///< Vertical Scrolling Definition
#define GC9106_MADCTL 0x36   ///< Memory Access Control
#define GC9106_VSCRSADD 0x37 ///< Vertical Scrolling Start Address
#define GC9106_PIXFMT 0x3A   ///< COLMOD: Pixel Format Set

// Color definitions
#define TFT_BLACK 0x0000       ///<   0,   0,   0
#define TFT_NAVY 0x000F        ///<   0,   0, 123
#define TFT_DARKGREEN 0x03E0   ///<   0, 125,   0
#define TFT_DARKCYAN 0x03EF    ///<   0, 125, 123
#define TFT_MAROON 0x7800      ///< 123,   0,   0
#define TFT_PURPLE 0x780F      ///< 123,   0, 123
#define TFT_OLIVE 0x7BE0       ///< 123, 125,   0
#define TFT_LIGHTGREY 0xC618   ///< 198, 195, 198
#define TFT_DARKGREY 0x7BEF    ///< 123, 125, 123
#define TFT_BLUE 0x001F        ///<   0,   0, 255
#define TFT_GREEN 0x07E0       ///<   0, 255,   0
#define TFT_CYAN 0x07FF        ///<   0, 255, 255
#define TFT_RED 0xF800         ///< 255,   0,   0
#define TFT_MAGENTA 0xF81F     ///< 255,   0, 255
#define TFT_YELLOW 0xFFE0      ///< 255, 255,   0
#define TFT_WHITE 0xFFFF       ///< 255, 255, 255
#define TFT_ORANGE 0xFD20      ///< 255, 165,   0
#define TFT_GREENYELLOW 0xAFE5 ///< 173, 255,  41
#define TFT_PINK 0xFC18        ///< 255, 130, 198

#define TFT_COLOR(r,g,b)        (((uint16_t)b & 0xF8) << 8) | (((uint16_t)r & 0xFC) << 3) | (((uint16_t)g & 0xF8) >> 3)
//(((uint16_t)r & 0x0F8) << 5) | (((uint16_t)b & 0x0F8)) | (((uint16_t)g & 0x0E0) >> 5) | (((uint16_t)g & 0x01C) << 11)

void display_init(gpio_pin_e pin_BK, gpio_pin_e pin_DC, gpio_pin_e pin_RST, gpio_pin_e pin_CS, gpio_pin_e pin_SCLK, gpio_pin_e pin_MOSI, uint16_t width, uint16_t height, uint8_t _rotation);
void display_fill_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color, uint32_t size);
void display_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void display_fill_screen(uint16_t color);
void display_set_rotation(uint8_t m);
uint16_t display_get_color(uint16_t r, uint16_t g, uint16_t b);
void display_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void backlight_turn_off();
void backlight_turn_on();

#endif // DISPLAY_H
