#include <driver/spi/spi.h>
#include <driver/gpio/gpio.h>
#include <phy_error.h>

#include "display.h"

#include <types.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

static uint8_t _colstart = 0, _rowstart = 0;
static uint16_t _width = 0;
static uint16_t _height = 0;
static uint8_t rotation = 0;
static gpio_pin_e gpio_DC = -1;
static gpio_pin_e gpio_BK = -1;

static SemaphoreHandle_t spi_done;


static void spi_handle_int(spi_evt_t *pevt)
{
    if (pevt->evt != SPI_TX_COMPLETED)
    {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(spi_done, &xHigherPriorityTaskWoken);

    /* Yield if xHigherPriorityTaskWoken is true. The
    actual macro used here is port specific. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void spi_transmit_and_wait(uint8_t *tx_buf, uint16_t tx_len)
{
    int ret = hal_spi_transmit_it(SPI0, tx_buf, tx_len);
    if (ret != PPlus_SUCCESS)
    {
        LOG("SPI TX ERROR, ret = %d", ret);
        return;
    }

    /* Wait for the interrupt */
    xSemaphoreTake(spi_done, portMAX_DELAY);
}

static void spi_transmit_same16_and_wait(uint16_t tx_data, uint16_t tx_buf_nums)
{
    /* switch to 16 bits temporarly */
    hal_spi_dfs_set(SPI0, SPI_2BYTE);

    int ret = hal_spi_transmit_same_it(SPI0, tx_data, tx_buf_nums);
    if (ret != PPlus_SUCCESS)
    {
        LOG("SPI TX ERROR, ret = %d, tx_buf_nums %d", ret, tx_buf_nums);
        return;
    }

    /* Wait for the interrupt */
    xSemaphoreTake(spi_done, portMAX_DELAY);

    /* switch back to 8 bits */
    hal_spi_dfs_set(SPI0, SPI_1BYTE);
}

// Inline the small helper functions for reduced overhead
static inline void ST7735_Command(uint8_t cmd)
{
    hal_gpio_write(gpio_DC, 0);
    spi_transmit_and_wait(&cmd, 1);
}

static inline void ST7735_Data(uint8_t *buff, uint8_t buff_size)
{
    hal_gpio_write(gpio_DC, 1);
    spi_transmit_and_wait(buff, buff_size);
}

static inline void ST7735_WriteCommand(uint8_t cmd)
{
    ST7735_Command(cmd);
}

static inline void ST7735_WriteData(uint8_t data)
{
    ST7735_Data(&data, 1);
}

static inline void ST7735_WriteDataMultiple(uint8_t *data, uint8_t size)
{
    ST7735_Data(data, size);
}

__ATTR_SECTION_XIP__
static void ST7735_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while (numCommands--)
    {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & 0x80 ? *addr++ : 0;
        numArgs &= 0x7F;

        if (numArgs)
        {
            ST7735_WriteDataMultiple((uint8_t *)addr, numArgs);
            addr += numArgs;
        }

        if (ms)
        {
            if (ms == 255)
                ms = 500;
            if (pdMS_TO_TICKS(ms) == 0)
                ms = 1;
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
    }
}

__ATTR_SECTION_XIP__
void display_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t data[4];

    // Column address set
    ST7735_WriteCommand(GC9106_CASET);
    data[0] = 0x00;
    data[1] = x0 + _colstart;
    data[2] = 0x00;
    data[3] = x1 + _colstart;
    ST7735_WriteDataMultiple(data, 4);

    // Row address set
    ST7735_WriteCommand(GC9106_PASET);
    data[0] = 0x00;
    data[1] = y0 + _rowstart;
    data[2] = 0x00;
    data[3] = y1 + _rowstart;
    ST7735_WriteDataMultiple(data, 4);

    // Memory write
    ST7735_WriteCommand(GC9106_RAMWR);
}

__ATTR_SECTION_XIP__
void display_fill_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color, uint32_t size)
{
    // Set the address window once
    display_set_addr_window(x0, y0, x1, y1);

    // Switch to data mode and transfer the color buffer
    hal_gpio_write(gpio_DC, 1);
    uint16_t *buffer = pvPortMalloc(size * 2);
    if (!buffer)
    {
        LOG("Slow for %d bytes", size * 2);
        /* TODO!!!: "color" needs to have the endinaness swapped, I think. */
        spi_transmit_same16_and_wait(color, size);
    }
    else
    {
        for (size_t i = 0; i < size; i++)
        {
            buffer[i] = color;
        }
        spi_transmit_and_wait((uint8_t*)buffer, size*2);
        vPortFree(buffer);
    }
}

static const uint8_t
    initcmd[] = {
        30, /* 29 commands in list: */
        //  (COMMAND_BYTE), n, data_bytes....
        0x01, ST_CMD_DELAY, 150, // Soft reset, then delay 150 ms
        (0x28), 0,               // Display Off
        (0xfe), 0,               // GC9106_ENAB1
        (0xfe), 0,
        (0xfe), 0,
        (0xef), 0,                // GC9106_ENAB2
        (0xb3), 1, 0x03,          // GC9106_ACCESS_F0_F1
        (GC9106_MADCTL), 1, 0xd8, // USER_MADCTL: BGR
        (GC9106_PIXFMT), 1, 0x05, // USER_COLMOD: 16 bits per pixel
        (0xb6), 1, 0x11,          // GC9106_ACCESS_A3_AA_AC
        (0xac), 1, 0x0b,          // undocumented
#ifdef USE_GC9107
        (0xb4), 1, 0x20,          // GC9106_INVCTR
#else
        (0xb4), 1, 0x21,          // GC9106_INVCTR
#endif
        (GC9106_INVON), 0,        // invert the LCD (watch LCD thing)
        (0xb0), 1, 0x00,          // GC9106_ACCESS_C0_C1_C2_C3_C6
        (0xb2), 1, 0x00,          // GC9106_ACCESS_E4_EB
        (0xb1), 1, 0xc0,          // GC9106_ACCESS_E6_E7
        (0xe6), 2, 0x50, 0x43,    // GC9106_VREG1 [50 43]
        (0xe7), 2, 0x56, 0x43,    // GC9106_VREG2 [38 43]
        (0xF0), 14, 0x1f, 0x41, 0x1B, 0x55, 0x36, 0x3d, 0x3e, 0x0, 0x16, 0x08, 0x09, 0x15, 0x14, 0xf,
        (0xF1), 14, 0x1f, 0x41, 0x1B, 0x55, 0x36, 0x3d, 0x3e, 0x0, 0x16, 0x08, 0x09, 0x15, 0x14, 0xf,
        (0xfe), 0,                 // GC9106_ENAB1
        (0xff), 0,                 //???
        (0x35), 1, 0x00,           // USER_TEON
        (0x44), 1, 0x00,           // GC9106_SETSCANLINE
        (0x11), ST_CMD_DELAY, 150, // USER_SLPOUT
        (0x29), 0,                 // USER_DISPON
#ifdef USE_GC9107
        (0x2A), 4, /***Set Column Address***/ 0x00, 0x18, 0x00, 0x67,
        (0x2B), 4, /***Set Page Address***/ 0x00, 0x00, 0x00, 0x9f,
#else
        (0x2A), 4, /***Set Column Address***/ 0x00, 0x18, 0x00, 0x67,
        (0x2B), 4, /***Set Page Address***/ 0x00, 0x00, 0x00, 0x9f,
#endif
        //(0x2c), 0,       //USER_MEMWR
        0x11, ST_CMD_DELAY, 150, // Exit Sleep, then delay 150 ms
        0x29, ST_CMD_DELAY, 150, // Main screen turn on, delay 150 ms
};

__ATTR_SECTION_XIP__
void display_setScrollMargins(uint16_t top, uint16_t bottom)
{
    // TFA+VSA+BFA must equal 480
    if (top + bottom <= _height)
    {
        uint16_t middle = _height - top - bottom;
        uint8_t data[6];
        data[0] = top >> 8;
        data[1] = top & 0xff;
        data[2] = middle >> 8;
        data[3] = middle & 0xff;
        data[4] = bottom >> 8;
        data[5] = bottom & 0xff;

        ST7735_WriteCommand(GC9106_VSCRDEF);
        ST7735_WriteDataMultiple((uint8_t *)data, 6);
    }
}

__ATTR_SECTION_XIP__
void display_scrollTo(uint16_t y)
{
    uint8_t data[2];
    data[0] = y >> 8;
    data[1] = y & 0xff;
    ST7735_WriteCommand(GC9106_VSCRSADD);
    ST7735_WriteDataMultiple((uint8_t *)data, 2);
}

__ATTR_SECTION_XIP__
void display_set_rotation(uint8_t m)
{
    rotation = m % 4; // can't be higher than 3
    switch (rotation)
    {
    case 0:
        m = (MADCTL_MX | MADCTL_ML | MADCTL_RGB);
#ifdef USE_GC9107
        _width = GC9107_TFTWIDTH;
        _height = GC9107_TFTHEIGHT;
        _colstart = 0;
        _rowstart = 0;
#else
        _width = GC9106_TFTWIDTH;
        _height = GC9106_TFTHEIGHT;
        _colstart = 24;
        _rowstart = 0;
#endif
        break;
    case 1:
        m = (MADCTL_MV | MADCTL_ML | MADCTL_RGB);
#ifdef USE_GC9107
        _width = GC9107_TFTHEIGHT;
        _height = GC9107_TFTWIDTH;
        _colstart = 0;
        _rowstart = 0;
#else
        _width = GC9106_TFTHEIGHT;
        _height = GC9106_TFTWIDTH;
        _colstart = 0;
        _rowstart = 24;
#endif
        break;
    case 2:
        m = (MADCTL_MY | MADCTL_RGB);
#ifdef USE_GC9107
        _width = GC9107_TFTWIDTH;
        _height = GC9107_TFTHEIGHT;
        _colstart = 0;
        _rowstart = 0;
#else
        _width = GC9106_TFTWIDTH;
        _height = GC9106_TFTHEIGHT;
        _colstart = 24;
        _rowstart = 0;
#endif
        break;
    case 3:
        m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_RGB);
#ifdef USE_GC9107
        _width = GC9107_TFTHEIGHT;
        _height = GC9107_TFTWIDTH;
        _colstart = 0;
        _rowstart = 0;
#else
        _width = GC9106_TFTHEIGHT;
        _height = GC9106_TFTWIDTH;
        _colstart = 0;
        _rowstart = 24;
#endif
        break;
    }
    m ^= 0x80; //.kbv

    display_setScrollMargins(0, 0); //.kbv
    display_scrollTo(0);

    ST7735_WriteCommand(ST77XX_MADCTL);
    ST7735_WriteData(m);

    display_set_addr_window(0, 0, _width, _height);
}

// Fill the screen with a specific color using chunked DMA-friendly transfers.
void display_fill_screen(uint16_t color)
{
    display_fill_window(0, 0, _width - 1, _height - 1, color, _width * _height);
}

__ATTR_SECTION_XIP__
uint16_t display_get_color(uint16_t r, uint16_t g, uint16_t b)
{
    // Convert to RGB565 format using swapped channel mappings to match custom defines:
    // ST77XX_RED  = 0x07E0 (green bits),
    // ST77XX_GREEN= 0x001F (blue bits),
    // ST77XX_BLUE = 0xF800 (red bits)
    // So map R→green field, G→blue field, B→red field:

    // [3:0] green
    // [7:4] blue
    // [12:8] red
    // [15:13] green low

    return ((r & 0x0F8) << 5)                          // R into bits 15:11
           | ((b & 0x0F8))                             // G into bits 10:5
           | ((g & 0x0E0) >> 5) | ((g & 0x01C) << 11); // G into bits 4:0
}

void display_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= _width || y >= _height)
        return;

    uint8_t color_data[2] = {color >> 8, color & 0xFF};

    display_set_addr_window(x, y, x, y);
    hal_gpio_write(gpio_DC, 1); // Data mode
    spi_transmit_and_wait(color_data, 2);
}


void display_init(gpio_pin_e pin_BK, gpio_pin_e pin_DC, gpio_pin_e pin_RST, gpio_pin_e pin_CS, gpio_pin_e pin_SCLK, gpio_pin_e pin_MOSI, uint16_t width, uint16_t height, uint8_t _rotation)
{
    _width = width;
    _height = height;
    rotation = _rotation;

    gpio_DC = pin_DC;
    gpio_BK = pin_BK;
    // GPIO initialization
    hal_gpio_pin_init(gpio_DC, GPIO_OUTPUT);
    hal_gpio_pin_init(pin_RST, GPIO_OUTPUT);
    hal_gpio_pin_init(pin_CS, GPIO_OUTPUT);

    // Reset Display
    hal_gpio_write(pin_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    hal_gpio_write(pin_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_done = xSemaphoreCreateBinary();

    hal_spi_init();

    // SPI configuration with an increased baudrate (adjust as necessary)
    spi_Cfg_t spi_cfg = {
        .sclk_pin = pin_SCLK,
        .ssn_pin = pin_CS,
        .MOSI = pin_MOSI,
        .MISO = GPIO_DUMMY,                 // not used
        .frequency = 20UL * 1000UL * 1000UL, /* 10MHz */
        .spi_scmod = SPI_MODE0,
        .spi_dfsmod = SPI_8BIT,
        .force_cs = SPI_FORCE_CS_DISABLED,
        .evt_handler = spi_handle_int,
    };
    hal_spi_bus_init(SPI0, spi_cfg);

    ST7735_ExecuteCommandList(initcmd);

    // Set rotation based on the caller’s parameter (now supports values 0-3)
    display_set_rotation(rotation);

    // Clear screen (black)
    display_fill_screen(ST77XX_BLACK);

    // Backlight control initialization
    hal_gpio_pin_init(gpio_BK, GPIO_OUTPUT);
    backlight_turn_on();
}

void backlight_turn_off()
{
    /* TODO!!!: convert to PWM once it's available */
    hal_gpio_write(gpio_BK, 1);
}

void backlight_turn_on()
{
    /* TODO!!!: convert to PWM once it's available */
    hal_gpio_write(gpio_BK, 0);
}
