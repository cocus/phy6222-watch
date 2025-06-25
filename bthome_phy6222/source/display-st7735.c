#include <driver/spi/spi.h>
#include <driver/gpio/gpio.h>
#include "display.h"

#include <types.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

static uint8_t tabcolor = INITR_GREENTAB;
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
    ST7735_WriteCommand(ST77XX_CASET);
    data[0] = 0x00;
    data[1] = x0 + _colstart;
    data[2] = 0x00;
    data[3] = x1 + _colstart;
    ST7735_WriteDataMultiple(data, 4);

    // Row address set
    ST7735_WriteCommand(ST77XX_RASET);
    data[0] = 0x00;
    data[1] = y0 + _rowstart;
    data[2] = 0x00;
    data[3] = y1 + _rowstart;
    ST7735_WriteDataMultiple(data, 4);

    // Memory write
    ST7735_WriteCommand(ST77XX_RAMWR);
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
    Bcmd[] = {                            // Init commands for 7735B screens
        18,                               // 18 commands in list:
        ST77XX_SWRESET, ST_CMD_DELAY,     //  1: Software reset, no args, w/delay
        50,                               //     50 ms delay
        ST77XX_SLPOUT, ST_CMD_DELAY,      //  2: Out of sleep mode, no args, w/delay
        255,                              //     255 = max (500 ms) delay
        ST77XX_COLMOD, 1 + ST_CMD_DELAY,  //  3: Set color mode, 1 arg + delay:
        0x05,                             //     16-bit color
        10,                               //     10 ms delay
        ST7735_FRMCTR1, 3 + ST_CMD_DELAY, //  4: Frame rate control, 3 args + delay:
        0x00,                             //     fastest refresh
        0x06,                             //     6 lines front porch
        0x03,                             //     3 lines back porch
        10,                               //     10 ms delay
        ST77XX_MADCTL, 1,                 //  5: Mem access ctl (directions), 1 arg:
        0x08,                             //     Row/col addr, bottom-top refresh
        ST7735_DISSET5, 2,                //  6: Display settings #5, 2 args:
        0x15,                             //     1 clk cycle nonoverlap, 2 cycle gate
                                          //     rise, 3 cycle osc equalize
        0x02,                             //     Fix on VTL
        ST7735_INVCTR, 1,                 //  7: Display inversion control, 1 arg:
        0x0,                              //     Line inversion
        ST7735_PWCTR1, 2 + ST_CMD_DELAY,  //  8: Power control, 2 args + delay:
        0x02,                             //     GVDD = 4.7V
        0x70,                             //     1.0uA
        10,                               //     10 ms delay
        ST7735_PWCTR2, 1,                 //  9: Power control, 1 arg, no delay:
        0x05,                             //     VGH = 14.7V, VGL = -7.35V
        ST7735_PWCTR3, 2,                 // 10: Power control, 2 args, no delay:
        0x01,                             //     Opamp current small
        0x02,                             //     Boost frequency
        ST7735_VMCTR1, 2 + ST_CMD_DELAY,  // 11: Power control, 2 args + delay:
        0x3C,                             //     VCOMH = 4V
        0x38,                             //     VCOML = -1.1V
        10,                               //     10 ms delay
        ST7735_PWCTR6, 2,                 // 12: Power control, 2 args, no delay:
        0x11, 0x15,
        ST7735_GMCTRP1, 16,     // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
        0x09, 0x16, 0x09, 0x20, //     (Not entirely necessary, but provides
        0x21, 0x1B, 0x13, 0x19, //      accurate colors)
        0x17, 0x15, 0x1E, 0x2B,
        0x04, 0x05, 0x02, 0x0E,
        ST7735_GMCTRN1, 16 + ST_CMD_DELAY, // 14: Gamma Adjustments (neg. polarity), 16 args + delay:
        0x0B, 0x14, 0x08, 0x1E,            //     (Not entirely necessary, but provides
        0x22, 0x1D, 0x18, 0x1E,            //      accurate colors)
        0x1B, 0x1A, 0x24, 0x2B,
        0x06, 0x06, 0x02, 0x0F,
        10,                          //     10 ms delay
        ST77XX_CASET, 4,             // 15: Column addr set, 4 args, no delay:
        0x00, 0x02,                  //     XSTART = 2
        0x00, 0x81,                  //     XEND = 129
        ST77XX_RASET, 4,             // 16: Row addr set, 4 args, no delay:
        0x00, 0x02,                  //     XSTART = 1
        0x00, 0x81,                  //     XEND = 160
        ST77XX_NORON, ST_CMD_DELAY,  // 17: Normal display on, no args, w/delay
        10,                          //     10 ms delay
        ST77XX_DISPON, ST_CMD_DELAY, // 18: Main screen turn on, no args, delay
        255},                        //     255 = max (500 ms) delay

    Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
        15,                           // 15 commands in list:
        ST77XX_SWRESET, ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
        150,                          //     150 ms delay
        ST77XX_SLPOUT, ST_CMD_DELAY,  //  2: Out of sleep mode, 0 args, w/delay
        255,                          //     500 ms delay
        ST7735_FRMCTR1, 3,            //  3: Framerate ctrl - normal mode, 3 arg:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR2, 3,            //  4: Framerate ctrl - idle mode, 3 args:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR3, 6,            //  5: Framerate - partial mode, 6 args:
        0x01, 0x2C, 0x2D,             //     Dot inversion mode
        0x01, 0x2C, 0x2D,             //     Line inversion mode
        ST7735_INVCTR, 1,             //  6: Display inversion ctrl, 1 arg:
        0x07,                         //     No inversion
        ST7735_PWCTR1, 3,             //  7: Power control, 3 args, no delay:
        0xA2,
        0x02,                         //     -4.6V
        0x84,                         //     AUTO mode
        ST7735_PWCTR2, 1,             //  8: Power control, 1 arg, no delay:
        0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
        ST7735_PWCTR3, 2,             //  9: Power control, 2 args, no delay:
        0x0A,                         //     Opamp current small
        0x00,                         //     Boost frequency
        ST7735_PWCTR4, 2,             // 10: Power control, 2 args, no delay:
        0x8A,                         //     BCLK/2,
        0x2A,                         //     opamp current small & medium low
        ST7735_PWCTR5, 2,             // 11: Power control, 2 args, no delay:
        0x8A, 0xEE, ST7735_VMCTR1, 1, // 12: Power control, 1 arg, no delay:
        0x0E, ST77XX_INVOFF, 0,       // 13: Don't invert display, no args
        ST77XX_MADCTL, 1,             // 14: Mem access ctl (directions), 1 arg:
        0xC8,                         //     row/col addr, bottom-top refresh
        ST77XX_COLMOD, 1,             // 15: set color mode, 1 arg, no delay:
        0x05},                        //     16-bit color

    Rcmd2green[] = {        // 7735R init, part 2 (green tab only)
        2,                  //  2 commands in list:
        ST77XX_CASET, 4,    //  1: Column addr set, 4 args, no delay:
        0x00, 0x02,         //     XSTART = 0
        0x00, 0x7F + 0x02,  //     XEND = 127
        ST77XX_RASET, 4,    //  2: Row addr set, 4 args, no delay:
        0x00, 0x01,         //     XSTART = 0
        0x00, 0x9F + 0x01}, //     XEND = 159

    Rcmd2red[] = {       // 7735R init, part 2 (red tab only)
        2,               //  2 commands in list:
        ST77XX_CASET, 4, //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x7F,      //     XEND = 127
        ST77XX_RASET, 4, //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x9F},     //     XEND = 159

    Rcmd2green144[] = {  // 7735R init, part 2 (green 1.44 tab)
        3,               //  3 commands in list:
        ST77XX_INVON, 0, //  1: Display is inverted
        ST77XX_CASET, 4, //  2: Column addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x7F,      //     XEND = 127
        ST77XX_RASET, 4, //  3: Row addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x7F},     //     XEND = 127

    Rcmd2green160x80[] = { // 7735R init, part 2 (mini 160x80)
        2,                 //  2 commands in list:
        ST77XX_CASET, 4,   //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,        //     XSTART = 0
        0x00, 0x4F,        //     XEND = 79
        ST77XX_RASET, 4,   //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,        //     XSTART = 0
        0x00, 0x9F},       //     XEND = 159

    Rcmd2green160x80plugin[] = { // 7735R init, part 2 (mini 160x80 with plugin FPC)
        3,                       //  3 commands in list:
        ST77XX_INVON, 0,         //   1: Display is inverted
        ST77XX_CASET, 4,         //  2: Column addr set, 4 args, no delay:
        0x00, 0x00,              //     XSTART = 0
        0x00, 0x4F,              //     XEND = 79
        ST77XX_RASET, 4,         //  3: Row addr set, 4 args, no delay:
        0x00, 255,              //     XSTART = 0
        0x00, 0x9F},             //     XEND = 159

    Rcmd3[] = {                                                                     // 7735R init, part 3 (red or green tab)
        4,                                                                          //  4 commands in list:
        ST7735_GMCTRP1, 16,                                                         //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
        0x02, 0x1c, 0x07, 0x12,                                                     //     (Not entirely necessary, but provides
        0x37, 0x32, 0x29, 0x2d,                                                     //      accurate colors)
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, ST7735_GMCTRN1, 16,         //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
        0x03, 0x1d, 0x07, 0x06,                                                     //     (Not entirely necessary, but provides
        0x2E, 0x2C, 0x29, 0x2D,                                                     //      accurate colors)
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, ST77XX_NORON, ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
        10,                                                                         //     10 ms delay
        ST77XX_DISPON, ST_CMD_DELAY,                                                //  4: Main screen turn on, no args w/delay
        100};                                                                       //     100 ms delay

__ATTR_SECTION_XIP__
static void ST7735_InitR(uint8_t options)
{
    // Execute command lists:
    ST7735_ExecuteCommandList(Rcmd1);

    if (options == INITR_GREENTAB)
    {
        ST7735_ExecuteCommandList(Rcmd2green);
        _colstart = 2;
        _rowstart = 1;
    }
    else if ((options == INITR_144GREENTAB) || (options == INITR_HALLOWING))
    {
        _height = ST7735_TFTHEIGHT_128;
        _width = ST7735_TFTWIDTH_128;
        ST7735_ExecuteCommandList(Rcmd2green144);
        _colstart = 2;
        _rowstart = 3; // For default rotation 0
    }
    else if (options == INITR_MINI160x80)
    {
        _height = ST7735_TFTWIDTH_80;
        _width = ST7735_TFTHEIGHT_160;
        ST7735_ExecuteCommandList(Rcmd2green160x80);
        _colstart = 24;
        _rowstart = 0;
    }
    else if (options == INITR_MINI160x80_PLUGIN)
    {
        _height = ST7735_TFTWIDTH_80;
        _width = ST7735_TFTHEIGHT_160;
        ST7735_ExecuteCommandList(Rcmd2green160x80plugin);
        _colstart = 26;
        _rowstart = 1;
        // invertOnCommand = ST77XX_INVOFF;
        // invertOffCommand = ST77XX_INVON;
    }
    else
    {
        // colstart, rowstart left at default '0' values
        ST7735_ExecuteCommandList(Rcmd2red);
    }
    ST7735_ExecuteCommandList(Rcmd3);

    // Black tab, change MADCTL color filter
    if ((options == INITR_BLACKTAB) || (options == INITR_MINI160x80))
    {
        uint8_t data = 0xC0;
        ST7735_Command(ST77XX_MADCTL);
        ST7735_Data(&data, 1);
    }

    if (options == INITR_HALLOWING)
    {
        // Hallowing is simply a 1.44" green tab upside-down:
        tabcolor = INITR_144GREENTAB;
        display_set_rotation(2);
    }
    else
    {
        tabcolor = options;
        display_set_rotation(0);
    }
}

__ATTR_SECTION_XIP__
void display_set_rotation(uint8_t m)
{
    uint8_t madctl = 0;

    rotation = m & 3; // can't be higher than 3

    // For ST7735 with GREEN TAB (including HalloWing)...
    if ((tabcolor == INITR_144GREENTAB) || (tabcolor == INITR_HALLOWING))
    {
        // ..._rowstart is 3 for rotations 0&1, 1 for rotations 2&3
        _rowstart = (rotation < 2) ? 3 : 1;
    }

    switch (rotation)
    {
    case 0:
        if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80))
        {
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
        }
        else
        {
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST7735_MADCTL_BGR;
        }

        if (tabcolor == INITR_144GREENTAB)
        {
            _height = ST7735_TFTHEIGHT_128;
            _width = ST7735_TFTWIDTH_128;
        }
        else if (tabcolor == INITR_MINI160x80 ||
                 tabcolor == INITR_MINI160x80_PLUGIN)
        {
            _height = ST7735_TFTHEIGHT_160;
            _width = ST7735_TFTWIDTH_80;
        }
        else
        {
            _height = ST7735_TFTHEIGHT_160;
            _width = ST7735_TFTWIDTH_128;
        }
        break;
    case 1:
        if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80))
        {
            madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
        }
        else
        {
            madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST7735_MADCTL_BGR;
        }

        if (tabcolor == INITR_144GREENTAB)
        {
            _width = ST7735_TFTHEIGHT_128;
            _height = ST7735_TFTWIDTH_128;
        }
        else if (tabcolor == INITR_MINI160x80 ||
                 tabcolor == INITR_MINI160x80_PLUGIN)
        {
            _width = ST7735_TFTHEIGHT_160;
            _height = ST7735_TFTWIDTH_80;
        }
        else
        {
            _width = ST7735_TFTHEIGHT_160;
            _height = ST7735_TFTWIDTH_128;
        }
        break;
    case 2:
        if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80))
        {
            madctl = ST77XX_MADCTL_RGB;
        }
        else
        {
            madctl = ST7735_MADCTL_BGR;
        }

        if (tabcolor == INITR_144GREENTAB)
        {
            _height = ST7735_TFTHEIGHT_128;
            _width = ST7735_TFTWIDTH_128;
        }
        else if (tabcolor == INITR_MINI160x80 ||
                 tabcolor == INITR_MINI160x80_PLUGIN)
        {
            _height = ST7735_TFTHEIGHT_160;
            _width = ST7735_TFTWIDTH_80;
        }
        else
        {
            _height = ST7735_TFTHEIGHT_160;
            _width = ST7735_TFTWIDTH_128;
        }
        break;
    case 3:
        if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80))
        {
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
        }
        else
        {
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST7735_MADCTL_BGR;
        }

        if (tabcolor == INITR_144GREENTAB)
        {
            _width = ST7735_TFTHEIGHT_128;
            _height = ST7735_TFTWIDTH_128;
        }
        else if (tabcolor == INITR_MINI160x80 ||
                 tabcolor == INITR_MINI160x80_PLUGIN)
        {
            _width = ST7735_TFTHEIGHT_160;
            _height = ST7735_TFTWIDTH_80;
        }
        else
        {
            _width = ST7735_TFTHEIGHT_160;
            _height = ST7735_TFTWIDTH_128;
        }
        break;
    }

    ST7735_WriteCommand(ST77XX_MADCTL);
    ST7735_WriteData(madctl);
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
    return ((b & 0xF8) << 8)    // B into bits 15:11
           | ((r & 0xFC) << 3)  // R into bits 5:0
           | ((g & 0xF8) >> 3); // G into bits 4:0
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

    // Initialize the display with chosen tab color; you may want to allow other options
    ST7735_InitR(INITR_144GREENTAB);

    // Set rotation based on the caller’s parameter (now supports values 0-3)
    // display_set_rotation(rotation);

    // Clear screen (black)
    display_fill_screen(ST77XX_BLACK);

    // Backlight control initialization
    hal_gpio_pin_init(gpio_BK, GPIO_OUTPUT);
    backlight_turn_on();
}

void backlight_turn_off()
{
    hal_gpio_write(gpio_BK, 1);
}

void backlight_turn_on()
{
    hal_gpio_write(gpio_BK, 0);
}
