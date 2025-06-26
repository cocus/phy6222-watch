
#include <types.h> /* for UNUSED */
#include <driver/adc/adc.h>
#include <driver/clock/clock.h>

#include <string.h>
#include <log/log.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "fonts/FreeMono9pt7b.h"
#include "display.h"


static SemaphoreHandle_t adc_done_sem;
static volatile float adc_values[2] = {0, 0};

static void adc_Poilling_evt(const adc_channels_t ch, const uint16_t *data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    float value = hal_adc_value_cal(ch, data, (MAX_ADC_SAMPLE_SIZE - 3));

    if (ch == CH9)
    {
        adc_values[0] = value;

        xSemaphoreGiveFromISR(adc_done_sem, &xHigherPriorityTaskWoken);

        /* Yield if xHigherPriorityTaskWoken is true. The
        actual macro used here is port specific. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        adc_values[1] = value;
    }
}

#define VU_LEFT_X 5
#define VU_LEFT_Y VU_LEFT_X
#ifdef USE_GC9107
#define VU_LEFT_W 32
#define VU_LEFT_H 96
#define VU_RIGHT_X (GC9107_TFTWIDTH - VU_LEFT_X - VU_LEFT_W)
#else
#define VU_LEFT_W 32
#define VU_LEFT_H 128
#define VU_RIGHT_X (GC9106_TFTWIDTH - VU_LEFT_X - VU_LEFT_W)
#endif

#define VU_RIGHT_Y VU_LEFT_Y
#define VU_RIGHT_W VU_LEFT_W
#define VU_RIGHT_H VU_LEFT_H

// LED
#define GPIO_LED GPIO_P00

// Vibrator
#define GPIO_VIBRATOR GPIO_P03

// Display
#define DC_PIN GPIO_P25
#define RST_PIN GPIO_P24
#define CS_PIN GPIO_P31
#define BKL_PIN GPIO_P02

#define SCLK_PIN GPIO_P34
#define MOSI_PIN GPIO_P32

// Button
#define BUTTON_PIN GPIO_P11

typedef struct
{
    float minValue;
    float maxValue;
    uint16_t dim;
    uint16_t bright;
} BarRegion;

static const BarRegion regions[3] =
{
    {-60, 0, TFT_COLOR(60, 80, 24), TFT_COLOR(100, 250, 0)}, // Green
    {0, 4, TFT_COLOR(90, 60, 24), TFT_COLOR(255, 200, 0)},   // Yellow
    {4, 20, TFT_COLOR(90, 20, 24), TFT_COLOR(255, 0, 0)}     // Red
};

int MapValueToYPos(const uint16_t height, const float value)
{
    float yValue = (height - 6) * (((float)value - regions[0].minValue) / (regions[2].maxValue - regions[0].minValue));
    return (int)yValue;
}

void MeterBarDraw(const uint16_t x, const uint16_t y0, const uint16_t width, const uint16_t height, const float value, const float y_peak)
{
    struct
    {
        uint8_t enabled;
        uint16_t y;
        uint16_t height;
        uint16_t color;
    } max_segments[8] =
    {
        {
            0,
        },
    };
    uint8_t curr_seg = 0;
    static const uint16_t peak_color = TFT_COLOR(250, 250, 250);

    for (int y = 0; y < (height - 6); y++)
    {
        // Map y (0=top, 119=bottom) to value range (-60 to 20)
        float yValue = regions[2].maxValue - (float)y * ((regions[2].maxValue - regions[0].minValue) / (height - 6));

        // Determine which region this y-value is in
        for (int i = 0; i < 3; i++)
        {
            if (ABS(yValue - y_peak) <= 0.5f)
            {
                if (max_segments[curr_seg].enabled == 1)
                {
                    if (max_segments[curr_seg].color == peak_color)
                    {
                        if (max_segments[curr_seg].y == y0 + y + 3)
                        {
                            break;
                        }
                        goto dismiss;
                    }

                    curr_seg++;
                }

                max_segments[curr_seg].enabled = 1;
                max_segments[curr_seg].color = peak_color;
                max_segments[curr_seg].y = y0 + y + 3;
                max_segments[curr_seg].height = 1;
                break;
            }
        dismiss:
            if ((yValue >= regions[i].minValue) && (yValue <= regions[i].maxValue))
            {
                uint16_t color = (value >= yValue) ? regions[i].bright : regions[i].dim;
                if (max_segments[curr_seg].enabled == 1)
                {
                    if (max_segments[curr_seg].color == color)
                    {
                        max_segments[curr_seg].height++;
                        break;
                    }

                    curr_seg++;
                }

                max_segments[curr_seg].enabled = 1;
                max_segments[curr_seg].color = color;
                max_segments[curr_seg].y = y0 + y + 3;
                max_segments[curr_seg].height = 1;
                // gfx_fill_rect(x + 3, y0 + y + 3, width - 6, 1, color);
                break;
            }
        }
    }

    for (size_t s = 0; s < sizeof(max_segments) / sizeof(max_segments[0]); s++)
    {
        if (max_segments[s].enabled == 0)
        {
            continue;
        }

        gfx_fill_rect(x + 3, max_segments[s].y, width - 6, max_segments[s].height, max_segments[s].color);
    }
}

void MeterBarDrawBorders(const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height)
{
    /* inner black border (2px) */
    gfx_draw_rect(x + 1, y + 1, width - 2, height - 2, display_get_color(0, 0, 0));
    gfx_draw_rect(x + 2, y + 2, width - 4, height - 4, display_get_color(0, 0, 0));

    /* outer grey border (1px)*/
    gfx_draw_rect(x + 0, y + 0, width - 0, height - 0, display_get_color(77, 77, 77));
}

float transform(float in)
{
    const float offset = 1100.0;
    const float max_val = 3200.0f - offset;
    float ret = in - 1100.0;

    if (ret < 0.0)
    {
        ret = 0;
    }

    return (ret * (20 + 60) / max_val) - 60.0f + 1.0;
}

void vumeter(void *argument)
{
    UNUSED(argument);

    adc_done_sem = xSemaphoreCreateBinary();

#ifdef USE_GC9107
    gfx_init(BKL_PIN, DC_PIN, RST_PIN, CS_PIN, SCLK_PIN, MOSI_PIN, GC9107_TFTWIDTH, GC9107_TFTHEIGHT, 0);
#else
    gfx_init(BKL_PIN, DC_PIN, RST_PIN, CS_PIN, SCLK_PIN, MOSI_PIN, GC9106_TFTWIDTH, GC9106_TFTHEIGHT, 0);
#endif
    gfx_set_font(&FreeMono9pt7b);
    gfx_fill_screen(TFT_COLOR(16, 16, 16));

    gfx_set_text_color(TFT_WHITE);
    gfx_set_text_size(1);
    gfx_set_text_wrap(true);
    gfx_set_cursor(0, 0);

    MeterBarDrawBorders(VU_LEFT_X, VU_LEFT_Y, VU_LEFT_W, VU_LEFT_H);
    gfx_draw_rect(VU_LEFT_X + 0, VU_LEFT_Y + VU_LEFT_H + 4 + 0, VU_LEFT_W - 0, 16 - 0, TFT_COLOR(0, 100, 200));
    gfx_fill_rect(VU_LEFT_X + 1, VU_LEFT_Y + VU_LEFT_H + 4 + 1, VU_LEFT_W - 2, 16 - 2, TFT_COLOR(0, 170, 245));
    gfx_draw_text(VU_LEFT_X + 11, VU_LEFT_Y + VU_LEFT_H + 4 + 1, "L", TFT_WHITE);

    MeterBarDrawBorders(VU_RIGHT_X, VU_RIGHT_Y, VU_RIGHT_W, VU_RIGHT_H);
    gfx_draw_rect(VU_RIGHT_X + 0, VU_RIGHT_Y + VU_RIGHT_H + 4 + 0, VU_RIGHT_W - 0, 16 - 0, TFT_COLOR(0, 100, 200));
    gfx_fill_rect(VU_RIGHT_X + 1, VU_RIGHT_Y + VU_RIGHT_H + 4 + 1, VU_RIGHT_W - 2, 16 - 2, TFT_COLOR(0, 170, 245));
    gfx_draw_text(VU_RIGHT_X + 11, VU_RIGHT_Y + VU_RIGHT_H + 4 + 1, "R", TFT_WHITE);

    adc_Cfg_t adc_cfg =
    {
        .enabled = 1,
        .continuously_sampled_mode = 0,
        .attenuated = 0,
        .sample_time = 255, /* slow it down */
    };

    hal_adc_init();
    hal_adc_clock_config(HAL_ADC_CLOCK_80K);
    hal_adc_configure_channel(CH9, adc_cfg);
    hal_adc_configure_channel(CH4, adc_cfg);

    hal_adc_start(1, 1, adc_Poilling_evt);

    float peakL = -60; // Start from bottom
    float peakR = -60; // Start from bottom
    const float decayRate = 0.75f;
    const float minPeak = -58.0f;

    while (1)
    {
        if (xSemaphoreTake(adc_done_sem, portMAX_DELAY) != pdTRUE)
        {
            LOG("FUCK");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        float refL = transform(osal_sys_tick % 3200);//adc_values[0]);
        float refR = transform(adc_values[1]);

        if (refL > peakL)
        {
            peakL = refL;
        }
        else if (peakL > minPeak)
        {
            peakL -= decayRate;
            /* clip lower */
            if (peakL < minPeak)
                peakL = minPeak;
            /* don't go under the current value */
            if (refL > peakL)
                peakL = refL;
        }

        if (refR > peakR)
        {
            peakR = refR;
        }
        else if (peakR > minPeak)
        {
            peakR -= decayRate;
            /* clip lower */
            if (peakR < minPeak)
                peakR = minPeak;
            /* don't go under the current value */
            if (refR > peakR)
                peakR = refR;
        }

        MeterBarDraw(VU_LEFT_X, VU_LEFT_Y, VU_LEFT_W, VU_LEFT_H, refL, peakL);
        MeterBarDraw(VU_RIGHT_X, VU_RIGHT_Y, VU_RIGHT_W, VU_RIGHT_H, refR, peakR);

        hal_adc_start(1, 1, adc_Poilling_evt);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vu_meter_init()
{
    xTaskCreate(vumeter, "vumeter", 256, NULL, tskIDLE_PRIORITY + 4, NULL);
}
