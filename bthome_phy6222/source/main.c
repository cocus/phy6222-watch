
#include <types.h> /* for UNUSED */
#include <phy62xx.h>
#include <driver/aes/aes.h>
#include <driver/clock/clock.h>
#include <driver/flash/flash.h>
#include <driver/gpio/gpio.h>
#include <driver/i2c/i2c.h>
#include <driver/pwm/pwm.h>

#include <string.h>
#include <log/log.h>

#include "FreeRTOS.h"
#include "task.h"

#include "osal_nuker.h"

// LED
#define GPIO_LED GPIO_P00
#define BKL_PIN GPIO_P02

// Button
#define BUTTON_PIN GPIO_P11

uint8_t aes[16] = { '\0' };

void genericTask(void *argument)
{
    UNUSED(argument);
    LOG("Hi from genericTask");

#if 0
    const uint8_t key[16] = {
        0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
        0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0
    };

    const uint8_t test_msg[16] = "Test data here!";

    const uint8_t iv[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x11, 0x22
    };

    uint8_t enc_tag[16];
    uint8_t enc_data[16];
    uint8_t aad[] = { 0xfe, 0x00 };

    /* encypt something */
    memset(enc_data, 0, sizeof(enc_data));
    memset(enc_tag, 0, sizeof(enc_tag));
    aes_gcm_128(key, iv, sizeof(iv),
                aad, sizeof(aad),
                test_msg, sizeof(test_msg),
                enc_data, enc_tag, 1);
    //LOG("ENCRYPT Result:");
    //LOG_DUMP_BYTE(aes, sizeof(aes));
    //LOG("ENCRYPT TAG:");
    //LOG_DUMP_BYTE(tag, sizeof(tag));

    /* decrypt something else */
    //memset(aes, 0, sizeof(aes));
    //memset(tag, 0, sizeof(tag));
    aes_gcm_128(key, iv, sizeof(iv),
                aad, sizeof(aad),
                enc_data, sizeof(enc_data),
                aes, enc_tag, 0);
    //LOG("DECRYPT Result:");
    //LOG_DUMP_BYTE(aes, sizeof(aes));
    //LOG("DECRYPT TAG:");
    //LOG_DUMP_BYTE(tag, sizeof(tag));

    if (memcmp(aes, test_msg, sizeof(test_msg)) == 0)
    {
        LOG("AES encrypt/decrypt ok!");
    } else {
        LOG("AES encrypt/decrypt fail");
    }
#endif

    //hal_gpio_pin_init(GPIO_LED, GPIO_OUTPUT);
    //hal_gpio_write(GPIO_LED, 1);


    const uint8_t ds3231_addr = 0b11010000 >> 1;
    hal_i2c_init(I2C_0, I2C_CLOCK_400K, GPIO_P07, GPIO_P20, 0);

    {
        /* Set a date/time */
        const uint8_t seconds = 42;
        const uint8_t minutes = 9;
        const uint8_t hours = 6;
        const uint8_t day_of_week = 7;
        const uint8_t date = 6;
        const uint8_t month = 7;
        const uint8_t year = 25;
        uint8_t wr_data[7] = {
            ((seconds/10)<<4) | (seconds%10),
            ((minutes/10)<<4) | (minutes%10),
            ((hours/10)<<4) | (hours%10),
            day_of_week,
            ((date/10)<<4) | (date%10),
            ((month/10)<<4) | (month%10),
            ((year/10)<<4) | (year%10)
        };
        hal_i2c_master_write_reg(I2C_0, ds3231_addr, 0x00, wr_data, sizeof(wr_data), 0xffffffff);
    }

    gpio_pin_e pin = GPIO_LED;
    hal_gpio_pin_init(pin, GPIO_INPUT);
    hal_gpio_pull_set(pin, WEAK_PULL_UP);

    PWMN_e pwmN = PWM_CH1;
    hal_pwm_init(pwmN, PWM_CLK_DIV_128, PWM_CNT_UP, PWM_POLARITY_RISING, pin);
    hal_pwm_set_count_top_val(pwmN, 0, 256);
    hal_pwm_enable();

    uint16_t val = 0;

    uint8_t temperature[2] = { 0, 0 };

    uint8_t rd[7] = { 0 };

    struct
    {
        uint8_t Hour;
        uint8_t Minute;
        uint8_t Second;
        uint8_t Year;
        uint8_t Month;
        uint8_t Date;
        uint8_t DayOfWeek;
    } Time;

    while(1)
    {
        if (val++ == 256)
        {
            val = 0;
        }
        hal_pwm_set_count_val(pwmN, val);

        if (val % 32 == 0)
        {
            memset(rd, 0, 7);

            hal_i2c_master_read_reg(I2C_0, ds3231_addr, 0x00, rd, 7, 0xffffffff);
            Time.Second = ((rd[0]&0x70)>>4)*10 + (rd[0]&0x0F);
            Time.Minute = ((rd[1]&0x70)>>4)*10 + (rd[1]&0x0F);
            Time.Hour = ((rd[2]&0x30)>>4)*10 + (rd[2]&0x0F);
            Time.DayOfWeek = rd[3];
            Time.Date = ((rd[4]&0x30)>>4)*10 + (rd[4]&0x0F);
            Time.Month = ((rd[5]&0x10)>>4)*10 + (rd[5]&0x0F);
            Time.Year = ((rd[6]&0xF0)>>4)*10 + (rd[6]&0x0F);
            LOG("%02d:%02d:%02d %02d/%02d/%02d", Time.Hour, Time.Minute, Time.Second, Time.Date, Time.Month, Time.Year);

            hal_i2c_master_read_reg(I2C_0, ds3231_addr, 0x11, temperature, 2, 0xffffffff);
            LOG("Temperature is %d.%d", (int8_t)temperature[0], temperature[1]);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

// void (*p)(void) = (void(*)(void))0;
// p();
#if 0
    LOG("NVIC:");
    LOG("  ISER:       %08x ICER:   %08x",
            NVIC->ISER[0], NVIC->ICER[0]);
    LOG("  ISPR:       %08x ICPR:   %08x",
            NVIC->ISPR[0], NVIC->ICPR[0]);
    LOG("  IRQ PRIO:   %08x %08x %08x %08x",
            NVIC->IP[0], NVIC->IP[1],
            NVIC->IP[2], NVIC->IP[3]);
    LOG("              %08x %08x %08x %08x",
            NVIC->IP[4], NVIC->IP[5],
            NVIC->IP[6], NVIC->IP[7]);

    LOG("SYSCON:");
    LOG("  CPUID:      %08x",
            SCB->CPUID);
    LOG("  ICSR:       %08x AIRCR:  %08x",
            SCB->ICSR, SCB->AIRCR);
    LOG("  SCR:        %08x CCR:    %08x",
            SCB->SCR, SCB->CCR);
    LOG("  SHPR2:      %08x SHPR3:  %08x",
            SCB->SHP[0], SCB->SHP[1]);

    for (;;)
    {
        // LOG("OFF");
        hal_gpio_write(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        // LOG("ON");
        hal_gpio_write(GPIO_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
}

#if 0
void genericTask2(void *argument)
{
    UNUSED(argument);

    LOG("Hi from genericTask2");

extern void app_init();
extern void app_update();

    app_init();

    // This is a simple loop to update the display and handle button presses
    for (;;)
    {
        app_update();
    }

    // Uncomment this block to blink the backlight pin instead of running the app

/*    hal_gpio_write(BKL_PIN, 1);

    for (;;)
    {
        // LOG("OFF");
        hal_gpio_write(BKL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        // LOG("ON");
        hal_gpio_write(BKL_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
*/
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Yet, another good itoa implementation
// returns: the length of the number string
int itoa_custom(int value, char *sp, int radix)
{
    char tmp[16]; // be carefLul with the length of the buffer
    char *tp = tmp;
    int i;
    unsigned v;

    int sign = (radix == 10 && value < 0);
    if (sign)
        v = -value;
    else
        v = (unsigned)value;

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix;
        if (i < 10)
            *tp++ = i + '0';
        else
            *tp++ = i + 'a' - 10;
    }

    int len = tp - tmp;

    if (sign)
    {
        *sp++ = '-';
        len++;
    }

    while (tp > tmp)
        *sp++ = *--tp;

    return len;
}

extern const char *digits;
uint8_t *str_bin2hex(uint8_t *d, uint8_t *s, int len)
{
    while (len--)
    {
        *d++ = digits[(*s >> 4) & 0xf];
        *d++ = digits[(*s++ >> 0) & 0xf];
    }
    return d;
}

uint8_t devInfoSerialNumber[19] = {0};
#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile uint8_t g_clk32K_config = CLK_32K_RCOSC; // CLK_32K_XTAL, CLK_32K_RCOSC

#include "rf_phy_driver.h"
#include "pwrmgr.h"

void hal_lowpower_init(void)
{
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);

    #define DCDC_REF_CLK_SETTING(x)                     subWriteReg(&(AON_PMCTL0),25,25, (0x01&(x)))
    #define DCDC_CONFIG_SETTING(x)                      subWriteReg(&(AON_PMCTL0),18,15, (0x0f&(x)))
    #define DIG_LDO_CURRENT_SETTING(x)                  subWriteReg(&(AON_PMCTL0),22,21, (0x03&(x)))

    DCDC_REFL_CLK_SETTING(1);
    DCDC_CONFIG_SETTING(0x0a);
    DIG_LDO_CURRENT_SETTING(0x01);
    // drv_pm_ram_retention(RET_SRAM0 | RET_SRAM1 | RET_SRAM2);
    // hal_pwrmgr_RAM_retention(RET_SRAM0);
    hal_pwrmgr_RAM_retention_set();
    hal_pwrmgr_LowCurrentLdo_enable();
    //========= low power module clk gate
#if (PHY_MCU_TYPE == MCU_BUMBEE_CK802)
    *(volatile uint32_t *)0x40000008 = 0x001961f1; //
    *(volatile uint32_t *)0x40000014 = 0x01e00278; //
#else

    *(volatile uint32_t *)0x40000008 = 0x001961f0; //
    *(volatile uint32_t *)0x40000014 = 0x01e00279; //
#endif
}
#endif

int main(void)
{
    /* init stuff as if OSAL was in charge */
    osal_nuker_init(SYS_CLK_DLL_96M, CLK_32K_RCOSC); // SYS_CLK_XTAL_16M);

    // hal_lowpower_init();


    // LOG("SDK Version ID %08x ", SDK_VER_RELEASE_ID);

    // hal_get_flash_info();
    // uint8_t *p = str_bin2hex(devInfoSerialNumber, (uint8_t *)&phy_flash.IdentificationID, 3);
    //*p++ = '-';
    // LOG("serialnum '%s'", devInfoSerialNumber);

    LOG("g_hclk %d", g_hclk);

    xTaskCreate(genericTask, "genericTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    //extern void vu_meter_init();
    //vu_meter_init();

#ifdef ENABLE_BTSTACK
    extern void port_thread(void *args);
    xTaskCreate(port_thread, "btstack_thread", 1024, NULL, 2, NULL);
#endif

    LOG("starting FreeRTOS scheduler");

    vTaskStartScheduler();

    return 0;
}
