
#include <types.h> /* for UNUSED */
#include <phy62xx.h>
#include <driver/clock/clock.h>
#include <driver/flash/flash.h>
#include <driver/gpio/gpio.h>

#include <string.h>
#include <log/log.h>

#include "FreeRTOS.h"
#include "task.h"

#include "osal_nuker.h"

// LED
#define GPIO_LED GPIO_P00

// Button
#define BUTTON_PIN GPIO_P11

void genericTask(void *argument)
{
    UNUSED(argument);
    LOG("Hi from genericTask");
    hal_gpio_pin_init(GPIO_LED, GPIO_OUTPUT);
    hal_gpio_write(GPIO_LED, 1);
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
#endif
    for (;;)
    {
        // LOG("OFF");
        hal_gpio_write(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        // LOG("ON");
        hal_gpio_write(GPIO_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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
    osal_nuker_init(SYS_CLK_DLL_96M); // SYS_CLK_XTAL_16M);

    // hal_lowpower_init();

    LOG("Build time: %s %s", __DATE__, __TIME__);

    // LOG("SDK Version ID %08x ", SDK_VER_RELEASE_ID);

    // hal_get_flash_info();
    // uint8_t *p = str_bin2hex(devInfoSerialNumber, (uint8_t *)&phy_flash.IdentificationID, 3);
    //*p++ = '-';
    // LOG("serialnum '%s'", devInfoSerialNumber);

    // osal_nuker_freertos_patch();

    LOG("g_hclk %d", g_hclk);

    // NVIC_SetPriority((IRQn_Type)PendSV_IRQn, 15);

    xTaskCreate(genericTask, "genericTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    extern void vu_meter_init();
    vu_meter_init();

#ifdef ENABLE_BTSTACK
    extern void port_thread(void *args);
    xTaskCreate(port_thread, "btstack_thread", 1024, NULL, 2, NULL);
#endif

    LOG("starting scheduler");

    vTaskStartScheduler();

    return 0;
}
