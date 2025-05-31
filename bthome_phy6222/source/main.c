#include "clock.h"
#include "flash.h"
#include "log.h"

#include "comdef.h"
#include "jump_function.h"
#include "FreeRTOS.h"
#include "task.h"

#include "osal_nuker.h"

// LED
#define GPIO_LED GPIO_P00

// Vibrator
#define GPIO_VIBRATOR GPIO_P03

// Display
#define DC_PIN GPIO_P25
#define RST_PIN GPIO_P24
#define CS_PIN GPIO_P31
#define BKL_PIN GPIO_P01

#define SCLK_PIN GPIO_P34
#define MOSI_PIN GPIO_P32

// Button
#define BUTTON_PIN GPIO_P11

void genericTask(void *argument)
{
    UNUSED(argument);
    LOG("Hi from genericTask");
    hal_gpio_write(GPIO_LED, 1);

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

void genericTask2(void *argument)
{
    UNUSED(argument);

    LOG("Hi from genericTask2");
    hal_gpio_write(BKL_PIN, 1);

    for (;;)
    {
        // LOG("OFF");
        hal_gpio_write(BKL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        // LOG("ON");
        hal_gpio_write(BKL_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////


const char *hex_ascii = {"0123456789ABCDEF"};
uint8_t *str_bin2hex(uint8_t *d, uint8_t *s, int len)
{
    while (len--)
    {
        *d++ = hex_ascii[(*s >> 4) & 0xf];
        *d++ = hex_ascii[(*s++ >> 0) & 0xf];
    }
    return d;
}

uint8 devInfoSerialNumber[19] = {0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile uint8 g_clk32K_config = CLK_32K_RCOSC; // CLK_32K_XTAL, CLK_32K_RCOSC

#include "rf_phy_driver.h"
#include "pwrmgr.h"

void hal_lowpower_init(void)
{
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);

    DCDC_REF_CLK_SETTING(1);
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

TaskHandle_t hbtstack_task;
int main(void)
{
    g_system_clk = SYS_CLK_DLL_48M; // SYS_CLK_XTAL_16M; // SYS_CLK_XTAL_16M, SYS_CLK_DBL_32M, SYS_CLK_DLL_64M

    spif_config(SYS_CLK_DLL_64M, 1, 0x801003b, 0, 0);

    /* init stuff as if OSAL was in charge */
    osal_nuker_init();

    clk_init(g_system_clk);

    hal_spif_cache_init(SYS_CLK_DLL_64M, XFRD_FCMD_READ_DUAL);

    hal_lowpower_init();

    /* init interrupt stuff related to what OSAL used */
    osal_nuker_interrupt_init();

    JUMP_FUNCTION(UART0_IRQ_HANDLER) = hal_UART0_IRQHandler;
    LOG_INIT();

    LOG("Build time: %s %s", __DATE__, __TIME__);

    LOG("SDK Version ID %08x ", SDK_VER_RELEASE_ID);

    hal_get_flash_info();
    uint8_t *p = str_bin2hex(devInfoSerialNumber, (uint8_t *)&phy_flash.IdentificationID, 3);
    *p++ = '-';
    LOG("serialnum '%s'", devInfoSerialNumber);

    osal_nuker_freertos_patch();

    LOG("g_hclk %d", g_hclk);




    // NVIC_SetPriority((IRQn_Type)PendSV_IRQn, 15);

    xTaskCreate(genericTask, "genericTask", 256, NULL, 1, NULL);
    // xTaskCreate(genericTask2, "genericTask2", 256, NULL, 1, NULL);

    extern void port_thread(void *args);
    xTaskCreate(port_thread, "btstack_thread", 4096, NULL, 2, NULL);

    LOG("starting scheduler");

    vTaskStartScheduler();

    return 0;
}
