#include "clock.h"
#include "flash.h"
#include "log.h"

#include "comdef.h"
#include "jump_function.h"
#include "FreeRTOS.h"
#include "task.h"

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
        //LOG("OFF");
        hal_gpio_write(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        //LOG("ON");
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
        //LOG("OFF");
        hal_gpio_write(BKL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        //LOG("ON");
        hal_gpio_write(BKL_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern void clear_timer(AP_TIM_TypeDef* TIMx);
extern void LL_evt_schedule(void);

void TIM1_IRQHandler1(void)
{
  /*  HAL_ENTER_CRITICAL_SECTION() */

  if (AP_TIM1->status & 0x1)
    {
      clear_timer_int(AP_TIM1);
      clear_timer(AP_TIM1);
      LL_evt_schedule();
    }

  /* HAL_EXIT_CRITICAL_SECTION(); */
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////

extern uint32_t osal_sys_tick;

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void SysTick_Handler(void);

void Custom_SysTick_Handler(void)
{
    //osal_sys_tick += 1; // not sure?

#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
        SysTick_Handler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif
}

void _hard_fault(uint32_t *arg)
{
    uint32_t *stk = (uint32_t *)((uint32_t)arg);
    LOG("\n[Hard fault handler]");
    LOG("R0   = 0x%08x", stk[9]);
    LOG("R1   = 0x%08x", stk[10]);
    LOG("R2   = 0x%08x", stk[11]);
    LOG("R3   = 0x%08x", stk[12]);
    LOG("R4   = 0x%08x", stk[1]);
    LOG("R5   = 0x%08x", stk[2]);
    LOG("R6   = 0x%08x", stk[3]);
    LOG("R7   = 0x%08x", stk[4]);
    LOG("R8   = 0x%08x", stk[5]);
    LOG("R9   = 0x%08x", stk[6]);
    LOG("R10  = 0x%08x", stk[7]);
    LOG("R11  = 0x%08x", stk[8]);
    LOG("R12  = 0x%08x", stk[13]);
    LOG("SP   = 0x%08x", stk[0]);
    LOG("LR   = 0x%08x", stk[14]);
    LOG("PC   = 0x%08x", stk[15]);
    LOG("PSR  = 0x%08x", stk[16]);
    LOG("ICSR = 0x%08x", *(volatile uint32_t *)0xE000ED04);

    while (1)
        ;
}

void Custom_HardFault_Handler(void)
{
    uint32_t arg = 0;
    _hard_fault(&arg);
}


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

TaskHandle_t hbtstack_task;
int main(void)
{
    g_system_clk = SYS_CLK_DLL_48M; // SYS_CLK_XTAL_16M; // SYS_CLK_XTAL_16M, SYS_CLK_DBL_32M, SYS_CLK_DLL_64M

    spif_config(SYS_CLK_DLL_64M, 1, 0x801003b, 0, 0);

    drv_irq_init();

    clk_init(g_system_clk);
	hal_rtc_clock_config((CLK32K_e) g_clk32K_config);

    hal_spif_cache_init(SYS_CLK_DLL_64M, XFRD_FCMD_READ_DUAL);


    DCDC_REF_CLK_SETTING(1);
    DCDC_CONFIG_SETTING(0x0a);
    DIG_LDO_CURRENT_SETTING(0x01);
    //drv_pm_ram_retention(RET_SRAM0 | RET_SRAM1 | RET_SRAM2);
    //hal_pwrmgr_RAM_retention(RET_SRAM0);
    hal_pwrmgr_RAM_retention_set();
    hal_pwrmgr_LowCurrentLdo_enable();
    //========= low power module clk gate
#if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)
    *(volatile uint32_t *)0x40000008 = 0x001961f1;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00278;  //
#else

    *(volatile uint32_t *)0x40000008 = 0x001961f0;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00279;  //
#endif

    LOG_INIT();

    LOG("Build time: %s %s", __DATE__, __TIME__);

    LOG("SDK Version ID %08x ", SDK_VER_RELEASE_ID);

    hal_get_flash_info();
    uint8_t *p = str_bin2hex(devInfoSerialNumber, (uint8_t *)&phy_flash.IdentificationID, 3);
    *p++ = '-';
    LOG("serialnum '%s'", devInfoSerialNumber);


    /* Disable all interrupts */
    NVIC->ICER[0] = 0xFFFFFFFF;

    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     /* ll_EVT */
    NVIC_SetPriority((IRQn_Type)TIM3_IRQn,  IRQ_PRIO_APP);      /* OSAL_TICK */
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     /* LL_EXA_ADV */

    NVIC_EnableIRQ((IRQn_Type)BB_IRQn);
    NVIC_EnableIRQ((IRQn_Type)TIM1_IRQn);                   /* ll_EVT */
    NVIC_EnableIRQ((IRQn_Type)TIM3_IRQn);

    portENABLE_INTERRUPTS();

    JUMP_FUNCTION(HARDFAULT_HANDLER) = (uint32_t)&Custom_HardFault_Handler;

    // Freertos stuff
    JUMP_FUNCTION(SVC_HANDLER) = (uint32_t)&vPortSVCHandler;
    LOG("SVC handler at %08x", JUMP_FUNCTION(SVC_HANDLER));

    JUMP_FUNCTION(PENDSV_HANDLER) = (uint32_t)&xPortPendSVHandler;
    LOG("PendSV handler at %08x", JUMP_FUNCTION(PENDSV_HANDLER));

    JUMP_FUNCTION(SYSTICK_HANDLER) = (uint32_t)&Custom_SysTick_Handler;
    LOG("SysTick handler at %08x", JUMP_FUNCTION(SYSTICK_HANDLER));

    LOG("g_hclk %d", g_hclk);

    // NVIC_SetPriority((IRQn_Type)PendSV_IRQn, 15);

    xTaskCreate(genericTask, "genericTask", 256, NULL, 1, NULL);
    //xTaskCreate(genericTask2, "genericTask2", 256, NULL, 1, NULL);


    extern void port_thread(void* args);
    xTaskCreate(port_thread, "btstack_thread", 4096, NULL, 1, NULL);

    LOG("starting scheduler");

    vTaskStartScheduler();

    return 0;
}
