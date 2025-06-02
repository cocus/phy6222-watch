/*************
 clock.h
 SDK_LICENSE
***************/
#ifndef _HAL_CLOCK_H
#define _HAL_CLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <phy62xx.h>

#define TIME_DELTA(x, y) ((x >= y) ? x - y : TIME_BASE - y + x)

    typedef enum
    {
        MOD_NONE = 0,
        MOD_CK802_CPU = 0,
        MOD_DMA = 3,
        MOD_AES = 4,
        MOD_IOMUX = 7,
        MOD_UART0 = 8,
        MOD_I2C0 = 9,
        MOD_I2C1 = 10,
        MOD_SPI0 = 11,
        MOD_SPI1 = 12,
        MOD_GPIO = 13,
        MOD_QDEC = 15,
        MOD_ADCC = 17,
        MOD_PWM = 18,
        MOD_SPIF = 19,
        MOD_VOC = 20,
        MOD_TIMER5 = 21,
        MOD_TIMER6 = 22,
        MOD_UART1 = 25,

        MOD_CP_CPU = 0 + 32,
        MOD_BB = MOD_CP_CPU + 3,
        MOD_TIMER = MOD_CP_CPU + 4,
        MOD_WDT = MOD_CP_CPU + 5,
        MOD_COM = MOD_CP_CPU + 6,
        MOD_KSCAN = MOD_CP_CPU + 7,
        MOD_BBREG = MOD_CP_CPU + 9,
        BBLL_RST = MOD_CP_CPU + 10,  // can reset,but not gate in here
        BBTX_RST = MOD_CP_CPU + 11,  // can reset,but not gate in here
        BBRX_RST = MOD_CP_CPU + 12,  // can reset,but not gate in here
        BBMIX_RST = MOD_CP_CPU + 13, // can reset,but not gate in here
        MOD_TIMER1 = MOD_CP_CPU + 21,
        MOD_TIMER2 = MOD_CP_CPU + 22,
        MOD_TIMER3 = MOD_CP_CPU + 23,
        MOD_TIMER4 = MOD_CP_CPU + 24,

        MOD_PCLK_CACHE = 0 + 64,
        MOD_HCLK_CACHE = MOD_PCLK_CACHE + 1,

        MOD_USR0 = 0 + 96,
        MOD_USR1 = MOD_USR0 + 1,
        MOD_USR2 = MOD_USR0 + 2,
        MOD_USR3 = MOD_USR0 + 3,
        MOD_USR4 = MOD_USR0 + 4,
        MOD_USR5 = MOD_USR0 + 5,
        MOD_USR6 = MOD_USR0 + 6,
        MOD_USR7 = MOD_USR0 + 7,
        MOD_USR8 = MOD_USR0 + 8,
        MOD_SYSTEM = 0xFF,
    } MODULE_e;

    typedef enum
    {
        CLK_32K_XTAL = 0,
        CLK_32K_RCOSC = 1,
    } CLK32K_e;

    typedef enum
    {
        XTAL_16M = 0,
        DBL_B_32M = 1,
        DBL_32 = 2,
        DLL_32M = 3,

    } ClkSrc_e;

    typedef enum _SYSCLK_SEL
    {
        SYS_CLK_RC_32M = 0,
        SYS_CLK_DBL_32M = 1,
        SYS_CLK_XTAL_16M = 2,
        SYS_CLK_DLL_48M = 3,
        SYS_CLK_DLL_64M = 4,
        SYS_CLK_DLL_96M = 5,
        SYS_CLK_8M = 6,
        SYS_CLK_4M = 7,
        SYS_CLK_NUM = 8,
    } sysclk_t;

#ifdef USE_ROMSYM_ALIAS
    extern sysclk_t _symrom_g_system_clk;
#else
extern sysclk_t g_system_clk;
#endif

    typedef enum
    {
        HCLK_CHANGE = 0,
        AP_CLK_CHANGE = 1,
        CP_CLK_CHANGE = 2,
    } clk_update_Type_t;

    typedef struct _clk_Evt_t
    {
        uint8_t flag;
    } clk_Evt_t;

    typedef void (*clk_Hdl_t)(clk_Evt_t *pev);

    typedef struct _clk_Contex_t
    {
        bool enable;
        clk_Hdl_t evt_handler;
    } clk_Ctx_t;

#define CLAER_RTC_COUNT AP_AON->RTCCTL |= BIT(1)
#define RUN_RTC AP_AON->RTCCTL |= BIT(0)
#define STOP_RTC AP_AON->RTCCTL &= ~BIT(0)

#define hal_system_init clk_init

    extern volatile uint32_t g_hclk;
    extern volatile uint32_t hclk,pclk;
    extern volatile uint32_t osal_sys_tick;

#define clk_get_hclk() g_hclk
    uint32_t clk_get_pclk(void);

    uint32_t sysclk_get_clk(void);

    void hal_clk_gate_enable(MODULE_e module);
    void hal_clk_gate_disable(MODULE_e module);
    int hal_clk_gate_get(MODULE_e module);
    void hal_clk_get_modules_state(uint32_t *buff);
    void hal_clk_reset(MODULE_e module);
    void hal_clk_rf_config(ClkSrc_e sel);
    void hal_clk_rxadc_config(ClkSrc_e sel);

    bool hal_clk_set_pclk(uint32_t div);
    int hal_clk_init(sysclk_t hclk_sel, clk_Hdl_t evt_handler);
    void hal_rtc_clock_config(CLK32K_e clk32Mode);

    uint32_t hal_systick(void);
    uint32_t hal_ms_intv(uint32_t tick);

    extern uint32_t rtc_get_counter(void);
    void WaitMs(uint32_t msecond);
    void WaitUs(uint32_t wtTime);
    void hal_system_soft_reset(void);

    extern int clk_init(sysclk_t h_system_clk_sel);
    extern void WaitRTCCount(uint32_t rtcDelyCnt);
    extern int clk_spif_ref_clk(sysclk_t spif_ref_sel);
    extern uint32_t getMcuPrecisionCount(void);

    extern uint32_t counter_tracking;
    extern uint32_t g_counter_traking_avg;

#define CHIP_RFEQ_OFF_FLASH_ADDRESS 0x11001e08 // было 0x11004008
#define CHIP_XTAK_CAP_FLASH_ADDRESS 0x11001e0c // было 0x1100400c

    void hal_rfPhyFreqOff_Set(void);
    void hal_xtal16m_cap_Set(void);

    void hal_rc32k_clk_tracking_init(void);

#ifdef __cplusplus
}
#endif

#endif
