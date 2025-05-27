/*
  main.c
*/

#include "bus_dev.h"
// FREERTOS: #include "config.h"
#include "gpio.h"
#include "clock.h"
#include "global_config.h"
#include "timer.h"
#include "jump_function.h"
#include "pwrmgr.h"
#include "mcu.h"
#include "gpio.h"
#include "log.h"
#include "rf_phy_driver.h"
#include "flash.h"
// #include "flash_eep.h"
#include "version.h"
#include "watchdog.h"
#include "adc.h"
// #include "ble_ota.h"
#include "mcu_phy_bumbee.h"

#define DEFAULT_UART_BAUD 115200

/*********************************************************************
 LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 EXTERNAL FUNCTIONS
 */

extern void init_config(void);
extern void app_main(void);
extern void hal_rom_boot_init(void);
/*********************************************************************
 CONNECTION CONTEXT RELATE DEFINITION
 */

#define BLE_MAX_ALLOW_CONNECTION 1
#define BLE_MAX_ALLOW_PKT_PER_EVENT_TX 3
#define BLE_MAX_ALLOW_PKT_PER_EVENT_RX 3
#define BLE_PKT_VERSION BLE_PKT_VERSION_5_1 // BLE_PKT_VERSION_4_0

/*	BLE_MAX_ALLOW_PER_CONNECTION
 {
 ...
 struct ll_pkt_desc *tx_conn_desc[MAX_LL_BUF_LEN];	   // new Tx data buffer
 struct ll_pkt_desc *rx_conn_desc[MAX_LL_BUF_LEN];

 struct ll_pkt_desc *tx_not_ack_pkt;
 struct ll_pkt_desc *tx_ntrm_pkts[MAX_LL_BUF_LEN];
 ...
 }
 tx_conn_desc[] + tx_ntrm_pkts[]	--> BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2
 rx_conn_desc[]				--> BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE
 tx_not_ack_pkt				--> 1*BLE_PKT_BUF_SIZE

 */

#define BLE_PKT_BUF_SIZE (((BLE_PKT_VERSION == BLE_PKT_VERSION_5_1) ? 1 : 0) * BLE_PKT51_LEN + ((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN + (sizeof(struct ll_pkt_desc) - 2))

#define BLE_MAX_ALLOW_PER_CONNECTION ((BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE * 2) + (BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE) + BLE_PKT_BUF_SIZE)

#define BLE_CONN_BUF_SIZE (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)

ALIGN4_U8 g_pConnectionBuffer[BLE_CONN_BUF_SIZE];
llConnState_t pConnContext[BLE_MAX_ALLOW_CONNECTION];

/*********************************************************************
 CTE IQ SAMPLE BUF config
 */
// #define BLE_SUPPORT_CTE_IQ_SAMPLE TRUE
#ifdef BLE_SUPPORT_CTE_IQ_SAMPLE
uint16 g_llCteSampleI[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
uint16 g_llCteSampleQ[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
#endif

/*********************************************************************
 OSAL LARGE HEAP CONFIG
 */
#define LARGE_HEAP_SIZE (4 * 1024)
ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];

#if 0 // SDK_VER_RELEASE_ID >= 0x030103 ?
#define LL_LINKBUF_CFG_NUM 0
#define LL_PKT_BUFSIZE 280
#define LL_LINK_HEAP_SIZE ((BLE_MAX_ALLOW_CONNECTION * 3 + LL_LINKBUF_CFG_NUM) * LL_PKT_BUFSIZE) // basic Space + configurable Space
ALIGN4_U8 g_llLinkHeap[LL_LINK_HEAP_SIZE];
#endif

/*********************************************************************
 GLOBAL VARIABLES
 */
volatile uint8 g_clk32K_config;
volatile sysclk_t g_spif_clk_config;

/*********************************************************************
 EXTERNAL VARIABLES
 */
// extern uint32_t __initial_sp;

static void hal_low_power_io_init(void)
{
    //========= disable all gpio pullup/down to preserve juice
    const ioinit_cfg_t ioInit[] = {
#if (SDK_VER_CHIP == __DEF_CHIP_QFN32__)
        {GPIO_P00, GPIO_FLOATING},
        {GPIO_P01, GPIO_FLOATING},
        {GPIO_P02, GPIO_FLOATING},
        {GPIO_P03, GPIO_FLOATING},
        {GPIO_P07, GPIO_FLOATING},
        {GPIO_P09, GPIO_PULL_UP},  // TX1
        {GPIO_P10, GPIO_PULL_UP},  // RX1
        {GPIO_P11, GPIO_FLOATING},
        {GPIO_P14, GPIO_FLOATING},
        {GPIO_P15, GPIO_FLOATING},
        {GPIO_P16, GPIO_FLOATING},
        {GPIO_P17, GPIO_FLOATING},
        {GPIO_P18, GPIO_FLOATING},
        {GPIO_P20, GPIO_FLOATING},
        {GPIO_P23, GPIO_FLOATING},
        {GPIO_P24, GPIO_FLOATING},
        {GPIO_P25, GPIO_FLOATING},
        {GPIO_P26, GPIO_FLOATING},
        //		{GPIO_P27, GPIO_FLOATING },
        {GPIO_P31, GPIO_FLOATING},
        {GPIO_P32, GPIO_FLOATING},
        {GPIO_P33, GPIO_FLOATING}, // I2C_SDA
        {GPIO_P34, GPIO_FLOATING}  // I2C_SCL
#else
        {GPIO_P02, GPIO_FLOATING},
        {GPIO_P03, GPIO_FLOATING},
        {GPIO_P07, GPIO_FLOATING},
        {GPIO_P09, GPIO_FLOATING},
        {GPIO_P10, GPIO_FLOATING},
        {GPIO_P11, GPIO_FLOATING},
        {GPIO_P14, GPIO_FLOATING},
        {GPIO_P15, GPIO_FLOATING},
        {GPIO_P18, GPIO_FLOATING},
        {GPIO_P20, GPIO_FLOATING},
        {GPIO_P34, GPIO_FLOATING},

#endif
    };

    for (uint8_t i = 0; i < sizeof(ioInit) / sizeof(ioinit_cfg_t); i++)
    {
        hal_gpio_pull_set(ioInit[i].pin, ioInit[i].type);
    }

    DCDC_CONFIG_SETTING(0x0a);
    DCDC_REF_CLK_SETTING(1);
    DIG_LDO_CURRENT_SETTING(1);
#if defined(__GNUC__)
    extern uint32 g_irqstack_top;
    // Check IRQ STACK (1KB) location

    /*
        if ((uint32_t) &g_irqstack_top > 0x1fffc000) {
            hal_pwrmgr_RAM_retention(RET_SRAM0 | RET_SRAM1 | RET_SRAM2);
        } else
    */
    if ((uint32_t)&g_irqstack_top > 0x1fff8000)
    {
        hal_pwrmgr_RAM_retention(RET_SRAM0 | RET_SRAM1);
    }
    else
    {
        hal_pwrmgr_RAM_retention(RET_SRAM0); // RET_SRAM0|RET_SRAM1|RET_SRAM2
    }
#else
#if DEBUG_INFO || SDK_VER_RELEASE_ID != 0x03010102
    hal_pwrmgr_RAM_retention(RET_SRAM0 | RET_SRAM1); // RET_SRAM0|RET_SRAM1|RET_SRAM2
#else
    hal_pwrmgr_RAM_retention(RET_SRAM0 | RET_SRAM1); // RET_SRAM0|RET_SRAM1|RET_SRAM2
#endif
#endif
    hal_pwrmgr_RAM_retention_set();
    subWriteReg(0x4000f014, 26, 26, 1); // hal_pwrmgr_LowCurrentLdo_enable();
                                        // hal_pwrmgr_LowCurrentLdo_disable();
}

static void ble_mem_init_config(void)
{
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
    LL_InitConnectContext(pConnContext, g_pConnectionBuffer,
                          BLE_MAX_ALLOW_CONNECTION,
                          BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                          BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                          BLE_PKT_VERSION);
#if (MAX_CONNECTION_SLAVE_NUM > 0)
    static ALIGN4_U8 g_llDevList[BLE_CONN_LL_DEV_LIST_SIZE];
    ll_multi_conn_llDevList_Init(g_llDevList);
#endif

#if MAX_NUM_LL_CONN > 1
    Host_InitContext(MAX_NUM_LL_CONN,
                     glinkDB, glinkCBs,
                     smPairingParam,
                     gMTU_Size,
                     gAuthenLink,
                     l2capReassembleBuf, l2capSegmentBuf,
                     gattClientInfo,
                     gattServerInfo);
#endif
#ifdef BLE_SUPPORT_CTE_IQ_SAMPLE
    LL_EXT_Init_IQ_pBuff(g_llCteSampleI, g_llCteSampleQ);
#endif
}

static void hal_rfphy_init(void)
{
    // Watchdog_Init(NULL);
    //============config the txPower
    g_rfPhyTxPower = RF_PHY_TX_POWER_0DBM;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet = RF_PHY_FREQ_FOFF_00KHZ; //	hal_rfPhyFreqOff_Set();
    //============config xtal 16M cap
    XTAL16M_CAP_SETTING(0x09); //	hal_xtal16m_cap_Set();
    XTAL16M_CURRENT_SETTING(0x01);

    hal_rc32k_clk_tracking_init();
    { /* замена hal_rom_boot_init() */
        extern void efuse_init(void);
        efuse_init();
        typedef void (*my_function)(void);
        my_function pFunc = (my_function)(0xa2e1);
        // ble_main();
        pFunc();
    }
    NVIC_SetPriority((IRQn_Type)BB_IRQn, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn, IRQ_PRIO_HIGH); // ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn, IRQ_PRIO_HIGH); // OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn, IRQ_PRIO_HIGH); // LL_EXA_ADV
    // ble memory init and config
    ble_mem_init_config();
}

static void hal_init(void)
{
    hal_low_power_io_init();
    clk_init(g_system_clk); // system init
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);
    hal_pwrmgr_init();

    // g_system_clk, SYS_CLK_DLL_64M, SYS_CLK_RC_32M / XFRD_FCMD_READ_QUAD, XFRD_FCMD_READ_DUAL
    hal_spif_cache_init(SYS_CLK_DLL_64M, XFRD_FCMD_READ_DUAL);
    hal_gpio_init();
    LOG_INIT();

    hal_adc_init();
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

// LED
#define GPIO_LED GPIO_P00

// Vibrator
#define GPIO_VIBRATOR GPIO_P03

// Display
#define DC_PIN   GPIO_P25
#define RST_PIN  GPIO_P24
#define CS_PIN   GPIO_P31
#define BKL_PIN  GPIO_P01

#define SCLK_PIN GPIO_P34
#define MOSI_PIN GPIO_P32

// Button
#define BUTTON_PIN GPIO_P11


#include "FreeRTOS.h"
#include "task.h"
void genericTask(void const * argument)
{
    LOG("Hi from genericTask");
    hal_gpio_write(GPIO_LED, 1);

    for(;;)
    {
        LOG("OFF");
        hal_gpio_write(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        LOG("ON");
        hal_gpio_write(GPIO_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void genericTask2(void const * argument)
{
    LOG("Hi from genericTask2");
    hal_gpio_write(BKL_PIN, 1);

    for(;;)
    {
        LOG("OFF");
        hal_gpio_write(BKL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        LOG("ON");
        hal_gpio_write(BKL_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

extern uint32_t osal_sys_tick;

void Systick_Handler_Wrapper(void)
{
    osal_sys_tick += 1; // not sure?

#if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
    xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
    g_system_clk = SYS_CLK_DLL_96M;// SYS_CLK_XTAL_16M; // SYS_CLK_XTAL_16M, SYS_CLK_DBL_32M, SYS_CLK_DLL_64M
    g_clk32K_config = CLK_32K_RCOSC; // CLK_32K_XTAL, CLK_32K_RCOSC

    // FREERTOS: drv_irq_init();
    init_config();

#if (HOST_CONFIG & OBSERVER_CFG)
    extern void ll_patch_advscan(void);
#else
    extern void ll_patch_slave(void);
    ll_patch_slave();
#endif

    // FREERTOS: hal_rfphy_init();
    hal_init();

    // FREERTOS: batt_start_measure();
    LOG("Build time: %s %s", __DATE__, __TIME__);

    LOG("SDK Version ID %08x ", SDK_VER_RELEASE_ID);
    LOG("rfClk %d rcClk %d sysClk %d tpCap[%02x %02x]", g_rfPhyClkSel, g_clk32K_config, g_system_clk, g_rfPhyTpCal0, g_rfPhyTpCal1);
    LOG("sizeof(struct ll_pkt_desc) = %d, buf size = %d", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);
    LOG("sizeof(g_pConnectionBuffer) = %d, sizeof(pConnContext) = %d, sizeof(largeHeap)=%d ",
        sizeof(g_pConnectionBuffer), sizeof(pConnContext), sizeof(g_largeHeap));

    LOG("[RESET CAUSE] %d", g_system_reset_cause);

    hal_get_flash_info();
    uint8_t *p = str_bin2hex(devInfoSerialNumber, (uint8_t *)&phy_flash.IdentificationID, 3);
    *p++ = '-';
    LOG("serialnum '%s'", devInfoSerialNumber);

    // Freertos stuff
    extern void vPortSVCHandler( void );
    JUMP_FUNCTION(SVC_HANDLER)                   =   (uint32_t)&vPortSVCHandler;
    LOG("SVC handler at %08x", JUMP_FUNCTION(SVC_HANDLER));

    extern void xPortPendSVHandler( void );
    JUMP_FUNCTION(PENDSV_HANDLER)                =   (uint32_t)&xPortPendSVHandler;
    LOG("PendSV handler at %08x", JUMP_FUNCTION(PENDSV_HANDLER));

    JUMP_FUNCTION(SYSTICK_HANDLER)               =   (uint32_t)&Systick_Handler_Wrapper;
    LOG("SysTick handler at %08x", JUMP_FUNCTION(SYSTICK_HANDLER));

    LOG("g_hclk %d", g_hclk);

    //NVIC_SetPriority((IRQn_Type)PendSV_IRQn, 15);

    xTaskCreate(genericTask, "genericTask", 256, NULL, 1, NULL);
    xTaskCreate(genericTask2, "genericTask2", 256, NULL, 1, NULL);

    LOG("starting scheduler");

    vTaskStartScheduler();

    return 0;
}
