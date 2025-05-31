#include "jump_function.h"
#include "osal_nuker.h"

#include "FreeRTOS.h" /* for portX functions */
#include "task.h"     /* for taskX functions */

#include "bus_dev.h"

#include "rf_phy_driver.h"

#include "log.h"

#include "ll.h" // for LL_*

#include "global_config.h" // for global config stuff

#include "OSAL.h"

#include "OSAL_Clock.h" // for osalTimeUpdate()

#include "OSAL_Tasks.h"

#include "../SDK/components/ble/hci/hci_tl.h" // for HCI_ProcessEvent

/* Stuff from ROM */
extern int m_in_critical_region;

#define		LARGE_HEAP_SIZE	 (4*1024)
ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];

void osal_nuker_init(void)
{
    /* first of all, set the "m_in_critical_region" to zero as what drv_irq_init() would */
    m_in_critical_region = 0;

    osal_mem_set_heap((osalMemHdr_t*) g_largeHeap, LARGE_HEAP_SIZE);

    /* so the osal_allocate and deallocate functions work */
    osal_init_system(); /* doesn't really do much */
}

int drv_disable_irq1(void)
{
    NVIC_DisableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(TIM4_IRQn) | BIT(BB_IRQn));
    /* TODO: maybe add a mutex here */
    return m_in_critical_region++ + 1;
}

int drv_enable_irq1(void)
{
    /* TODO: maybe add a mutex here */
    int result = m_in_critical_region-- - 1;
    if (!m_in_critical_region)
    {
        NVIC_EnableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(TIM4_IRQn) | BIT(BB_IRQn));
    }
    return result;
}

static void LL_evt_schedule1(void)
{
    extern uint32_t llScanT1;
    LOG("LL_evt_schedule1 CALLED YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
    LOG("llScanT1 is %08x PREVIOUSLY", llScanT1);
    void (*func)(void) = (void (*)(void))0x8011;
    func();
    LOG("llScanT1 is %08x AFTER!", llScanT1);
}


void osal_nuker_interrupt_init(void)
{
    /* Disable all interrupts */
    //portDISABLE_INTERRUPTS();
    NVIC->ICER[0] = 0xFFFFFFFF;

    /* Patch out the disable/enable IRQ functions that OSAL used to use */
    JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE) = (uint32_t)&drv_disable_irq1;
    LOG("New HAL_DRV_IRQ_DISABLE at %08x", JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE));
    JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE) = (uint32_t)&drv_enable_irq1;
    LOG("New HAL_DRV_IRQ_ENABLE at %08x", JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE));

    //JUMP_FUNCTION(TIM1_IRQ_HANDLER) = (uint32_t)&TIM1_IRQHandler1;
    //LOG("New TIM1_IRQHandler at %08x", JUMP_FUNCTION(TIM1_IRQ_HANDLER));

    //JUMP_FUNCTION(LL_SCHEDULER) = (uint32_t)&LL_scheduler1;
    //LOG("New LL_SCHEDULER at %08x", JUMP_FUNCTION(LL_SCHEDULER));

    //JUMP_FUNCTION(LL_EVT_SCHEDULE) = (uint32_t)&LL_evt_schedule1;
    //LOG("New LL_EVT_SCHEDULE at %08x", JUMP_FUNCTION(LL_EVT_SCHEDULE));

    hal_clk_gate_enable(MOD_TIMER); /* systick */

    hal_clk_gate_enable(MOD_BB);
    hal_clk_gate_enable(MOD_BBREG);

    /* this seems to modify stuff inside the BB region, so they need to be clkd before */
    extern void efuse_init(void);
    efuse_init(); // _rom_sec_boot_init

    //hal_clk_gate_enable(MOD_TIMER1);
    //hal_clk_gate_enable(MOD_TIMER3);
    //hal_clk_gate_enable(MOD_TIMER2);
    //hal_clk_gate_enable(MOD_TIMER4);

    typedef void (*my_function)(void);
    my_function pFunc = (my_function)(0xa2e1);

    /*
      This calls boot_init(), wakeup_init(), rf_init() and rf_calibrate().
      GlobalConfig [14] needs to be set before calling it.
     */
    //pGlobal_config[CLOCK_SETTING] = g_system_clk; //CLOCK_32MHZ;
    extern void init_config(void);
    init_config();

    // Own Device Public Address
    extern uint8 ownPublicAddr[ LL_DEVICE_ADDR_LEN ];     // index 0..5 is LSO..MSB

    ownPublicAddr[3] = 0xc0; // LSO
    ownPublicAddr[4] = 0x00; // LSO
    ownPublicAddr[5] = 0x01; // MSB

    pFunc();

    //NVIC_SetPriority((IRQn_Type)BB_IRQn, IRQ_PRIO_REALTIME);
    //NVIC_SetPriority((IRQn_Type)TIM1_IRQn, IRQ_PRIO_HIGH); /* ll_EVT */
    //NVIC_SetPriority((IRQn_Type)TIM3_IRQn, IRQ_PRIO_APP);  /* OSAL_TICK */
    //NVIC_SetPriority((IRQn_Type)TIM4_IRQn, IRQ_PRIO_HIGH); /* LL_EXA_ADV */

    portENABLE_INTERRUPTS();
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
    {

    }

    extern void rf_phy_ini1(void);
    //rf_phy_ini1();

    extern void hal_rom_boot_init(void);
    // hal_rom_boot_init();
}

void osal_nuker_ble_init(void)
{
#if (HOST_CONFIG & OBSERVER_CFG)
    extern void ll_patch_advscan(void);
#else
    extern void ll_patch_slave(void);
    ll_patch_slave();

    //extern void ll_patch_master(void);
    //ll_patch_master();

    //extern void ll_patch_multi(void);
    //ll_patch_multi();
#endif

//    extern void init_config(void);
//    init_config();

    hal_rfphy_init();

    LL_Init(0xc1); // 0xc1 is the task ID for LL

    HCI_Init(0xc0);
}

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void SysTick_Handler(void);

static void Custom_SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
        SysTick_Handler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif
}

static void _hard_fault(void *arg)
{
    uint32_t *stk = (uint32_t*)(arg);
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

static void Custom_HardFault_Handler(void)
{
    uint32_t arg = 0;
    _hard_fault(&arg);
}

static struct
{
    uint8_t *task_id; // Task ID
    uint16_t events; // Events for the task
    pTaskEventHandlerFn handler; // Event handler function
} osal_fake_tasks_stuff[] =
{
    { &LL_TaskID, 0, LL_ProcessEvent }, // LL task ID is 0
    { &hciTaskID, 0, HCI_ProcessEvent } // HCI task ID is 1
};

uint8 osal_fake_set_event(uint8 task_id, uint16 events)
{
    for (int i = 0; i < sizeof(osal_fake_tasks_stuff) / sizeof(osal_fake_tasks_stuff[0]); i++)
    {
        if (!osal_fake_tasks_stuff[i].task_id)
        {
            continue;
        }

        if (*osal_fake_tasks_stuff[i].task_id == task_id)
        {
            //LOG("----> Setting event %04x for task %d (was %04x)", events, task_id, osal_fake_tasks_stuff[i].events);
            osal_fake_tasks_stuff[i].events |= events;
            return 0; // Event set successfully
        }
    }

    //LOG("----> Failed to set event %04x for task %d (task not found)", events, task_id);
    return 3;
}
void osal_simulate_task_event(uint8 idx)
{
    uint16_t buffered_ev = osal_fake_tasks_stuff[idx].events;
    if (buffered_ev != 0)
    {
        LOG("Simulating task event for task %d with events %04x", *osal_fake_tasks_stuff[idx].task_id, buffered_ev);
    }
    osal_fake_tasks_stuff[idx].events = osal_fake_tasks_stuff[idx].handler(*osal_fake_tasks_stuff[idx].task_id, buffered_ev);
}

void osal_fake_timer(void *arg)
{
    (void)arg; // unused
    while (1)
    {
        osalTimeUpdate();
        vTaskDelay(pdMS_TO_TICKS(10));

        osal_simulate_task_event(0);
        //osal_simulate_task_event(1);
        //extern int OSAL_timeSeconds;
        //LOG("tick OSAL_timeSeconds = %d", OSAL_timeSeconds);
    }
}

void powerconserve_stub(void)
{
    // This is a stub for OSAL_POWER_CONSERVE, which is not used in FreeRTOS.
    // It can be left empty or log a message if needed.
    LOG("OSAL_POWER_CONSERVE called, but not implemented in FreeRTOS.");
}



void osal_nuker_freertos_patch(void)
{
    JUMP_FUNCTION(HARDFAULT_HANDLER) = (uint32_t)&Custom_HardFault_Handler;
    LOG("New HardFault handler at %08x", JUMP_FUNCTION(HARDFAULT_HANDLER));

    /* FreeRTOS requires the SVC, PendSV and SysTick handlers routed to them */
    JUMP_FUNCTION(SVC_HANDLER) = (uint32_t)&vPortSVCHandler;
    LOG("New SVC handler at %08x", JUMP_FUNCTION(SVC_HANDLER));

    JUMP_FUNCTION(PENDSV_HANDLER) = (uint32_t)&xPortPendSVHandler;
    LOG("New PendSV handler at %08x", JUMP_FUNCTION(PENDSV_HANDLER));

    JUMP_FUNCTION(SYSTICK_HANDLER) = (uint32_t)&Custom_SysTick_Handler;
    LOG("New SysTick handler at %08x", JUMP_FUNCTION(SYSTICK_HANDLER));

    //JUMP_FUNCTION(OSAL_POWER_CONSERVE) = (uint32_t)&powerconserve_stub;
    //LOG("New OSAL_POWER_CONSERVE at %08x", JUMP_FUNCTION(OSAL_POWER_CONSERVE));


    JUMP_FUNCTION(OSAL_SET_EVENT) = (uint32_t)&osal_fake_set_event;
    LOG("New OSAL_SET_EVENT at %08x", JUMP_FUNCTION(OSAL_SET_EVENT));


    xTaskCreate(osal_fake_timer, "osal_fake_timer", 256, NULL, 4, NULL);
}
