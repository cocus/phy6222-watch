#ifndef PHY6222_CMSIS_H
#define PHY6222_CMSIS_H

/* --------  Configuration of the Cortex-M0 Processor and Core Peripherals  ------- */
#define __CM0_REV 0              /*!< Core Revision r0p0                            */
#define __MPU_PRESENT 0          /*!< M0  provides an MPU                    */
#define __VTOR_PRESENT 0         /*!< Vector  Table  Register supported             */
#define __NVIC_PRIO_BITS 2       /*!< M0 uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used  */

/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
    /* -------------------  Cortex-M0 Processor Exceptions Numbers  ------------------- */
    NonMaskableInt_IRQn = -14, /*  2 Non Maskable Interrupt */
    HardFault_IRQn = -13,      /*  3 HardFault Interrupt */

    SVCall_IRQn = -5, /* 11 SV Call Interrupt */

    PendSV_IRQn = -2,  /* 14 Pend SV Interrupt */
    SysTick_IRQn = -1, /* 15 System Tick Interrupt */

    /* ----------------------  PHY BUMBEE M0 Interrupt Numbers  --------------------- */
    BB_IRQn = 4,    /* Base band Interrupt */
    KSCAN_IRQn = 5, /* Key scan Interrupt */
    RTC_IRQn = 6,   /* RTC Timer Interrupt */

    WDT_IRQn = 10,   /* Watchdog Timer Interrupt */
    UART0_IRQn = 11, /* UART0 Interrupt */
    I2C0_IRQn = 12,  /* I2C0 Interrupt */
    I2C1_IRQn = 13,  /* I2C1 Interrupt */
    SPI0_IRQn = 14,  /* SPI0 Interrupt */
    SPI1_IRQn = 15,  /* SPI1 Interrupt */
    GPIO_IRQn = 16,  /* GPIO Interrupt */
    UART1_IRQn = 17, /* UART1 Interrupt */
    SPIF_IRQn = 18,  /* SPIF Interrupt */
    DMAC_IRQn = 19,  /* DMAC Interrupt */
    TIM1_IRQn = 20,  /* Timer1 Interrupt */
    TIM2_IRQn = 21,  /* Timer2 Interrupt */
    TIM3_IRQn = 22,  /* Timer3 Interrupt */
    TIM4_IRQn = 23,  /* Timer4 Interrupt */
    TIM5_IRQn = 24,  /* Timer5 Interrupt */
    TIM6_IRQn = 25,  /* Timer6 Interrupt */

    AES_IRQn = 28,  /* AES Interrupt */
    ADCC_IRQn = 29, /* ADC Interrupt */
    QDEC_IRQn = 30, /* QDEC Interrupt */
    RNG_IRQn = 31   /* RNG Interrupt */
} IRQn_Type;

#endif /* PHY6222_CMSIS_H */
