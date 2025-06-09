/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup phy6222
  * @{
  */

#ifndef __MCU_BUMBEE_M0__
#define __MCU_BUMBEE_M0__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
/**
  * @brief Configuration of the Cortex-M0 Processor and Core Peripherals
 */
#define __CM0_REV                  0U       /*!< Core Revision r0p0                           */
#define __MPU_PRESENT              0U       /*!< M0 doesn't provide an MPU                    */
#define __VTOR_PRESENT             0U       /*!< Vector Table Register not supported          */
#define __NVIC_PRIO_BITS           2U       /*!< M0 uses 2 Bits for the Priority Levels       */
#define __Vendor_SysTickConfig     0U       /*!< Set to 1 if different SysTick Config is used */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief PHY6222 Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */

 /*!< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!<  2 Non Maskable Interrupt                            */
  HardFault_IRQn              = -13,    /*!<  3 HardFault Interrupt                               */
  SVCall_IRQn                 = -5,     /*!< 11 SV Call Interrupt                                 */
  PendSV_IRQn                 = -2,     /*!< 14 Pend SV Interrupt                                 */
  SysTick_IRQn                = -1,     /*!< 15 System Tick Interrupt                             */

/******  PHY6222 M0 specific Interrupt Numbers ****************************************************/
  BB_IRQn                     = 4,      /*!< Base band Interrupt                                  */
  KSCAN_IRQn                  = 5,      /*!< Key scan Interrupt                                   */
  RTC_IRQn                    = 6,      /*!< RTC Timer Interrupt                                  */
  WDT_IRQn                    = 10,     /*!< Watchdog Timer Interrupt                             */
  UART0_IRQn                  = 11,     /*!< UART0 Interrupt                                      */
  I2C0_IRQn                   = 12,     /*!< I2C0 Interrupt                                       */
  I2C1_IRQn                   = 13,     /*!< I2C1 Interrupt                                       */
  SPI0_IRQn                   = 14,     /*!< SPI0 Interrupt                                       */
  SPI1_IRQn                   = 15,     /*!< SPI1 Interrupt                                       */
  GPIO_IRQn                   = 16,     /*!< GPIO Interrupt                                       */
  UART1_IRQn                  = 17,     /*!< UART1 Interrupt                                      */
  SPIF_IRQn                   = 18,     /*!< SPIF Interrupt                                       */
  DMAC_IRQn                   = 19,     /*!< DMAC Interrupt                                       */
  TIM1_IRQn                   = 20,     /*!< Timer1 Interrupt                                     */
  TIM2_IRQn                   = 21,     /*!< Timer2 Interrupt                                     */
  TIM3_IRQn                   = 22,     /*!< Timer3 Interrupt                                     */
  TIM4_IRQn                   = 23,     /*!< Timer4 Interrupt                                     */
  TIM5_IRQn                   = 24,     /*!< Timer5 Interrupt                                     */
  TIM6_IRQn                   = 25,     /*!< Timer6 Interrupt                                     */
  AES_IRQn                    = 28,     /*!< AES Interrupt                                        */
  ADCC_IRQn                   = 29,     /*!< ADC Interrupt                                        */
  QDEC_IRQn                   = 30,     /*!< QDEC Interrupt                                       */
  RNG_IRQn                    = 31,     /*!< RNG Interrupt                                        */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm0.h" /* Processor and core peripherals */
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief COM?
  */
typedef struct
{
  __IO uint32_t CH0_AP_MBOX;        /*!< 0x00 */
  __IO uint32_t CH0_CP_MBOX;        /*!< 0x04 */
  __IO uint32_t CH1_AP_MBOX;        /*!< 0x08 */
  __IO uint32_t CH1_CP_MBOX;        /*!< 0x0c */
  __IO uint32_t AP_STATUS;          /*!< 0x10 */
  __IO uint32_t CP_STATUS;          /*!< 0x14 */
  __IO uint32_t AP_INTEN;           /*!< 0x18 */
  __IO uint32_t CP_INTEN;           /*!< 0x1c */
  __IO uint32_t remap;              /*!< 0x20 */
  __IO uint32_t RXEV_EN;            /*!< 0x24 */
  __IO uint32_t STCALIB;            /*!< 0x28 */
  __IO uint32_t PERI_MASTER_SELECT; /*!< 0x2c */
} AP_COM_TypeDef;

/**
  * @brief Code Cache
  */
typedef struct
{
  __IO uint32_t CTRL0;            /*!< 0x40 */
  __IO uint32_t CTRL1;            /*!< 0x44 */
  uint32_t      RESERVED[13];
  __IO uint32_t REMAP_TABLE;      /*!< 0x7c */
  __IO uint32_t REMAP_CTRL[32];   /*!< 0x80 */
} AP_CACHE_TypeDef;

/**
  * @brief Watchdog Timer
  */
typedef struct
{
  __IO uint8_t  CR;               /*!< 0x0 */
  uint8_t       RESERVED0[3];
  __IO uint32_t TORR;             /*!< 0x4 */
  __O uint32_t  CCVR;             /*!< 0x8 */
  __IO uint32_t CRR;              /*!< 0xc */
  uint8_t       STAT;             /*!< 0x10 */
  uint8_t       RESERVED1[3];
  __IO uint8_t  EOI;              /*!< 0x14 */
  uint8_t       RESERVED2[3];
} AP_WDT_TypeDef;

/**
  * @brief TIM Timers
  */
typedef struct
{
  __IO uint32_t LoadCount;        /*!< 0x0 */
  __IO uint32_t CurrentCount;     /*!< 0x4 */
  __IO uint32_t ControlReg;       /*!< 0x8 */
  __IO uint32_t EOI;              /*!< 0xc */
  __IO uint32_t status;           /*!< 0x10 */

} AP_TIM_TypeDef;

/**
  * @brief Systick Timer
  */
typedef struct
{
  __IO uint32_t IntStatus;        /*!< 0x0 */
  __IO uint32_t EOI;              /*!< 0x4 */
  __IO uint32_t unMaskIntStatus;  /*!< 0x8 */
  __IO uint32_t version;          /*!< 0xc */
} AP_TIM_SYS_TypeDef;

/**
  * @brief Universal Asynchronous Receiver Transmitter (UART)
  */
typedef struct
{
  union
  {
    __I uint8_t RBR;
    __IO uint8_t THR;
    __IO uint8_t DLL;
    uint32_t RESERVED0;           /*!< 0x0 */
  };
  union
  {
    __IO uint8_t DLM;
    __IO uint32_t IER;            /*!< 0x4 */
  };
  union
  {
    __I uint32_t IIR;             /*!< 0x8 */
    __IO uint8_t FCR;
  };
  __IO uint8_t  LCR;              /*!< 0xc */
  uint8_t       RESERVED1[3];
  __IO uint32_t MCR;              /*!< 0x10 */
  __I uint8_t   LSR;              /*!< 0x14 */
  uint8_t       RESERVED2[3];
  __IO uint32_t MSR;              /*!< 0x18 */
  __IO uint8_t  SCR;              /*!< 0x1c */
  uint8_t       RESERVED3[3];
  __IO uint32_t LPDLL;            /*!< 0x20 */
  __IO uint32_t LPDLH;            /*!< 0x24 */
  __IO uint32_t recerved[2];
  union
  {
    __IO uint32_t SRBR[16];       /*!< 0x30~0x6c */
    __IO uint32_t STHR[16];
  };
  __IO uint32_t FAR;              /*!< 0x70 */
  __IO uint32_t TFR;              /*!< 0x74 */
  __IO uint32_t RFW;              /*!< 0x78 */
  __IO uint32_t USR;              /*!< 0x7c */
  __IO uint32_t TFL;              /*!< 0x80 */
  __IO uint32_t RFL;              /*!< 0x84 */
  __IO uint32_t SRR;              /*!< 0x88 */
  __IO uint32_t SRTS;             /*!< 0x8c */
  __IO uint32_t SBCR;             /*!< 0x90 */
  __IO uint32_t SDMAM;            /*!< 0x94 */
  __IO uint32_t SFE;              /*!< 0x98 */
  __IO uint32_t SRT;              /*!< 0x9c */
  __IO uint32_t STET;             /*!< 0xa0 */
  __IO uint32_t HTX;              /*!< 0xa4 */
  __IO uint32_t DMASA;            /*!< 0xa8 */
  __IO uint32_t reserved[18];
  __IO uint32_t CPR;              /*!< 0xf4 */
  __IO uint32_t UCV;              /*!< 0xf8 */
  __IO uint32_t CTR;              /*!< 0xfc */
} AP_UART_TypeDef;

/**
  * @brief Inter Integrated Circuit Interface (I2C)
  */
typedef struct
{
  __IO uint32_t IC_CON;           /*!< 0x0 */
  __IO uint32_t IC_TAR;           /*!< 0x4 */
  __IO uint32_t IC_SAR;           /*!< 0x8 */
  __IO uint32_t IC_HS_MADDR;      /*!< 0xc */
  __IO uint32_t IC_DATA_CMD;      /*!< 0x10 */
  __IO uint32_t IC_SS_SCL_HCNT;   /*!< 0x14 */
  __IO uint32_t IC_SS_SCL_LCNT;   /*!< 0x18 */
  __IO uint32_t IC_FS_SCL_HCNT;   /*!< 0x1c */
  __IO uint32_t IC_FS_SCL_LCNT;   /*!< 0x20 */
  __IO uint32_t IC_HS_SCL_HCNT;   /*!< 0x24 */
  __IO uint32_t IC_HS_SCL_LCNT;   /*!< 0x28 */
  __IO uint32_t IC_INTR_STAT;     /*!< 0x2c */
  __IO uint32_t IC_INTR_MASK;     /*!< 0x30 */
  __IO uint32_t IC_RAW_INTR_STAT; /*!< 0x34 */
  __IO uint32_t IC_RX_TL;         /*!< 0x38 */
  __IO uint32_t IC_TX_TL;         /*!< 0x3c */
  __IO uint32_t IC_CLR_INTR;      /*!< 0x40 */
  __IO uint32_t IC_CLR_UNDER;     /*!< 0x44 */
  __IO uint32_t IC_CLR_RX_OVER;   /*!< 0x48 */
  __IO uint32_t IC_CLR_TX_OVER;   /*!< 0x4c */
  __IO uint32_t IC_CLR_RD_REG;    /*!< 0x50 */
  __IO uint32_t IC_CLR_TX_ABRT;   /*!< 0x54 */
  __IO uint32_t IC_CLR_RX_DONE;   /*!< 0x58 */
  __IO uint32_t IC_CLR_ACTIVITY;  /*!< 0x5c */
  __IO uint32_t IC_CLR_STOP_DET;  /*!< 0x60 */
  __IO uint32_t IC_CLR_START_DET; /*!< 0x64 */
  __IO uint32_t IC_CLR_GEN_CALL;  /*!< 0x68 */
  __IO uint32_t IC_ENABLE;        /*!< 0x6c */
  __IO uint32_t IC_STATUS;        /*!< 0x70 */
  __IO uint32_t IC_TXFLR;         /*!< 0x74 */
  __IO uint32_t IC_RXFLR;         /*!< 0x78 */
  __IO uint32_t IC_SDA_HOLD;      /*!< 0x7c */
  __IO uint32_t IC_TX_ABRT_SOURCE;/*!< 0x80 */
  __IO uint32_t IC_SLV_DATA_NACK_ONLY;/*!< 0x84 */
  __IO uint32_t IC_DMA_CR;        /*!< 0x88 */
  __IO uint32_t IC_DMA_TDLR;      /*!< 0x8c */
  __IO uint32_t IC_DMA_RDLR;      /*!< 0x90 */
  __IO uint32_t IC_SDA_SETUP;     /*!< 0x94 */
  __IO uint32_t IC_ACK_GENERAL_CALL;/*!< 0x98 */
  __IO uint32_t IC_ENABLE_STATUS; /*!< 0x9c */
  __IO uint32_t IC_FS_SPKLEN;     /*!< 0xa0 */
  __IO uint32_t IC_HS_SPKLEN;     /*!< 0xa4 */
} AP_I2C_TypeDef;

/**
  * @brief Inter Integrated Circuit Sound (I2S)
  */
typedef struct
{
  __IO uint32_t IER;              /*!< 0x0 */
  __IO uint32_t IRER;             /*!< 0x4 */
  __IO uint32_t ITER;             /*!< 0x8 */
  __IO uint32_t CER;              /*!< 0xc */
  __IO uint32_t CCR;              /*!< 0x10 */
  __IO uint32_t RXFFR;            /*!< 0x14 */
  __IO uint32_t TXFFR;            /*!< 0x18 */
} AP_I2S_BLOCK_TypeDef;

typedef struct
{
  union
  {
      __IO uint32_t LRBR;         /*!< 0x20 */
      __IO uint32_t LTHR;         /*!< 0x20 */
  };
  union
  {
      __IO uint32_t RRBR;         /*!< 0x24 */
      __IO uint32_t RTHR;         /*!< 0x24 */
  };
  __IO uint32_t RER;              /*!< 0x28 */
  __IO uint32_t TER;              /*!< 0x2c */
  __IO uint32_t RCR;              /*!< 0x30 */
  __IO uint32_t TCR;              /*!< 0x34 */
  __IO uint32_t ISR;              /*!< 0x38 */
  __IO uint32_t IMR;              /*!< 0x3c */
  __IO uint32_t ROR;              /*!< 0x40 */
  __IO uint32_t TOR;              /*!< 0x44 */
  __IO uint32_t RFCR;             /*!< 0x48 */
  __IO uint32_t TFCR;             /*!< 0x4c */
  __IO uint32_t RFF;              /*!< 0x50 */
  __IO uint32_t TFF;              /*!< 0x54 */

} AP_I2S_TypeDef;

/**
  * @brief General Purpose Input/Output (GPIO)
  */
typedef struct
{
  __IO uint32_t swporta_dr;       /*!< 0x00 */
  __IO uint32_t swporta_ddr;      /*!< 0x04 */
  __IO uint32_t swporta_ctl;      /*!< 0x08 */
  uint32_t      RESERVED0[9];     /*!< 0x18-0x2c portC&D */
  __IO uint32_t inten;            /*!< 0x30 */
  __IO uint32_t intmask;          /*!< 0x34 */
  __IO uint32_t inttype_level;    /*!< 0x38 */
  __IO uint32_t int_polarity;     /*!< 0x3c */
  __I uint32_t  int_status;       /*!< 0x40 */
  __IO uint32_t raw_instatus;     /*!< 0x44 */
  __IO uint32_t debounce;         /*!< 0x48 */
  __O uint32_t  porta_eoi;        /*!< 0x4c */
  __I uint32_t  ext_porta;        /*!< 0x50 */
  uint32_t      RESERVED1[3];     /*!< 0x58 0x5c */
  __IO uint32_t ls_sync;          /*!< 0x60 */
  __I uint32_t  id_code;          /*!< 0x64 */
  uint32_t      RESERVED2[1];     /*!< 0x68 */
  __I uint32_t  ver_id_code;      /*!< 0x6c */
  __I uint32_t  config_reg2;      /*!< 0x70 */
  __I uint32_t  config_reg1;      /*!< 0x74 */
} AP_GPIO_TypeDef;

/**
  * @brief Serial Peripheral Interface (SPI)
  */
typedef struct
{
  __IO uint16_t CR0;              /*!< 0x00: Control Register 0 (R/W) */
  uint16_t      RESERVED0;
  __IO uint16_t CR1;              /*!< 0x04: Control Register 1 (R/W) */
  uint16_t      RESERVED1;
  __IO uint8_t  SSIEN;            /*!< 0x08 */
  uint8_t       RESERVED2[3];
  __IO uint8_t  MWCR;             /*!< 0x0c */
  uint8_t       RESERVED3[3];
  __IO uint8_t  SER;              /*!< 0x10 */
  uint8_t       RESERVED4[3];
  __IO uint32_t BAUDR;            /*!< 0x14 */
  __IO uint32_t TXFTLR;           /*!< 0x18 */
  __IO uint32_t RXFTLR;           /*!< 0x1c */
  __O uint32_t  TXFLR;            /*!< 0x20 */
  __O uint32_t  RXFLR;            /*!< 0x24 */
  __IO uint8_t  SR;               /*!< 0x28 */
  uint8_t       RESERVED5[3];
  __IO uint32_t IMR;              /*!< 0x2c */
  __IO uint32_t ISR;              /*!< 0x30 */
  __IO uint32_t RISR;             /*!< 0x34 */
  __IO uint32_t TXOICR;           /*!< 0x38 */
  __IO uint32_t RXOICR;           /*!< 0x3c */
  __IO uint32_t RXUICR;           /*!< 0x40 */
  __IO uint32_t MSTICR;           /*!< 0x44 */
  __IO uint32_t ICR;              /*!< 0x48 */
  __IO uint32_t DMACR;            /*!< 0x4c */
  __IO uint32_t DMATDLR;          /*!< 0x50 */
  __IO uint32_t DMARDLR;          /*!< 0x54 */
  __IO uint32_t IDR;              /*!< 0x5c */
  __IO uint32_t SSI_COM_VER;      /*!< 0x5c */
  __IO uint32_t DataReg;          /*!< 0x60 */
} AP_SSI_TypeDef;

/**
  * @brief I/O MUX
  */
typedef struct
{
  __IO uint32_t Analog_IO_en;     /*!< 0x00 */
  __IO uint32_t SPI_debug_en;     /*!< 0x04 */
  __IO uint32_t debug_mux_en;     /*!< 0x08 */
  __IO uint32_t full_mux0_en;     /*!< 0x0c */
  __IO uint32_t full_mux1_en;     /*!< 0x10 reserved in some soc */
  __IO uint32_t gpio_pad_en;      /*!< 0x14 */
  __IO uint32_t gpio_sel[9];      /*!< 0x18 */
  __IO uint32_t pad_pe0;          /*!< 0x3c */
  __IO uint32_t pad_pe1;          /*!< 0x40 */
  __IO uint32_t pad_ps0;          /*!< 0x44 */
  __IO uint32_t pad_ps1;          /*!< 0x48 */
  __IO uint32_t keyscan_in_en;    /*!< 0x4c */
  __IO uint32_t keyscan_out_en;   /*!< 0x50 */
} IOMUX_TypeDef;

/**
  * @brief Always-on power domain
  */
typedef struct
{
  __IO uint32_t PWROFF;           /*!< 0x00 = 0x5a5aa5a5 enter system off mode */
  __IO uint32_t PWRSLP;           /*!< 0x04 = 0xa5a55a5a system sleep mode */
  __IO uint32_t IOCTL[3];         /*!< 0x08 0x0c 0x10 */
  __IO uint32_t PMCTL0;           /*!< 0x14 */
  __IO uint32_t PMCTL1;           /*!< 0x18 */
  __IO uint32_t PMCTL2_0;         /*!< 0x1c bit6 enable software control 32k_clk */
  __IO uint32_t PMCTL2_1;         /*!< 0x20 */
  __IO uint32_t RTCCTL;           /*!< 0x24 bit20 - enable comparator0 envent, bit18 counter overflow interrupt, bit15 enable comparator0 inerrupt, */
  __IO uint32_t RTCCNT;           /*!< 0x28 current RTC counter */
  __IO uint32_t RTCCC0;           /*!< 0x2c */
  __IO uint32_t RTCCC1;           /*!< 0x30 */
  __IO uint32_t RTCCC2;           /*!< 0x34 */
  __IO uint32_t RTCFLAG;          /*!< 0x38 */
  __IO uint32_t RTCCLK0;          /*!< 0x3C bit3:0 = sysclk_t: 1 dll 32m,  2 xtal 16m, 3 dll 48m, 4 dll 64m, 5 dll 96m */
  __IO uint32_t RTCCLK1;          /*!< 0x40 bit18 - xtal output to digital enable */
  __IO uint32_t RTCCFG1;          /*!< 0x44 - [bit16] enable digclk 96M, [bit7] enable DLL, 25:24 g_rxAdcClkSel 26:25 sel_rxadc_dbl_clk_32M_polarity, 23:22 g_rfPhyClkSel, 6:5 trim dll/dbl ldo vout */
  __IO uint32_t reserved0[5];     /*!< 0x48 4c 50 54 58 */
  __IO uint32_t RTCCFG2;          /*!< 0x5C - [bit16] 16M [bit8:4] cnt [bit3] track_en_rc32k */
  __IO uint32_t reserved1;        /*!< 0x60 */
  __IO uint32_t RTCTRCCNT;        /*!< 0x64 RC 32KHz tracking counter, calculate 16MHz ticks number per RC32KHz cycle, counter_tracking_wakeup */
  __IO uint32_t RTCTRCNT;         /*!< 0x68 */
  __IO uint32_t reserved2[13];    /*!< 0x6c 70 74 78 7c 80 84 88 8c 90 94 98 9c */
  __IO uint32_t REG_S9;           /*!< 0xa0 */
  __IO uint32_t REG_S10;          /*!< 0xa4 */
  __IO uint32_t REG_S11;          /*!< 0xa8 bit0 sleep_flag */
  __IO uint32_t IDLE_REG;         /*!< 0xac */
  __IO uint32_t GPIO_WAKEUP_SRC[2];/*!< 0xb0 b4 */
  __IO uint32_t PCLK_CLK_GATE;    /*!< 0xb8 bit0 pclk_clk_gate_en */
  __IO uint32_t XTAL_16M_CTRL;    /*!< 0xbc */
  __IO uint32_t SLEEP_R[4];       /*!< 0xc0 c4 c8 cc: SLEEP_R[0]: flags =2 RSTC_OFF_MODE, =4 RSTC_WARM_NDWC */
} AP_AON_TypeDef;

/**
  * @brief Real-Time Clock
  */
typedef struct
{
  __IO uint32_t RTCCTL;           /*!< 0x24 */
  __IO uint32_t RTCCNT;           /*!< 0x28 */
  __IO uint32_t RTCCC0;           /*!< 0x2c */
  __IO uint32_t RTCCC1;           /*!< 0x30 */
  __IO uint32_t RTCCC2;           /*!< 0x34 */
  __IO uint32_t RTCFLAG;          /*!< 0x38 */
} AP_RTC_TypeDef;

/**
  * @brief Wakeup/Sleep?
  */
typedef struct
{
  __IO uint32_t io_wu_mask_31_0;  /*!< 0xa0 */
  __IO uint32_t io_wu_mask_34_32; /*!< 0xa4 */
} AP_Wakeup_TypeDef;

/**
  * @brief Power Clock Reset (PCR)
  */
typedef struct
{
  __IO uint32_t SW_RESET0;        /*!< 0x0 */
  __IO uint32_t SW_RESET1;        /*!< 0x4 */
  __IO uint32_t SW_CLK;           /*!< 0x8 */
  __IO uint32_t SW_RESET2;        /*!< 0xc */
  __IO uint32_t SW_RESET3;        /*!< 0x10 bit 1: M0 cpu reset pulse, bit 0: M0 system reset pulse. */
  __IO uint32_t SW_CLK1;          /*!< 0x14 */
  __IO uint32_t APB_CLK;          /*!< 0x18 bit7-4: HCLK/PCLK ratio (minus 1) */
  __IO uint32_t APB_CLK_UPDATE;   /*!< 0x1c */
  __IO uint32_t CACHE_CLOCK_GATE; /*!< 0x20 */
  __IO uint32_t CACHE_RST;        /*!< 0x24 */
  __IO uint32_t CACHE_BYPASS;     /*!< 0x28 */
} AP_PCR_TypeDef;

typedef struct
{
  __IO uint32_t CLKSEL;           /*!< 0x3c */
  __IO uint32_t CLKHF_CTL0;       /*!< 0x40 */
  __IO uint32_t CLKHF_CTL1;       /*!< 0x44: bit8 doubler enabled */
  __IO uint32_t ANA_CTL;          /*!< 0x48 */
  __IO uint32_t mem_0_1_dvs;      /*!< 0x4c */
  __IO uint32_t mem_2_3_4_dvs;    /*!< 0x50 */
  __IO uint32_t efuse_cfg;        /*!< 0x54 */
  __IO uint32_t chip_state;       /*!< 0x58 */
  __IO uint32_t cal_rw;           /*!< 0x5c */
  __IO uint32_t cal_ro0;          /*!< 0x60 */
  __IO uint32_t cal_ro1;          /*!< 0x64 */
  __IO uint32_t cal_ro2;          /*!< 0x68 */
  __IO uint32_t ADC_CTL0;         /*!< 0x6c */
  __IO uint32_t ADC_CTL1;         /*!< 0x70 */
  __IO uint32_t ADC_CTL2;         /*!< 0x74 */
  __IO uint32_t ADC_CTL3;         /*!< 0x78 */
  __IO uint32_t ADC_CTL4;         /*!< 0x7c */
  uint32_t      RESERVED0[48];
  __IO uint32_t EFUSE_PROG[2];    /*!< 0x140 */
  uint32_t      RESERVED1[6];
  __IO uint32_t EFUSE0[2];        /*!< 0x160 */
  __IO uint32_t EFUSE1[2];        /*!< 0x168 */
  __IO uint32_t EFUSE2[2];        /*!< 0x170 */
  __IO uint32_t EFUSE3[2];        /*!< 0x178 */
  __IO uint32_t SECURTY_STATE;    /*!< 0x180 */
} AP_PCRM_TypeDef;

/**
  * @brief Analog to Digital Converter (ADC)
  */
typedef struct
{
  __IO uint32_t enable;           /*!< 0x00: ADCC voice enable. Setting this to “1” will enable voice core work */
  __IO uint32_t RESERVED0[2];     /*!< 0x04~0x08 */
  __IO uint32_t control_1;        /*!< 0x0c: ADCC voice control 1 */
  __IO uint32_t control_2;        /*!< 0x10: ADCC voice control 2 */
  __IO uint32_t control_3;        /*!< 0x14: ADCC voice control 3 */
  __IO uint32_t control_4;        /*!< 0x18: ADCC voice control 4 */
  __IO uint32_t compare_reset;    /*!< 0x1c */
  __IO uint32_t int_pointer_ch0_ch3;/*!< 0x20 */
  __IO uint32_t int_pointer_ch4_ch7;/*!< 0x24 */
  //__IO uint32_t    int_pointer[2];          //0x20~0x24
  __IO uint32_t RESERVED1[3];     /*!< 0x28~0x30 */
  __IO uint32_t intr_mask;        /*!< 0x34 */
  __IO uint32_t intr_clear;       /*!< 0x38 */
  __IO uint32_t intr_status;      /*!< 0x3c */
  __IO uint32_t compare_cfg[8];   /*!< 0x40~0x5c */
} AP_ADCC_TypeDef;

/**
  * @brief SPI Flash
  */
typedef struct
{
  __IO uint32_t config;           /*!< 0x0,QSPI Configuration Register,R/W */
  __IO uint32_t read_instr;       /*!< 0x4,Device Read Instruction Register,R/W */
  __IO uint32_t write_instr;      /*!< 0x8,Device Write Instruction Register,R/W */
  __IO uint32_t delay;            /*!< 0xC,QSPI Device Delay Register,R/W */
  __IO uint32_t rddata_capture;   /*!< 0x10,Read Data Capture Register,R/W */
  __IO uint32_t dev_size;         /*!< 0x14,Device Size Register,R/W */
  __IO uint32_t sram_part;        /*!< 0x18,SRAM Partition Register,R/W */
  __IO uint32_t indirect_ahb_addr_trig;/*!< 0x1C,Indirect AHB Address Trigger Register,R/W */
  __IO uint32_t dma_peripheral;   /*!< 0x20,DMA Peripheral Register,R/W */
  __IO uint32_t remap;            /*!< 0x24,Remap Address Register,R/W */
  __IO uint32_t mode_bit;         /*!< 0x28,Mode Bit Register,R/W */
  __IO uint32_t sram_fill_level;  /*!< 0x2C,SRAM Fill Level Register,RO */
  __IO uint32_t tx_threshold;     /*!< 0x30,TX Threshold Register,R/W */
  __IO uint32_t rx_threshold;     /*!< 0x34,RX Threshold Register,R/W */
  __IO uint32_t wr_completion_ctrl;/*!<  0x38,Write Completion Control Register,R/W */
  __IO uint32_t poll_expire;      /*!< 0x3C,Polling Expiration Register,R/W */
  __IO uint32_t int_status;       /*!< 0x40,Interrupt Status Register,R/W */
  __IO uint32_t int_mask;         /*!< 0x44,Interrupt Mask,R/W */
  __I uint32_t  RESERVED0[2];     /*!< 0x48~0x4c,Empty */
  __IO uint32_t low_wr_protection;/*!< 0x50,Lower Write Protection Register,R/W */
  __IO uint32_t up_wr_protection; /*!< 0x54,Upper Write Protection Register,R/W */
  __IO uint32_t wr_protection;    /*!< 0x58,Write Protection Register,R/W */
  __I uint32_t  RESERVED1;        /*!< 0x5c,Empty */
  __IO uint32_t indirect_rd;      /*!< 0x60,Indirect Read Transfer Register,R/W */
  __IO uint32_t indirect_rd_watermark;/*!< 0x64,Indirect Read Transfer Watermark Register,R/W */
  __IO uint32_t indirect_rd_start_addr;/*!< 0x68,Indirect Read Transfer Start Address Register,R/W */
  __IO uint32_t indirect_rd_num;  /*!< 0x6C,Indirect Read Transfer Number Bytes Register,R/W */
  __IO uint32_t indirect_wr;      /*!< 0x70,Indirect Write Transfer Register,R/W */
  __IO uint32_t indirect_wr_watermark;        /*!< 0x74,Indirect Write Transfer Watermark Register,R/W */
  __IO uint32_t indirect_wr_start_addr;       /*!< 0x78,Indirect Write Transfer Start Address Register,R/W */
  __IO uint32_t indirect_wr_cnt;  /*!< 0x7C,Indirect Write Transfer Count Register,R/W */
  __IO uint32_t indirect_ahb_trig_addr_range; /*!< 0x80,Indirect AHB Trigger Address Range Register,R/W */
  __I uint32_t  RESERVED2[3];     /*!< 0x84~0x8c,Empty */
  __IO uint32_t fcmd;             /*!< 0x90,Flash Command Register,R/W */
  __IO uint32_t fcmd_addr;        /*!< 0x94,Flash Command Address Registers,R/W */
  __I uint32_t  RESERVED3[2];     /*!< 0x98~0x9c,Empty */
  __IO uint32_t fcmd_rddata[2];   /*!< 0xA0,Flash Command Read Data Register (low-a0, up-a4),RO */
  __IO uint32_t fcmd_wrdata[2];   /*!< 0xA8,Flash Command Write Data Register (low-a8, up-ac),R/W */
  __IO uint32_t poll_fstatus;     /*!< 0xB0,Polling Flash Status Register,RO */
  //__IO uint32_t    ;  //0xFC,Module ID Register,RO
} AP_SPIF_TypeDef;

/**
  * @brief Keyboard Scan
  */
typedef struct
{
  __IO uint32_t ctrl0;            /*!< 0xc0 */
  __IO uint32_t ctrl1;            /*!< 0xc4 */
  __IO uint32_t mk_in_en;         /*!< 0xc8 */
  __IO uint32_t mkc[6];           /*!< 0xcc~0xe0 */
} AP_KSCAN_TypeDef;

/**
  * @brief Pulse-width modulation (PWM)
  */
typedef struct
{
  __IO uint32_t pwmen;            /*!< 0x00? */
} AP_PWM_TypeDef;

typedef struct
{
  __IO uint32_t ctrl0;            /*!< 0x00? */
  __IO uint32_t ctrl1;            /*!< 0x04? */
} AP_PWMCTRL_TypeDef;

/**
  * @brief Direct memory access controller (DMA)
  */
typedef struct
{
  __IO uint32_t SAR;              /*!< 0x0? */
  __IO uint32_t SAR_H;            /*!< 0x4? */
  __IO uint32_t DAR;              /*!< 0x8? */
  __IO uint32_t DAR_H;            /*!< 0xc? */
  __IO uint32_t LLP;              /*!< 0x10? */
  __IO uint32_t LLP_H;            /*!< 0x14? */
  __IO uint32_t CTL;              /*!< 0x18? */
  __IO uint32_t CTL_H;            /*!< 0x1c? */
  __IO uint32_t SSTAT;            /*!< 0x20? */
  __IO uint32_t SSTAT_H;          /*!< 0x24? */
  __IO uint32_t DSTAT;            /*!< 0x28? */
  __IO uint32_t DSTAT_L;          /*!< 0x2c? */
  __IO uint32_t SSTATAR;          /*!< 0x30? */
  __IO uint32_t SSTATAR_H;        /*!< 0x34? */
  __IO uint32_t DSTATAR;          /*!< 0x38? */
  __IO uint32_t DSTATAR_H;        /*!< 0x3c? */
  __IO uint32_t CFG;              /*!< 0x40? */
  __IO uint32_t CFG_H;            /*!< 0x44? */
  __IO uint32_t RESERVED0[4];
} AP_DMA_CH_TypeDef;

typedef struct
{
  __IO uint32_t RawTfr;           /*!< 0x2c0 */
  __IO uint32_t RawTfr_H;         /*!< 0x2c4 */
  __IO uint32_t RawBlock;         /*!< 0x2c8 */
  __IO uint32_t RawBlock_H;       /*!< 0x2cc */
  __IO uint32_t RawSrcTran;       /*!< 0x2d0 */
  __IO uint32_t RawSrcTran_H;     /*!< 0x2d4 */
  __IO uint32_t RawDstTran;       /*!< 0x2d8 */
  __IO uint32_t RawDstTran_H;     /*!< 0x2dc */
  __IO uint32_t RawErr;           /*!< 0x2e0 */
  __IO uint32_t RawErr_H;         /*!< 0x2e4 */
  __IO uint32_t StatusTfr;        /*!< 0x2e8 */
  __IO uint32_t StatusTfr_H;      /*!< 0x2ec */
  __IO uint32_t StatusBlock;      /*!< 0x2f0 */
  __IO uint32_t StatusBlock_H;    /*!< 0x2f4 */
  __IO uint32_t StatusSrcTran;    /*!< 0x2f8 */
  __IO uint32_t StatusSrcTran_H;  /*!< 0x2fc */
  __IO uint32_t StatusDstTran;    /*!< 0x300 */
  __IO uint32_t StatusDstTran_H;  /*!< 0x304 */
  __IO uint32_t StatusErr;        /*!< 0x308 */
  __IO uint32_t StatusErr_H;      /*!< 0x30c */
  __IO uint32_t MaskTfr;          /*!< 0x310 */
  __IO uint32_t MaskTfr_H;        /*!< 0x314 */
  __IO uint32_t MaskBlock;        /*!< 0x318 */
  __IO uint32_t MaskBlock_H;      /*!< 0x31c */
  __IO uint32_t MaskSrcTran;      /*!< 0x320 */
  __IO uint32_t MaskSrcTran_H;    /*!< 0x324 */
  __IO uint32_t MaskDstTran;      /*!< 0x328 */
  __IO uint32_t MaskDstTran_H;    /*!< 0x32c */
  __IO uint32_t MaskErr;          /*!< 0x330 */
  __IO uint32_t MaskErr_H;        /*!< 0x334 */
  __IO uint32_t ClearTfr;         /*!< 0x338 */
  __IO uint32_t ClearTfr_H;       /*!< 0x33c */
  __IO uint32_t ClearBlock;       /*!< 0x340 */
  __IO uint32_t ClearBlock_H;     /*!< 0x344 */
  __IO uint32_t ClearSrcTran;     /*!< 0x348 */
  __IO uint32_t ClearSrcTran_H;   /*!< 0x34c */
  __IO uint32_t ClearDstTran;     /*!< 0x350 */
  __IO uint32_t ClearDstTran_H;   /*!< 0x354 */
  __IO uint32_t ClearErr;         /*!< 0x358 */
  __IO uint32_t ClearErr_H;       /*!< 0x35c */
  __IO uint32_t StatusInt;        /*!< 0x360 */
  __IO uint32_t StatusInt_H;      /*!< 0x364 */
} AP_DMA_INT_TypeDef;

typedef struct
{
  __IO uint32_t ReqSrcReg;        /*!< 0x368 */
  __IO uint32_t ReqSrcReg_H;      /*!< 0x36c */
  __IO uint32_t ReqDstReg;        /*!< 0x370 */
  __IO uint32_t ReqDstReg_H;      /*!< 0x374 */
  __IO uint32_t SglReqSrcReg;     /*!< 0x378 */
  __IO uint32_t SglReqSrcReg_H;   /*!< 0x37c */
  __IO uint32_t SglReqDstReg;     /*!< 0x380 */
  __IO uint32_t SglReqDstReg_H;   /*!< 0x384 */
  __IO uint32_t LstSrcReg;        /*!< 0x388 */
  __IO uint32_t LstSrcReg_H;      /*!< 0x38c */
  __IO uint32_t LstDstReg;        /*!< 0x390 */
  __IO uint32_t LstDstReg_H;      /*!< 0x394 */
} AP_DMA_SW_HANDSHAKE_TypeDef;

typedef struct
{
  __IO uint32_t DmaCfgReg;        /*!< 0x398 */
  __IO uint32_t DmaCfgReg_H;      /*!< 0x39c */
  __IO uint32_t ChEnReg;          /*!< 0x3a0 */
  __IO uint32_t ChEnReg_H;        /*!< 0x3a4 */
  __IO uint32_t DmaIdReg;         /*!< 0x3a8 */
  __IO uint32_t DmaIdReg_H;       /*!< 0x3ac */
  __IO uint32_t DmaTestReg;       /*!< 0x3b0 */
  __IO uint32_t DmaTestReg_H;     /*!< 0x3b4 */
  __IO uint32_t RESERVED0[4];
  __IO uint32_t DMA_COMP_PARAMS_6;/*!< 0x3c8 */
  __IO uint32_t DMA_COMP_PARAMS_6_H;/*!< 0x3cc */
  __IO uint32_t DMA_COMP_PARAMS_5;/*!< 0x3d0 */
  __IO uint32_t DMA_COMP_PARAMS_5_H;/*!< 0x3d4 */
  __IO uint32_t DMA_COMP_PARAMS_4;/*!< 0x3d8 */
  __IO uint32_t DMA_COMP_PARAMS_4_H;/*!< 0x3dc */
  __IO uint32_t DMA_COMP_PARAMS_3;/*!< 0x3e0 */
  __IO uint32_t DMA_COMP_PARAMS_3_H;/*!< 0x3e4 */
  __IO uint32_t DMA_COMP_PARAMS_2;/*!< 0x3e8 */
  __IO uint32_t DMA_COMP_PARAMS_2_H;/*!< 0x3ec */
  __IO uint32_t DMA_COMP_PARAMS_1;/*!< 0x3f0 */
  __IO uint32_t DMA_COMP_PARAMS_1_H;/*!< 0x3f4 */
  __IO uint32_t DMA_ID;           /*!< 0x3f8 */
  __IO uint32_t DMA_ID_H;         /*!< 0x3fc */
} AP_DMA_MISC_TypeDef;

// TODO!!!: EFUSES, MDM

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define AP_APB0_BASE          0x40000000UL /*!< Peripheral base address in the alias region */
#define AP_APB2_BASE          0x4000F000UL /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map (APB0) */
#define AP_PCR_BASE           (AP_APB0_BASE + 0x0000)
#define AP_TIM1_BASE          (AP_APB0_BASE + 0x1000)
#define AP_TIM2_BASE          (AP_APB0_BASE + 0x1014)
#define AP_TIM3_BASE          (AP_APB0_BASE + 0x1028)
#define AP_TIM4_BASE          (AP_APB0_BASE + 0x103c)
#define AP_TIM5_BASE          (AP_APB0_BASE + 0x1050)
#define AP_TIM6_BASE          (AP_APB0_BASE + 0x1064)
#define AP_TIM_SYS_BASE       (AP_APB0_BASE + 0x10a0)
#define AP_WDT_BASE           (AP_APB0_BASE + 0x2000)
#define AP_COM_BASE           (AP_APB0_BASE + 0x3000)
#define AP_IOMUX_BASE         (AP_APB0_BASE + 0x3800)
#define AP_UART0_BASE         (AP_APB0_BASE + 0x4000)
#define AP_I2C0_BASE          (AP_APB0_BASE + 0x5000)
#define AP_I2C1_BASE          (AP_APB0_BASE + 0x5800)
#define AP_SPI0_BASE          (AP_APB0_BASE + 0x6000)
#define AP_SPI1_BASE          (AP_APB0_BASE + 0x7000)
#define AP_GPIOA_BASE         (AP_APB0_BASE + 0x8000)
#define AP_UART1_BASE         (AP_APB0_BASE + 0x9000)
#define AP_DMIC_BASE          (AP_APB0_BASE + 0xA000)
#define AP_QDEC_BASE          (AP_APB0_BASE + 0xB000)
#define AP_CACHE_BASE         (AP_APB0_BASE + 0xC000)
#define AP_SPIF_BASE          (AP_APB0_BASE + 0xC800)
#define AP_KSCAN_BASE         (AP_APB0_BASE + 0xD0C0)
#define AP_PWM_BASE           (AP_APB0_BASE + 0xE000)

/*!< Peripheral memory map (APB2) */
#define AP_AON_BASE           (AP_APB2_BASE + 0x0000)
#define AP_RTC_BASE           (AP_APB2_BASE + 0x0024)
#define AP_PCRM_BASE          (AP_APB2_BASE + 0x003c)
#define AP_WAKEUP_BASE        (AP_APB2_BASE + 0x00a0)

/*!< Peripheral memory map (others) */
#define AP_DMAC_BASE          (0x40010000UL)
#define AP_MDM_BASE           (0x40030000UL)
#define AP_AES_BASE           (0x40040000UL)
#define ADCC_BASE_ADDR        (0x40050000UL)

#define FLASH_BASE_ADDR       (0x11000000)
#define SPIF_BASE_ADDR        (0x11080000)

#define SRAM0_BASE_ADDRESS    (0x1FFF0000)
#define SRAM1_BASE_ADDRESS    (0x1FFF4000)
#define SRAM2_BASE_ADDRESS    (0x1FFF8000)

/*!< Combination of all SRAMs */
#define SRAM_BASE_ADDR        (SRAM0_BASE_ADDRESS)
#define SRAM_END_ADDR         (SRAM_BASE_ADDR+0x10000-1)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define AP_PCR              ((AP_PCR_TypeDef *)AP_PCR_BASE)
#define AP_TIM1             ((AP_TIM_TypeDef *)AP_TIM1_BASE)
#define AP_TIM2             ((AP_TIM_TypeDef *)AP_TIM2_BASE)
#define AP_TIM3             ((AP_TIM_TypeDef *)AP_TIM3_BASE)
#define AP_TIM4             ((AP_TIM_TypeDef *)AP_TIM4_BASE)
#define AP_TIM5             ((AP_TIM_TypeDef *)AP_TIM5_BASE)
#define AP_TIM6             ((AP_TIM_TypeDef *)AP_TIM6_BASE)
#define AP_TIMS             ((AP_TIM_SYS_TypeDef *)AP_TIM_SYS_BASE)
#define AP_WDT              ((AP_WDT_TypeDef *)AP_WDT_BASE)
#define AP_COM              ((AP_COM_TypeDef *)AP_COM_BASE)
#define AP_IOMUX            ((IOMUX_TypeDef *)AP_IOMUX_BASE)
#define AP_UART0            ((AP_UART_TypeDef *)AP_UART0_BASE)
#define AP_I2C0             ((AP_I2C_TypeDef *)AP_I2C0_BASE)
#define AP_I2C1             ((AP_I2C_TypeDef *)AP_I2C1_BASE)
#define AP_SPI0             ((AP_SSI_TypeDef *)AP_SPI0_BASE)
#define AP_SPI1             ((AP_SSI_TypeDef *)AP_SPI1_BASE)
#define AP_GPIO             ((AP_GPIO_TypeDef *)AP_GPIOA_BASE)
#define AP_UART1            ((AP_UART_TypeDef *)AP_UART1_BASE)
#define AP_CACHE            ((AP_CACHE_TypeDef *)AP_CACHE_BASE)
#define AP_SPIF             ((AP_SPIF_TypeDef *)AP_SPIF_BASE)
#define AP_KSCAN            ((AP_KSCAN_TypeDef *)AP_KSCAN_BASE)
#define AP_PWM              ((AP_PWM_TypeDef *)AP_PWM_BASE)
#define AP_PWM_CTRL(n)      ((AP_PWMCTRL_TypeDef *)(AP_PWM_BASE + 4 + n * 12))
#define AP_AON              ((AP_AON_TypeDef *)AP_AON_BASE)
#define AP_RTC              ((AP_RTC_TypeDef *)AP_RTC_BASE)
#define AP_PCRM             ((AP_PCRM_TypeDef *)AP_PCRM_BASE)
#define AP_WAKEUP           ((AP_Wakeup_TypeDef *)AP_WAKEUP_BASE)
#define AP_ADCC             ((AP_ADCC_TypeDef *)ADCC_BASE_ADDR)
#define AP_DMA_CH_CFG(n)    ((AP_DMA_CH_TypeDef *)(AP_DMAC_BASE + 0x58 * n))
#define AP_DMA_INT          ((AP_DMA_INT_TypeDef *)(AP_DMAC_BASE + 0x2c0))
#define AP_DMA_SW_HANDSHAKE ((AP_DMA_SW_HANDSHAKE_TypeDef *)(AP_DMAC_BASE + 0x368))
#define AP_DMA_MISC         ((AP_DMA_MISC_TypeDef *)(AP_DMAC_BASE + 0x398))

/**
  * @}
  */


/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Hardware_Constant_Definition
    * @{
    */
#define TIME_BASE                       0x003fffff /*!< 24bit count shift 2 bit as 1us/bit */
  /**
    * @}
    */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                             SPI FLASH registers                            */
/*                                                                            */
/******************************************************************************/

/*!< Endpoint-specific registers */
#define SPIF_FCMD                           (AP_SPIF->fcmd)

/********************  Bit definition for FCMD register  **********************/
#define SPIF_FCMD_EXEC_Pos                  (0U)
#define SPIF_FCMD_EXEC_Msk                  (0x1UL << SPIF_FCMD_EXEC_Pos)       /*!< 0x00000001 */
#define SPIF_FCMD_EXEC                      SPIF_FCMD_EXEC_Msk                  /*!< SPIF exec flag, maybe? */

#define SPIF_FCMD_BUSY_Pos                  (1U)
#define SPIF_FCMD_BUSY_Msk                  (0x1UL << SPIF_FCMD_BUSY_Pos)       /*!< 0x00000002 */
#define SPIF_FCMD_BUSY                      SPIF_FCMD_BUSY_Msk                  /*!< SPIF Busy flag, maybe? */

#define SPIF_FCMD_WREN_Pos                  (15U)
#define SPIF_FCMD_WREN_Msk                  (0x1UL << SPIF_FCMD_WREN_Pos)       /*!< 0x0008000 */
#define SPIF_FCMD_WREN                      SPIF_FCMD_WREN_Msk                  /*!< SPIF Write enable flag? */

#define SPIF_FCMD_MBIT_Pos                  (18U)
#define SPIF_FCMD_MBIT_Msk                  (0x1UL << SPIF_FCMD_MBIT_Pos)       /*!< 0x00040000 */
#define SPIF_FCMD_MBIT                      SPIF_FCMD_MBIT_Msk                  /*!< SPIF MBIT flag? */

#define SPIF_FCMD_ADDREN_Pos                (19U)
#define SPIF_FCMD_ADDREN_Msk                (0x1UL << SPIF_FCMD_ADDREN_Pos)     /*!< 0x00080000 */
#define SPIF_FCMD_ADDREN                    SPIF_FCMD_ADDREN_Msk                /*!< SPIF Address enable flag? */

#define SPIF_FCMD_RDEN_Pos                  (23U)
#define SPIF_FCMD_RDEN_Msk                  (0x1UL << SPIF_FCMD_RDEN_Pos)       /*!< 0x00800000 */
#define SPIF_FCMD_RDEN                      SPIF_FCMD_RDEN_Msk                  /*!< SPIF Read enable flag? */

#define SPIF_FCMD_OPS_Pos                   (24U)

#define SPIF_CMD_RESET                      (0x99UL)
#define SPIF_CMD_ENRST                      (0x66UL)
#define SPIF_CMD_WREN                       (0x06UL)
#define SPIF_CMD_WRDIS                      (0x04UL)
#define SPIF_CMD_VSRWREN                    (0x50UL)
#define SPIF_CMD_CERASE                     (0x60UL)
#define SPIF_CMD_SERASE                     (0x20UL)
#define SPIF_CMD_BERASE32                   (0x52UL)
#define SPIF_CMD_BERASE64                   (0xD8UL)
#define SPIF_CMD_DPWRDN                     (0xB9UL)
#define SPIF_CMD_RLSDPD                     (0xABUL)
#define SPIF_CMD_WRST                       (0x01UL)
#define SPIF_CMD_RDID                       (0x9FUL)
#define SPIF_CMD_RDST                       (0x05UL)
#define SPIF_CMD_RDST_H                     (0x35UL)
#define SPIF_CMD_PPROG                      (0x02UL)
#define SPIF_CMD_READ                       (0x03UL)
#define SPIF_CMD_READF                      (0x0BUL)
#define SPIF_CMD_READDO                     (0x3BUL)
#define SPIF_CMD_READDIO                    (0xBBUL)
#define SPIF_CMD_READQO                     (0x6BUL)
#define SPIF_CMD_READQIO                    (0xeBUL)
#define SPIF_CMD_READQIOW                   (0xe7UL)

#define SPIF_FCMD_OP_RESET                  (SPIF_CMD_RESET << SPIF_FCMD_OPS_Pos)    /*!< 0x99000000: reset */
#define SPIF_FCMD_OP_ENRST                  (SPIF_CMD_ENRST << SPIF_FCMD_OPS_Pos)    /*!< 0x66000000: enable reset */
#define SPIF_FCMD_OP_WREN                   (SPIF_CMD_WREN << SPIF_FCMD_OPS_Pos)     /*!< 0x06000000: Write Enable */
#define SPIF_FCMD_OP_WRDIS                  (SPIF_CMD_WRDIS << SPIF_FCMD_OPS_Pos)    /*!< 0x04000000: write disable */
#define SPIF_FCMD_OP_VSRWREN                (SPIF_CMD_VSRWREN << SPIF_FCMD_OPS_Pos)  /*!< 0x50000000: Volatile SR Write Enable */
#define SPIF_FCMD_OP_CERASE                 (SPIF_CMD_CERASE << SPIF_FCMD_OPS_Pos)   /*!< 0x60000000: (or 0xC7)chip erase */
#define SPIF_FCMD_OP_SERASE                 (SPIF_CMD_SERASE << SPIF_FCMD_OPS_Pos)   /*!< 0x20000000: sector erase */
#define SPIF_FCMD_OP_BERASE32               (SPIF_CMD_BERASE32 << SPIF_FCMD_OPS_Pos) /*!< 0x52000000: block erease 32k */
#define SPIF_FCMD_OP_BERASE64               (SPIF_CMD_BERASE64 << SPIF_FCMD_OPS_Pos) /*!< 0xD8000000: block erease 64k */
#define SPIF_FCMD_OP_DPWRDN                 (SPIF_CMD_DPWRDN << SPIF_FCMD_OPS_Pos)   /*!< 0xB9000000: deep power down */
#define SPIF_FCMD_OP_RLSDPD                 (SPIF_CMD_RLSDPD << SPIF_FCMD_OPS_Pos)   /*!< 0xAB000000: release from powerdown (and read device id) */
#define SPIF_FCMD_OP_WRST                   (SPIF_CMD_WRST << SPIF_FCMD_OPS_Pos)     /*!< 0x01000000: Write Status */
#define SPIF_FCMD_OP_RDID                   (SPIF_CMD_RDID << SPIF_FCMD_OPS_Pos)     /*!< 0x9F000000: read ID */
#define SPIF_FCMD_OP_RDST                   (SPIF_CMD_RDST << SPIF_FCMD_OPS_Pos)     /*!< 0x05000000: read status */
#define SPIF_FCMD_OP_RDST_H                 (SPIF_CMD_RDST_H << SPIF_FCMD_OPS_Pos)   /*!< 0x35000000: read status high byte */
#define SPIF_FCMD_OP_PPROG                  (SPIF_CMD_PPROG << SPIF_FCMD_OPS_Pos)    /*!< 0x02000000: page program */
#define SPIF_FCMD_OP_READ                   (SPIF_CMD_READ << SPIF_FCMD_OPS_Pos)     /*!< 0x03000000: read */
#define SPIF_FCMD_OP_READF                  (SPIF_CMD_READF << SPIF_FCMD_OPS_Pos)    /*!< 0x0B000000: fast read */
#define SPIF_FCMD_OP_READDO                 (SPIF_CMD_READDO << SPIF_FCMD_OPS_Pos)   /*!< 0x3B000000: dual output fast read */
#define SPIF_FCMD_OP_READDIO                (SPIF_CMD_READDIO << SPIF_FCMD_OPS_Pos)  /*!< 0xBB000000: dual I/O fast read */
#define SPIF_FCMD_OP_READQO                 (SPIF_CMD_READQO << SPIF_FCMD_OPS_Pos)   /*!< 0x6B000000: quad output fast read */
#define SPIF_FCMD_OP_READQIO                (SPIF_CMD_READQIO << SPIF_FCMD_OPS_Pos)  /*!< 0xeB000000: quad I/O fast read */
#define SPIF_FCMD_OP_READQIOW               (SPIF_CMD_READQIOW << SPIF_FCMD_OPS_Pos) /*!< 0xe7000000: quad I/O fast read word */


/******************************************************************************/
/*                                                                            */
/*                       RTC Control in AON register                          */
/*                                                                            */
/******************************************************************************/

/*!< Endpoint-specific registers */
#define AON_PWROFF                          (AP_AON->PWROFF)
#define AON_RTCCTL                          (AP_AON->RTCCTL)
#define AON_PMCTL0                          (AP_AON->PMCTL0)
#define AON_PMCTL2_0                        (AP_AON->PMCTL2_0)
#define AON_SLEEPR0                         (AP_AON->SLEEP_R[0])
#define AON_SLEEPR1                         (AP_AON->SLEEP_R[1])

/*******************  Bit definition for RTCCTL register  *********************/
#define AON_RTCCTL_RTC_Pos                  (0U)
#define AON_RTCCTL_RTC_Msk                  (0x1UL << AON_RTCCTL_RTC_Pos)       /*!< 0x00000001 */
#define AON_RTCCTL_RTC                      AON_RTCCTL_RTC_Msk                  /*!< RTC Run/Stop Control Flag */

#define AON_RTCCTL_RTCCLR_Pos               (1U)
#define AON_RTCCTL_RTCCLR_Msk               (0x1UL << AON_RTCCTL_RTCCLR_Pos)    /*!< 0x00000002 */
#define AON_RTCCTL_RTCCLR                   AON_RTCCTL_RTCCLR_Msk               /*!< RTC Count Clear Flag */

#define AON_RTCCTL_COMP0INT_Pos             (15U)
#define AON_RTCCTL_COMP0INT_Msk             (0x1UL << AON_RTCCTL_COMP0INT_Pos)  /*!< 0x0008000 */
#define AON_RTCCTL_COMP0INT                 AON_RTCCTL_COMP0INT_Msk             /*!< Enable Comparator 0 Interrupt */

#define AON_RTCCTL_COUNTOVF_Pos             (18U)
#define AON_RTCCTL_COUNTOVF_Msk             (0x1UL << AON_RTCCTL_COUNTOVF_Pos)  /*!< 0x0040000 */
#define AON_RTCCTL_COUNTOVF                 AON_RTCCTL_COUNTOVF_Msk             /*!< Enable Counter Overflow Interrupt */

#define AON_RTCCTL_COMP0EVT_Pos             (20U)
#define AON_RTCCTL_COMP0EVT_Msk             (0x1UL << AON_RTCCTL_COMP0EVT_Pos)  /*!< 0x0100000 */
#define AON_RTCCTL_COMP0EVT                 AON_RTCCTL_COMP0EVT_Msk             /*!< Enable Comparator 0 Event Flag */

/******************************************************************************/
/*                                                                            */
/*                       Clock gating for PCR                                 */
/*                                                                            */
/******************************************************************************/

/*!< Endpoint-specific registers */
#define PCR_SWCLK                           (AP_PCR->SW_CLK)
#define PCR_SWCLK1                          (AP_PCR->SW_CLK1)
#define PCR_CACHE_CLOCK_GATE                (AP_PCR->CACHE_CLOCK_GATE)
#define PCR_SW_RESET0                       (AP_PCR->SW_RESET0)
#define PCR_SW_RESET1                       (AP_PCR->SW_RESET1)
#define PCR_SW_RESET2                       (AP_PCR->SW_RESET2)
#define PCR_CACHE_RST                       (AP_PCR->CACHE_RST)

/*******************  Bit definition for SW_CLK register  *********************/
#define PCR_SWCTL_CK802_Pos                 (0U)
#define PCR_SWCTL_CK802_Msk                 (0x1UL << PCR_SWCTL_CK802_Pos)      /*!< 0x00000001 */
#define PCR_SWCTL_CK802                     PCR_SWCTL_CK802_Msk                 /*!< Unknown CK802 CPU clock gate control  */

#define PCR_SWCTL_DMA_Pos                   (3U)
#define PCR_SWCTL_DMA_Msk                   (0x1UL << PCR_SWCTL_DMA_Pos)        /*!< 0x00000008 */
#define PCR_SWCTL_DMA                       PCR_SWCTL_DMA_Msk                   /*!< DMA clock gate control  */

#define PCR_SWCTL_AES_Pos                   (4U)
#define PCR_SWCTL_AES_Msk                   (0x1UL << PCR_SWCTL_AES_Pos)        /*!< 0x00000010 */
#define PCR_SWCTL_AES                       PCR_SWCTL_AES_Msk                   /*!< AES128 clock gate control  */

#define PCR_SWCTL_IOMUX_Pos                 (7U)
#define PCR_SWCTL_IOMUX_Msk                 (0x1UL << PCR_SWCTL_IOMUX_Pos)      /*!< 0x00000080 */
#define PCR_SWCTL_IOMUX                     PCR_SWCTL_IOMUX_Msk                 /*!< IOMUX clock gate control  */

#define PCR_SWCTL_UART0_Pos                 (8U)
#define PCR_SWCTL_UART0_Msk                 (0x1UL << PCR_SWCTL_UART0_Pos)      /*!< 0x00000100 */
#define PCR_SWCTL_UART0                     PCR_SWCTL_UART0_Msk                 /*!< UART0 clock gate control  */

#define PCR_SWCTL_I2C0_Pos                  (9U)
#define PCR_SWCTL_I2C0_Msk                  (0x1UL << PCR_SWCTL_I2C0_Pos)       /*!< 0x00000200 */
#define PCR_SWCTL_I2C0                      PCR_SWCTL_I2C0_Msk                  /*!< I2C0 clock gate control  */

#define PCR_SWCTL_I2C1_Pos                  (10U)
#define PCR_SWCTL_I2C1_Msk                  (0x1UL << PCR_SWCTL_I2C1_Pos)       /*!< 0x00000400 */
#define PCR_SWCTL_I2C1                      PCR_SWCTL_I2C1_Msk                  /*!< I2C1 clock gate control  */

#define PCR_SWCTL_SPI0_Pos                  (11U)
#define PCR_SWCTL_SPI0_Msk                  (0x1UL << PCR_SWCTL_SPI0_Pos)       /*!< 0x00000800 */
#define PCR_SWCTL_SPI0                      PCR_SWCTL_SPI0_Msk                  /*!< SPI0 clock gate control  */

#define PCR_SWCTL_SPI1_Pos                  (12U)
#define PCR_SWCTL_SPI1_Msk                  (0x1UL << PCR_SWCTL_SPI1_Pos)       /*!< 0x00001000 */
#define PCR_SWCTL_SPI1                      PCR_SWCTL_SPI1_Msk                  /*!< SPI1 clock gate control  */

#define PCR_SWCTL_GPIO_Pos                  (13U)
#define PCR_SWCTL_GPIO_Msk                  (0x1UL << PCR_SWCTL_GPIO_Pos)       /*!< 0x00002000 */
#define PCR_SWCTL_GPIO                      PCR_SWCTL_GPIO_Msk                  /*!< GPIO clock gate control  */

#define PCR_SWCTL_QDEC_Pos                  (15U)
#define PCR_SWCTL_QDEC_Msk                  (0x1UL << PCR_SWCTL_QDEC_Pos)       /*!< 0x00008000 */
#define PCR_SWCTL_QDEC                      PCR_SWCTL_QDEC_Msk                  /*!< QDEC clock gate control  */

#define PCR_SWCTL_ADCC_Pos                  (17U)
#define PCR_SWCTL_ADCC_Msk                  (0x1UL << PCR_SWCTL_ADCC_Pos)       /*!< 0x00020000 */
#define PCR_SWCTL_ADCC                      PCR_SWCTL_ADCC_Msk                  /*!< ADCC clock gate control  */

#define PCR_SWCTL_PWM_Pos                   (18U)
#define PCR_SWCTL_PWM_Msk                   (0x1UL << PCR_SWCTL_PWM_Pos)        /*!< 0x00040000 */
#define PCR_SWCTL_PWM                       PCR_SWCTL_PWM_Msk                   /*!< PWM clock gate control  */

#define PCR_SWCTL_SPIF_Pos                  (19U)
#define PCR_SWCTL_SPIF_Msk                  (0x1UL << PCR_SWCTL_SPIF_Pos)       /*!< 0x00080000 */
#define PCR_SWCTL_SPIF                      PCR_SWCTL_SPIF_Msk                  /*!< SPIF clock gate control  */

#define PCR_SWCTL_VOC_Pos                   (20U)
#define PCR_SWCTL_VOC_Msk                   (0x1UL << PCR_SWCTL_VOC_Pos)        /*!< 0x00100000 */
#define PCR_SWCTL_VOC                       PCR_SWCTL_VOC_Msk                   /*!< VOC clock gate control  */

#define PCR_SWCTL_TIM5_Pos                  (21U)
#define PCR_SWCTL_TIM5_Msk                  (0x1UL << PCR_SWCTL_TIM5_Pos)       /*!< 0x00200000 */
#define PCR_SWCTL_TIM5                      PCR_SWCTL_TIM5_Msk                  /*!< TIM5 clock gate control  */

#define PCR_SWCTL_TIM6_Pos                  (22U)
#define PCR_SWCTL_TIM6_Msk                  (0x1UL << PCR_SWCTL_TIM6_Pos)       /*!< 0x00400000 */
#define PCR_SWCTL_TIM6                      PCR_SWCTL_TIM6_Msk                  /*!< TIM6 clock gate control  */

#define PCR_SWCTL_UART1_Pos                 (25U)
#define PCR_SWCTL_UART1_Msk                 (0x1UL << PCR_SWCTL_UART1_Pos)      /*!< 0x02000000 */
#define PCR_SWCTL_UART1                     PCR_SWCTL_UART1_Msk                 /*!< UART1 clock gate control  */

/*******************  Bit definition for SW_CLK1 register  ********************/
// SW_CLK1 -->0x4000f014
#define _CLK_M0_CPU                         (BIT(0))
#define _CLK_BB                             (BIT(3))
#define _CLK_TIMER                          (BIT(4))
#define _CLK_WDT                            (BIT(5))
#define _CLK_COM                            (BIT(6))
#define _CLK_KSCAN                          (BIT(7))
#define _CLK_BBREG                          (BIT(9))
#define _CLK_TIMER1                         (BIT(21))
#define _CLK_TIMER2                         (BIT(22))
#define _CLK_TIMER3                         (BIT(23))
#define _CLK_TIMER4                         (BIT(24))

/**
  * @}
*/

/**
  * @}
*/

/** @addtogroup Exported_macro
  * @{
  */


/*********************************** AON **************************************/
#define AON_CLEAR_RTC_COUNT               (AON_RTCCTL |= AON_RTCCTL_RTCCLR)
#define AON_CLEAR_XTAL_TRACKING_AND_CALIB AON_SLEEPR1 = 0

/*********************************** WDT **************************************/
#define AP_WDT_ENABLE_STATE ((AP_WDT->CR & 0x01)) // 1:enable 0:disable
#define AP_WDT_FEED         \
    do                      \
    {                       \
        AP_WDT->CRR = 0x76; \
    } while (0)



/******************************** IRQ Prios ***********************************/
#define IRQ_PRIO_REALTIME               0 /*!< */
#define IRQ_PRIO_HIGH                   1 /*!< */
#define IRQ_PRIO_HAL                    2 /*!< */
#define IRQ_PRIO_THREAD                 3 /*!< */
#define IRQ_PRIO_APP                    3 /*!< */

    // #define ROM_SRAM_JUMPTABLE SRAM_BASE_ADDR
    // #define ROM_SRAM_GLOBALCFG (ROM_SRAM_JUMPTABLE + 0x400)
    // #define ROM_SRAM_JUMPTABLE_MIRROR 0x1fffd000
    // #define ROM_SRAM_GLOBALCFG_MIRROR (ROM_SRAM_JUMPTABLE_MIRROR + 0x400)
    // #define ROM_SRAM_HEAP 0x1fffe000
    // #define ROM_SRAM_HEAP_SIZE (1024 * 8)
    // #define ROM_SRAM_DWC_BUF 0x1ffffc00
    // #define APP_SRAM_START_ADDR 0x1fff2000

    // 0x4000f05C - [bit16] 16M [bit8:4] cnt [bit3] track_en_rc32k
    // 0x4000f064 - RC 32KHz tracking counter, calculate 16MHz ticks number per RC32KHz cycle
    // uint32_t  counter_tracking // 24bit tracking counter, read from 0x4000f064
    // counter_tracking = g_counter_traking_avg = STD_RC32_16_CYCLE_16MHZ_CYCLE; hal_rc32k_clk_tracking_init()
    // 0x4000f0C0 - SLEEP_R[0] flags =2 RSTC_OFF_MODE, =4 RSTC_WARM_NDWC
    // 0x4000f0C4 - SLEEP_R[1] bit7 - first wakeupinit, tracking flags
    // 0x4000f0C8 - SLEEP_R[2] использую для сохранения UTC счета времени при перезагрузке
    // 0x4000f0CС - SLEEP_R[3] использую для сохранения UTC счета времени при перезагрузке
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
  }
#endif /* __cplusplus */

#endif /* __MCU_BUMBEE_M0__ */
