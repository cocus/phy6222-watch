/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_GPIO_H
#define _HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <phy62xx.h>

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */

/**
  * @brief  PHY62xx GPIOs in either GPIO_P** or P** format.
  */
typedef enum
{
  GPIO_P00 = 0,             /*!< GPIO 00: QFN32 pin 1 */
  P0 = GPIO_P00,

  GPIO_P01 = 1,             /*!< GPIO 01: QFN32 pin 2 */
  P1 = GPIO_P01,

  GPIO_P02 = 2,             /*!< GPIO 02/SWD_IO: QFN32 pin 3 */
  P2 = GPIO_P02,

  GPIO_P03 = 3,             /*!< GPIO 03/SWD_CLK: QFN32 pin 4 */
  P3 = GPIO_P03,

  GPIO_P07 = 4,             /*!< GPIO 07: QFN32 pin 6 */
  P7 = GPIO_P07,

  // TODO!!!: Investigate if GPIO08 is actually TM

  GPIO_P09 = 5,             /*!< GPIO 09: QFN32 pin 8 */
  P9 = GPIO_P09,

  GPIO_P10 = 6,             /*!< GPIO 10: QFN32 pin 9 */
  P10 = GPIO_P10,

  GPIO_P11 = 7,             /*!< GPIO 11/AIO_0: QFN32 pin 10 */
  P11 = GPIO_P11,
  Analog_IO_0 = GPIO_P11,   /*!< P11 can also be muxed as a single ended (AIO_0) or differential (Input B-) analog IO */

  GPIO_P14 = 8,             /*!< GPIO 14/AIO_3: QFN32 pin 11 */
  P14 = GPIO_P14,
  Analog_IO_3 = GPIO_P14,   /*!< P14 can also be muxed as a single ended (AIO_3) or differential (Input C+) analog IO */

  GPIO_P15 = 9,             /*!< GPIO 15/AIO_4/MICBIAS: QFN32 pin 15 */
  P15 = GPIO_P15,
  Analog_IO_4 = GPIO_P15,   /*!< P15 can also be muxed as a single ended (AIO_4) or differential (Input D-) analog IO */

  GPIO_P16 = 10,            /*!< GPIO 16/XTAL32K_I: QFN32 pin 18 */
  P16 = GPIO_P16,
  Analog_IO_5 = GPIO_P16,   /*!< P16 can't be muxed as an analog IO (AIO_5, according to the datasheet) */
  XTALI = GPIO_P16,

  GPIO_P17 = 11,            /*!< GPIO 17/XTAL32K_O: QFN32 pin 19 */
  P17 = GPIO_P17,
  Analog_IO_6 = GPIO_P17,   /*!< P16 can't be muxed as an analog IO (AIO_6, according to the datasheet) */
  XTALO = GPIO_P17,

  GPIO_P18 = 12,            /*!< GPIO 18/AIO_7/PGA-: QFN32 pin 20 */
  P18 = GPIO_P18,
  Analog_IO_7 = GPIO_P18,   /*!< P18 can also be muxed as a differential (AIO_7, Input A+) analog IO */

  GPIO_P20 = 13,            /*!< GPIO 20/AIO_9/PGA+: QFN32 pin 21 */
  P20 = GPIO_P20,
  Analog_IO_9 = GPIO_P20,   /*!< P20 can also be muxed as a single ended (AIO_9) or differential (Input D+) analog IO */

  GPIO_P23 = 14,            /*!< GPIO 23/AIO_1/MICBIASREF: QFN32 pin 25 */
  P23 = GPIO_P23,
  Analog_IO_1 = GPIO_P23,   /*!< P23 can also be muxed as a single ended (AIO_1) or differential (Input B+) analog IO */

  GPIO_P24 = 15,            /*!< GPIO 24/AIO_2: QFN32 pin 26 */
  P24 = GPIO_P24,
  Analog_IO_2 = GPIO_P24,   /*!< P23 can also be muxed as a single ended (AIO_2) or differential (Input C-) analog IO */

  GPIO_P25 = 16,            /*!< GPIO 25/AIO_8: QFN32 pin 27 */
  P25 = GPIO_P25,
  Analog_IO_8 = GPIO_P25,   /*!< P25 can also be muxed as a differential (AIO_8, Input A-) analog IO */

  GPIO_P26 = 17,            /*!< GPIO 26: QFN32 pin 28 */
  P26 = GPIO_P26,

  GPIO_P27 = 18,            /*!< GPIO 27: QFN32 pin ? */
  P27 = GPIO_P27,

  GPIO_P31 = 19,            /*!< GPIO 31: QFN32 pin 29 */
  P31 = GPIO_P31,
  GPIO_P32 = 20,            /*!< GPIO 32: QFN32 pin 30 */
  P32 = GPIO_P32,
  GPIO_P33 = 21,            /*!< GPIO 33: QFN32 pin 31 */
  P33 = GPIO_P33,
  GPIO_P34 = 22,            /*!< GPIO 34: QFN32 pin 32 */
  P34 = GPIO_P34,
  GPIO_NUM = 23,
  GPIO_DUMMY = 0xff,
} gpio_pin_e;

/**
  * @brief  PHY62xx Function Mux for GPIOs
  */
typedef enum
{
  /*!< I2C0 */
  FMUX_IIC0_SCL = 0,        /*!< I2C0: SCL signal */
  FMUX_IIC0_SDA = 1,        /*!< I2C0: SDA signal */

  /*!< I2C1 */
  FMUX_IIC1_SCL = 2,        /*!< I2C1: SCL signal */
  FMUX_IIC1_SDA = 3,        /*!< I2C1: SDA signal */

  /*!< UART0 */
  FMUX_UART0_TX = 4,        /*!< UART0: TX signal */
  FMUX_UART0_RX = 5,        /*!< UART0: RX signal */

  FMUX_RF_RX_EN = 6,
  FMUX_RF_TX_EN = 7,

  /*!< UART1 */
  FMUX_UART1_TX = 8,        /*!< UART1: TX signal */
  FMUX_UART1_RX = 9,        /*!< UART1: RX signal */

  /*!< PWM */
  FMUX_PWM0 = 10,           /*!< PWM0 signal */
  FMUX_PWM1 = 11,           /*!< PWM1 signal */
  FMUX_PWM2 = 12,           /*!< PWM2 signal */
  FMUX_PWM3 = 13,           /*!< PWM3 signal */
  FMUX_PWM4 = 14,           /*!< PWM4 signal */
  FMUX_PWM5 = 15,           /*!< PWM5 signal */

  /*!< SPI0 */
  FMUX_SPI_0_SCK = 16,      /*!< SPI0: SCK signal */
  FMUX_SPI_0_SSN = 17,      /*!< SPI0: CS signal */
  FMUX_SPI_0_TX = 18,       /*!< SPI0: MOSI signal */
  FMUX_SPI_0_RX = 19,       /*!< SPI0: MISO signal */

  /*!< SPI1 */
  FMUX_SPI_1_SCK = 20,      /*!< SPI1: SCK signal */
  FMUX_SPI_1_SSN = 21,      /*!< SPI1: CS signal */
  FMUX_SPI_1_TX = 22,       /*!< SPI1: MOSI signal */
  FMUX_SPI_1_RX = 23,       /*!< SPI1: MISO signal */

  FMUX_CHAX = 24,
  FMUX_CHBX = 25,
  FMUX_CHIX = 26,
  FMUX_CHAY = 27,
  FMUX_CHBY = 28,
  FMUX_CHIY = 29,
  FMUX_CHAZ = 30,
  FMUX_CHBZ = 31,
  FMUX_CHIZ = 32,

  /*!< DMIC */
  FMUX_CLK1P28M = 33,       /*!< DMIC: I2S CLK 28MHz out signal */
  FMUX_ADCC = 34,           /*!< DMIC: I2S DATA_IN input signal*/

  FMUX_ANT_SEL_0 = 35,
  FMUX_ANT_SEL_1 = 36,
  FMUX_ANT_SEL_2 = 37,
} gpio_fmux_e;

/**
 * @brief  Frequency selection for the Debug Mux
 */
typedef enum
{
  FRE_HCLK_DIV8 = 0,
  FRE_PCLK_DIV4 = 1,
  FRE_CLK_1P28M = 2,
  FRE_CLK_RC32K = 6,
  FRE_XTAL_CLK32768 = 7,
} Freq_Type_e;

/**
 * @brief  GPIO Bit SET and Bit RESET enumeration
 */
typedef enum
{
  GPIO_PIN_RESET = 0u,      /*!< GPIO has a logical value of 0 (low) */
  GPIO_PIN_SET              /*!< GPIO has a logical value of 1 (high) */
} GPIO_PinState;

/**
 * @brief  GPIO Direction enumeration
 */
typedef enum
{
  GPIO_INPUT = 0,           /*!< GPIO set as input */
  GPIO_OUTPUT = 1           /*!< GPIO set as output */
} gpio_dir_t;

/**
 * @brief  GPIO Interrupt polarity enumeration
 */
typedef enum
{
  POL_FALLING = 0,
  POL_ACT_LOW = 0,
  POL_RISING = 1,
  POL_ACT_HIGH = 1
} gpio_polarity_e;

typedef enum
{
  Bit_DISABLE = 0,          /*!< FMUX disable */
  Bit_ENABLE,               /*!< FMUX enable */
} bit_action_e;

/**
 * @brief  GPIO Pull up/Pull down enumeration
 */
typedef enum
{
  GPIO_FLOATING = 0x00,     /*!< No pull (floating) */
  FLOATING = GPIO_FLOATING,

  GPIO_PULL_UP_S = 0x01,    /*!< Strong (150k) pull up */
  STRONG_PULL_UP = GPIO_PULL_UP_S,

  GPIO_PULL_UP = 0x02,      /*!< Weak (1M) pull up */
  WEAK_PULL_UP = GPIO_PULL_UP,

  GPIO_PULL_DOWN = 0x03,    /*!< Strong (150k) pull down */
  PULL_DOWN = GPIO_PULL_DOWN,
} gpio_pupd_e;

/**
 * @brief  GPIO initialization structure used to easily set-up all GPIOs used in a project
 */
typedef struct
{
  gpio_pin_e pin;           /*!< GPIO pin */
  gpio_pupd_e type;         /*!< GPIO Pull type */
  gpio_dir_t dir;           /*!< GPIO direction */
  GPIO_PinState def_state;  /*!< Value to write on GPIO (if output) */
} ioinit_cfg_t;

/**
 * @brief  GPIO Internal structure for changing pull up/down
 */
typedef struct
{
  uint8_t reg_i;
  uint8_t bit_h;
  uint8_t bit_l;
} PULL_TypeDef;

/*!< Software-defined GPIO interrupt handler */
typedef void (*gpioin_Hdl_t)(gpio_pin_e pin, gpio_polarity_e type);

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */
#define NEGEDGE POL_FALLING
#define POSEDGE POL_RISING
  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_Exported_Functions
  * @{
  */
    /**
      * @brief  Initializes GPIO subsystem.
      * @note   Nulls out the GPIO context structure, disables the interrupt mask
      *         and wakeup interrupt for all GPIOs, registers and enables the GPIO IRQ.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_INVALID_STATE if already initialized.
      */
    int hal_gpio_init(void);

    /**
      * @brief  Configures a given pin as a GPIO, sets its direction.
      * @param  pin: Pin to configure as a GPIO, from the gpio_pin_e enum.
      *         type: GPIO_INPUT for input, GPIO_OUTPUT for output.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_INVALID_STATE if already initialized.
      */
    int hal_gpio_pin_init(gpio_pin_e pin, gpio_dir_t type);

    void hal_gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type);


    void hal_gpio_write(gpio_pin_e pin, uint8_t en);
    void hal_gpio_fast_write(gpio_pin_e pin, uint8_t en);
    GPIO_PinState hal_gpio_read(gpio_pin_e pin);

    void hal_gpio_fmux(gpio_pin_e pin, bit_action_e value);
    void hal_gpio_fmux_set(gpio_pin_e pin, gpio_fmux_e type);


    void hal_gpio_ds_control(gpio_pin_e pin, bit_action_e value);
    int hal_gpio_cfg_analog_io(gpio_pin_e pin, bit_action_e value);
    void hal_gpio_pin2pin3_control(gpio_pin_e pin, uint8_t en);

    void hal_gpio_wakeup_set(gpio_pin_e pin, gpio_polarity_e type);
    void hal_gpio_wakeup_control(gpio_pin_e pin, bit_action_e value);

    int hal_gpioin_enable(gpio_pin_e pin);
    int hal_gpioin_register(gpio_pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl);
    int hal_gpioin_unregister(gpio_pin_e pin);
    int hal_gpioin_disable(gpio_pin_e pin);

    int hal_gpioretention_unregister(gpio_pin_e pin);
    int hal_gpioretention_register(gpio_pin_e pin);

    void hal_gpio_debug_mux_enable(Freq_Type_e fre);
    void hal_gpio_debug_mux_disable(Freq_Type_e fre);

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup GPIO_Exported_ROM_Functions GPIO Exported ROM Functions
  * @{
  */
    // rom api
    ATTR_ROM_FN int gpio_write(gpio_pin_e pin, uint8_t en);
    ATTR_ROM_FN GPIO_PinState gpio_read(gpio_pin_e pin);
    ATTR_ROM_FN void gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type);

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup GPIO_Exported_ROM_Variables GPIO Exported ROM Variables
  * @{
  */
    ATTR_ROM_CONST PULL_TypeDef c_gpio_pull[GPIO_NUM];
    ATTR_ROM_CONST uint8_t c_gpio_index[GPIO_NUM];

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
#endif

#endif /* _HAL_GPIO_H */

