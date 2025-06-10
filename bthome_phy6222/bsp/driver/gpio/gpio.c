/**
  ******************************************************************************
  * @file    gpio.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   GPIO BSP module driver.
  *          This file provides firmware functions to manage the GPIOs and their
  *          routing (mux) on the MCU:
  *           + Route a peripheral's signal to a given GPIO
  *           + Set direction/Pull of a GPIO
  *           + Read/Write a GPIO logic value
  *           + Configure a GPIO input as an interrupt/wake-up source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

#include <driver/pwrmgr/pwrmgr.h> /* for hal_pwrmgr_register */

#include <jump_function.h> /* for JUMP_FUNCTION */

#include <string.h> /* for memset() */

#include <types.h> /* for subWriteReg and BIT */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup GPIO
  * @brief GPIO BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @addtogroup GPIO_Private_Typedef GPIO Private Typedefs
  * @{
  */

/**
 * @brief  PHY62xx pin assignment enum
 */
typedef enum
{
    GPIO_PIN_ASSI_NONE = 0,
    GPIO_PIN_ASSI_OUT,
    GPIO_PIN_ASSI_IN,
} GPIO_PIN_ASSI_t;

typedef enum
{
    GPIO_IN_DISABLED = 0U,
    GPIO_IN_ENABLED = 1U,
} GPIO_IN_ENA_t;

/**
 * @brief  PHY62xx GPIO Input context structure
 */
typedef struct
{
    uint8_t enable;
    uint8_t pin_state;
    gpioin_Hdl_t posedgeHdl;
    gpioin_Hdl_t negedgeHdl;
} gpioin_Ctx_t;

typedef enum
{
    GPIO_CTX_NOT_INITALIZED = 0U,       /*!< Context is not initialized yet */
    GPIO_CTX_INITIALIZED = 1U           /*!< Context is initialized */
} GPIO_CTX_INIT_t;

/**
 * @brief  PHY62xx GPIO context structure
 */
typedef struct
{
    GPIO_CTX_INIT_t state;
    uint8_t pin_assignments[GPIO_NUM];
    uint32_t pin_retention_status;
    gpioin_Ctx_t irq_ctx[GPIO_NUM];
} gpio_Ctx_t;

/**
  * @}
  */


/* Private define ------------------------------------------------------------*/
/** @addtogroup GPIO_Private_Constants GPIO Private Constants
  * @{
  */
#ifndef USE_ROM_GPIO
#define USE_ROM_GPIO 1
#endif

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants FLASH Private macros
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Variables FLASH Private variables
  * @{
  */
static gpio_Ctx_t m_gpioCtx = {
    .state = GPIO_CTX_NOT_INITALIZED,
    .pin_assignments = {
        0,
    },
    .pin_retention_status = 0
};

static const uint8_t retention_reg[GPIO_NUM][2] = {
    /* AP_AON->IOCTL[] offset, Bit number */
    { 0, 13 }, // p0
    { 0, 14 }, // p1
    { 0, 16 }, // p2
    { 0, 17 }, // p3
    { 0, 19 }, // p7
    { 0, 20 }, // p9
    { 1, 7 },  // p10
    { 1, 8 },  // p11
    { 1, 10 }, // p14
    { 1, 11 }, // p15
    { 1, 28 }, // p16
    { 1, 29 }, // p17
    { 2, 4 },  // p18
    { 2, 5 },  // p20
    { 2, 7 },  // p23
    { 2, 8 },  // p24
    { 2, 25 }, // p25
    { 2, 26 }, // p26
    { 2, 28 }, // p27
    { 2, 29 }, // p31
    { 3, 1 },  // p32
    { 3, 2 },  // p33
    { 3, 23 }, // p34
};

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup GPIO_Private_Functions GPIO Private functions
  * @{
  */
static int hal_gpio_interrupt_disable(gpio_pin_e pin)
{
    /* mask the pin */
    AP_GPIO->intmask |= BIT(pin);
    /* disable the int */
    AP_GPIO->inten &= ~BIT(pin);
    return PPlus_SUCCESS;
}

static void hal_gpio_retention_enable(gpio_pin_e pin, uint8_t en)
{
    if (en)
    {
        if ((pin == P32) || (pin == P33) || (pin == P34))
        {
            AP_AON->PMCTL0 |= BIT(retention_reg[pin][1]);
        }
        else
        {
            AP_AON->IOCTL[retention_reg[pin][0]] |= BIT(retention_reg[pin][1]);
        }
    }
    else
    {
        if ((pin == P32) || (pin == P33) || (pin == P34))
        {
            AP_AON->PMCTL0 &= ~BIT(retention_reg[pin][1]);
        }
        else
        {
            AP_AON->IOCTL[retention_reg[pin][0]] &= ~BIT(retention_reg[pin][1]);
        }
    }
}

static void hal_gpio_sleep_handler(void)
{
    int i;
    gpio_polarity_e pol;

    for (i = 0; i < GPIO_NUM; i++)
    {
        // config wakeup
        if ((m_gpioCtx.pin_assignments[i] == GPIO_PIN_ASSI_OUT) && (m_gpioCtx.pin_retention_status & BIT(i)))
        {
            hal_gpio_retention_enable((gpio_pin_e)i, Bit_ENABLE);
        }

        if (m_gpioCtx.pin_assignments[i] == GPIO_PIN_ASSI_IN)
        {
#ifdef XOSC_PIN_ALLOW
            if ((i == P16) || (i == P17))
            {
                hal_gpio_cfg_analog_io((gpio_pin_e)i, Bit_DISABLE);
                subWriteReg(&(AP_AON->PMCTL2_0), 6, 6, 0x01);
                WaitUs(50);
                pol = hal_gpio_read((gpio_pin_e)i) ? POL_FALLING : POL_RISING;
                subWriteReg(&(AP_AON->PMCTL2_0), 6, 6, 0x00);
            }
            else
#endif
            {
                pol = hal_gpio_read((gpio_pin_e)i) ? POL_FALLING : POL_RISING;
            }

            hal_gpio_wakeup_set((gpio_pin_e)i, pol);
            m_gpioCtx.irq_ctx[i].pin_state = hal_gpio_read((gpio_pin_e)i);
        }
    }
}

static int hal_gpio_interrupt_enable(gpio_pin_e pin, gpio_polarity_e type)
{
    uint32_t gpio_tmp;

    gpio_tmp = AP_GPIO->inttype_level;
    gpio_tmp |= BIT(pin); // edge sensitive
    AP_GPIO->inttype_level = gpio_tmp;

    gpio_tmp = AP_GPIO->intmask;
    gpio_tmp &= ~BIT(pin); // unmask interrupt
    AP_GPIO->intmask = gpio_tmp;

    gpio_tmp = AP_GPIO->int_polarity;
    if (type == POL_RISING)
        gpio_tmp |= BIT(pin);
    else
        gpio_tmp &= ~BIT(pin);
    AP_GPIO->int_polarity = gpio_tmp;

    gpio_tmp = AP_GPIO->inten;
    gpio_tmp |= BIT(pin); // enable interrupt
    AP_GPIO->inten = gpio_tmp;

    return PPlus_SUCCESS;
}

static void hal_gpioin_event_pin(gpio_pin_e pin, gpio_polarity_e type)
{
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

    if (p_irq_ctx[pin].posedgeHdl && (type == POL_RISING))
    {
        p_irq_ctx[pin].posedgeHdl(pin, POL_RISING); // LOG("POS\n");
    }
    else if (p_irq_ctx[pin].negedgeHdl && (type == POL_FALLING))
    {
        p_irq_ctx[pin].negedgeHdl(pin, POL_FALLING); // LOG("NEG\n");
    }
}

static void hal_gpioin_wakeup_trigger(gpio_pin_e pin)
{
    uint8_t pin_state = (uint8_t)hal_gpio_read(pin);
    gpio_polarity_e type = pin_state ? POL_RISING : POL_FALLING;

    if (m_gpioCtx.irq_ctx[pin].pin_state != pin_state)
        hal_gpioin_event_pin(pin, type);
}

static void hal_gpioin_event(uint32_t int_status, uint32_t polarity)
{
    int i;
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
    //    LOG("GI:%x,%x\n",int_status,polarity);

    for (i = 0; i < GPIO_NUM; i++)
    {
        if (int_status & (1ul << i))
        {
            gpio_polarity_e type = (polarity & BIT(i)) ? POL_RISING : POL_FALLING;
            hal_gpioin_event_pin((gpio_pin_e)i, type);

            // reconfig interrupt
            if (p_irq_ctx[i].posedgeHdl && p_irq_ctx[i].negedgeHdl) // both raise and fall
            {
                type = (type == POL_RISING) ? POL_FALLING : POL_RISING;
                hal_gpio_interrupt_enable((gpio_pin_e)i, type);
            }
            else if (p_irq_ctx[i].posedgeHdl) // raise
            {
                hal_gpio_interrupt_enable((gpio_pin_e)i, POL_RISING);
            }
            else if (p_irq_ctx[i].negedgeHdl) // fall
            {
                hal_gpio_interrupt_enable((gpio_pin_e)i, POL_FALLING);
            }
        }
    }
}

static void hal_gpio_wakeup_handler(void)
{
    int i;

    NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_APP);
    NVIC_EnableIRQ(GPIO_IRQn);

#ifdef XOSC_PIN_ALLOW
    if (pGlobal_config[LL_SWITCH] & LL_RC32K_SEL)
    {
        subWriteReg(&(AP_AON->PMCTL2_0), 16, 7, 0x3fb); // software control 32k_clk
        subWriteReg(&(AP_AON->PMCTL2_0), 6, 6, 0x01);   // enable software control
    }
    else
    {
        subWriteReg(&(AP_AON->PMCTL2_0), 9, 8, 0x03); // software control 32k_clk
        subWriteReg(&(AP_AON->PMCTL2_0), 6, 6, 0x00); // disable software control
    }
#endif

    for (i = 0; i < GPIO_NUM; i++)
    {
        if (m_gpioCtx.pin_assignments[i] != GPIO_PIN_ASSI_NONE)
        {
            if ((i == P2) || (i == P3))
                hal_gpio_pin2pin3_control((gpio_pin_e)i, 1);

#ifdef XOSC_PIN_ALLOW

            if ((i == P16) || (i == P17))
                hal_gpio_cfg_analog_io((gpio_pin_e)i, Bit_DISABLE);

#endif
        }

        if ((m_gpioCtx.pin_assignments[i] == GPIO_PIN_ASSI_OUT) && (m_gpioCtx.pin_retention_status & BIT(i)))
        {
            GPIO_PinState pol = hal_gpio_read((gpio_pin_e)i);
            hal_gpio_write((gpio_pin_e)i, pol);
            hal_gpio_retention_enable((gpio_pin_e)i, Bit_DISABLE);
        }

        if (m_gpioCtx.irq_ctx[i].enable)
        {
            hal_gpioin_enable((gpio_pin_e)i);         // resume gpio irq
            hal_gpioin_wakeup_trigger((gpio_pin_e)i); // trigger gpio irq manually
        }
    }
}

static void hal_GPIO_IRQHandler(void)
{
    uint32_t polarity = AP_GPIO->int_polarity;
    uint32_t st = AP_GPIO->int_status;

    /* clear interrupt */
    AP_GPIO->porta_eoi = st;

    /* process the interrupt */
    hal_gpioin_event(st, polarity);
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup GPIO_Exported_Functions GPIO related exported functions
  * @{
  */
int hal_gpio_init(void)
{
    if (m_gpioCtx.state == GPIO_CTX_INITIALIZED)
        return PPlus_ERR_INVALID_STATE;

    memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));
    m_gpioCtx.state = GPIO_CTX_INITIALIZED;

    /* disable all channel irq, unmask all channel */
    AP_GPIO->inten = 0;
    AP_GPIO->intmask = 0;

    /* disable all wakeup pin */
    AP_WAKEUP->io_wu_mask_31_0 = 0;
    AP_WAKEUP->io_wu_mask_34_32 = 0;

    JUMP_FUNCTION(GPIO_IRQ_HANDLER) = (uint32_t)&hal_GPIO_IRQHandler;

    NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_APP);
    NVIC_EnableIRQ(GPIO_IRQn);

    hal_pwrmgr_register(MOD_GPIO, hal_gpio_sleep_handler, hal_gpio_wakeup_handler);

    return PPlus_SUCCESS;
}

int hal_gpio_pin_init(gpio_pin_e pin, gpio_dir_t type)
{
    //    if((m_gpioCtx.pin_assignments[pin] == GPIO_PIN_ASSI_OUT) &&
    //            (m_gpioCtx.pin_retention_status & BIT(pin)) && (type == GPIO_INPUT))
    //        return PPlus_ERR_INVALID_PARAM;

    hal_gpio_fmux(pin, Bit_DISABLE);

    if ((pin == P2) || (pin == P3))
        hal_gpio_pin2pin3_control(pin, 1);

    hal_gpio_cfg_analog_io(pin, Bit_DISABLE);

    if (type == GPIO_OUTPUT)
    {
        AP_GPIO->swporta_ddr |= BIT(pin);
        m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_OUT;
    }
    else
    {
        AP_GPIO->swporta_ddr &= ~BIT(pin);
        m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN;
        m_gpioCtx.pin_retention_status &= ~BIT(pin);
    }

    return PPlus_SUCCESS;
}

#if (USE_ROM_GPIO == 0)
void hal_gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type)
{

    uint8_t i = c_gpio_pull[pin].reg_i;
    uint8_t h = c_gpio_pull[pin].bit_h;
    uint8_t l = c_gpio_pull[pin].bit_l;

    if (pin < P31)
        subWriteReg(&(AP_AON->IOCTL[i]), h, l, type);
    else
        subWriteReg(&(AP_AON->PMCTL0), h, l, type);
}
#endif

void hal_gpio_write(gpio_pin_e pin, uint8_t en)
{

    if (en)
        AP_GPIO->swporta_dr |= BIT(pin);
    else
        AP_GPIO->swporta_dr &= ~BIT(pin);

    hal_gpio_pin_init(pin, GPIO_OUTPUT);
}

void hal_gpio_fast_write(gpio_pin_e pin, uint8_t en)
{

    if (en)
        AP_GPIO->swporta_dr |= BIT(pin);
    else
        AP_GPIO->swporta_dr &= ~BIT(pin);
}

#if (USE_ROM_GPIO == 0)
GPIO_PinState hal_gpio_read(gpio_pin_e pin)
{
    uint32_t r;

    if (AP_GPIO->swporta_ddr & BIT(pin))
        r = AP_GPIO->swporta_dr;
    else
        r = AP_GPIO->ext_porta;

    return (int)((r >> pin) & 1);
}

void hal_gpio_fmux(gpio_pin_e pin, bit_action_e value)
{
    if (value == Bit_ENABLE)
    {
        //        if((pin == P2) || (pin == P3))
        //            hal_gpio_pin2pin3_control(pin,1);
        AP_IOMUX->full_mux0_en |= BIT(pin);
    }
    else
    {
        AP_IOMUX->full_mux0_en &= ~BIT(pin);
    }
}
#endif

void hal_gpio_fmux_set(gpio_pin_e pin, gpio_fmux_e type)
{
    uint8_t h = 0, l = 0;
    uint32_t reg_index;
    uint32_t bit_index;

    if (pin != GPIO_DUMMY)
    {
        reg_index = pin >> 2;
        bit_index = pin & 0x03;
        l = 8 * bit_index;
        h = l + 5;
        hal_gpioin_disable(pin);
        subWriteReg(&(AP_IOMUX->gpio_sel[reg_index]), h, l, type);
        hal_gpio_fmux(pin, Bit_ENABLE);
    }
}

#if (USE_ROM_GPIO == 0)
void hal_gpio_wakeup_control(gpio_pin_e pin, bit_action_e value)
{
    if (pin < P32)
    {
        if (value)
            AP_AON->REG_S9 |= BIT(c_gpio_index[pin]);
        else
            AP_AON->REG_S9 &= ~BIT(c_gpio_index[pin]);
    }
    else
    {
        if (value)
            AP_AON->REG_S10 |= BIT(c_gpio_index[pin] - 32);
        else
            AP_AON->REG_S10 &= ~BIT(c_gpio_index[pin] - 32);
    }
}

void hal_gpio_ds_control(gpio_pin_e pin, bit_action_e value)
{
    if (value)
        AP_IOMUX->pad_ps0 |= BIT(pin);
    else
        AP_IOMUX->pad_ps0 &= ~BIT(pin);
}
#endif

int hal_gpioretention_unregister(gpio_pin_e pin)
{
    if (m_gpioCtx.pin_assignments[pin] != GPIO_PIN_ASSI_OUT)
        return PPlus_ERR_INVALID_PARAM;

    m_gpioCtx.pin_retention_status &= ~BIT(pin);
    return PPlus_SUCCESS;
}

int hal_gpioin_unregister(gpio_pin_e pin)
{
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

    hal_gpioin_disable(pin);
    p_irq_ctx[pin].negedgeHdl = NULL;
    p_irq_ctx[pin].posedgeHdl = NULL;
    return PPlus_SUCCESS;
}

#if (USE_ROM_GPIO == 0)
int hal_gpio_cfg_analog_io(gpio_pin_e pin, bit_action_e value)
{
    if ((pin < P11) || (pin > P25))
        return PPlus_ERR_INVALID_PARAM;

    if (value)
    {
        hal_gpio_pull_set(pin, GPIO_FLOATING);
        AP_IOMUX->Analog_IO_en |= BIT(pin - P11);
    }
    else
    {
        AP_IOMUX->Analog_IO_en &= ~BIT(pin - P11);
    }

    return PPlus_SUCCESS;
}
#endif

void hal_gpio_wakeup_set(gpio_pin_e pin, gpio_polarity_e type)
{
    uint8_t i = c_gpio_pull[pin].reg_i;
    uint8_t p = c_gpio_pull[pin].bit_l - 1;

    if (m_gpioCtx.pin_assignments[pin] != GPIO_PIN_ASSI_IN)
        return;

    AP_GPIO->inttype_level |= BIT(pin); // edge sensitive

    if (pin < P31)
    {
        if (POL_FALLING == type)
            AP_AON->IOCTL[i] |= BIT(p);
        else
            AP_AON->IOCTL[i] &= ~BIT(p);
    }
    else
    {
        if (POL_FALLING == type)
            AP_AON->PMCTL0 |= BIT(p);
        else
            AP_AON->PMCTL0 &= ~BIT(p);
    }

    hal_gpio_wakeup_control(pin, Bit_ENABLE); // enable wakeup function
}

void hal_gpio_pin2pin3_control(gpio_pin_e pin, uint8_t en) // 0:sw,1:other func
{
    if (en)
        AP_IOMUX->gpio_pad_en |= BIT(pin - 2);
    else
        AP_IOMUX->gpio_pad_en &= ~BIT(pin - 2);
}

int hal_gpioin_disable(gpio_pin_e pin)
{
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

    p_irq_ctx[pin].enable = GPIO_IN_DISABLED;
    m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_NONE;

    hal_gpio_pin_init(pin, GPIO_INPUT);

    return hal_gpio_interrupt_disable(pin);
}

int hal_gpioin_enable(gpio_pin_e pin)
{
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
    gpio_polarity_e type = POL_FALLING;
    GPIO_PinState pinVal = GPIO_PIN_RESET;

    if (p_irq_ctx[pin].posedgeHdl == NULL && p_irq_ctx[pin].negedgeHdl == NULL)
        return PPlus_ERR_NOT_REGISTED;

    m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN;
    p_irq_ctx[pin].enable = GPIO_IN_ENABLED;

    hal_gpio_pin_init(pin, GPIO_INPUT);

    /* this code is really weird */
    if (p_irq_ctx[pin].posedgeHdl && p_irq_ctx[pin].negedgeHdl) // both raise and fall
    {
        pinVal = hal_gpio_read(pin);
        type = pinVal == GPIO_PIN_SET ? POL_FALLING : POL_RISING;
    }
    else if (p_irq_ctx[pin].posedgeHdl) // raise
    {
        type = POL_RISING;
    }
    else if (p_irq_ctx[pin].negedgeHdl) // fall
    {
        type = POL_FALLING;
    }

    hal_gpio_interrupt_enable(pin, type);

    return PPlus_SUCCESS;
}

int hal_gpioretention_register(gpio_pin_e pin)
{
    hal_gpio_pin_init(pin, GPIO_OUTPUT);
    m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_OUT;

    m_gpioCtx.pin_retention_status |= BIT(pin);
    return PPlus_SUCCESS;
}

int hal_gpioin_register(gpio_pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl)
{
    int ret;
    gpioin_Ctx_t *p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

    hal_gpioin_disable(pin);
    p_irq_ctx[pin].posedgeHdl = posedgeHdl;
    p_irq_ctx[pin].negedgeHdl = negedgeHdl;
    ret = hal_gpioin_enable(pin);

    if (ret != PPlus_SUCCESS)
        hal_gpioin_disable(pin);

    return ret;
}

void hal_gpio_debug_mux_enable(Freq_Type_e fre)
{
    AP_IOMUX->debug_mux_en |= BIT(fre);
}

void hal_gpio_debug_mux_disable(Freq_Type_e fre)
{
    AP_IOMUX->debug_mux_en &= ~BIT(fre);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
