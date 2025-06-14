/**
  ******************************************************************************
  * @file    adc.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   ADC BSP module driver.
  *          This file provides firmware functions to manage the ADCs.
  *           + 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include <driver/clock/clock.h> /* for g_system_clk */

#include <driver/pwrmgr/pwrmgr.h> /* for hal_pwrmgr_register */

#include <driver/gpio/gpio.h> /* for gpio_pin_e */

#include <log/log.h> /* for LOG */

#include <jump_function.h> /* for JUMP_FUNCTION */

#include <types.h> /* for subWriteReg and BIT */

#include <stddef.h> /* for NULL */

#define SPIF_RSVD_AREA_1                 (0x1000)
#define pSPIF_RSVD1_ADC_CALIBRATE       ((volatile uint32_t*)(FLASH_BASE_ADDR + SPIF_RSVD_AREA_1))
#define SPIF_RSVD1_ADC_CALIBRATE        (FLASH_BASE_ADDR + SPIF_RSVD_AREA_1)


typedef enum
{
    ADC_CTX_NOT_INITALIZED = 0U,        /*!< Context is not initialized yet */
    ADC_CTX_INITIALIZED = 1U            /*!< Context is initialized */
} ADC_CTX_INIT_t;

typedef struct _adc_Contex_t
{
    ADC_CTX_INIT_t state;

    uint16_t irq_bits;

    uint16_t enabled_channels;
    uint16_t highres_channels;

    uint8_t continue_mode;

    // sysclk_t    clk_src;
    uint16_t adc_cal_postive;
    uint16_t adc_cal_negtive;

    int adc_cal_sum;
    int adc_cal_diff;


    adc_Hdl_t evt_handler;
} adc_Ctx_t;

static adc_Ctx_t mAdc_Ctx = {
    .state = ADC_CTX_NOT_INITALIZED,
    .irq_bits = 0x00,
    //.chs_en_shadow = 0x00,
    .continue_mode = 0,
    .adc_cal_postive = 0xFFF,
    .adc_cal_negtive = 0xFFF,
    .evt_handler = NULL
};

static const struct {
    adc_CH_t    hal_adc;
    adc_channels_t ch;
    gpio_pin_e  single_ended_pin;
    uint8_t     pmctl2_1_bit;               /*!< Number if ADC single-ended ADC bit 7 = AIO_9, 6 = AIO_8, 5 = AIO_7, 4 = AIO_4, 3 = AIO_3, 2 = AIO_2, 1 = AIO_1, 0 = 0) */
    uint8_t     aio_differential_pair;
    uint8_t     buffer_index;
    uint16_t    irq_mask_bit;
    __IO uint32_t *adc_ctl;
    uint32_t    adc_ctl_val;
} adcs[] = {
    /* aio<0> = gpio<11> */
    [0] = { ADC_CH1N_P11, CH0, P11, 0, 1, 3, BIT(3), &PCRM_ADCCTL1, PCRM_ADCCTL_NCHEN },
    /* aio<1> = gpio<23>, input b pos, microphone bias ref voltage */
    [1] = { ADC_CH1P_P23, CH1, P23, 1, 0, 2, BIT(2), &PCRM_ADCCTL1, PCRM_ADCCTL_PCHEN },

    /* aio<2> = gpio<24>, input c neg */
    [2] = { ADC_CH2N_P24, CH2, P24, 2, 3, 5, BIT(5), &PCRM_ADCCTL2, PCRM_ADCCTL_NCHEN },
    /* aio<3> = gpio<14>, input c pos */
    [3] = { ADC_CH2P_P14, CH3, P14, 3, 2, 4, BIT(4), &PCRM_ADCCTL2, PCRM_ADCCTL_PCHEN },

    /* aio<4> = gpio<15>, input d neg */
    [4] = { ADC_CH3N_P15, CH4, P15, 4, 7, 7, BIT(7), &PCRM_ADCCTL3, PCRM_ADCCTL_NCHEN },
    /* aio<9> = gpio<20>, input d pos, PGA neg */
    [9] = { ADC_CH3P_P20, CH9, P20, 7, 4, 6, BIT(6), &PCRM_ADCCTL3, PCRM_ADCCTL_PCHEN },

    /* aio<5> = gpio<16>, 32k xtal input */
    [5] = { 0, CH5, P16, 0, 0, 0, 0, NULL, 0 },
    /* aio<6> = gpio<17>, 32k xtal output */
    [6] = { 0, CH6, P17, 0, 0, 0, 0, NULL, 0 },

    /* aio<7> = gpio<18>, input a pos, PGA pos */
    [7] = { ADC_CH0DIFF, CH7, P18, 0, 0, 0, 0, &PCRM_ADCCTL0, PCRM_ADCCTL_NCHEN },
    /* aio<8> = gpio<25>, input a neg */
    [8] = { ADC_CH0DIFF, CH8, P25, 0, 0, 0, 0, &PCRM_ADCCTL0, PCRM_ADCCTL_PCHEN },
};


static const struct {
    gpio_pin_e      pin_pos;
    gpio_pin_e      pin_neg;
    uint8_t         differential_anactl;
    uint8_t         mask;
} adcs_diff[] = {
    { P18, P25, 1, 0x01 },
    { P23, P11, 2, 0x04 },
    { P14, P24, 3, 0x10 },
    { P20, P15, 4, 0x40 }
};



#if (SDK_VER_CHIP == __DEF_CHIP_QFN32__)
const unsigned int adc_Lambda[MAX_ADC_CH - MIN_ADC_CH + 1] =
    {
        4519602, // P11
        4308639, // P23
        4263287, // P24
        4482718, // P14
        4180401, // P15
        4072069, // P20
};

#elif (SDK_VER_CHIP == __DEF_CHIP_TSOP16__)
const unsigned int adc_Lambda[MAX_ADC_CH - MIN_ADC_CH + 1] =
    {
        4488156, // P11
        4308639, // P23,
        4263287, // P24,
        4467981, // P14
        4142931, // P15
        4054721, // P20
};
#endif

/**************************************************************************************
    @fn          hal_adc_value

    @brief       This function process for get adc value

    input parameters

    @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE

    output parameters

    @param       None.

    @return      ADC value
 **************************************************************************************/
static void hal_adc_load_calibration_value(void)
{
    uint32_t adc_cal = read_reg(SPIF_RSVD1_ADC_CALIBRATE);
    mAdc_Ctx.adc_cal_negtive = (uint16_t)(adc_cal & 0x0fff);
    mAdc_Ctx.adc_cal_postive = (uint16_t)((adc_cal >> 16) & 0x0fff);
    LOG("AD_CAL[%x %x]", mAdc_Ctx.adc_cal_negtive, mAdc_Ctx.adc_cal_postive);

    if ((mAdc_Ctx.adc_cal_negtive < 0x733) || (mAdc_Ctx.adc_cal_negtive > 0x8cc) ||
        (mAdc_Ctx.adc_cal_postive < 0x733) || (mAdc_Ctx.adc_cal_postive > 0x8cc))
    {
        mAdc_Ctx.adc_cal_negtive = 0xfff;
        mAdc_Ctx.adc_cal_postive = 0xfff;
        LOG("->AD_CAL[%x %x]", mAdc_Ctx.adc_cal_negtive, mAdc_Ctx.adc_cal_postive);
    }
    else
    {
        mAdc_Ctx.adc_cal_sum = mAdc_Ctx.adc_cal_postive + mAdc_Ctx.adc_cal_negtive;
        mAdc_Ctx.adc_cal_diff = mAdc_Ctx.adc_cal_postive - mAdc_Ctx.adc_cal_negtive;
    }
}


float hal_adc_value_cal(adc_CH_t ch, uint16_t *buf, uint32_t size, uint8_t high_resol, uint8_t diff_mode)
{
    uint32_t i;
    int adc_sum = 0;
    volatile float result = 0.0;
    uint16_t adc_cal_postive = mAdc_Ctx.adc_cal_postive;
    uint16_t adc_cal_negtive = mAdc_Ctx.adc_cal_negtive;

    for (i = 0; i < size; i++)
    {
        adc_sum += (buf[i] & 0xfff);
    }
    result = ((float)adc_sum) / size;
    if ((adc_cal_postive != 0xfff) && (adc_cal_negtive != 0xfff))
    {
        float delta = (mAdc_Ctx.adc_cal_diff) / 2.0;

        if (ch & 0x01)
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / mAdc_Ctx.adc_cal_sum)
                                 : ((result - delta) / mAdc_Ctx.adc_cal_sum);
        }
        else
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / mAdc_Ctx.adc_cal_sum)
                                 : ((result + delta) / mAdc_Ctx.adc_cal_sum);
        }
    }
    else
    {
        result = (diff_mode) ? (float)(result / 2048 - 1) : (float)(result / 4096);
    }

    if (high_resol == 1)
    {
        result *= 800.0;
    }
    else
    {
        result = (float)result * (float)adc_Lambda[ch - 2] * 0.8 / 1000;
    }
    return result;
}

static void set_channel_res(uint8_t aio, uint8_t is_high)
{
    uint8_t h = 0;
    uint8_t l = 0;

    if (is_high)
    {
        l = 1;
    }
    else
    {
        h = 1;
    }

    /* PMCTL2_1 [15:8] => Attenuatin bits for AIO_9, 8, 7, 4, 3, 2, 1, 0 (set = attenuate 1/4, unset = no atten)*/
    subWriteReg(&(AON_PMCTL2_1), (aio + 8), (aio + 8), h);
    /* PMCTL2_1 [7:0] => Pass control bits for AIO_9, 8, 7, 4, 3, 2, 1, 0 (set = connect, unset = disconnect) */
    subWriteReg(&(AON_PMCTL2_1), aio, aio, l);
}

static void clear_adcc_cfg(void)
{
    mAdc_Ctx.irq_bits = 0x00;
    //mAdc_Ctx.chs_en_shadow = 0x00;
    mAdc_Ctx.continue_mode = 0;
    mAdc_Ctx.evt_handler = NULL;
}

/////////////// adc ////////////////////////////
/**************************************************************************************
    @fn          hal_ADC_IRQHandler

    @brief       This function process for adc interrupt

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_ADC_IRQHandler(void)
{
    int ch, ch2, status = 0, n;
    static uint16_t adc_data[MAX_ADC_SAMPLE_SIZE - 2];

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        /* clear all IRQs */
        AP_ADCC->intr_clear = 0x3FF;
        AP_ADCC->intr_mask = 0;
        return;
    }

    /* Read the IRQ status */
    status = AP_ADCC->intr_status & 0x3ff;
    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((mAdc_Ctx.enabled_channels & BIT(i)) == 0)
        {
            if (adcs[i].irq_mask_bit & status)
            {
                LOG("FUCK on %d, status = 0x%08x", i, status);
                /* An IRQ happened for a channel that wasn't enabled! */
                AP_ADCC->intr_mask &= ~BIT(i);
                AP_ADCC->intr_clear = BIT(i);
            }
            continue;
        }

        if (mAdc_Ctx.continue_mode == 0)
        {
            AP_ADCC->intr_mask &= ~adcs[i].irq_mask_bit;   // MASK coresponding channel
            mAdc_Ctx.enabled_channels &= ~adcs[i].irq_mask_bit; // disable channel
        }

        uint16_t buffer_index = adcs[i].buffer_index;

        for (n = 0; n < (MAX_ADC_SAMPLE_SIZE - 3); n++)
        {
            //adc_data[n] = (uint16_t)(read_reg(ADC_CH_BASE + (single_ended_aio * 0x80) + ((n + 2) * 4)) & 0xfff);
            //adc_data[n + 1] = (uint16_t)((read_reg(ADC_CH_BASE + (single_ended_aio * 0x80) + ((n + 2) * 4)) >> 16) & 0xfff);
            adc_data[n] = (uint16_t)(AP_ADCC->adc_data[buffer_index][n + 2] & 0xfff);
            adc_data[n + 1] = (uint16_t)((AP_ADCC->adc_data[buffer_index][n + 2] >> 16) & 0xfff);
        }

        /* Clear the Interrupt of this channel */
        AP_ADCC->intr_clear = adcs[i].irq_mask_bit;

        if (mAdc_Ctx.evt_handler)
        {
            adc_Evt_t evt;
            evt.type = HAL_ADC_EVT_DATA;
            LOG("i = %d, buffer_index = %d", i, buffer_index);
            evt.is_high = (mAdc_Ctx.highres_channels & BIT(i)) ? 1 : 0;
            evt.is_diff = 0;
            evt.ch = adcs[i].hal_adc;
            evt.data = adc_data;
            evt.size = MAX_ADC_SAMPLE_SIZE - 3;
            mAdc_Ctx.evt_handler(&evt);
        }
    }

    // LOG("> %x",mAdc_Ctx.irq_bits);
    if ((mAdc_Ctx.enabled_channels == 0) && (mAdc_Ctx.continue_mode == 0)) //
    {
        hal_adc_stop();
    }
}

static void adc_wakeup_hdl(void)
{
    NVIC_SetPriority((IRQn_Type)ADCC_IRQn, IRQ_PRIO_HAL);
}

/**************************************************************************************
    @fn          hal_adc_init

    @brief       This function process for adc initial

    input parameters

    @param       ADC_MODE_e mode: adc sample mode select;1:SAM_MANNUAL(mannual mode),0:SAM_AUTO(auto mode)
                ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
                ADC_SEMODE_e semode: signle-ended mode negative side enable; 1:SINGLE_END(single-ended mode) 0:DIFF(Differentail mode)
                IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/


int hal_adc_init(void)
{
    if (mAdc_Ctx.state == ADC_CTX_INITIALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    /* Setup the ADC IRQ handler NOW */
    NVIC_DisableIRQ((IRQn_Type)ADCC_IRQn);
    JUMP_FUNCTION(ADCC_IRQ_HANDLER) = (uint32_t)&hal_ADC_IRQHandler;

    hal_pwrmgr_register(MOD_ADCC, NULL, adc_wakeup_hdl);

    clear_adcc_cfg();

    hal_adc_load_calibration_value();

    /* Analog LDO off */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;

    /* ADC Disabled */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Set all ADC channels to be switched off (attn = 0, pass = 0 for each bit) */
    AON_PMCTL2_1 = 0x00;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    /* CLK_1P28M_ENABLE */
    PCRM_CLKSEL |= PCRM_CLKSEL_1P28M;

    // ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    PCRM_CLKHF_CTL0 |= PCRM_CLKHF_CTL0_XTALOUT;

    // ENABLE_DLL;                  //enable DLL
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_DLL;

    // ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADCDBL_Msk; // check

    // ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_ADC;

    // set adc mode,1:mannual,0:auto mode
    PCRM_ADCCTL4 |= PCRM_ADCCTL4_MODE_MANUAL;
    PCRM_ADCCTL4 |= BIT(0); /* TBD */

    /* Channel enable bit set to 0 on all channels */
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_PCHEN;

    /* Disable Mic Bias */
    PCRM_ANACTL &= ~PCRM_ANACTL_MICBIAS;

    mAdc_Ctx.state = ADC_CTX_INITIALIZED;

    return PPlus_SUCCESS;
}

int hal_adc_clock_config(adc_CLOCK_SEL_t clk)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    subWriteReg(&(PCRM_ADCCTL4), 2, PCRM_ADCCTL4_SEL_Pos, clk);

    return PPlus_SUCCESS;
}
#if 0

static void set_sampling_resolution(adc_CH_t channel, uint8_t is_high_resolution, uint8_t is_differential_mode)
{
    for (size_t i = 0; i < (sizeof(adcs)/sizeof(adcs[0])); i++)
    {
        if (adcs[i].hal_adc == channel)
        {
            set_channel_res(adcs[i].pmctl2_1_bit, is_high_resolution);
            set_channel_res(adcs[i].aio_differential_pair, is_high_resolution);
            return;
        }
    }
}

static void set_sampling_resolution_auto(uint8_t channel, uint8_t is_high_resolution, uint8_t is_differential_mode)
{
    uint8_t i_channel;
    adc_CH_t a_channel;
    AON_PMCTL2_1 = 0x00;

    for (i_channel = MIN_ADC_CH; i_channel <= MAX_ADC_CH; i_channel++)
    {
        if (channel & BIT(i_channel))
        {
            a_channel = (adc_CH_t)i_channel;
            set_sampling_resolution(a_channel,
                                    (is_high_resolution & BIT(i_channel)),
                                    (is_differential_mode & BIT(i_channel)));
        }
    }
}

int hal_adc_start1(void)
{
    uint8_t irq_bits2 = /*(((mAdc_Ctx.chs_en_shadow & 0x80) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x40) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x20) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x10) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x08) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x04) << 1));*/ 0;

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    // LOG("irq_bits2:0x%x",irq_bits2);
    hal_pwrmgr_lock(MOD_ADCC);

    for (int i = MIN_ADC_CH; i <= MAX_ADC_CH; i++)
    {
        if (irq_bits2 & (BIT(i)))
        {
            switch (i)
            {
            case ADC_CH1N_P11:
                PCRM_ADCCTL1 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH1P_P23:
                PCRM_ADCCTL1 |= PCRM_ADCCTL_PCHEN;
                break;

            case ADC_CH2N_P24:
                PCRM_ADCCTL2 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH2P_P14:
                PCRM_ADCCTL2 |= PCRM_ADCCTL_PCHEN;
                break;

            case ADC_CH3N_P15:
                PCRM_ADCCTL3 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH3P_P20:
                PCRM_ADCCTL3 |= PCRM_ADCCTL_PCHEN;
                break;
            }
        }
    }

    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO; // new

    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;
    AP_ADCC->intr_mask = mAdc_Ctx.irq_bits; // ENABLE_ADC_INT;

    // disableSleep();
    return PPlus_SUCCESS;
}

int hal_adc_config_channel(adc_Cfg_t cfg, adc_Hdl_t evt_handler)
{
    uint8_t i;
    uint8_t chn_sel = 0;
    gpio_pin_e pin, pin_neg;

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    if (evt_handler == NULL)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if ((cfg.channel & BIT(0)) || (cfg.channel & BIT(1)))
    {
        return PPlus_ERR_NOT_SUPPORTED;
    }

    if ((!cfg.channel & BIT(1)) && (cfg.is_differential_mode && (cfg.channel & BIT(1))))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if (cfg.is_differential_mode != 0)
    {
        if ((cfg.is_differential_mode != 0x80) && (cfg.is_differential_mode != 0x20) && (cfg.is_differential_mode != 0x08))
        {
            return PPlus_ERR_INVALID_PARAM;
        }
    }

    clear_adcc_cfg();

    /* Set all ADC channels to be switched off (attn = 0, pass = 0 for each bit) */
    AON_PMCTL2_1 = 0x00;

    /* Analog LDO off */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;

    /* ADC Disabled */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    mAdc_Ctx.continue_mode = cfg.is_continue_mode;
    mAdc_Ctx.irq_bits = cfg.channel & 0x03;

    for (i = 2; i < 8; i++)
    {
        if (cfg.channel & BIT(i))
        {
            if (i % 2)
            {
                /* 3=>2, 5=>4, 7=>6*/
                mAdc_Ctx.irq_bits |= BIT(i - 1);
            }
            else
            {
                /* 2=>3, 4=>5, 6=>7 */
                mAdc_Ctx.irq_bits |= BIT(i + 1);
            }
        }
    }
    ////////mAdc_Ctx.chs_en_shadow = mAdc_Ctx.irq_bits;
    // LOG("cfg.channel:0x%x",cfg.channel);
    /// LOG("mAdc_Ctx.irq_bits:0x%x", mAdc_Ctx.irq_bits);

    /* Re-enable the clock of the ADCC module */
    hal_clk_gate_enable(MOD_ADCC);

    /* CLK_1P28M_ENABLE; this might be used only for the DMIC? */
    PCRM_CLKSEL |= PCRM_CLKSEL_1P28M;

    // ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    PCRM_CLKHF_CTL0 |= PCRM_CLKHF_CTL0_XTALOUT;

    // ENABLE_DLL;                  //enable DLL
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_DLL;
    // ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADCDBL_Msk; // check
    // subWriteReg(0x4000F044,21,20,3);
    // ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_ADC;

    // subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
    PCRM_ADCCTL4 |= PCRM_ADCCTL4_MODE_MANUAL;
    PCRM_ADCCTL4 |= BIT(0);

    set_sampling_resolution_auto(cfg.channel, cfg.is_high_resolution, cfg.is_differential_mode);

    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ANACTL &= ~PCRM_ANACTL_MICBIAS; // disable micbias

    mAdc_Ctx.evt_handler = evt_handler;

    if (cfg.is_differential_mode == 0)
    {
        PCRM_ANACTL |= PCRM_ANACTL_DIFF1;
        PCRM_ANACTL |= PCRM_ANACTL_DIFF2;

        PCRM_ADCCTL4 &= ~PCRM_ADCCTL4_MODE_Msk; // enable auto mode
        for (i = MIN_ADC_CH; i <= MAX_ADC_CH; i++)
        {
            if (cfg.channel & BIT(i))
            {
                gpio_pin_e pin = s_pinmap[i];
                hal_gpio_ds_control(pin, Bit_ENABLE);
                hal_gpio_cfg_analog_io(pin, Bit_ENABLE);
            }
        }
    }
    else
    {
        switch (cfg.is_differential_mode)
        {
        case 0x80:
            pin = P20;
            pin_neg = P15;
            chn_sel = 0x04;
            break;

        case 0x20:
            pin = P14;
            pin_neg = P24;
            chn_sel = 0x03;
            break;

        case 0x08:
            pin = P23;
            pin_neg = P11;
            chn_sel = 0x02;
            break;

        case 0x02:
            pin = P18;
            pin_neg = P25;
            chn_sel = 0x01;

            AON_PMCTL2_1 = 0x0060; /* AIO_7 and AIO_8 passthrough enable */
            break;

        default:
            break;
        }

        hal_gpio_ds_control(pin, Bit_ENABLE);
        subWriteReg(&(PCRM_ANACTL), 7, 5, chn_sel);

        PCRM_ANACTL &= ~PCRM_ANACTL_DIFF1;
        PCRM_ANACTL &= ~PCRM_ANACTL_DIFF2;

    // LOG("%d %d %x",pin,pin_neg,*(volatile int*)0x40003800);
        hal_gpio_cfg_analog_io(pin, Bit_ENABLE);
        hal_gpio_cfg_analog_io(pin_neg, Bit_ENABLE);
        // LOG("%d %d %x",pin,pin_neg,*(volatile int*)0x40003800);
        mAdc_Ctx.irq_bits = (cfg.is_differential_mode >> 1);
    }

    return PPlus_SUCCESS;
}
#endif

int hal_adc_start(void)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    // LOG("irq_bits2:0x%x",irq_bits2);
    hal_pwrmgr_lock(MOD_ADCC);

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((mAdc_Ctx.enabled_channels & BIT(i)) == 0)
        {
            continue;
        }

        /* Sets only the "Channel Enable" bit on the appropriate ADCCTLx */
        /* TODO!!!: investigate the other stuff on the register */
        *adcs[i].adc_ctl |= adcs[i].adc_ctl_val;
    }

    /* Just in case, clear pending IRQs */
    AP_ADCC->intr_clear = 0x1FF;

    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO; // new

    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;
    AP_ADCC->intr_mask = mAdc_Ctx.irq_bits; // ENABLE_ADC_INT;

    // disableSleep();
    return PPlus_SUCCESS;
}

int hal_adc_config_single_ended(uint16_t channels, uint16_t high_res_channels, adc_Hdl_t evt_handler)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    if (channels & 0x3ff == 0)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    clear_adcc_cfg();

    mAdc_Ctx.enabled_channels = channels;
    mAdc_Ctx.highres_channels = high_res_channels;
    mAdc_Ctx.evt_handler = evt_handler;

    /* single ended mode */
    PCRM_ANACTL |= PCRM_ANACTL_DIFF1;
    PCRM_ANACTL |= PCRM_ANACTL_DIFF2;

    /* Enable auto mode (i.e. continuous) */
    PCRM_ADCCTL4 &= ~PCRM_ADCCTL4_MODE_MANUAL;

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((channels & BIT(i)) == 0)
        {
            continue;
        }

        mAdc_Ctx.irq_bits |= adcs[i].irq_mask_bit;

        set_channel_res(adcs[i].pmctl2_1_bit, (high_res_channels & BIT(i)) ? 1 : 0);

        hal_gpio_ds_control(adcs[i].single_ended_pin, Bit_ENABLE);
        hal_gpio_cfg_analog_io(adcs[i].single_ended_pin, Bit_ENABLE);
    }

    /* Re-enable the clock of the ADCC module */
    hal_clk_gate_enable(MOD_ADCC);

    return PPlus_SUCCESS;
}

int hal_adc_stop(void)
{
    int i;

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    AON_PMCTL2_1 = 0x00;
    NVIC_DisableIRQ((IRQn_Type)ADCC_IRQn);
    AP_ADCC->intr_clear = 0x1FF;

    if (g_system_clk != SYS_CLK_DBL_32M)
    {
        PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADC;
    }

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((mAdc_Ctx.enabled_channels & BIT(i)) == 0)
        {
            continue;
        }

        hal_gpio_cfg_analog_io(adcs[i].single_ended_pin, Bit_DISABLE);
    }

    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO; // Power down analog LDO

    hal_clk_reset(MOD_ADCC);
    hal_clk_gate_disable(MOD_ADCC);

    /* WHY???? */
    clear_adcc_cfg();

    // enableSleep();
    hal_pwrmgr_unlock(MOD_ADCC);
    return PPlus_SUCCESS;
}

