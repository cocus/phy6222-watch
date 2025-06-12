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


#define    ADC_CH_BASE             (ADCC_BASE_ADDR + 0x00000400UL)

typedef enum
{
    ADC_CTX_NOT_INITALIZED = 0U,        /*!< Context is not initialized yet */
    ADC_CTX_INITIALIZED = 1U            /*!< Context is initialized */
} ADC_CTX_INIT_t;

typedef struct _adc_Contex_t
{
    ADC_CTX_INIT_t state;

    uint16_t all_channel;
    uint16_t chs_en_shadow;

    uint16_t my_channels;
    uint16_t my_high_res;

    uint8_t continue_mode;

    // sysclk_t    clk_src;
    uint16_t adc_cal_postive;
    uint16_t adc_cal_negtive;

    adc_Hdl_t evt_handler;
} adc_Ctx_t;

static adc_Ctx_t mAdc_Ctx = {
    .state = ADC_CTX_NOT_INITALIZED,
    .all_channel = 0x00,
    .chs_en_shadow = 0x00,
    .continue_mode = 0,
    .adc_cal_postive = 0xFFF,
    .adc_cal_negtive = 0xFFF,
    .evt_handler = NULL
};

gpio_pin_e s_pinmap[ADC_CH_NUM] = {
    GPIO_DUMMY, // ADC_CH0 =0,
    GPIO_DUMMY, // ADC_CH1 =1,
    P11,        // ADC_CH1N =2,
    P23,        // ADC_CH1P =3,  ADC_CH1DIFF = 3,
    P24,        // ADC_CH2N =4,
    P14,        // ADC_CH2P =5,  ADC_CH2DIFF = 5,
    P15,        // ADC_CH3N =6,
    P20,        // ADC_CH3P =7,  ADC_CH3DIFF = 7,
    GPIO_DUMMY, // ADC_CH_VOICE =8,
};

static const struct {
    adc_CH_t    hal_adc;
    adc_channels_t ch;
    gpio_pin_e  single_ended_pin;
    uint8_t     adc_ctl_bit_num;
    uint8_t     aio;
    uint8_t     aio_differential_pair;
    uint8_t     bitfield_bit;
    __IO uint32_t *adc_ctl;
    uint32_t    adc_ctl_val;
} adcs[] = {
    /*{ P25, 1, 0,  0, 8, 9, 0 },
    { P18, 0, 1,  0, 9, 8, 1 },*/

    [0] = { ADC_CH1N_P11, CH0, P11, 20, 0, 1, 3, &PCRM_ADCCTL1, PCRM_ADCCTL1_CH1N },
    [1] = { ADC_CH1P_P23, CH1, P23,  4, 1, 0, 2, &PCRM_ADCCTL1, PCRM_ADCCTL1_CH1P },

    [2] = { ADC_CH2N_P24, CH2, P24, 20, 2, 3, 5, &PCRM_ADCCTL2, PCRM_ADCCTL2_CH2N },
    [3] = { ADC_CH2P_P14, CH3, P14,  4, 3, 2, 4, &PCRM_ADCCTL2, PCRM_ADCCTL2_CH2P },

    [4] = { ADC_CH3N_P15, CH4, P15, 20, 4, 7, 7, &PCRM_ADCCTL3, PCRM_ADCCTL3_CH3N },
    [9] = { ADC_CH3P_P20, CH9, P20,  4, 7, 4, 6, &PCRM_ADCCTL3, PCRM_ADCCTL3_CH3P },

    [5] = { 0, CH5, P16, 0, 0, 0, 0, NULL, 0 },
    [6] = { 0, CH6, P17, 0, 0, 0, 0, NULL, 0 },

    [7] = { ADC_CH0DIFF, CH7, P18, 0, 0, 0, 0, &PCRM_ADCCTL0, PCRM_ADCCTL1_CH1N },
    [8] = { ADC_CH0DIFF, CH8, P25, 0, 0, 0, 0, &PCRM_ADCCTL0, PCRM_ADCCTL1_CH1P },
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

#define ADC_CFGS (sizeof(adcs)/sizeof(adcs[0]))

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
}

static void set_channel_res(uint8_t aio, uint8_t diff_pair, uint8_t is_diff, uint8_t is_high)
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

    if (is_diff)
    {
        subWriteReg(&(AON_PMCTL2_1), (diff_pair + 8), (diff_pair + 8), h);
        subWriteReg(&(AON_PMCTL2_1), diff_pair, diff_pair, l);
    }

    subWriteReg(&(AON_PMCTL2_1), (aio + 8), (aio + 8), h);
    subWriteReg(&(AON_PMCTL2_1), aio, aio, l);
}

static void set_sampling_resolution(adc_CH_t channel, uint8_t is_high_resolution, uint8_t is_differential_mode)
{
    for (size_t i = 0; i < (sizeof(adcs)/sizeof(adcs[0])); i++)
    {
        if (adcs[i].hal_adc == channel)
        {
            set_channel_res(adcs[i].aio, adcs[i].aio_differential_pair, is_differential_mode, is_high_resolution);
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

static void clear_adcc_cfg(void)
{
    mAdc_Ctx.all_channel = 0x00;
    mAdc_Ctx.chs_en_shadow = 0x00;
    mAdc_Ctx.continue_mode = 0;
    mAdc_Ctx.evt_handler = NULL;
}

#if 0
static void disable_channel(adc_CH_t ch)
{
	switch (ch)
	{
		case ADC_CH1N_P11:
			PCRM_ADCCTL1 &= ~BIT(20);
			break;

		case ADC_CH1P_P23:
			PCRM_ADCCTL1 &= ~BIT(4);
			break;

		case ADC_CH2N_P24:
			PCRM_ADCCTL2 &= ~BIT(20);
			break;

		case ADC_CH2P_P14:
			PCRM_ADCCTL2 &= ~BIT(4);
			break;

		case ADC_CH3N_P15:
			PCRM_ADCCTL3 &= ~BIT(20);
			break;

		case ADC_CH3P_P20:
			PCRM_ADCCTL3 &= ~BIT(4);
			break;
	}
}
#endif
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
        if ((mAdc_Ctx.my_channels & BIT(i)) == 0)
        {
            if (BIT(adcs[i].bitfield_bit) & status)
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
            AP_ADCC->intr_mask &= ~BIT(i);   // MASK coresponding channel
            mAdc_Ctx.my_channels &= ~BIT(i); // disable channel
        }

        uint16_t bitfield_bit = adcs[i].bitfield_bit;

        for (n = 0; n < (MAX_ADC_SAMPLE_SIZE - 3); n++)
        {
            adc_data[n] = (uint16_t)(read_reg(ADC_CH_BASE + (bitfield_bit * 0x80) + ((n + 2) * 4)) & 0xfff);
            adc_data[n + 1] = (uint16_t)((read_reg(ADC_CH_BASE + (bitfield_bit * 0x80) + ((n + 2) * 4)) >> 16) & 0xfff);
        }

        /* Clear the Interrupt of this channel */
        AP_ADCC->intr_clear = BIT(i);

        if (mAdc_Ctx.evt_handler)
        {
            adc_Evt_t evt;
            evt.type = HAL_ADC_EVT_DATA;
            LOG("i = %d, bitfield = %d", i, bitfield_bit);
            evt.is_high = (mAdc_Ctx.my_high_res & BIT(i)) ? 1 : 0;
            evt.is_diff = 0;
            evt.ch = adcs[i].hal_adc;
            evt.data = adc_data;
            evt.size = MAX_ADC_SAMPLE_SIZE - 3;
            mAdc_Ctx.evt_handler(&evt);
        }
    }

    // LOG("> %x",mAdc_Ctx.all_channel);
    if ((mAdc_Ctx.my_channels == 0) && (mAdc_Ctx.continue_mode == 0)) //
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

int hal_adc_start1(void)
{
    uint8_t all_channel2 = (((mAdc_Ctx.chs_en_shadow & 0x80) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x40) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x20) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x10) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x08) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x04) << 1));

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    // LOG("all_channel2:0x%x",all_channel2);
    hal_pwrmgr_lock(MOD_ADCC);

    for (int i = MIN_ADC_CH; i <= MAX_ADC_CH; i++)
    {
        if (all_channel2 & (BIT(i)))
        {
            switch (i)
            {
            case ADC_CH1N_P11:
                PCRM_ADCCTL1 |= PCRM_ADCCTL1_CH1N;
                break;

            case ADC_CH1P_P23:
                PCRM_ADCCTL1 |= PCRM_ADCCTL1_CH1P;
                break;

            case ADC_CH2N_P24:
                PCRM_ADCCTL2 |= PCRM_ADCCTL2_CH2N;
                break;

            case ADC_CH2P_P14:
                PCRM_ADCCTL2 |= PCRM_ADCCTL2_CH2P;
                break;

            case ADC_CH3N_P15:
                PCRM_ADCCTL3 |= PCRM_ADCCTL3_CH3N;
                break;

            case ADC_CH3P_P20:
                PCRM_ADCCTL3 |= PCRM_ADCCTL3_CH3P;
                break;
            }
        }
    }

    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO; // new

    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;
    AP_ADCC->intr_mask = mAdc_Ctx.all_channel; // ENABLE_ADC_INT;

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

    AON_PMCTL2_1 = 0x00;
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    mAdc_Ctx.continue_mode = cfg.is_continue_mode;
    mAdc_Ctx.all_channel = cfg.channel & 0x03;

    for (i = 2; i < 8; i++)
    {
        if (cfg.channel & BIT(i))
        {
            if (i % 2)
            {
                /* 3=>2, 5=>4, 7=>6*/
                mAdc_Ctx.all_channel |= BIT(i - 1);
            }
            else
            {
                /* 2=>3, 4=>5, 6=>7 */
                mAdc_Ctx.all_channel |= BIT(i + 1);
            }
        }
    }
    mAdc_Ctx.chs_en_shadow = mAdc_Ctx.all_channel;
    // LOG("cfg.channel:0x%x",cfg.channel);
    /// LOG("mAdc_Ctx.all_channel:0x%x", mAdc_Ctx.all_channel);

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

    PCRM_ADCCTL0 &= ~BIT(20);
    PCRM_ADCCTL0 &= ~BIT(4);

    PCRM_ADCCTL1 &= ~PCRM_ADCCTL1_CH1N;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL1_CH1P;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL2_CH2N;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL2_CH2P;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL3_CH3N;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL3_CH3P;

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

            AON_PMCTL2_1 = 0x0060;
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
        mAdc_Ctx.all_channel = (cfg.is_differential_mode >> 1);
    }

    return PPlus_SUCCESS;
}

int hal_adc_start(void)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    // LOG("all_channel2:0x%x",all_channel2);
    hal_pwrmgr_lock(MOD_ADCC);

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((mAdc_Ctx.my_channels & BIT(i)) == 0)
        {
            continue;
        }

        /* Set the appropriate ADCCTLx register with the appropriate value */
        *adcs[i].adc_ctl |= adcs[i].adc_ctl_val;
    }

    /* Just in case, clear pending IRQs */
    AP_ADCC->intr_clear = 0x1FF;

    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO; // new

    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;
    AP_ADCC->intr_mask = mAdc_Ctx.all_channel; // ENABLE_ADC_INT;

    // disableSleep();
    return PPlus_SUCCESS;
}

int hal_adc_config_single_ended(uint16_t channels, uint16_t high_res_channels, adc_Hdl_t evt_handler)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    uint16_t new_channel = 0;

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((channels & BIT(i)) == 0)
        {
            continue;
        }

        new_channel |= BIT(adcs[i].bitfield_bit);
    }

    if (new_channel == 0)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    /* Analog LDO off */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;

    /* ADC Disabled */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Set ADC resolution of all channels to low */
    AON_PMCTL2_1 = 0x00;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    clear_adcc_cfg();

    mAdc_Ctx.my_channels = channels;
    mAdc_Ctx.my_high_res = high_res_channels;
    mAdc_Ctx.evt_handler = evt_handler;
    mAdc_Ctx.all_channel = new_channel;
    mAdc_Ctx.chs_en_shadow = mAdc_Ctx.all_channel; /* ??? */

    /* CLK_1P28M_ENABLE; this might be used only for the DMIC? */
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

    PCRM_ADCCTL0 &= ~BIT(20);
    PCRM_ADCCTL0 &= ~BIT(4);
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL1_CH1N;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL1_CH1P;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL2_CH2N;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL2_CH2P;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL3_CH3N;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL3_CH3P;

    /* Disable Mic Bias */
    PCRM_ANACTL &= ~PCRM_ANACTL_MICBIAS;

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

        set_channel_res(adcs[i].aio, adcs[i].aio_differential_pair, 0, (high_res_channels & BIT(i)) ? 1 : 0);

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
        if ((mAdc_Ctx.my_channels & BIT(i)) == 0)
        {
            continue;
        }

        hal_gpio_cfg_analog_io(s_pinmap[i], Bit_DISABLE);
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
        float delta = ((int)(adc_cal_postive - adc_cal_negtive)) / 2.0;

        if (ch & 0x01)
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / (adc_cal_postive + adc_cal_negtive))
                                 : ((result - delta) / (adc_cal_postive + adc_cal_negtive));
        }
        else
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / (adc_cal_postive + adc_cal_negtive))
                                 : ((result + delta) / (adc_cal_postive + adc_cal_negtive));
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
