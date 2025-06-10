/**************************************************************************************************

  Phyplus Microelectronics Limited confidential and proprietary.
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics
  Limited ("Phyplus"). Your use of this Software is limited to those
  specific rights granted under  the terms of the business contract, the
  confidential agreement, the non-disclosure agreement and any other forms
  of agreements as a customer or a partner of Phyplus. You may not use this
  Software unless you agree to abide by the terms of these agreements.
  You acknowledge that the Software may not be modified, copied,
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
  (BLE) integrated circuit, either as a product or is integrated into your
  products.  Other than for the aforementioned purposes, you may not use,
  reproduce, copy, prepare derivative works of, modify, distribute, perform,
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
  Filename:       gpio_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "pwrmgr.h"
#include "error.h"
#include "key.h"

/*********************************************************************
 * timer_Task
 * Task timer sample code
 */
static uint8 timer_TaskID;
#define TIMER_1S_ONCE 0x0001
#define TIMER_2S_CYCLE 0x0004

void timer_int_process(uint8_t evt)
{
    switch (evt)
    {
    case HAL_EVT_TIMER_5:
        LOG("t5");
        break;

    case HAL_EVT_TIMER_6:
        LOG("t6");
        break;

    case HAL_EVT_WAKEUP:
        LOG("wakeup");
        break;

    case HAL_EVT_SLEEP:
        LOG("sleep");
        break;

    default:
        LOG("err ");
        break;
    }
}

void Timer_Demo_Init(uint8 task_id)
{
    timer_TaskID = task_id;

    LOG("task id is %d", task_id);
    // LOG("%s: %s() -> %d", __FUNCTION__, "osal_start_reload_timer", osal_start_timerEx(timer_TaskID, TIMER_1S_ONCE, 1000));
    LOG("%s() -> %d", "osal_start_reload_timer", osal_start_reload_timer(timer_TaskID, TIMER_2S_CYCLE, 2000));
    hal_timer_init(timer_int_process);
}

uint16 Timer_Demo_ProcessEvent(uint8 task_id, uint16 events)
{
    static uint8 count1 = 0, count2 = 0;
    static bool timer_cycle_enable = TRUE;
    static bool LED = FALSE;

    LOG("task id is %d (timer is %d), events is %.4x", task_id, timer_TaskID, events);

    if (task_id != timer_TaskID)
    {
        return 0;
    }

    if (events & TIMER_1S_ONCE)
    {
        LOG("1s:once only mode");
        osal_start_timerEx(timer_TaskID, TIMER_1S_ONCE, 1000);

        if (timer_cycle_enable == FALSE)
        {
            if (++count1 >= 10)
            {
                osal_start_reload_timer(timer_TaskID, TIMER_2S_CYCLE, 2000);

                LOG("2s:recycle mode start");
                timer_cycle_enable = TRUE;
                count1 = 0;
            }
        }
        return (events ^ TIMER_1S_ONCE);
    }

    if (events & TIMER_2S_CYCLE)
    {
        LOG("2s:recycle mode");

        if (LED == FALSE)
            LED = TRUE;
        else
            LED = FALSE;
        gpio_write(GPIO_P00, LED);

        /*if (++count2 >= 5)
        {
            osal_stop_timerEx(timer_TaskID, TIMER_2S_CYCLE);

            LOG("2s:recycle mode stop");
            timer_cycle_enable = FALSE;
            count2 = 0;
        }*/

        return (events ^ TIMER_2S_CYCLE);
    }

    return 0;
}

/*********************************************************************
    gpio_wakeup_Task
    Task gpio wakeup sample code
    The followinng code shows P14 wakeup the system when there is a posedge or negedge.
*/
static uint8 gpio_wakeup_TaskID;
void posedge_callback_wakeup(gpio_pin_e pin, gpio_polarity_e type)
{
    if (type == POSEDGE)
    {
        LOG("wakeup(pos):gpio:%d type:%d", pin, type);
    }
    else
    {
        LOG("error");
    }
}

void negedge_callback_wakeup(gpio_pin_e pin, gpio_polarity_e type)
{
    if (type == NEGEDGE)
    {
        LOG("wakeup(neg):gpio:%d type:%d", pin, type);
    }
    else
    {
        LOG("wakeup(pos):gpio:%d type:%d", pin, type);
    }
}

/*
     P00~P03:default jtag,we can use it as wakeup pin when no debug.
     P04~P07,P11~P15,P18~P30:default gpio,use it easily.
     P08:mode select pin,cannot used as other usage.
     P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
     P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
     P31~P34:default spif,we can use it as wakeup pin directly,we driver have completed its multiplex config.
*/

void Gpio_Wakeup_Init(uint8 task_id)
{
    uint8_t i = 0;
    gpio_wakeup_TaskID = task_id;
    LOG("gpio wakeup demo start task id is %d", task_id);

    // hal_gpio_pull_set(P14,WEAK_PULL_UP);
    //for (i = 0; i < GPIO_WAKEUP_PIN_NUM; i++)
    {
        LOG("%s: %d", "hal_gpioin_register", hal_gpioin_register(GPIO_P11, posedge_callback_wakeup, negedge_callback_wakeup));
        LOG("gpioin_state: P11 %d", hal_gpio_read(GPIO_P11));
        hal_gpio_wakeup_set(GPIO_P11, POSEDGE);
    }
}

uint16 Gpio_Wakeup_ProcessEvent(uint8 task_id, uint16 events)
{
    //LOG("task_id %d, ev %d", task_id, events);

    if (task_id != gpio_wakeup_TaskID)
    {
        return 0;
    }


    return 0;
}

/*********************************************************************
*********************************************************************/
