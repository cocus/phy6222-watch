/****************************************************************************
 * arch/arm/src/phy62xx/phy62xx_ble.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

// #include <sys/types.h>

// #include <sys/socket.h>

// #include <stdbool.h>
// #include <stdio.h>
#include <string.h>
// #include <unistd.h>
// #include <errno.h>
// #include <debug.h>
/* #include "OSAL.h" */

/* #include "hci_tl.h" */

#include "jump_function.h"
#include "rom_sym_def.h"
#include "phy62xx_ble.h"
#include "bus_dev.h"
#include "log.h"
// #include <nuttx/nuttx.h>
// #include <nuttx/kmalloc.h>
// #include <nuttx/wqueue.h>
// #include <nuttx/net/bluetooth.h>
// #include <nuttx/wireless/bluetooth/bt_driver.h>
// #include <nuttx/wireless/bluetooth/bt_uart.h>

#define H4_HEADER_SIZE  1

#define H4_CMD           0x01
#define H4_ACL           0x02
#define H4_SCO           0x03
#define H4_EVT           0x04
#define H4_ISO           0x05

#define BLE_BUF_SIZE 512

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pplus_ble_recv_cb
 *
 * Description:
 *   BLE receive callback function when BLE hardware receive packet
 *
 * Input Parameters:
 *   data - BLE packet data pointer
 *   len  - BLE packet length
 *
 * Returned Value:
 *   0 on success or a negated value on failure.
 *
 ****************************************************************************/

int pplus_ble_recv_cb_h4(uint8_t *data, uint16_t len)
{
    int ret = 0;
    // struct pplus_ble_priv_s *priv = &g_pplus_ble;

    // ret = bt_netdev_receive(&priv->drv, BT_EVT,
    //                             &data[H4_HEADER_SIZE],
    //                             len - H4_HEADER_SIZE);
    if (ret < 0)
    {
        LOG("ERROR: Failed to receive ret=%d\n", ret);
    }

    return ret;
}

int pplus_ble_recv_cb_acl(uint8_t *data, uint16_t len)
{
    int ret = 0;
    // struct pplus_ble_priv_s *priv = &g_pplus_ble;

    // ret = bt_netdev_receive(&priv->drv, BT_ACL_IN,
    //                             &data[H4_HEADER_SIZE],
    //                             len - H4_HEADER_SIZE);
    if (ret < 0)
    {
        LOG("ERROR: Failed to receive ret=%d\n", ret);
    }

    return ret;
}

uint8_t pplus_ble_recv_msg(uint8_t destination_task, uint8_t *msg_ptr)
{
    UNUSED(destination_task);

    /* int ret = 0; */
#if 0
    hciPacket_t *hci_msg = (hciPacket_t *)msg_ptr;
    uint8_t *pdata = hci_msg->pData;
    uint8_t dataLength;
    uint16_t hcipkt_len;

    /* gpio_write(P25, 1); */

    if (pdata[0] == H4_EVT)
    {
        dataLength = pdata[2];
        hcipkt_len = HCI_EVENT_MIN_LENGTH + dataLength;
        /* logx("len %d\n",dataLength);
         * for(int k = 0;k<hcipkt_len ;k++)
         * logx("%02X,",pdata[k]);
         * logx("\n");
         */

        pplus_ble_recv_cb_h4(pdata, hcipkt_len);

        /* ret = pplus_ble_recv_cb_h4(pdata, hcipkt_len); */
    }
    else if (pdata[0] == H4_ACL)
    {
        dataLength = pdata[3];
        hcipkt_len = HCI_DATA_MIN_LENGTH + dataLength;
        pdata[2] &= 0x3f;

        /* logx("len %d\n",dataLength);
         * for(int k = 0;k<hcipkt_len ;k++)
         * logx("%02X,",pdata[k]);
         * logx("\n");
         */

        pplus_ble_recv_cb_acl(pdata, hcipkt_len);

        /* ret = pplus_ble_recv_cb_acl(pdata, hcipkt_len); */
    }

    /* gpio_write(P25, 0); */

    osal_bm_free(pdata);

    return osal_msg_deallocate(msg_ptr);
#endif
    return 0;
}

#if 0
/****************************************************************************
 * Name: pplus_ble_send
 *
 * Description:
 *   ESP32-C3 BLE send callback function for BT driver.
 *
 * Input Parameters:
 *   drv  - BT driver pointer
 *   type - BT packet type
 *   data - BT packet data buffer pointer
 *   len  - BT packet length
 *
 * Returned Value:
 *   Sent bytes on success or a negated value on failure.
 *
 ****************************************************************************/

extern void hciProcessHostToCtrlCmd(uint8_t *pData);

static int pplus_ble_send(struct bt_driver_s *drv,
                          enum bt_buf_type_e type,
                          void *data, size_t len)
{
    /* uint8_t *hdr = (uint8_t *)data - drv->head_reserve;
     * printf("pplus_ble_send");
     */

    if ((len + H4_HEADER_SIZE) > BLE_BUF_SIZE)
    {
        return -EINVAL;
    }

    if (type == BT_CMD)
    {
        /* *hdr = H4_CMD; */

        hciProcessHostToCtrlCmd((uint8_t *)data);
    }
    else if (type == BT_ACL_OUT)
    {
        uint16_t connHandle;
        uint16_t param;
        uint8_t pbFlag;
        uint16_t pktLen;
        uint8_t *acldata = (uint8_t *)data;
        uint8_t *send_buf = NULL;

        param = BUILD_UINT16(acldata[0], acldata[1]);
        connHandle = param & 0xfff;
        pbFlag = (param & 0x3000) >> 12;
        pktLen = BUILD_UINT16(acldata[2], acldata[3]);

        send_buf = (uint8_t *)HCI_bm_alloc(pktLen);

        if (!send_buf)
        {
            return -ENOMEM;
        }

        memcpy(send_buf, &acldata[4], pktLen);

        int ret = LL_TxData(connHandle, send_buf, pktLen, pbFlag);

        if (ret == HCI_SUCCESS)
        {
            osal_bm_free(send_buf);
            return -ret;
        }
    }
    else
    {
        return -EINVAL;
    }

    return len;
}

/****************************************************************************
 * Name: pplus_ble_close
 *
 * Description:
 *   ESP32-C3 BLE close callback function for BT driver.
 *
 * Input Parameters:
 *   drv  - BT driver pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* static void pplus_ble_close(struct bt_driver_s *drv)
 * {
 * }
 */

/****************************************************************************
 * Name: pplus_ble_open
 *
 * Description:
 *   ESP32-C3 BLE open callback function for BT driver.
 *
 * Input Parameters:
 *   drv - BT driver pointer
 *
 * Returned Value:
 *   OK on success or a negated value on failure.
 *
 ****************************************************************************/

static int pplus_ble_open(struct bt_driver_s *drv)
{
    return 0;
}
#endif

extern uint8_t hciCtrlCmdToken;
int drv_disable_irq1(void)
{
    NVIC_DisableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(BB_IRQn));
    return 0;
}

int drv_enable_irq1(void)
{
    NVIC_EnableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(BB_IRQn));
    return 0;
}

int pplus_ble_initialize(void)
{
    int ret = 0;

    JUMP_FUNCTION(OSAL_MSG_SEND) = (uint32_t)&pplus_ble_recv_msg;
    JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE) = (uint32_t)&drv_disable_irq1;
    JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE) = (uint32_t)&drv_enable_irq1;
    hciCtrlCmdToken = 1;

    if (ret < 0)
    {
        LOG("ERROR: bt_netdev_register error: %d\n", ret);
        return ret;
    }

    return ret;
}
