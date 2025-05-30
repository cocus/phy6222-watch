/*
 * Copyright (C) 2019 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 *  Made for BlueKitchen by OneWave with <3
 *      Author: ftrefou@onewave.io
 */
#define BTSTACK_FILE__ "btstack_port.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "btstack_config.h"

#include "hci_dump_embedded_stdout.h"

#include "hci_transport.h"

#include "log.h"
#include "jump_function.h"
#include "rom_sym_def.h"
#include "OSAL.h"

/********************************************
 *      system calls implementation
 *******************************************/
ssize_t _write(int fd, const void *buf, size_t count)
{
    UNUSED(fd);
    const uint8_t *ptr = buf;
    // expand '/n' to '/r/n'
    size_t pos;
    for (pos = 0; pos < count; pos++)
    {
        uint8_t next_char = *ptr++;
        if (next_char == '\n')
        {
            const uint8_t NEWLINE[] = "\r";
            hal_uart_send_buff(UART0, (uint8_t *)&NEWLINE[0], 2);
        }
        else
        {
            hal_uart_send_buff(UART0, &next_char, 1);
        }
    }

    return count;
}

ssize_t _read(int file, void *buf, size_t count)
{
    uint8_t *ptr = buf;
    UNUSED(file);
    UNUSED(buf);
    UNUSED(ptr);
    // HAL_UART_Receive(&hTuart, ptr, count, 1000);
    count = 0;
    return count;
}

int _close(int file)
{
    UNUSED(file);
    return -1;
}

int _isatty(int file)
{
    UNUSED(file);
    return -1;
}

int _lseek(int file)
{
    UNUSED(file);
    return -1;
}

int _fstat(int file)
{
    UNUSED(file);
    return -1;
}

int _kill(pid_t pid, int sig)
{
    UNUSED(pid);
    UNUSED(sig);
    return -1;
}

pid_t _getpid(void)
{
    return 0;
}

extern volatile uint32_t osal_sys_tick;
uint32_t hal_time_ms(void)
{
    return osal_sys_tick;
}

/*******************************************
 *       transport implementation
 ******************************************/

#include "btstack.h"
#include "btstack_config.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_freertos.h"
#include "btstack_tlv_none.h"
#include "hci_event.h"
#include "hci_transport.h"
#include "le_device_db_tlv.h"
#include "hci.h"
#include "hci_dump.h"
#include "btstack_debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#undef HCI_H
#include "../SDK/components/ble/include/hci.h"
#include "../SDK/components/ble/hci/hci_tl.h"
#include "osal_bufmgr.h"

//
// Controller
//
static uint8_t send_hardware_error;
static bool send_transport_sent;

static uint8_t hci_outgoing_event[258];
static bool hci_outgoing_event_ready = false;

static void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);

// data source for integration with BTstack Runloop
static btstack_data_source_t transport_data_source;

// run from main thread

static void transport_send_hardware_error(uint8_t error_code)
{
    uint8_t event[] = {HCI_EVENT_HARDWARE_ERROR, 1, error_code};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

static void transport_notify_ready(void)
{
    // notify upper stack that it transport is ready
    uint8_t event[] = {HCI_EVENT_TRANSPORT_READY, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

static void transport_emit_hci_event(const hci_event_t *event, ...)
{
    va_list argptr;
    va_start(argptr, event);
    uint16_t length = hci_event_create_from_template_and_arglist(hci_outgoing_event, sizeof(hci_outgoing_event), event, argptr);
    va_end(argptr);
    transport_packet_handler(HCI_EVENT_PACKET, hci_outgoing_event, length);
}

static void send_command_complete(uint16_t opcode, uint8_t status, const uint8_t *result, uint16_t len)
{
    hci_event_create_from_template_and_arguments(hci_outgoing_event, sizeof(hci_outgoing_event),
                                                 &hci_event_command_complete, /* num commands */ 1, opcode, status, len, result);
    hci_outgoing_event_ready = true;
    btstack_run_loop_poll_data_sources_from_irq();
}

static void fake_command_complete(uint16_t opcode)
{
    hci_event_create_from_template_and_arguments(hci_outgoing_event, sizeof(hci_outgoing_event),
                                                 &hci_event_command_complete, /* num commands */ 1, opcode, ERROR_CODE_SUCCESS, 0, NULL);
    hci_outgoing_event_ready = true;
    btstack_run_loop_poll_data_sources_from_irq();
}

extern uint8_t hciCtrlCmdToken;

static void transport_process(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type)
{
    UNUSED(ds);
    UNUSED(callback_type);
    // deliver command complete events caused by command processor
    if (hci_outgoing_event_ready)
    {
        hci_outgoing_event_ready = false;

        uint8_t size = hci_outgoing_event[1] + 2;
        uint8_t *packet = (uint8_t *)&hci_outgoing_event[0];
        dbg_printf("DUMP DUMP DUMP dump pkt: len %d\n", size);
        for (int k = 0; k < size; k++)
            dbg_printf("%02X,", packet[k]);
        dbg_printf("\n");
        dbg_printf("hciCtrlCmdToken %02x\n", hciCtrlCmdToken);

        LOG(" -> deliver command complete event %x, size %d", hci_outgoing_event[0], hci_outgoing_event[1] + 2);
        transport_packet_handler(HCI_EVENT_PACKET, hci_outgoing_event, hci_outgoing_event[1] + 2);
    }

    if (send_hardware_error != 0)
    {
        uint8_t error_code = send_hardware_error;
        send_hardware_error = 0;
        LOG(" -> notify upper stack of hardware error %02X", error_code);
        transport_emit_hci_event(&hci_event_hardware_error, error_code);
    }

    if (send_transport_sent)
    {
        send_transport_sent = false;
        LOG(" -> notify upper stack that it might be possible to send again");
        // notify upper stack that it might be possible to send again
        transport_emit_hci_event(&hci_event_transport_packet_sent);
    }
}

int drv_disable_irq1(void)
{
    NVIC_DisableIRQs(BIT(TIM1_IRQn) | BIT(TIM3_IRQn) | BIT(BB_IRQn));
    return 0;
}

int drv_enable_irq1(void)
{
    NVIC_EnableIRQs(BIT(TIM1_IRQn) | BIT(TIM3_IRQn) | BIT(BB_IRQn));
    return 0;
}

#define H4_HEADER_SIZE 1

//
// Minimum length for EVENT packet is 1+1+1
// | Packet Type (1) | Event Code(1) | Length(1) |
//
#define HCI_EVENT_MIN_LENGTH 3

//
// Minimum length for DATA packet is 1+2+2
// | Packet Type (1) | Handler(2) | Length(2) |
//
#define HCI_DATA_MIN_LENGTH 5

typedef struct __attribute__((packed))
{
    uint8_t evtcode;
    uint8_t plen;
    uint8_t payload[1];
} hcievt_t;

typedef struct __attribute__((packed))
{
    uint8_t type;
    hcievt_t evt;
} hcievtserial_t;

typedef struct __attribute__((packed))
{
    uint8_t type;
    uint16_t handle;
    uint16_t length;
    uint8_t acl_data[1];
} acldataserial_t;

uint8_t pplus_ble_recv_msg(uint8_t destination_task, uint8_t *msg_ptr)
{
    if (destination_task != hciTaskID)
    {
        return 0;
    }

    hciPacket_t *hci_msg = (hciPacket_t *)msg_ptr;
    hcievtserial_t *evp = (hcievtserial_t *)hci_msg->pData;
    acldataserial_t *aclp = (hcievtserial_t *)hci_msg->pData;

    uint16_t size = 0;
    uint8_t *packet = NULL;

    LOG("<<<<< pplus_ble_recv_msg: packet type %02X",
        evp->type);

    switch (evp->type)
    {
    case HCI_EVENT_PACKET:

        size = evp->evt.plen + 2;
        packet = (uint8_t *)&evp->evt;
        packet[2] = 1;

        /* Send buffer to upper stack */
        memcpy(&hci_outgoing_event[0], packet, size);
        hci_outgoing_event_ready = true;
        /*transport_packet_handler(
            evp->type,
            packet,
            size);*/
        break;

        /*case HCI_ACL_DATA_PACKET:
            acl = &(((acldatapacket_t *)hcievt)->AclDataSerial);
            // Send buffer to upper stack
            transport_packet_handler(
                acl->type,
                &((uint8_t*)acl)[1],
                acl->length+4);
            break;*/

    default:
        LOG(" <<<<< pplus_ble_recv_msg: invalid packet type %02X", evp->type);
        send_hardware_error = 0x01;
        break;
    }
#if 0

    if (pdata->pkt == HCI_EVENT_PACKET)
    {
        dataLength = pdata->plen;
        hcipkt_len = HCI_EVENT_MIN_LENGTH + dataLength;
        //pdata[3] = 1;

        dbg_printf("DUMP DUMP DUMP dump pkt: len %d\n", dataLength);
        for (int k = 0; k < hcipkt_len; k++)
            dbg_printf("%02X,", pdata[k]);
        dbg_printf("\n");
        dbg_printf("hciCtrlCmdToken %02x\n", hciCtrlCmdToken);

        // Send buffer to upper stack
        transport_packet_handler(
            pdata->pkt,
            &pdata[H4_HEADER_SIZE],
            hcipkt_len - H4_HEADER_SIZE);
    }
    else if (pdata[0] == HCI_ACL_DATA_PACKET)
    {
        dataLength = pdata[3];
        hcipkt_len = HCI_DATA_MIN_LENGTH + dataLength;
        pdata[2] &= 0x3f;

        /* logx("len %d",dataLength);
         * for(int k = 0;k<hcipkt_len ;k++)
         * logx("%02X,",pdata[k]);
         * logx("");
         */

        // Send buffer to upper stack
        transport_packet_handler(
            pdata[0],
            &pdata[H4_HEADER_SIZE],
            hcipkt_len - H4_HEADER_SIZE);
    }
    else
    {
        send_hardware_error = 0x01;
    }
    /* gpio_write(P25, 0); */
#endif
    // osal_bm_free(hcievt->header.pData);

    uint8_t r = osal_msg_deallocate(msg_ptr);
    btstack_run_loop_poll_data_sources_from_irq();
    return r;
}

/**
 * init transport
 * @param transport_config
 */
static void transport_init(const void *transport_config)
{
    UNUSED(transport_config);
    log_info("transport_init");

    // set up polling data_source
    btstack_run_loop_set_data_source_handler(&transport_data_source, &transport_process);
    btstack_run_loop_enable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&transport_data_source);

    transport_notify_ready();
}

/**
 * open transport connection
 */
static int transport_open(void)
{
    log_info("transport_open");
    return 0;
}

/**
 * close transport connection
 */
static int transport_close(void)
{
    log_info("transport_close");
    return 0;
}

/**
 * register packet handler for HCI packets: ACL and Events
 */
static void transport_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size))
{
    log_info("transport_register_packet_handler");
    transport_packet_handler = handler;
}

/**
 * support async transport layers, e.g. IRQ driven without buffers
 */
static int transport_can_send_packet_now(uint8_t packet_type)
{
    LOG("transport_can_send_packet_now: packet_type %02X, send_transport_sent %d, hci_outgoing_event_ready %d",
        packet_type, send_transport_sent, hci_outgoing_event_ready);
    if (send_transport_sent)
        return 0;
    switch (packet_type)
    {
    case HCI_COMMAND_DATA_PACKET:
        return hci_outgoing_event_ready ? 0 : 1;
    case HCI_ACL_DATA_PACKET:
        return 1;
    default:
        btstack_assert(false);
        break;
    }
    return 0;
}

static void controller_handle_hci_command(uint8_t *packet, uint16_t size)
{
    UNUSED(size);
    btstack_assert(hci_outgoing_event_ready == false);

    const uint8_t local_supported_features[] = {0, 0, 0, 0, 0x40, 0, 0, 0};
    // const uint8_t read_buffer_size_result[] = { 0x1b, 0, HCI_NUM_TX_BUFFERS_STACK };
    hciStatus_t status;
    uint16_t opcode = little_endian_read_16(packet, 0);

    switch (opcode)
    {
    case HCI_OPCODE_HCI_RESET:
        // extern hciStatus_t HCI_ResetCmd( void );
        LOG("HCI_RESET command received");
        status = HCI_ResetCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_RESET failed with status %02x", status);
        }
        // transport_notify_cmd_complete(opcode);
        break;
    case HCI_OPCODE_HCI_READ_LOCAL_SUPPORTED_FEATURES:
        // No. 37, byte 4, bit 6 = LE Supported (Controller)
        LOG("HCI_READ_LOCAL_SUPPORTED_FEATURES command received");

        /*
        bad:

        [1707] transport_send_packet: >>>>> transport_send_packet: packet_type 01, size 3
        [1726] controller_handle_hci_command: HCI_READ_LOCAL_SUPPORTED_FEATURES command received
        [1747] pplus_ble_recv_msg: PATCH MSG RX!!!! destination_task C0, msg_ptr 1FFF08EC
        [1767] pplus_ble_recv_msg: <<<<< pplus_ble_recv_msg: packet type 04
        DUMP DUMP DUMP dump pkt: len 14
        0E,0C,54,03,10,00,00,00,00,00,60,00,00,00,
        hciCtrlCmdToken 00
        [1807] packet_handler: ++++++++++++++++++++ packet_handler called with packet_type 0x04, size 14

        good:
        [1757] transport_send_packet: >>>>> transport_send_packet: packet_type 01, size 3
        [1777] controller_handle_hci_command: HCI_READ_LOCAL_SUPPORTED_FEATURES command received
        DUMP DUMP DUMP dump pkt: len 14
        0E,0C,01,03,10,00,00,00,00,00,40,00,00,00,
        hciCtrlCmdToken 54
        [1824] transport_process:  -> deliver command complete event e, size 14
        [1842] packet_handler: ++++++++++++++++++++ packet_handler called with packet_type 0x04, size 14


        bad2:
        [1775] controller_handle_hci_command: HCI_READ_LOCAL_SUPPORTED_FEATURES command received
        [1796] pplus_ble_recv_msg: PATCH MSG RX!!!! destination_task C0, msg_ptr 1FFF08EC
        [1816] pplus_ble_recv_msg: <<<<< pplus_ble_recv_msg: packet type 04
        DUMP DUMP DUMP dump pkt: len 14
        0E,0C,01,03,10,00,00,00,00,00,60,00,00,00,
        hciCtrlCmdToken 00
        [1856] packet_handler: ++++++++++++++++++++ packet_handler called with packet_type 0x04, size 14
        */

        status = HCI_ReadLocalSupportedFeaturesCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_READ_LOCAL_SUPPORTED_FEATURES failed with status %02x", status);
        }
        // send_command_complete(opcode, 0, local_supported_features, 8);
        //  send_command_complete(opcode, 0, local_supported_features, 8);
        break;
    case HCI_OPCODE_HCI_LE_READ_BUFFER_SIZE:
        LOG("HCI_LE_READ_BUFFER_SIZE command received");
        status = HCI_LE_ReadBufSizeCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_READ_BUFFER_SIZE failed with status %02x", status);
        }
        // send_command_complete(opcode, 0, read_buffer_size_result, 8);
        break;
    case HCI_OPCODE_HCI_LE_SET_ADVERTISING_PARAMETERS:
        LOG("HCI_LE_SET_ADVERTISING_PARAMETERS command received");
        status = HCI_LE_SetAdvParamCmd(
            little_endian_read_16(packet, 3),
            little_endian_read_16(packet, 5),
            packet[7],
            packet[8],
            packet[9],
            &packet[10],
            packet[16],
            packet[17]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_ADVERTISING_PARAMETERS failed with status %02x", status);
        }
        // send_command_complete(opcode, status, NULL, 0);*/
        break;
    case HCI_OPCODE_HCI_LE_SET_ADVERTISING_DATA:
        LOG("HCI_LE_SET_ADVERTISING_DATA command received");
        status = HCI_LE_SetAdvDataCmd(packet[3], &packet[4]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_ADVERTISING_DATA failed with status %02x", status);
        }
        // send_command_complete(opcode, status, NULL, 0);
        break;
    case HCI_OPCODE_HCI_LE_SET_ADVERTISE_ENABLE:
        LOG("HCI_LE_SET_ADVERTISE_ENABLE command received");
        status = HCI_LE_SetAdvEnableCmd(packet[3]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_ADVERTISE_ENABLE failed with status %02x", status);
        }
        // status = ll_set_advertise_enable(packet[3]);
        // send_command_complete(opcode, status, NULL, 0);
        break;
    case HCI_OPCODE_HCI_LE_SET_SCAN_ENABLE:
        LOG("HCI_LE_SET_SCAN_ENABLE command received, scanEnable = %d, filterDuplicates = %d",
            packet[3], packet[4]);
        status = HCI_LE_SetScanEnableCmd(packet[3], packet[4]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_SCAN_ENABLE failed with status %02x", status);
        }
        // ll_set_scan_enable(packet[3], packet[4]);
        // fake_command_complete(opcode);
        break;
    case HCI_OPCODE_HCI_LE_SET_SCAN_PARAMETERS:
        LOG("HCI_LE_SET_SCAN_PARAMETERS command received");
        status = HCI_LE_SetScanParamCmd(
            packet[3],
            little_endian_read_16(packet, 4),
            little_endian_read_16(packet, 6),
            packet[8],
            packet[9]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_SCAN_PARAMETERS failed with status %02x", status);
        }
        break;
    case HCI_OPCODE_HCI_READ_LOCAL_VERSION_INFORMATION:
        LOG("HCI_READ_LOCAL_VERSION_INFORMATION command received");
        status = HCI_ReadLocalVersionInfoCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_READ_LOCAL_VERSION_INFORMATION failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_READ_LOCAL_SUPPORTED_COMMANDS:
        LOG("HCI_READ_LOCAL_SUPPORTED_COMMANDS command received");
        status = HCI_ReadLocalSupportedCommandsCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_READ_LOCAL_SUPPORTED_COMMANDS failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_READ_BD_ADDR:
        LOG("HCI_READ_BD_ADDR command received");
        status = HCI_ReadBDADDRCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_READ_BD_ADDR failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_SET_EVENT_MASK:
        LOG("HCI_SET_EVENT_MASK command received");
        // set event mask
        status = HCI_SetEventMaskCmd(&packet[3]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_SET_EVENT_MASK failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_SET_EVENT_MASK:
        LOG("HCI_LE_SET_EVENT_MASK command received");
        // set LE event mask
        status = HCI_LE_SetEventMaskCmd(&packet[3]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_EVENT_MASK failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_RAND:
        LOG("HCI_LE_RAND command received");
        status = HCI_LE_RandCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_RAND failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_READ_WHITE_LIST_SIZE:
        LOG("HCI_LE_READ_WHITE_LIST_SIZE command received");
        status = HCI_LE_ReadWhiteListSizeCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_READ_WHITE_LIST_SIZE failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_CREATE_CONNECTION:
        LOG("HCI_LE_CREATE_CONNECTION command received");
        status = HCI_LE_CreateConnCmd(
            little_endian_read_16(packet, 3),   // scan interval
            little_endian_read_16(packet, 5),   // scan window
            packet[7],                          // initiator filter policy
            packet[8],                          // peer address type
            &packet[9],                         // peer address
            packet[15],                         // own address type
            little_endian_read_16(packet, 16),  // conn interval min
            little_endian_read_16(packet, 18),  // conn interval max
            little_endian_read_16(packet, 20),  // conn latency
            little_endian_read_16(packet, 22),  // supervision timeout
            little_endian_read_16(packet, 24),  // minimum ce length
            little_endian_read_16(packet, 26)); // maximum ce length
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_CREATE_CONNECTION failed with status %02x", status);
        }
        break;

    default:
        LOG("!!!!!!!!!!!!!!!! CMD opcode %02x not handled yet !!!!!!!!!!!!!!!! ", opcode);
        // try with "OK"
        fake_command_complete(opcode);
        break;
    }
}

/**
 * send packet
 */
static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{

    uint16_t connHandle;
    uint16_t param;
    uint8_t pbFlag;
    uint16_t pktLen;
    uint8_t *acldata = (uint8_t *)packet;
    uint8_t *send_buf = NULL;

    // TL_CmdPacket_t *ble_cmd_buff = &BleCmdBuffer;
    LOG(">>>>> transport_send_packet: packet_type %02X, size %d", packet_type, size);
    switch (packet_type)
    {
    case HCI_COMMAND_DATA_PACKET:
        // ble_cmd_buff->cmdserial.type = packet_type;
        // ble_cmd_buff->cmdserial.cmd.plen = size;
        // memcpy((void *)&ble_cmd_buff->cmdserial.cmd, packet, size);
        // TL_BLE_SendCmd(NULL, 0);
        controller_handle_hci_command(packet, size);
        send_transport_sent = true;
        break;

    case HCI_ACL_DATA_PACKET:
        param = BUILD_UINT16(acldata[0], acldata[1]);
        connHandle = param & 0xfff;
        pbFlag = (param & 0x3000) >> 12;
        pktLen = BUILD_UINT16(acldata[2], acldata[3]);

        send_buf = (uint8_t *)HCI_bm_alloc(pktLen);

        if (!send_buf)
        {
            break;
        }

        memcpy(send_buf, &acldata[4], pktLen);

        int ret = LL_TxData(connHandle, send_buf, pktLen, pbFlag);

        LOG("LL_TxData returned %d", ret);

        osal_bm_free(send_buf);

        send_transport_sent = true;
        break;

    default:
        send_hardware_error = 0x01; // invalid HCI packet
        break;
    }
    return 0;
}

static const hci_transport_t transport = {
    "phy62xx-vhci",
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_packet,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
};

static const hci_transport_t *transport_get_instance(void)
{
    return &transport;
}

/*******************************************************************************
    Global Var
*/
extern volatile uint8_t g_rfPhyTpCal0;       //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1;       //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTpCal0_2Mbps; //** two point calibraion result0            **//
extern volatile uint8_t g_rfPhyTpCal1_2Mbps; //** two point calibraion result1            **//
extern volatile uint8_t g_rfPhyTxPower;      //** rf pa output power setting [0x00 0x1f]  **//
extern volatile uint8_t g_rfPhyPktFmt;       //** rf_phy pkt format config                **//
extern volatile uint32 g_rfPhyRxDcIQ;        //** rx dc offset cal result                 **//
extern volatile int8_t g_rfPhyFreqOffSet;

#define XTAL16M_CAP_SETTING(x) subWriteReg(0x4000f0bc, 4, 0, (0x1f & (x)))

#define XTAL16M_CURRENT_SETTING(x) subWriteReg(0x4000f0bc, 6, 5, (0x03 & (x)))
#define DIG_LDO_CURRENT_SETTING(x) subWriteReg(0x4000f014, 22, 21, (0x03 & (x)))

#define RF_PHY_LO_LDO_SETTING(x) subWriteReg(0x400300cc, 11, 10, (0x03 & (x)))
#define RF_PHY_PA_VTRIM_SETTING(x) subWriteReg(0x400300dc, 9, 7, (0x03 & (x)))
#define RF_PHY_LNA_LDO_SETTING(x) subWriteReg(0x400300dc, 6, 5, (0x03 & (x)))

#define PKT_FMT_ZIGBEE 0
#define PKT_FMT_BLE1M 1
#define PKT_FMT_BLE2M 2
#define PKT_FMT_BLR500K 3
#define PKT_FMT_BLR125K 4

#define RF_PHY_TX_POWER_EXTRA_MAX 0x3f
#define RF_PHY_TX_POWER_MAX 0x1f
#define RF_PHY_TX_POWER_MIN 0x00

#define RF_PHY_TX_POWER_5DBM 0x3f
#define RF_PHY_TX_POWER_0DBM 0x1f
#define RF_PHY_TX_POWER_N2DBM 0x0f
#define RF_PHY_TX_POWER_N5DBM 0x0a
#define RF_PHY_TX_POWER_N10DBM 0x04
#define RF_PHY_TX_POWER_N15DBM 0x02
#define RF_PHY_TX_POWER_N20DBM 0x01

#define RF_PHY_FREQ_FOFF_00KHZ 0
#define RF_PHY_FREQ_FOFF_20KHZ 5
#define RF_PHY_FREQ_FOFF_40KHZ 10
#define RF_PHY_FREQ_FOFF_60KHZ 15
#define RF_PHY_FREQ_FOFF_80KHZ 20
#define RF_PHY_FREQ_FOFF_100KHZ 25
#define RF_PHY_FREQ_FOFF_120KHZ 30
#define RF_PHY_FREQ_FOFF_140KHZ 35
#define RF_PHY_FREQ_FOFF_160KHZ 40
#define RF_PHY_FREQ_FOFF_180KHZ 45
#define RF_PHY_FREQ_FOFF_200KHZ 50
#define RF_PHY_FREQ_FOFF_N20KHZ -5
#define RF_PHY_FREQ_FOFF_N40KHZ -10
#define RF_PHY_FREQ_FOFF_N60KHZ -15
#define RF_PHY_FREQ_FOFF_N80KHZ -20
#define RF_PHY_FREQ_FOFF_N100KHZ -25
#define RF_PHY_FREQ_FOFF_N120KHZ -30
#define RF_PHY_FREQ_FOFF_N140KHZ -35
#define RF_PHY_FREQ_FOFF_N160KHZ -40
#define RF_PHY_FREQ_FOFF_N180KHZ -45
#define RF_PHY_FREQ_FOFF_N200KHZ -50

#define RF_PHY_DTM_MANUL_NULL 0x00
#define RF_PHY_DTM_MANUL_FOFF 0x01
#define RF_PHY_DTM_MANUL_TXPOWER 0x02
#define RF_PHY_DTM_MANUL_XTAL_CAP 0x04
#define RF_PHY_DTM_MANUL_MAX_GAIN 0x08

#define RF_PHY_DTM_MANUL_ALL 0xFF

static void efuse_init(void)
{
    write_reg(0x4000f054, 0x0);
    write_reg(0x4000f140, 0x0);
    write_reg(0x4000f144, 0x0);
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
        efuse_init();
        // typedef void (*my_function)(void);
        // my_function pFunc = (my_function)(0xa2e1);
        //  ble_main();
        //  pFunc();
    }

    extern void hal_rom_boot_init(void);
    hal_rom_boot_init();
}
#include "ll_def.h"
extern llConnState_t *conn_param;
extern uint8 llState, llSecondaryState;
int phy6220_ll_info_show()
{
    LOG("ll_recv_ctrl_pkt_cnt        : %d", conn_param[0].pmCounter.ll_recv_ctrl_pkt_cnt);
    LOG("ll_recv_data_pkt_cnt        : %d", conn_param[0].pmCounter.ll_recv_data_pkt_cnt);
    LOG("ll_recv_invalid_pkt_cnt     : %d", conn_param[0].pmCounter.ll_recv_invalid_pkt_cnt);
    LOG("ll_recv_abnormal_cnt        : %d", conn_param[0].pmCounter.ll_recv_abnormal_cnt);
    LOG("ll_send_data_pkt_cnt        : %d", conn_param[0].pmCounter.ll_send_data_pkt_cnt);
    LOG("ll_conn_event_cnt           : %d", conn_param[0].pmCounter.ll_conn_event_cnt);
    LOG("ll_recv_crcerr_event_cnt    : %d", conn_param[0].pmCounter.ll_recv_crcerr_event_cnt);
    LOG("ll_conn_event_timeout_cnt   : %d", conn_param[0].pmCounter.ll_conn_event_timeout_cnt);
    LOG("ll_to_hci_pkt_cnt           : %d", conn_param[0].pmCounter.ll_to_hci_pkt_cnt);
    LOG("ll_hci_to_ll_pkt_cnt        : %d", conn_param[0].pmCounter.ll_hci_to_ll_pkt_cnt);
    LOG("ll_hci_buffer_alloc_err_cnt : %d", conn_param[0].pmCounter.ll_hci_buffer_alloc_err_cnt);
    LOG("ll_miss_master_evt_cnt      : %d", conn_param[0].pmCounter.ll_miss_master_evt_cnt);
    LOG("ll_miss_slave_evt_cnt       : %d", conn_param[0].pmCounter.ll_miss_slave_evt_cnt);
    LOG("ll_tbd_cnt1                 : %d", conn_param[0].pmCounter.ll_tbd_cnt1);
    LOG("ll_tbd_cnt2                 : %d", conn_param[0].pmCounter.ll_tbd_cnt2);
    LOG("ll_tbd_cnt3                 : %d", conn_param[0].pmCounter.ll_tbd_cnt3);
    LOG("ll_tbd_cnt4                 : %d", conn_param[0].pmCounter.ll_tbd_cnt4);

    LOG("ll_send_undirect_adv_cnt    : %d", g_pmCounters.ll_send_undirect_adv_cnt);
    LOG("ll_send_nonconn_adv_cnt     : %d", g_pmCounters.ll_send_nonconn_adv_cnt);
    LOG("ll_send_scan_adv_cnt        : %d", g_pmCounters.ll_send_scan_adv_cnt);

    LOG("ll_send_scan_rsp_cnt        : %d", g_pmCounters.ll_send_scan_rsp_cnt);
    LOG("ll_send_scan_req_cnt        : %d", g_pmCounters.ll_send_scan_req_cnt);
    LOG("ll_send_conn_rsp_cnt        : %d", g_pmCounters.ll_send_conn_rsp_cnt);
    LOG("ll_recv_adv_pkt_cnt         : %d", g_pmCounters.ll_recv_adv_pkt_cnt);
    LOG("ll_recv_conn_req_cnt        : %d", g_pmCounters.ll_recv_conn_req_cnt);
    LOG("ll_recv_scan_req_cnt        : %d", g_pmCounters.ll_recv_scan_req_cnt);
    LOG("ll_recv_scan_req_cnt        : %d", g_pmCounters.ll_recv_scan_req_cnt);
    LOG("ll_recv_scan_rsp_cnt        : %d", g_pmCounters.ll_recv_scan_rsp_cnt);
    LOG("ll_recv_scan_req_cnt        : %d", g_pmCounters.ll_recv_scan_req_cnt);
    LOG("ll_conn_adv_pending_cnt        : %d", g_pmCounters.ll_conn_adv_pending_cnt);
    LOG("ll_conn_scan_pending_cnt        : %d", g_pmCounters.ll_conn_scan_pending_cnt);

    LOG("llAdjBoffUpperLimitFailure  : %d", g_pmCounters.ll_tbd_cnt4);
    LOG("scanInfo.numSuccess         : %d", scanInfo.numSuccess);
    LOG("scanInfo.numFailure         : %d", scanInfo.numFailure);
    LOG("scanInfo.currentBackoff     : %d", scanInfo.currentBackoff);
    LOG("llState llSecStatae         : %d %d", llState, llSecondaryState);
    LOG("ll_trigger_err              : %d ", g_pmCounters.ll_trigger_err);
    return 0;
}

extern void init_config(void);
void port_thread(void *args)
{
    UNUSED(args);
    LOG("hi from bt");

#if (HOST_CONFIG & OBSERVER_CFG)
    extern void ll_patch_advscan(void);
#else
    extern void ll_patch_slave(void);
    ll_patch_slave();
#endif

    init_config();

    hal_rfphy_init();
    LOG("bt2");

    LOG("hciTaskID was %d, now %d", hciTaskID, 0xc0);
    hciTaskID = 0xc0;

    LOG("Patching OSAL_MSG_SEND, was %08X, now %08X",
        JUMP_FUNCTION(OSAL_MSG_SEND), (uint32_t)&pplus_ble_recv_msg);

    JUMP_FUNCTION(OSAL_MSG_SEND) = (uint32_t)&pplus_ble_recv_msg;

    JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE) = (uint32_t)&drv_disable_irq1;
    JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE) = (uint32_t)&drv_enable_irq1;
    LOG("hciCtrlCmdToken was %d, now 1", hciCtrlCmdToken);
    hciCtrlCmdToken = 1;

    osal_init_system();
    // osal_mem_init();

    phy6220_ll_info_show();

    NVIC_SetPriority((IRQn_Type)BB_IRQn, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn, IRQ_PRIO_HIGH); // ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn, IRQ_PRIO_HIGH); // OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM3_IRQn, IRQ_PRIO_HIGH); // OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn, IRQ_PRIO_HIGH); // LL_EXA_ADV

    // uncomment to enable packet logger
    // #define ENABLE_HCI_DUMP

    // config packet logger
#ifdef ENABLE_HCI_DUMP
#ifdef ENABLE_SEGGER_RTT
    // Disable sleep modes as shown here:
    // https://github.com/STMicroelectronics/STM32CubeWB/blob/master/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_HeartRate/Core/Src/app_debug.c#L180
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    // with this, RTT works in Skip mode but in Block mode
    hci_dump_init(hci_dump_segger_rtt_stdout_get_instance());
#else
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif
#endif
    LOG("a");
    /// GET STARTED with BTstack ///
    btstack_memory_init();
    LOG("b");

    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    LOG("c");

    const btstack_tlv_t *btstack_tlv_impl = btstack_tlv_none_init_instance();
    // setup global tlv
    btstack_tlv_set_instance(btstack_tlv_impl, NULL);

    // setup LE Device DB using TLV
    le_device_db_tlv_configure(btstack_tlv_impl, NULL);

    // init HCI
    hci_init(transport_get_instance(), NULL);
    LOG("d");

    const char *argv[] = {
        "main", "-a", "dd:44:00:12:1e:d9"};

    extern int btstack_main(int argc, const char *argv[]);
    btstack_main(3, argv);

    /*extern int btstack_main(void);
    btstack_main();*/
    LOG("f");

    log_info("btstack executing run loop...");
    btstack_run_loop_execute();
}
