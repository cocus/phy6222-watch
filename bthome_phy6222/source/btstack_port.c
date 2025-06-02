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

#include <log.h>
#include <jump_function.h>

/*******************************************
 *       transport implementation
 ******************************************/

#include <btstack.h>
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
#include "hci_dump_embedded_stdout.h"
#include "btstack_debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#undef HCI_H
#include <ble/include/hci.h>
#include <ble/hci/hci_tl.h>
#include <osal/osal_bufmgr.h>
#include <driver/uart/uart.h>

#include "osal_nuker.h"


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
        //LOG(" -> notify upper stack that it might be possible to send again");
        // notify upper stack that it might be possible to send again
        transport_emit_hci_event(&hci_event_transport_packet_sent);
    }
}

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

#define HCI_TASK_CUSTOM_ID 0xc0

uint8_t pplus_ble_recv_msg(uint8_t destination_task, uint8_t *msg_ptr)
{
    UNUSED(destination_task);
    hciPacket_t *hci_msg = (hciPacket_t *)msg_ptr;
    hcievtserial_t *evp = (hcievtserial_t *)hci_msg->pData;
    acldataserial_t *aclp = (hcievtserial_t *)hci_msg->pData;

    uint16_t size = 0;
    uint8_t *packet = NULL;

    LOG("<<<<< pplus_ble_recv_msg: event %d, status %d, packet type %02X, ptr %p",
        hci_msg->hdr.event, hci_msg->hdr.status, evp->type, msg_ptr);

    switch (evp->type)
    {
    case HCI_EVENT_PACKET:

        if (hci_outgoing_event_ready)
        {
            //LOG(" <<<<< pplus_ble_recv_msg: hci_outgoing_event_ready is true, dropping packet");
//            send_hardware_error = 0x01; // hardware error
            goto cleanup;
        }
        size = evp->evt.plen + 2;
        packet = (uint8_t *)&evp->evt;
        packet[2] = 1;

        /* Send buffer to upper stack */
        memcpy(&hci_outgoing_event[0], packet, size);
        hci_outgoing_event_ready = true;
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

cleanup:
    /*if (hci_msg->hdr.event == 3 || hci_msg->hdr.event == 1)
    {
        LOG("bm_free on %08x", *(void **)(msg_ptr + 12));
        osal_bm_free(*(void **)(msg_ptr + 12));
    }*/

    //uint8_t r = osal_msg_deallocate(msg_ptr);
    btstack_run_loop_poll_data_sources_from_irq();
    return PPlus_SUCCESS;
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
    if (send_transport_sent)
        return 0;
    switch (packet_type)
    {
    case HCI_COMMAND_DATA_PACKET:
        LOG("can send? hci_outgoing_event_ready = %d", hci_outgoing_event_ready);
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
        break;

    case HCI_OPCODE_HCI_READ_LOCAL_SUPPORTED_FEATURES:
        LOG("HCI_READ_LOCAL_SUPPORTED_FEATURES command received");
        status = HCI_ReadLocalSupportedFeaturesCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_READ_LOCAL_SUPPORTED_FEATURES failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_READ_BUFFER_SIZE:
        LOG("HCI_LE_READ_BUFFER_SIZE command received");
        status = HCI_LE_ReadBufSizeCmd();
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_READ_BUFFER_SIZE failed with status %02x", status);
        }
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
        break;

    case HCI_OPCODE_HCI_LE_SET_ADVERTISING_DATA:
        LOG("HCI_LE_SET_ADVERTISING_DATA command received");
        status = HCI_LE_SetAdvDataCmd(packet[3], &packet[4]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_ADVERTISING_DATA failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_SET_ADVERTISE_ENABLE:
        LOG("HCI_LE_SET_ADVERTISE_ENABLE command received");
        status = HCI_LE_SetAdvEnableCmd(packet[3]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_ADVERTISE_ENABLE failed with status %02x", status);
        }
        break;

    case HCI_OPCODE_HCI_LE_SET_SCAN_ENABLE:
        LOG("HCI_LE_SET_SCAN_ENABLE command received, scanEnable = %d, filterDuplicates = %d",
            packet[3], packet[4]);
        status = HCI_LE_SetScanEnableCmd(packet[3], packet[4]);
        if (status != HCI_SUCCESS)
        {
            LOG("HCI_LE_SET_SCAN_ENABLE failed with status %02x", status);
        }
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

    //LOG(">>>>> transport_send_packet: packet_type %02X, size %d", packet_type, size);
    switch (packet_type)
    {
    case HCI_COMMAND_DATA_PACKET:
        controller_handle_hci_command(packet, size);
        send_transport_sent = true;
        break;

    case HCI_ACL_DATA_PACKET:
        LOG(">>>>> transport_send_packet: packet_type %02X, size %d", packet_type, size);

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


void port_thread(void *args)
{
    UNUSED(args);
    LOG("hi from bt");

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
    /// GET STARTED with BTstack ///
    btstack_memory_init();

    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());

    const btstack_tlv_t *btstack_tlv_impl = btstack_tlv_none_init_instance();
    // setup global tlv
    btstack_tlv_set_instance(btstack_tlv_impl, NULL);

    // setup LE Device DB using TLV
    le_device_db_tlv_configure(btstack_tlv_impl, NULL);

    vTaskDelay(pdMS_TO_TICKS(200));
    // init HCI
    hci_init(transport_get_instance(), NULL);

    /*const char *argv[] = {
        "main", "-a", "dd:44:00:12:1e:d9"};

    extern int btstack_main(int argc, const char *argv[]);*/
    extern int btstack_main(void);
    btstack_main();

    /*
    btstack_main();*/

    log_info("btstack executing run loop...");
    btstack_run_loop_execute();
}
