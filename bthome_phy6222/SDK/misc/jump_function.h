/**
 ****************************************************************************************

    @file jump_fucntion.h

    @brief This file contains the definitions of the macros and functions that are
    architecture dependent.  The implementation of those is implemented in the
    appropriate architecture directory.


    $Rev:  $

 SDK_LICENSE

 ****************************************************************************************
*/


#ifndef _JUMP_FUNC_H_
#define _JUMP_FUNC_H_
#include <stdint.h>

// =====================  MACROS =======================
#if 0
#define JUMP_BASE_ADDR 0x1fff0000
#define JUMP_FUNCTION(x)    (*(uint32 *)(JUMP_BASE_ADDR + (x << 2)))
#else
extern const uint32_t* jump_table_base[];
#define JUMP_FUNCTION(x)    (*(uint32_t *)(jump_table_base + x))
#endif

// ROM function entries


// 0 - 10 for common
#define     OSAL_INIT_TASKS                      1
#define     TASKS_ARRAY                          2
#define     TASK_COUNT                           3
#define     TASK_EVENTS                          4
#define     OSAL_MEM_INIT                        5

#define     LL_INIT                              11
#define     LL_PROCESS_EVENT                     12
#define     LL_RESET                             13
#define     LL_TXDATA                            14
#define     LL_DISCONNECT                        15
#define     LL_SET_ADV_PARAM                     16
#define     LL_SET_ADV_DATA                      17
#define     LL_SET_ADV_CONTROL                   18
#define     LL_SET_DEFAULT_CONN_PARAM            19

#define     LL_EXT_SET_TX_POWER                  20

#define     LL_CLEAR_WHITE_LIST                  21
#define     LL_ADD_WHITE_LIST_DEV                22
#define     LL_REMOVE_WHITE_LIST_DEV             23
#define     LL_READ_WHITE_LIST_SIZE              24
#define     LL_NUM_EMPTY_WL_ENTRIES              25

#define     LL_SLAVE_EVT_ENDOK                   26
#define     LL_SETUP_NEXT_SLAVE_EVT              27
#define     LL_CHK_LSTO_DURING_SL                28
#define     LL_PROCESS_SLAVE_CTRL_PROC           29

#define     LL_PROCESS_SLAVE_CTRL_PKT            30
#define     LL_SLAVE_EVT_ABORT                   31
#define     LL_PROCESS_RX_DATA                   32
#define     LL_PROCESS_TX_DATA                   33
#define     LL_CONN_TERMINATE                    34
#define     LL_WRITE_TX_DATA                     35

#define     LL_EVT_SCHEDULE                      36
#define     LL_MOVE_TO_SLAVE_FUNCTION            37
#define     LL_SLAVE_CONN_EVENT                  38

#define     LL_SETUP_ADV                         39

#define     LL_SETUP_UNDIRECT_ADV                40
#define     LL_SETUP_NOCONN_ADV                  41

#define     LL_SETUP_SCAN_ADV                    42
#define     LL_SETUP_DIRECT_ADV                  43

#define     LL_CALC_TIMER_DRIFT                  44
#define     LL_GENERATE_TX_BUFFER                45
#define     LL_READ_RX_FIFO                      46
#define     LL_READ_TX_FIFO_RTLP                 47
#define     LL_READ_TX_FIFO_PKT                  48

#define     LL_HW_PROCESS_RTO                    49

#define     LL_HW_SET_TIMING                     50
#define     LL_RELEASE_CONN_ID                   51

#define     LL_READ_TX_PWR_LVL                   52   //  A1 ROM metal change add
#define     LL_READ_ADV_TX_PWR_LVL               53   //  A1 ROM metal change add
#define     LL_READ_RSSI                         54   //  A1 ROM metal change add
#define     LL_READ_REMOTE_USE_FEATURES          55   //  A1 ROM metal change add
#define     LL_ENCRYPT                           56   //  A1 ROM metal change add

#define     LL_DIRECT_TEST_END                   57   //  A1 ROM metal change add
#define     LL_DIRECT_TEST_TX_TEST               58   //  A1 ROM metal change add
#define     LL_DIRECT_TEST_RX_TEST               59   //  A1 ROM metal change add

#define     OSAL_POWER_CONSERVE                  60
#define     ENTER_SLEEP_PROCESS                  61
#define     WAKEUP_PROCESS                       62
#define     CONFIG_RTC                           63
#define     ENTER_SLEEP_OFF_MODE                 64   //  A1 ROM metal change add

#define     HAL_PROCESS_POLL                     65   //  A1 ROM metal change add
#define     LL_HW_GO                             66   //  A1 ROM metal change add
#define     LL_HW_TRIGGER                        67   //  A1 ROM metal change add
#define     LL_SET_TX_PWR_LVL                    68   //  A1 ROM metal change add

// LL AES
#define     LL_AES128_ENCRYPT                    70   //  A1 ROM metal change add
#define     LL_GEN_TRUE_RANDOM                   71   //  A1 ROM metal change add
#define     LL_GEN_DEVICE_SKD                    72   //  A1 ROM metal change add
#define     LL_GEN_DEVICE_IV                     73   //  A1 ROM metal change add
#define     LL_GENERATE_NOUNCE                   74   //  A1 ROM metal change add
#define     LL_ENC_ENCRYPT                       75   //  A1 ROM metal change add
#define     LL_ENC_DECRYPT                       76   //  A1 ROM metal change add

// host entries
#define     SMP_INIT                             80
#define     SMP_PROCESS_EVENT                    81

// l2cap entries
#define     L2CAP_PARSE_PACKET                   82
#define     L2CAP_ENCAP_PACKET                   83
#define     L2CAP_PKT_TO_SEGBUFF                 84
#define     L2CAP_SEGBUFF_TO_LINKLAYER           85
#define     L2CAP_PROCESS_FREGMENT_TX_DATA       86

//gap linkmgr entries
#define     GAP_LINK_MGR_PROCESS_CONNECT_EVT     87
#define     GAP_LINK_MGR_PROCESS_DISCONNECT_EVT  88

// hci tl
#define     HCI_INIT                             90   //  A1 ROM metal change add
#define     HCI_PROCESS_EVENT                    91   //  A1 ROM metal change add



// app entries
#define     APP_SLEEP_PROCESS                    100
#define     APP_WAKEUP_PROCESS                   101
#define     RF_INIT                              102
#define     WAKEUP_INIT                          103
#define     BOOT_INIT                            104
#define     DEBUG_PRINT                          105
#define     RF_CALIBRATTE                        106    //  A1 ROM metal change add
#define     RF_PHY_CHANGE                        107    //  A1 ROM metal change add

// LL master, A2 ROM metal change add
#define     LL_MASTER_EVT_ENDOK                  110
#define     LL_SETUP_NEXT_MASTER_EVT             111
#define     LL_PROCESS_MASTER_CTRL_PROC          112
#define     LL_PROCESS_MASTER_CTRL_PKT           113
#define     LL_MOVE_TO_MASTER_FUNCTION           114
#define     LL_MASTER_CONN_EVENT                 115

#define     LL_SET_SCAN_CTRL                     116
#define     LL_SET_SCAN_PARAM                    117

#define     LL_CREATE_CONN                       118
#define     LL_CREATE_CONN_CANCEL                119

#define     LL_START_ENCRYPT                     120

#define     LL_SETUP_SCAN                        121

#define     LL_SETUP_SEC_NOCONN_ADV              122
#define     LL_SETUP_SEC_SCAN                    123
#define     LL_SEC_ADV_ALLOW                     124
#define     LL_CALC_MAX_SCAN_TIME                125

// A2 multi-connection
#define     LL_SETUP_SEC_ADV_ENTRY               126
#define     LL_SETUP_SEC_CONN_ADV                127
#define     LL_SETUP_SEC_SCANNABLE_ADV           128




//DLE
#define     LL_SET_DATA_LENGTH                   130
#define     LL_PDU_LENGTH_UPDATE                 131
#define     LL_TRX_NUM_ADJUST                    132
//PHY UPDATE
#define     LL_SET_PHY_MODE                      133
#define     LL_PHY_MODE_UPDATE                   134
#define     LL_SET_NEXT_PHY_MODE                 135

#define     LL_ADP_ADJ_NEXT_TIME                 136
#define     LL_ADP_SMART_WINDOW                  137
#define     LL_SET_NEXT_DATA_CHN                 138
#define     LL_PLUS_DISABLE_LATENCY              139
#define     LL_PLUS_ENABLE_LATENCY               140

#define     LL_SETUP_EXT_ADV_EVENT               141
#define     LL_SETUP_PRD_ADV_EVENT               142
#define     LL_SETUP_ADV_EXT_IND_PDU             143
#define     LL_SETUP_AUX_ADV_IND_PDU             144
#define     LL_SETUP_AUX_SYNC_IND_PDU            145
#define     LL_SETUP_AUX_CHAIN_IND_PDU           146
#define     LL_SETUP_AUX_CONN_REQ_PDU            147
#define     LL_SETUP_AUX_CONN_RSP_PDU            148

#define     LL_SCHEDULER                         149
#define     LL_ADD_TASK                          150
#define     LL_DEL_TASK                          151

#define     LL_ADV_SCHEDULER                     152
#define     LL_ADV_ADD_TASK                      153
#define     LL_ADV_DEL_TASK                      154

#define     LL_ADV_SCHEDULER_PRD                 155
#define     LL_ADV_ADD_TASK_PRD                  156
#define     LL_ADV_DEL_TASK_PRD                  157

#define     LL_GET_NEXT_AUX_CHN                  158
#define     LL_SETUP_AUX_SCAN_RSP_PDU            159

#define     LL_PROCESSBASICIRQ_SRX               160
#define     LL_PROCESSBASICIRQ_SECADVTRX         161
#define     LL_PROCESSBASICIRQ_SCANTRX           162
#define     LL_PROCESSBASICIRQ_SECSCANSRX        163
#define     LL_PROCESSBASICIRQ_SECINITSRX        164

// 2020-02-13 Add for CTE
#define LL_CONNLESS_CTE_TX_PARAM                203
#define LL_CONNLESS_CTE_TX_ENABLE               204
#define LL_CONNLESS_IQ_SAMPLE_ENABLE            205
#define LL_CONN_CTE_RECV_PARAM                  206
#define LL_CONN_CTE_REQ_EN                      207
#define LL_CONN_CTE_TX_PARAM                    208
#define LL_CONN_CTE_RSP_EN                      209

//OSAL
#define     OSAL_SET_EVENT                       210
#define     OSAL_MSG_SEND                        211
#define     HAL_DRV_IRQ_INIT                     212
#define     HAL_DRV_IRQ_ENABLE                   213
#define     HAL_DRV_IRQ_DISABLE                  214

#define     HAL_WATCHDOG_INIT                    215

// interrupt request handler
#define     NMI_HANDLER                          219
#define     HARDFAULT_HANDLER                    220
#define     SVC_HANDLER                          221
#define     PENDSV_HANDLER                       222
#define     SYSTICK_HANDLER                      223

#define     V0_IRQ_HANDLER                       224
#define     V1_IRQ_HANDLER                       225
#define     V2_IRQ_HANDLER                       226
#define     V3_IRQ_HANDLER                       227
#define     V4_IRQ_HANDLER                       228
#define     V5_IRQ_HANDLER                       229
#define     V6_IRQ_HANDLER                       230
#define     V7_IRQ_HANDLER                       231
#define     V8_IRQ_HANDLER                       232
#define     V9_IRQ_HANDLER                       233
#define     V10_IRQ_HANDLER                      234
#define     V11_IRQ_HANDLER                      235
#define     V12_IRQ_HANDLER                      236
#define     V13_IRQ_HANDLER                      237
#define     V14_IRQ_HANDLER                      238
#define     V15_IRQ_HANDLER                      239
#define     V16_IRQ_HANDLER                      240
#define     V17_IRQ_HANDLER                      241
#define     V18_IRQ_HANDLER                      242
#define     V19_IRQ_HANDLER                      243
#define     V20_IRQ_HANDLER                      244
#define     V21_IRQ_HANDLER                      245
#define     V22_IRQ_HANDLER                      246
#define     V23_IRQ_HANDLER                      247
#define     V24_IRQ_HANDLER                      248
#define     V25_IRQ_HANDLER                      249
#define     V26_IRQ_HANDLER                      250
#define     V27_IRQ_HANDLER                      251
#define     V28_IRQ_HANDLER                      252
#define     V29_IRQ_HANDLER                      253
#define     V30_IRQ_HANDLER                      254
#define     V31_IRQ_HANDLER                      255


#endif // _JUMP_FUNC_H_
