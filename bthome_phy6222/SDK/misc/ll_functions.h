#ifndef LL_FUNCTIONS_H
#define LL_FUNCTIONS_H

#include <stdint.h>
#include "types.h"
#include "ll_def.h"
#include "ll_sleep.h"

#include "hci.h"
#include "l2cap.h"

// ================== FUNCTIONS  ==================================
void move_to_slave_function0(void);
void LL_slave_conn_event0(void);
llStatus_t llSetupAdv0(void);
void llSetupUndirectedAdvEvt0(void);
void llSetupNonConnectableAdvEvt0( void );
void llSetupScannableAdvEvt0( void );
void llSetupDirectedAdvEvt0( void );
void LL_evt_schedule0(void);

void llCalcTimerDrift0( uint32    connInterval,
                        uint16   slaveLatency,
                        uint8    sleepClkAccuracy,
                        uint32*   timerDrift ) ;

uint16 ll_generateTxBuffer0(int txFifo_vacancy, uint16* pSave_ptr);

void ll_hw_read_tfifo_rtlp0(void);

void ll_read_rxfifo0(void);

int ll_hw_read_tfifo_packet0(uint8* pkt);

void ll_hw_process_RTO0(uint32 ack_num);

void LL_set_default_conn_params0(llConnState_t* connPtr);

// =====
void enterSleepProcess0(uint32 time);

void wakeupProcess0(void);

void config_RTC0(uint32 time);

void enter_sleep_off_mode0(Sleep_Mode mode);

void llSlaveEvt_TaskEndOk0( void );

uint8 llSetupNextSlaveEvent0( void );

uint8 llCheckForLstoDuringSL0( llConnState_t* connPtr );

uint8 llProcessSlaveControlProcedures0( llConnState_t* connPtr );

void llProcessSlaveControlPacket0( llConnState_t* connPtr,
                                   uint8*         pBuf );

void llSlaveEvt_TaskAbort0(void );

// ------
void llMasterEvt_TaskEndOk0( void );
void llProcessMasterControlPacket0( llConnState_t* connPtr,
                                    uint8*         pBuf );
uint8 llProcessMasterControlProcedures0( llConnState_t* connPtr );
uint8 llSetupNextMasterEvent0( void );

void move_to_master_function0(void);
void LL_master_conn_event0(void);

llStatus_t LL_SetScanControl0( uint8 scanMode,
                               uint8 filterReports );
llStatus_t LL_SetScanParam0( uint8  scanType,
                             uint16 scanInterval,
                             uint16 scanWindow,
                             uint8  ownAddrType,
                             uint8  scanWlPolicy );

llStatus_t LL_CreateConn0( uint16 scanInterval,
                           uint16 scanWindow,
                           uint8  initWlPolicy,
                           uint8  peerAddrType,
                           uint8*  peerAddr,
                           uint8  ownAddrType,
                           uint16 connIntervalMin,
                           uint16 connIntervalMax,
                           uint16 connLatency,
                           uint16 connTimeout,
                           uint16 minLength,
                           uint16 maxLength );
llStatus_t LL_CreateConnCancel0( void );

llStatus_t LL_StartEncrypt0( uint16 connId,
                             uint8*  rand,
                             uint8*  eDiv,
                             uint8*  ltk );

void llSetupScan0( uint8 chan );

//  ================== ll.c
void LL_Init0( uint8 taskId );
uint16 LL_ProcessEvent0( uint8 task_id, uint16 events );
llStatus_t LL_Reset0( void );
llStatus_t LL_TxData0( uint16 connId, uint8* pBuf, uint8  pktLen, uint8  fragFlag );
llStatus_t LL_Disconnect0( uint16 connId, uint8  reason );
llStatus_t LL_SetAdvParam0( uint16 advIntervalMin,
                            uint16 advIntervalMax,
                            uint8  advEvtType,
                            uint8  ownAddrType,
                            uint8  directAddrType,
                            uint8*  directAddr,
                            uint8  advChanMap,
                            uint8  advWlPolicy );
llStatus_t LL_SetAdvData0( uint8  advDataLen, uint8* advData );
llStatus_t LL_SetAdvControl0( uint8 advMode );

llStatus_t LL_EXT_SetTxPower0( uint8 txPower, uint8* cmdComplete );

llStatus_t LL_ClearWhiteList0( void );
llStatus_t LL_AddWhiteListDevice0( uint8* devAddr, uint8 addrType );
llStatus_t LL_RemoveWhiteListDevice0( uint8* devAddr, uint8 addrType );
llStatus_t LL_ReadWlSize0( uint8* numEntries );
llStatus_t LL_ReadTxPowerLevel0( uint8 connId, uint8 type, int8*  txPower );
llStatus_t LL_SetTxPowerLevel0( int8  txPower );
llStatus_t LL_ReadAdvChanTxPower0( int8* txPower );
llStatus_t LL_ReadRssi0( uint16 connId, int8*   lastRssi );
llStatus_t LL_ReadRemoteUsedFeatures0( uint16 connId );
llStatus_t LL_Encrypt0( uint8* key, uint8* plaintextData, uint8* encryptedData );

llStatus_t LL_DirectTestEnd0( void );
llStatus_t LL_DirectTestTxTest0( uint8 txFreq, uint8 payloadLen, uint8 payloadType );
llStatus_t LL_DirectTestRxTest0( uint8 rxFreq );

// ================ ll_common.c
void llProcessTxData0( llConnState_t* connPtr, uint8 context );
uint8 llProcessRxData0( void );
uint8 llWriteTxData0( llConnState_t* connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8*         pBuf );
void llConnTerminate0( llConnState_t* connPtr, uint8 reason );
void llReleaseConnId0( llConnState_t* connPtr );

// ================ ll_enc.c
void LL_ENC_AES128_Encrypt0( uint8* key,
                             uint8* plaintext,
                             uint8* ciphertext );
uint8 LL_ENC_GenerateTrueRandNum0( uint8* buf,
                                   uint8 len );
void LL_ENC_GenDeviceSKD0( uint8* SKD );
void LL_ENC_GenDeviceIV0( uint8* IV );
void LL_ENC_GenerateNonce0( uint32 pktCnt,
                            uint8  direction,
                            uint8*  nonce );
void LL_ENC_Encrypt0( llConnState_t* connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8*         pBuf );
uint8 LL_ENC_Decrypt0( llConnState_t* connPtr,
                       uint8          pktHdr,
                       uint8          pktLen,
                       uint8*         pBuf );

// =================== osal
void osal_pwrmgr_powerconserve0( void ) ;

// =================== ll_hw_drv.c
void ll_hw_set_timing0(uint8 pktFmt);
void ll_hw_go0(void);
void ll_hw_trigger0(void);

// ================== SMP functions
void SM_Init0( uint8 task_id );
uint16 SM_ProcessEvent0( uint8 task_id, uint16 events );

// ================== HCI_TL functions
void HCI_Init0( uint8 task_id );
uint16 HCI_ProcessEvent0( uint8 task_id, uint16 events );


// ======= OSAL memory
void osal_mem_init0(void);

// =========== ROM -> APP function
void app_sleep_process(void);

void app_wakeup_process(void);

void rf_init(void);

void boot_init0(void);

void wakeup_init0(void);

void debug_print(uint32 state);

void rf_calibrate0(void);

void rf_phy_change_cfg(uint8 pktFmt);

// ========== A2, for conn-adv, conn-scan
uint8 llSetupSecNonConnectableAdvEvt0( void );
uint8 llSecAdvAllow0(void);
uint32 llCalcMaxScanTime0(void);
void llSetupSecScan0( uint8 chan );

uint8 llSetupSecAdvEvt0( void );
uint8 llSetupSecConnectableAdvEvt0( void );
uint8 llSetupSecScannableAdvEvt0( void );



//=============== gap_linkmgr.c
void gapProcessDisconnectCompleteEvt0( hciEvt_DisconnComplete_t* pPkt );
void gapProcessConnectionCompleteEvt0( hciEvt_BLEConnComplete_t* pPkt );


//=============== l2cap_util.c
uint8 l2capParsePacket0( l2capPacket_t* pPkt, hciDataEvent_t* pHciMsg );
uint8 l2capEncapSendData0( uint16 connHandle, l2capPacket_t* pPkt );
uint8 l2capPktToSegmentBuff0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf, uint8 blen,uint8* pBuf);
void l2capPocessFragmentTxData0(uint16 connHandle);
uint8 l2capSegmentBuffToLinkLayer0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf);
void l2capPocessFragmentTxData0(uint16 connHandle);

//=============== DLE
llStatus_t LL_SetDataLengh0( uint16 connId,uint16 TxOctets,uint16 TxTime );
void llPduLengthUpdate0(uint16 connHandle);
void llTrxNumAdaptiveConfig0(void);

//===============LL ADJ WINDOW
void ll_adptive_adj_next_time0(uint32 nextTime);
void ll_adptive_smart_window0(uint32 irq_status,uint32 anchor_point);
void llSetNextDataChan0( llConnState_t* connPtr );

//=============== PHY UPDATE
llStatus_t LL_SetPhyMode0( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions);
llStatus_t LL_PhyUpdate0( uint16 connId );
void llSetNextPhyMode0( llConnState_t* connPtr );

llStatus_t LL_PLUS_DisableSlaveLatency0(uint8 connId);
llStatus_t LL_PLUS_EnableSlaveLatency0(uint8 connId);

// ================= BBB
void ll_scheduler0(uint32 time);
void ll_addTask0(uint8 connId, uint32 time);
void ll_deleteTask0(uint8 connId);

void ll_adv_scheduler0(void);
void ll_add_adv_task0(extAdvInfo_t* pExtAdv);
void ll_delete_adv_task0(uint8 index);

void ll_adv_scheduler_periodic0(void);
void ll_add_adv_task_periodic0(periodicAdvInfo_t* pPrdAdv, extAdvInfo_t* pExtAdv);
void ll_delete_adv_task_periodic0(uint8 index);
uint8 llSetupExtAdvEvent0(extAdvInfo_t*  pAdvInfo);
uint8 llSetupPrdAdvEvent0(periodicAdvInfo_t* pPrdAdv, extAdvInfo_t* pExtAdv);

void llSetupAdvExtIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxAdvIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxChainIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxSyncIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxConnectReqPDU0(void);
void llSetupAuxScanRspPDU0(extAdvInfo_t*  pAdvInfo);
void llSetupAuxConnectRspPDU0(extAdvInfo_t*  pAdvInfo);

uint8  llGetNextAuxAdvChn0(uint8 current);


//=============== OSAL
uint8 osal_set_event0( uint8 task_id, uint16 event_flag );
uint8 osal_msg_send0( uint8 destination_task, uint8* msg_ptr );

//=============== _HAL_IRQ_
void drv_irq_init0(void);
int drv_enable_irq0(void);
int drv_disable_irq0(void);

// 2020-02-13 cte jumpfunction
llStatus_t LL_ConnectionlessCTE_TransmitParam0(              uint8 advertising_handle,
                                                             uint8 len,
                                                             uint8 type,
                                                             uint8 count,
                                                             uint8 Pattern_LEN,
                                                             uint8* AnaIDs);

llStatus_t LL_ConnectionlessCTE_TransmitEnable0(             uint8 advertising_handle,uint8 enable);
llStatus_t LL_ConnectionlessIQ_SampleEnable0(             uint16 sync_handle,
                                                          uint8 enable,
                                                          uint8 slot_Duration,
                                                          uint8 MaxSampledCTEs,
                                                          uint8 pattern_len,
                                                          uint8* AnaIDs);
llStatus_t LL_Set_ConnectionCTE_ReceiveParam0(                uint16 connHandle,
                                                              uint8 enable,
                                                              uint8 slot_Duration,
                                                              uint8 pattern_len,
                                                              uint8* AnaIDs);
llStatus_t LL_Connection_CTE_Request_Enable0(               uint16 connHandle,
                                                            uint8 enable,
                                                            uint16 Interval,
                                                            uint8 len,
                                                            uint8 type);

llStatus_t LL_Set_ConnectionCTE_TransmitParam0(              uint16 connHandle,
                                                             uint8 type,
                                                             uint8 pattern_len,
                                                             uint8* AnaIDs);

llStatus_t LL_Connection_CTE_Response_Enable0(             uint16 connHandle,uint8 enable);

#endif /* LL_FUNCTIONS_H */
