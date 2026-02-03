/*! *********************************************************************************
* Copyright (c) 2014 - 2025, Freescale Semiconductor, Inc.
* Copyright 2016-2026 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */
#include "SMAC.h"
#include "PhyInterface.h"
#include "AspInterface.h"
#include "EmbeddedTypes.h"

#include "SMAC_Config.h"
#include "fsl_component_mem_manager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"

#if (defined(FSL_FEATURE_SOC_LTC_COUNT) && (FSL_FEATURE_SOC_LTC_COUNT > 0))
#include "fsl_ltc.h"
#endif

/* -------------------------------------------------------------------------- */
/*                               Private macros                               */
/* -------------------------------------------------------------------------- */

#if gSmacUseSecurity_c
#define ENC_BLOCK_SIZE (16)
#endif


/* -------------------------------------------------------------------------- */
/*                               Private memory                               */
/* -------------------------------------------------------------------------- */

static const char* mSmacVersionString = (char*)gSmacVerString_c;

RegisterModuleInfo(SMAC, /* DO NOT MODIFY */
                   mSmacVersionString, /* DO NOT MODIFY */
                   gSmacModuleId_c, /* DO NOT MODIFY, EDIT in SMAC.h */
                   gSmacVerMajor_c, /* DO NOT MODIFY, EDIT in SMAC.h */
                   gSmacVerMinor_c, /* DO NOT MODIFY, EDIT in SMAC.h */
                   gSmacVerPatch_c, /* DO NOT MODIFY, EDIT in SMAC.h */
                   gSmacBuildNo_c); /* DO NOT MODIFY, EDIT in SMAC.h */

static smacInternalAttrib_t maSmacAttributes[gSmacMaxPan_c];

static smacMultiPanInstances_t mSmacActivePan;

static uint8_t mSmacInitialized;

#if gSmacUseSecurity_c

static const uint8_t TEST_KEY[ENC_BLOCK_SIZE] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

static const uint8_t TEST_IV[ENC_BLOCK_SIZE]  = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

#if (defined(FSL_FEATURE_SOC_LTC_COUNT) && (FSL_FEATURE_SOC_LTC_COUNT > 0))

static const uint8_t TEST_DKEY[ENC_BLOCK_SIZE] = {
    0xd0, 0x14, 0xf9, 0xa8, 0xc9, 0xee, 0x25, 0x89,
    0xe1, 0x3f, 0x0c, 0xc8, 0xb6, 0x63, 0x0c, 0xa6
};

#endif /* FSL_FEATURE_SOC_LTC_COUNT */

#endif /* gSmacUseSecurity_c */

/* -------------------------------------------------------------------------- */
/*                               Public memory                                */
/* -------------------------------------------------------------------------- */

uint8_t gTotalChannels;

/* -------------------------------------------------------------------------- */
/*                              Private functions                             */
/* -------------------------------------------------------------------------- */

#if gSmacUseSecurity_c

static void SMAC_SetIVKey(uint8_t* KEY, uint8_t* IV)
{
    FLib_MemCpy(maSmacAttributes[mSmacActivePan].secInit.KEY, KEY, ENC_BLOCK_SIZE);
    FLib_MemCpy(maSmacAttributes[mSmacActivePan].secInit.IV, IV, ENC_BLOCK_SIZE);

#if (defined(FSL_FEATURE_SOC_LTC_COUNT) && (FSL_FEATURE_SOC_LTC_COUNT > 0))
    // This API is not working on K4W1 FPGA, see KFOURWONE-311 jira ticket
    //LTC_AES_GenerateDecryptKey(LTC0, maSmacAttributes[mSmacActivePan].secInit.KEY, maSmacAttributes[mSmacActivePan].secInit.DKEY, ENC_BLOCK_SIZE);
    FLib_MemCpy(maSmacAttributes[mSmacActivePan].secInit.DKEY, (uint8_t*)TEST_DKEY, ENC_BLOCK_SIZE);
#endif
}

static void SMAC_Encrypt(uint8_t* pIn, uint8_t* pOut, uint8_t *len, smacMultiPanInstances_t panID)
{
    *len = AES_128_CBC_Encrypt_And_Pad(pIn, (uint32_t)(*len),
                                     maSmacAttributes[panID].secInit.IV,
                                     maSmacAttributes[panID].secInit.KEY,
                                     pOut);
}

static void SMAC_Decrypt(uint8_t* pIn, uint8_t* pOut, uint8_t *len, smacMultiPanInstances_t panID)
{
#if (defined(FSL_FEATURE_SOC_LTC_COUNT) && (FSL_FEATURE_SOC_LTC_COUNT > 0))
    *len = AES_128_CBC_Decrypt_And_Depad(pIn, (uint32_t)(*len),
                                       maSmacAttributes[panID].secInit.IV,
                                       maSmacAttributes[panID].secInit.DKEY,
                                       pOut);
#else
    *len = AES_128_CBC_Decrypt_And_Depad(pIn, (uint32_t)(*len),
                                       maSmacAttributes[panID].secInit.IV,
                                       maSmacAttributes[panID].secInit.KEY,
                                       pOut);
#endif
}

#endif /* gSmacUseSecurity_c */

static void BackoffTimeElapsed(void* param)
{
    uint32_t lsmacInstance = (uint32_t)param;

    uint8_t u8PhyRes = MAC_PD_SapHandler(maSmacAttributes[lsmacInstance].gSmacDataMessage, 0);
    if(u8PhyRes != gPhySuccess_c)
    {
        OSA_InterruptDisable();
        maSmacAttributes[lsmacInstance].smacState = mSmacStateIdle_c;
        OSA_InterruptEnable();

        MSG_Free(maSmacAttributes[lsmacInstance].gSmacDataMessage);
        maSmacAttributes[lsmacInstance].gSmacDataMessage = NULL;
    }
}

static bool_t SMACPacketCheck(pdDataToMacMessage_t* pMsgFromPhy, smacMultiPanInstances_t instance)
{
    //check if packet is of type Data
    if((pMsgFromPhy->msgData.dataInd.pPsdu[0] & 0x07) != 0x01)
    {
        return FALSE;
    }

    //check if PSDU length is at least of SMAC header size.
    if(pMsgFromPhy->msgData.dataInd.psduLength < gSmacHeaderBytes_c)
    {
        return FALSE;
    }

    //check if PSDU length is greater than the maximum configured SMAC packet size.
    if(pMsgFromPhy->msgData.dataInd.psduLength > (maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->u8MaxDataLength + gSmacHeaderBytes_c))
    {
        return FALSE;
    }

    return TRUE;
}

static phyStatus_t PD_SMAC_SapHandler(void* pMsg, instanceId_t instance)
{
    phyStatus_t status = gPhySuccess_c;
    smacToAppDataMessage_t* pSmacMsg;
    smacMultiPanInstances_t lSmacInstanceBackup;
    pdDataToMacMessage_t* pDataMsg = (pdDataToMacMessage_t*)pMsg;

    switch(pDataMsg->msgType)
    {
    case gPdDataCnf_c:
        if(NULL == maSmacAttributes[instance].gSmacDataMessage)
        {
            status = gPhySuccess_c;
        }
        else
        {
            MSG_Free(maSmacAttributes[instance].gSmacDataMessage);
            maSmacAttributes[instance].gSmacDataMessage = NULL;

            pSmacMsg = MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
            if(pSmacMsg == NULL)
            {
                status = gPhySuccess_c;
            }
            else
            {
                pSmacMsg->msgType = gMcpsDataCnf_c;
                pSmacMsg->msgData.dataCnf.status = gErrorNoError_c;
                maSmacAttributes[instance].gSMAC_APP_MCPS_SapHandler(pSmacMsg, instance);
            }

            OSA_InterruptDisable();
            maSmacAttributes[instance].smacState = mSmacStateIdle_c;
            OSA_InterruptEnable();
        }
        break;

    case gPdDataInd_c:
        if(FALSE == SMACPacketCheck(pDataMsg, (smacMultiPanInstances_t)instance))
        {
            //if timeout is asked and packet fails the check, send message with abort status
            if(maSmacAttributes[instance].mSmacTimeoutAsked)
            {
                pSmacMsg = MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
                pSmacMsg->msgType = gMcpsDataInd_c;
                pSmacMsg->msgData.dataInd.pRxPacket = maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer;
                pSmacMsg->msgData.dataInd.pRxPacket->rxStatus = rxAbortedStatus_c;
                maSmacAttributes[instance].gSMAC_APP_MCPS_SapHandler(pSmacMsg, instance);

                OSA_InterruptDisable();
                maSmacAttributes[instance].smacState = mSmacStateIdle_c;
                OSA_InterruptEnable();
            }

            status = gPhySuccess_c;
        }
        else
        {
            maSmacAttributes[instance].smacLastDataRxParams.linkQuality = ((pdDataToMacMessage_t*)pMsg)->msgData.dataInd.ppduLinkQuality;
            maSmacAttributes[instance].smacLastDataRxParams.timeStamp   = (phyTime_t)((pdDataToMacMessage_t*)pMsg)->msgData.dataInd.timeStamp;
            maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->rxStatus = rxSuccessStatus_c;

#if gSmacUseSecurity_c
            uint8_t len = pDataMsg->msgData.dataInd.psduLength - gSmacHeaderBytes_c - gPhyFCSSize_c;
            SMAC_Decrypt(pDataMsg->msgData.dataInd.pPsdu + gSmacHeaderBytes_c, pDataMsg->msgData.dataInd.pPsdu + gSmacHeaderBytes_c,
                       &len,
                       (smacMultiPanInstances_t)instance);
            pDataMsg->msgData.dataInd.psduLength = len + gSmacHeaderBytes_c + gPhyFCSSize_c;
#endif

            // in case no timeout was asked we need to unset RXOnWhenIdle Pib.
            if(!maSmacAttributes[instance].mSmacTimeoutAsked)
            {
                lSmacInstanceBackup = mSmacActivePan;
                MLMESetActivePan((smacMultiPanInstances_t)instance);
                (void)MLMERXDisableRequest();
                MLMESetActivePan(lSmacInstanceBackup);
            }

            maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->u8DataLength 
                = pDataMsg->msgData.dataInd.psduLength - gSmacHeaderBytes_c - gPhyFCSSize_c;

            FLib_MemCpy(&maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->smacHeader,
                      ((smacHeader_t*)pDataMsg->msgData.dataInd.pPsdu),
                      gSmacHeaderBytes_c);
            FLib_MemCpy(&maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->smacPdu,
                      ((smacPdu_t*)(pDataMsg->msgData.dataInd.pPsdu + gSmacHeaderBytes_c)),
                      maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->u8DataLength);

            pSmacMsg = MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
            if(pSmacMsg == NULL)
            {
                status = gPhySuccess_c;
            }
            else
            {
                pSmacMsg->msgType = gMcpsDataInd_c;
                pSmacMsg->msgData.dataInd.pRxPacket = maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer;
#if gMpmMaxPANs_c == 2
                pSmacMsg->msgData.dataInd.pRxPacket->instanceId = (smacMultiPanInstances_t)instance;
#else
                pSmacMsg->msgData.dataInd.pRxPacket->instanceId = (smacMultiPanInstances_t)0;
#endif
                pSmacMsg->msgData.dataInd.u8LastRxRssi = ((pdDataToMacMessage_t *)pMsg)->msgData.dataInd.ppduRssi;
                maSmacAttributes[instance].gSMAC_APP_MCPS_SapHandler(pSmacMsg, instance);
            }

            OSA_InterruptDisable();
            maSmacAttributes[instance].smacState = mSmacStateIdle_c;
            OSA_InterruptEnable();
        }
        break;

    default:
        break;
    }

    MSG_Free(pMsg);
    return status;
}

static phyStatus_t PLME_SMAC_SapHandler(void* pMsg, instanceId_t instance)
{
    uint32_t backOffTime = 0;
    smacMultiPanInstances_t lSmacInstanceBackup;
    smacToAppMlmeMessage_t* pSmacToApp;
    smacToAppDataMessage_t* pSmacMsg;
    plmeToMacMessage_t* pPlmeMsg = (plmeToMacMessage_t*)pMsg;

    if (maSmacAttributes[instance].gSmacMlmeMessage)
    {
        MSG_Free(maSmacAttributes[instance].gSmacMlmeMessage);
        maSmacAttributes[instance].gSmacMlmeMessage = NULL;
    }

    switch(pPlmeMsg->msgType)
    {
    case gPlmeCcaCnf_c:
        if(pPlmeMsg->msgData.ccaCnf.status == gPhyChannelBusy_c && maSmacAttributes[instance].smacState == mSmacStateTransmitting_c)
        {
            if(maSmacAttributes[instance].txConfigurator.ccaBeforeTx)
            {
                if(maSmacAttributes[instance].txConfigurator.retryCountCCAFail > maSmacAttributes[instance].u8CCARetryCounter)
                {
                    maSmacAttributes[instance].u8CCARetryCounter++;
                    HAL_RngGetData(&backOffTime, sizeof(backOffTime));
#if !defined(RW610N_BT_CM3_SERIES)
                    /* TODO: MCRED-2 support timers on Redfinch */
                    TM_Start(maSmacAttributes[instance].u8BackoffTimerId, kTimerModeSingleShot, ((backOffTime & gMaxBackoffTime_c) + gMinBackoffTime_c));
#endif
                }
                else
                {
                    MSG_Free(maSmacAttributes[instance].gSmacDataMessage);
                    maSmacAttributes[instance].gSmacDataMessage = NULL;

                    //retries failed so create message for the application
                    pSmacMsg = MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
                    if(pSmacMsg != NULL)
                    {
                        //error type : Channel Busy
                        pSmacMsg->msgData.dataCnf.status = gErrorChannelBusy_c;
                        //type is Data Confirm
                        pSmacMsg->msgType = gMcpsDataCnf_c;
                        maSmacAttributes[instance].gSMAC_APP_MCPS_SapHandler(pSmacMsg, instance);
                    }

                    //place SMAC into idle state
                    OSA_InterruptDisable();
                    maSmacAttributes[instance].smacState = mSmacStateIdle_c;
                    OSA_InterruptEnable();
                }
            }

            MSG_Free(pMsg);
            return gPhySuccess_c;
        }

        // if SMAC isn't in TX then definitely it is a CCA confirm allocate a message for the application
        pSmacToApp = MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
        if(pSmacToApp != NULL)
        {
            pSmacToApp->msgType = gMlmeCcaCnf_c;

            //Channel status translated into SMAC messages: idle channel means no error.
            if(pPlmeMsg->msgData.ccaCnf.status == gPhyChannelIdle_c)
            {
                pSmacToApp->msgData.ccaCnf.status = gErrorNoError_c;
            }
            else
            {
                pSmacToApp->msgData.ccaCnf.status = gErrorChannelBusy_c;
            }
        }
        break;

    case gPlmeEdCnf_c:
        //allocate a message for the application
        pSmacToApp = MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
        if(pSmacToApp != NULL)
        {
            //message type is ED Confirm
            pSmacToApp->msgType = gMlmeEdCnf_c;
            if(pPlmeMsg->msgData.edCnf.status == gPhySuccess_c)
            {
                pSmacToApp->msgData.edCnf.status = gErrorNoError_c;
                pSmacToApp->msgData.edCnf.energyLevel = pPlmeMsg->msgData.edCnf.energyLevel;
                pSmacToApp->msgData.edCnf.energyLeveldB = pPlmeMsg->msgData.edCnf.energyLeveldB;

                lSmacInstanceBackup = mSmacActivePan;
                MLMESetActivePan((smacMultiPanInstances_t)instance);
                MLMESetActivePan(lSmacInstanceBackup);
            }
            else
            {
                pSmacToApp->msgData.edCnf.status = gErrorBusy_c;
            }
        }
        break;

    case gPlmeTimeoutInd_c:
    case gPlmeAbortInd_c:
        if(maSmacAttributes[instance].smacState == mSmacStateTransmitting_c)
        {
            if(maSmacAttributes[instance].txConfigurator.autoAck)
            {
                //re-arm retries for channel busy at retransmission.
                maSmacAttributes[instance].u8CCARetryCounter = 0;

                if(maSmacAttributes[instance].txConfigurator.retryCountAckFail > maSmacAttributes[instance].u8AckRetryCounter)
                {
                    maSmacAttributes[instance].u8AckRetryCounter++;

                    HAL_RngGetData(&backOffTime, sizeof(backOffTime));
                    //start event timer. After time elapses, Data request will be fired.
#if !defined(RW610N_BT_CM3_SERIES)
                    /* TODO: MCRED-2 support timers on Redfinch */
                    TM_Start(maSmacAttributes[instance].u8BackoffTimerId, kTimerModeSingleShot, ((backOffTime & gMaxBackoffTime_c) + gMinBackoffTime_c));
#endif
                }
                else
                {
                    (void)MSG_Free(maSmacAttributes[instance].gSmacDataMessage);
                    maSmacAttributes[instance].gSmacDataMessage = NULL;

                    //retries failed so create message for the application
                    pSmacMsg = MEM_BufferAlloc(sizeof(smacToAppDataMessage_t));
                    if(pSmacMsg != NULL)
                    {
                        //set error code: No Ack
                        pSmacMsg->msgData.dataCnf.status = gErrorNoAck_c;
                        //type is Data Confirm
                        pSmacMsg->msgType = gMcpsDataCnf_c;

                        maSmacAttributes[instance].gSMAC_APP_MCPS_SapHandler(pSmacMsg, instance);
                    }

                    OSA_InterruptDisable();
                    maSmacAttributes[instance].smacState = mSmacStateIdle_c;
                    OSA_InterruptEnable();
                }
            }

            MSG_Free(pMsg);
            return gPhySuccess_c;
        }

        //if no ack timeout was received then it is definitely a RX timeout
        pSmacToApp = MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
        if(pSmacToApp != NULL)
        {
            if(maSmacAttributes[instance].smacState == mSmacStateReceiving_c)
            {
                maSmacAttributes[instance].smacProccesPacketPtr.smacRxPacketPointer->rxStatus = rxTimeOutStatus_c;
            }
            pSmacToApp->msgType = gMlmeTimeoutInd_c;
        }
        break;

    case gPlme_UnexpectedRadioResetInd_c:
        pSmacToApp = MEM_BufferAlloc(sizeof(smacToAppMlmeMessage_t));
        if(pSmacToApp != NULL)
        {
            pSmacToApp->msgType = gMlme_UnexpectedRadioResetInd_c;
        }
        break;

    default:
        MSG_Free(pMsg);
        return gPhySuccess_c;
        break;
    }

    OSA_InterruptDisable();
    maSmacAttributes[instance].smacState = mSmacStateIdle_c;
    OSA_InterruptEnable();

    if(pSmacToApp != NULL)
    {
        maSmacAttributes[instance].gSMAC_APP_MLME_SapHandler(pSmacToApp, instance);
    }

    MSG_Free(pMsg);
    return gPhySuccess_c;
}

/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

void Smac_RegisterSapHandlers(SMAC_APP_MCPS_SapHandler_t pSMAC_APP_MCPS_SapHandler,
                              SMAC_APP_MLME_SapHandler_t pSMAC_APP_MLME_SapHandler,
                              instanceId_t smacInstanceId)
{
    maSmacAttributes[smacInstanceId].gSMAC_APP_MCPS_SapHandler = pSMAC_APP_MCPS_SapHandler;
    maSmacAttributes[smacInstanceId].gSMAC_APP_MLME_SapHandler = pSMAC_APP_MLME_SapHandler;
}

void InitSmac(void)
{
    uint32_t u32RandomNo = 0;

    HAL_RngInit();

#if gSmacUseSecurity_c
    SecLib_Init();
#endif

    gTotalChannels = 26;

#if(TRUE == smacInitializationValidation_d)
    mSmacInitialized = TRUE;
#endif

    mSmacActivePan = gSmacPan0_c;
    while(mSmacActivePan < gSmacMaxPan_c)
    {
        maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
        maSmacAttributes[mSmacActivePan].smacLastDataRxParams.linkQuality = 0;
        maSmacAttributes[mSmacActivePan].smacLastDataRxParams.timeStamp = 0;

        /*clear defer tx flag*/
        macToPlmeMessage_t lMsg;
        lMsg.ctx_id = mSmacActivePan;
        lMsg.msgType = gPlmeSetReq_c;
        lMsg.msgData.setReq.PibAttribute = gPhyPibDeferTxIfRxBusy_c;
        lMsg.msgData.setReq.PibAttributeValue = (uint64_t)FALSE;
        (void)MAC_PLME_SapHandler(&lMsg, 0);

        maSmacAttributes[mSmacActivePan].txConfigurator.autoAck = FALSE;
        maSmacAttributes[mSmacActivePan].txConfigurator.enhAck = FALSE;
        maSmacAttributes[mSmacActivePan].txConfigurator.ccaBeforeTx = FALSE;
        maSmacAttributes[mSmacActivePan].txConfigurator.retryCountAckFail = 0;
        maSmacAttributes[mSmacActivePan].txConfigurator.retryCountCCAFail = 0;
#if !defined(RW610N_BT_CM3_SERIES)
        /* TODO: MCRED-2 support timers on Redfinch */
        (void)TM_Open(maSmacAttributes[mSmacActivePan].u8BackoffTimerId);
        (void)TM_InstallCallback((timer_handle_t)maSmacAttributes[mSmacActivePan].u8BackoffTimerId, BackoffTimeElapsed, (void*)mSmacActivePan);
#endif

        (void)SMACSetShortSrcAddress(gNodeAddress_c);
        (void)SMACSetPanID(gDefaultPanID_c);

        HAL_RngGetData(&u32RandomNo, sizeof(u32RandomNo));
        maSmacAttributes[mSmacActivePan].u8SmacSeqNo = (uint8_t)u32RandomNo;

#if gSmacUseSecurity_c
        SMAC_SetIVKey((uint8_t*)TEST_KEY, (uint8_t*)TEST_IV);
#endif
        mSmacActivePan = (smacMultiPanInstances_t)(mSmacActivePan + 1);
    }
    mSmacActivePan = gSmacPan0_c;

    //Notify the PHY what function to call for communicating with SMAC
    Phy_RegisterSapHandlers((PD_MAC_SapHandler_t)PD_SMAC_SapHandler, (PLME_MAC_SapHandler_t)PLME_SMAC_SapHandler, 0);
}

smacErrors_t MCPSDataRequest (txPacket_t *psTxPacket)
{
    macToPdDataMessage_t *pMsg;
    phyStatus_t u8PhyRes = gPhySuccess_c;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

#if(TRUE == smacParametersValidation_d)
    if((NULL == psTxPacket) || (gMaxSmacSDULength_c < psTxPacket->u8DataLength))
    {
        return gErrorOutOfRange_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

#if !gSmacUseSecurity_c
    pMsg = MSG_Alloc(sizeof(macToPdDataMessage_t) +
                         psTxPacket->u8DataLength + gSmacHeaderBytes_c);
#else
    pMsg = MSG_Alloc(sizeof(macToPdDataMessage_t) +
                         psTxPacket->u8DataLength + gSmacHeaderBytes_c + ENC_BLOCK_SIZE);
#endif

    if(pMsg == NULL )
    {
        return gErrorNoResourcesAvailable_c;
    }

    maSmacAttributes[mSmacActivePan].u8SmacSeqNo++;
    maSmacAttributes[mSmacActivePan].u8AckRetryCounter = 0;
    maSmacAttributes[mSmacActivePan].u8CCARetryCounter = 0;

    /* Fill with Phy related data */
    pMsg->ctx_id = mSmacActivePan;
    pMsg->msgType = gPdDataReq_c;
    pMsg->msgData.dataReq.startTime = gPhySeqStartAsap_c;

    if(maSmacAttributes[mSmacActivePan].txConfigurator.autoAck &&
            psTxPacket->smacHeader.destAddr != 0xFFFF
#if !gEnhAckMode8
         && psTxPacket->smacHeader.panId != 0xFFFF
#endif
      )
    {
                                    //Turn@       +       phy payload(symbols)+ Turn@ + ACK
        pMsg->msgData.dataReq.txDuration = 12 + (gSmacHeaderBytes_c + psTxPacket->u8DataLength + 2)*2 + 12 + 42;

        if(maSmacAttributes[mSmacActivePan].txConfigurator.ccaBeforeTx)
        {
            pMsg->msgData.dataReq.txDuration += 0x08; //CCA Duration: 8 symbols
        }

#if gSmacUseSecurity_c
        pMsg->msgData.dataReq.txDuration += (ENC_BLOCK_SIZE - ((psTxPacket->u8DataLength - gSmacHeaderBytes_c
                                                            + ENC_BLOCK_SIZE) & (ENC_BLOCK_SIZE - 1))) * 2;
#endif
    }
    else
    {
        pMsg->msgData.dataReq.txDuration = 0xFFFFFFFF;
    }

    pMsg->msgData.dataReq.psduLength = psTxPacket->u8DataLength + gSmacHeaderBytes_c + gPhyFCSSize_c; // include FCS bytes in data psdu length
    pMsg->msgData.dataReq.pPsdu = (uint8_t*)pMsg + sizeof(macToPdDataMessage_t);

    FLib_MemCpy(pMsg->msgData.dataReq.pPsdu, &(psTxPacket->smacHeader), gSmacHeaderBytes_c);
    FLib_MemCpy(pMsg->msgData.dataReq.pPsdu + gSmacHeaderBytes_c, &(psTxPacket->smacPdu), psTxPacket->u8DataLength);

    if(maSmacAttributes[mSmacActivePan].txConfigurator.ccaBeforeTx)
    {
        pMsg->msgData.dataReq.CCABeforeTx = gPhyCCAMode1_c;
    }
    else
    {
        pMsg->msgData.dataReq.CCABeforeTx = gPhyNoCCABeforeTx_c;
    }

    if(maSmacAttributes[mSmacActivePan].txConfigurator.autoAck &&
            psTxPacket->smacHeader.destAddr != 0xFFFF
#if !gEnhAckMode8
         && psTxPacket->smacHeader.panId != 0xFFFF
#endif
      )
    {
    //set frame control option: ACK.
        pMsg->msgData.dataReq.pPsdu[0] |=   gFrameCtrlAckReqMsk_c;
        pMsg->msgData.dataReq.ackRequired = gPhyRxAckRqd_c;

        if (maSmacAttributes[mSmacActivePan].txConfigurator.enhAck) {
            // Set version 2
            pMsg->msgData.dataReq.pPsdu[1] |= (2 << 4);

#if gSmacUseExtendedAddr_c
#if !gEnhAckMode8
            // Set PAN ID compression to false
            pMsg->msgData.dataReq.pPsdu[0] &= 0xBF;
#endif
#endif
        }
    }
    else
    {
        pMsg->msgData.dataReq.ackRequired = gPhyNoAckRqd_c;
    }

#if gSmacUseSecurity_c
    uint8_t inputLen = pMsg->msgData.dataReq.psduLength - gSmacHeaderBytes_c - gPhyFCSSize_c;

    SMAC_Encrypt(pMsg->msgData.dataReq.pPsdu + gSmacHeaderBytes_c, pMsg->msgData.dataReq.pPsdu + gSmacHeaderBytes_c,
               &(inputLen),
               mSmacActivePan);
    pMsg->msgData.dataReq.psduLength = inputLen + gSmacHeaderBytes_c + gPhyFCSSize_c;
#endif

    pMsg->msgData.dataReq.pPsdu[2] = maSmacAttributes[mSmacActivePan].u8SmacSeqNo;

    maSmacAttributes[mSmacActivePan].gSmacDataMessage = pMsg;      //Store pointer for freeing later

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateTransmitting_c;
    OSA_InterruptEnable();

    u8PhyRes = MAC_PD_SapHandler(pMsg, 0);

    if(u8PhyRes == gPhySuccess_c)
    {
        return gErrorNoError_c;
    }

    MSG_Free(maSmacAttributes[mSmacActivePan].gSmacDataMessage);
    maSmacAttributes[mSmacActivePan].gSmacDataMessage = NULL;

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
    OSA_InterruptEnable();

    return gErrorNoResourcesAvailable_c;
}

void MLMETXDisableRequest(void)
{
    macToPlmeMessage_t lMsg;

    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgType     = gPlmeSetTRxStateReq_c;
    lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
    (void)MAC_PLME_SapHandler(&lMsg, 0);

    if(maSmacAttributes[mSmacActivePan].gSmacDataMessage != NULL)
    {
        (void)MSG_Free(maSmacAttributes[mSmacActivePan].gSmacDataMessage);
        maSmacAttributes[mSmacActivePan].gSmacDataMessage = NULL;
    }

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
    OSA_InterruptEnable();
}

smacErrors_t MLMERXEnableRequest(rxPacket_t *gsRxPacket, smacTime_t stTimeout)
{
    macToPlmeMessage_t lMsg;

#if(TRUE == smacParametersValidation_d)
#if gSmacUseSecurity_c
    if((NULL == gsRxPacket) || (gMaxSmacSDULength_c + 16 < gsRxPacket->u8MaxDataLength))
#else
    if((NULL == gsRxPacket) || (gMaxSmacSDULength_c < gsRxPacket->u8MaxDataLength))
#endif
    {
        return gErrorOutOfRange_c;
    }
#endif

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

    lMsg.ctx_id = mSmacActivePan;
    if(stTimeout)
    {
        lMsg.msgType = gPlmeSetTRxStateReq_c;
        lMsg.msgData.setTRxStateReq.startTime = gPhySeqStartAsap_c;
        lMsg.ctx_id = mSmacActivePan;
        lMsg.msgData.setTRxStateReq.state = gPhySetRxOn_c;
        lMsg.msgData.setTRxStateReq.rxDuration = stTimeout;
    }
    else
    {
        lMsg.msgType = gPlmeSetReq_c;
        lMsg.msgData.setReq.PibAttribute = gPhyPibRxOnWhenIdle;
        lMsg.msgData.setReq.PibAttributeValue = (uint64_t)1;
    }

    maSmacAttributes[mSmacActivePan].mSmacTimeoutAsked = (stTimeout > 0);

    gsRxPacket->rxStatus = rxProcessingReceptionStatus_c;
    maSmacAttributes[mSmacActivePan].smacProccesPacketPtr.smacRxPacketPointer = gsRxPacket;

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateReceiving_c;
    OSA_InterruptEnable();

    if (MAC_PLME_SapHandler(&lMsg, 0) == gPhySuccess_c)
    {
        return gErrorNoError_c;
    }

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
    OSA_InterruptEnable();

    return gErrorNoResourcesAvailable_c;
}

smacErrors_t MLMERXDisableRequest(void)
{
    macToPlmeMessage_t lMsg;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if((mSmacStateReceiving_c != maSmacAttributes[mSmacActivePan].smacState) &&
       (mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState))
    {
        return gErrorNoValidCondition_c;
    }

    lMsg.ctx_id = mSmacActivePan;

    OSA_InterruptDisable();
    smacStates_t lState = maSmacAttributes[mSmacActivePan].smacState;
    maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
    OSA_InterruptEnable();

    if(!maSmacAttributes[mSmacActivePan].mSmacTimeoutAsked)
    {
        lMsg.msgType                          = gPlmeSetReq_c;
        lMsg.msgData.setReq.PibAttribute      = gPhyPibRxOnWhenIdle;
        lMsg.msgData.setReq.PibAttributeValue = (uint64_t)0;

        if (MAC_PLME_SapHandler(&lMsg, 0) != gPhySuccess_c)
        {
            OSA_InterruptDisable();
            maSmacAttributes[mSmacActivePan].smacState = lState;
            OSA_InterruptEnable();

            return gErrorBusy_c;
        }
    }
    else
    {
        maSmacAttributes[mSmacActivePan].mSmacTimeoutAsked = FALSE;

        lMsg.msgType = gPlmeSetTRxStateReq_c;
        lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
        (void)MAC_PLME_SapHandler(&lMsg, 0);
    }

    return gErrorNoError_c;
}

smacErrors_t MLMESetActivePan(smacMultiPanInstances_t panID)
{
#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(panID >= gSmacMaxPan_c)
    {
        return gErrorOutOfRange_c;
    }

    if(panID == mSmacActivePan)
    {
        return gErrorNoError_c;
    }

    OSA_InterruptDisable();
    mSmacActivePan = panID;
    OSA_InterruptEnable();

    return gErrorNoError_c;
}

smacErrors_t MLMEConfigureDualPanSettings(bool_t bUseAutoMode,
                                          bool_t bModifyDwell,
                                          uint8_t u8Prescaler,
                                          uint8_t u8Scale)
{
#if gMpmMaxPANs_c != 2
    return gErrorNoValidCondition_c;
#else

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[gSmacPan0_c].smacState ||
       mSmacStateIdle_c != maSmacAttributes[gSmacPan1_c].smacState)
    {
        return gErrorBusy_c;
    }

    if((bModifyDwell == TRUE) && (u8Prescaler > 3 || u8Scale > 63))
    {
        return gErrorOutOfRange_c;
    }

    mpmConfig_t lMpmConfig;
    MPM_GetConfig(&lMpmConfig);

    if(bModifyDwell)
    {
        lMpmConfig.dwellTime = ((u8Prescaler << mDualPanDwellPrescallerShift_c) |
                                (u8Scale << mDualPanDwellTimeShift_c));
    }

    lMpmConfig.autoMode = bUseAutoMode;
    MPM_SetConfig(&lMpmConfig);

    return gErrorNoError_c;
#endif
}

smacErrors_t MLMESetChannelRequest(channels_t newChannel)
{
    uint8_t errorVal;
    macToPlmeMessage_t lMsg;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

    lMsg.msgType = gPlmeSetReq_c;
    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgData.setReq.PibAttribute = gPhyPibCurrentChannel_c;
    lMsg.msgData.setReq.PibAttributeValue = (uint64_t) newChannel;

    errorVal = MAC_PLME_SapHandler(&lMsg, 0);
    switch (errorVal)
    {
    case gPhyBusy_c:
        return gErrorBusy_c;
        break;

    case gPhyInvalidParameter_c:
        return gErrorOutOfRange_c;
        break;

    case gPhySuccess_c:
        return gErrorNoError_c;
        break;

    default:
        return gErrorOutOfRange_c;
        break;
    }
}

channels_t MLMEGetChannelRequest(void)
{
    macToPlmeMessage_t lMsg;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gChannelInvalid_c;
    }
#endif

    lMsg.msgType = gPlmeGetReq_c;
    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgData.getReq.PibAttribute = gPhyPibCurrentChannel_c;

    MAC_PLME_SapHandler(&lMsg, 0);

    return (channels_t)lMsg.msgData.getReq.PibAttributeValue;
}

smacErrors_t SMACSetShortSrcAddress(address_size_t nwShortAddress)
{
    macToPlmeMessage_t lMsg;

    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgType = gPlmeSetReq_c;
    lMsg.msgData.setReq.PibAttribute = gPhyPibShortAddress_c;
    lMsg.msgData.setReq.PibAttributeValue = (uint64_t)nwShortAddress;

    phyStatus_t u8PhyRes = MAC_PLME_SapHandler(&lMsg,0);
    if(u8PhyRes == gPhyBusy_c || u8PhyRes == gPhyBusyTx_c || u8PhyRes == gPhyBusyRx_c)
    {
        return gErrorBusy_c;
    }

    if(u8PhyRes != gPhySuccess_c)
    {
        return gErrorNoResourcesAvailable_c;
    }

    maSmacAttributes[mSmacActivePan].u16ShortSrcAddress = nwShortAddress;
    return gErrorNoError_c;
}

smacErrors_t SMACSetExtendedSrcAddress(uint64_t nwExtendedAddress)
{
    macToPlmeMessage_t lMsg;

    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgType = gPlmeSetReq_c;
    lMsg.msgData.setReq.PibAttribute = gPhyPibLongAddress_c;
    lMsg.msgData.setReq.PibAttributeValue = nwExtendedAddress;

    phyStatus_t u8PhyRes = MAC_PLME_SapHandler(&lMsg,0);
    if(u8PhyRes == gPhyBusy_c || u8PhyRes == gPhyBusyTx_c || u8PhyRes == gPhyBusyRx_c)
    {
        return gErrorBusy_c;
    }

    if(u8PhyRes != gPhySuccess_c)
    {
        return gErrorNoResourcesAvailable_c;
    }

    maSmacAttributes[mSmacActivePan].u64ExtendedSrcAddress = nwExtendedAddress;
    return gErrorNoError_c;
}

smacErrors_t SMACSetPanID(address_size_t nwShortPanID)
{
    macToPlmeMessage_t lMsg;

    lMsg.ctx_id = mSmacActivePan;
    lMsg.msgType = gPlmeSetReq_c;
    lMsg.msgData.setReq.PibAttribute = gPhyPibPanId_c;
    lMsg.msgData.setReq.PibAttributeValue = (uint64_t)nwShortPanID;

    phyStatus_t u8PhyRes = MAC_PLME_SapHandler(&lMsg,0);
    if(u8PhyRes == gPhyBusy_c || u8PhyRes == gPhyBusyTx_c || u8PhyRes == gPhyBusyRx_c)
    {
        return gErrorBusy_c;
    }

    if(u8PhyRes != gPhySuccess_c)
    {
        return gErrorNoResourcesAvailable_c;
    }

    maSmacAttributes[mSmacActivePan].u16PanID = nwShortPanID;
    return gErrorNoError_c;
}

smacErrors_t MLMEPAOutputAdjust( uint8_t u8PaValue)
{
    AppToAspMessage_t   msg;

    /* To silence IAR/GCC wrongly complaining about unused var */
    NOT_USED(msg);

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif /* TRUE == smacInitializationValidation_d */

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

    msg.msgType                                = aspMsgTypeSetPowerLevel_c;
    msg.msgData.aspSetPowerLevelReq.powerLevel = u8PaValue;

    if (APP_ASP_SapHandler(&msg, 0) == gAspSuccess_c)
    {
        return gErrorNoError_c;
    }

    return gErrorOutOfRange_c;
}

uint8_t MLMELinkQuality(void)
{
#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return 0;
    }
#endif

  return maSmacAttributes[mSmacActivePan].smacLastDataRxParams.linkQuality;
}

smacErrors_t MLMEPhySoftReset(void)
{
    macToPlmeMessage_t lMsg;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    lMsg.ctx_id                       = mSmacActivePan;
    lMsg.msgType                      = gPlmeSetTRxStateReq_c;
    lMsg.msgData.setTRxStateReq.state = gPhyForceTRxOff_c;
    (void)MAC_PLME_SapHandler(&lMsg, 0);

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].smacState= mSmacStateIdle_c;
    OSA_InterruptEnable();

    if(maSmacAttributes[mSmacActivePan].gSmacDataMessage != NULL)
    {
        MSG_Free(maSmacAttributes[mSmacActivePan].gSmacDataMessage);
        maSmacAttributes[mSmacActivePan].gSmacDataMessage = NULL;
    }

    if(maSmacAttributes[mSmacActivePan].gSmacMlmeMessage != NULL)
    {
        MSG_Free(maSmacAttributes[mSmacActivePan].gSmacMlmeMessage);
        maSmacAttributes[mSmacActivePan].gSmacMlmeMessage = NULL;
    }

    return gErrorNoError_c;
}

smacErrors_t MLMEScanRequest(channels_t u8ChannelToScan)
{
    smacErrors_t err = gErrorNoError_c;
    phyStatus_t u8PhyRes;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

    if(u8ChannelToScan != MLMEGetChannelRequest())
    {
        err = MLMESetChannelRequest(u8ChannelToScan);

        if(err != gErrorNoError_c)
        {
            return err;
        }
    }

    macToPlmeMessage_t* pMsg = (macToPlmeMessage_t*)MSG_Alloc(sizeof(macToPlmeMessage_t));

    pMsg->ctx_id  = mSmacActivePan;
    pMsg->msgType = gPlmeEdReq_c;
    pMsg->msgData.edReq.startTime = gPhySeqStartAsap_c;

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].gSmacMlmeMessage = pMsg;
    maSmacAttributes[mSmacActivePan].smacState        = mSmacStateScanningChannels_c;
    OSA_InterruptEnable();

    u8PhyRes = MAC_PLME_SapHandler(pMsg,0);
    if(u8PhyRes != gPhySuccess_c)
    {
        OSA_InterruptDisable();
        maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
        OSA_InterruptEnable();

        MSG_Free(maSmacAttributes[mSmacActivePan].gSmacMlmeMessage);
        maSmacAttributes[mSmacActivePan].gSmacMlmeMessage = NULL;

        return gErrorBusy_c;
    }

    return gErrorNoError_c;
}

smacErrors_t MLMECcaRequest()
{
    macToPlmeMessage_t* pMsg;

#if(TRUE == smacInitializationValidation_d)
    if(FALSE == mSmacInitialized)
    {
        return gErrorNoValidCondition_c;
    }
#endif

    if(mSmacStateIdle_c != maSmacAttributes[mSmacActivePan].smacState)
    {
        return gErrorBusy_c;
    }

    pMsg = (macToPlmeMessage_t*)MSG_Alloc(sizeof(macToPlmeMessage_t));

    pMsg->ctx_id = mSmacActivePan;
    pMsg->msgType = gPlmeCcaReq_c;
    pMsg->msgData.ccaReq.ccaType = gPhyCCAMode1_c;
    pMsg->msgData.ccaReq.contCcaMode = gPhyContCcaDisabled;

    OSA_InterruptDisable();
    maSmacAttributes[mSmacActivePan].gSmacMlmeMessage = pMsg;
    maSmacAttributes[mSmacActivePan].smacState = mSmacStatePerformingCca_c;
    OSA_InterruptEnable();

    if (MAC_PLME_SapHandler(pMsg, 0) != gPhySuccess_c)
    {
        OSA_InterruptDisable();
        maSmacAttributes[mSmacActivePan].smacState = mSmacStateIdle_c;
        OSA_InterruptEnable();

        MSG_Free(maSmacAttributes[mSmacActivePan].gSmacMlmeMessage);
        maSmacAttributes[mSmacActivePan].gSmacMlmeMessage = NULL;

        return gErrorBusy_c;
    }

    return gErrorNoError_c;
}

void SMACSetTxAutoAck(bool_t enable)
{
    maSmacAttributes[mSmacActivePan].txConfigurator.autoAck = enable;
}

void SMACSetTxEnhAck(bool_t enable)
{
    maSmacAttributes[mSmacActivePan].txConfigurator.enhAck = enable;
}

void SMACFillHeader(smacHeader_t* pSmacHeader, address_size_t destAddr)
{
    pSmacHeader->frameControl = gSmacDefaultFrameCtrl_c;

#if !gEnhAckMode8
    pSmacHeader->panId        = maSmacAttributes[mSmacActivePan].u16PanID;
#endif
    pSmacHeader->seqNo        = gSmacDefaultSeqNo_c;
    pSmacHeader->srcAddr      = maSmacAttributes[mSmacActivePan].u16ShortSrcAddress;
    pSmacHeader->destAddr     = destAddr;
}
