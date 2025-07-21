/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2018, 2023-2025 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#include "PhyInterface.h"
#include "Phy.h"
#include "PhySec.h"
#include "PhyPacket.h"
#include "dbg_io.h"
#include "fsl_component_mem_manager.h"
#include "fsl_component_messaging.h"

#if (defined(HDI_MODE) && (HDI_MODE == 1L))
#if (defined(KW43B43ZC7_SERIES) || defined(KW43B43ZC7_NBU_SERIES))
#include "hdi.h"
#endif
#endif

#if defined(PHY_WLAN_COEX)
#include "PhyWlanCoex.h"
#endif

#if (gMWS_Enabled_d) || (gMWS_UseCoexistence_d)
#include "MWS.h"
#include "nxp2p4_xcvr.h"

#include "nxp_xcvr_oqpsk_802p15p4_config.h"
#include "nxp_xcvr_coding_config.h"
#include "nxp_xcvr_ext_ctrl.h"

#include "nxp_xcvr_gfsk_bt_0p5_h_0p5_config.h"
#endif

#if defined(K32W1480_SERIES) || defined(CPU_KW45B41Z83AFPA_NBU) ||                 \
    defined(MCXW727C_cm33_core0_SERIES) ||  defined(MCXW727C_cm33_core1_SERIES) || \
    defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES)
#include "fwk_platform_genfsk.h"

#ifndef gAppMaxTxPowerDbm_c
#define gAppMaxTxPowerDbm_c 10
#endif
#endif

#if defined(FFU_DEVICE_LIMIT_VISIBILITY)
#include "PhyDeviceFiltering.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define MSG_Pending(anchor)             ((anchor)->head != 0)
#define MSG_DeQueue(anchor)             MSG_QueueRemoveHead((anchor))
#define MSG_Queue(anchor, element)      MSG_QueueAddTail((anchor), (element))
#define MSG_QueueHead(anchor, element)  MSG_QueueAddHead((anchor), (element))

#define DELAYED_TX_RANGE                (uint32_t)(0xFFFF * 10) /* maximum csl phase time in symbols */

#define TX_DONE_RESET_REGISTER          (ZLL_BASE + 0x88)
#define TX_DONE_RESET_REGISTER_MASK     0x04

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void Phy24Task(Phy_PhyLocalStruct_t *ctx);

static phyStatus_t Phy_Handle_RxReq(Phy_PhyLocalStruct_t *ctx, macToPlmeMessage_t *pMsg, uint32_t dt);

static phyStatus_t Phy_Handle_PdDataReq(Phy_PhyLocalStruct_t *ctx, macToPdDataMessage_t *pMsg);

static phyStatus_t Phy_Handle_PlmeCcaEdRequest(Phy_PhyLocalStruct_t *ctx, macToPlmeMessage_t *pMsg);

static void Phy_EnterIdle(Phy_PhyLocalStruct_t *ctx);

void PLME_SendMessage(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msgType);

static void PD_SendMessage(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msgType);

static void Phy_SendLatePD(uint32_t param);
static void Phy_SendLatePLME(uint32_t param);

#if (gMWS_Enabled_d) || (gMWS_UseCoexistence_d)
static uint32_t MWS_802_15_4_Callback(mwsEvents_t event);
#endif

#ifdef CTX_SCHED
static void ctx_plme_msg_all(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msg_type);
static void sched_reset();
void sched_enable();
void sched_start_timer(uint32_t ticks);
void sched_stop_timer();
void PHY_InterruptHandler_base(uint8_t xcvseqCopy, uint32_t irqStatus);
void ProtectFromXcvrInterrupt_base(Phy_PhyLocalStruct_t *ctx);
void UnprotectFromXcvrInterrupt_base(Phy_PhyLocalStruct_t *ctx);
uint8_t PhyPpGetState_base(Phy_PhyLocalStruct_t *ctx);
void PhyAbort_base(Phy_PhyLocalStruct_t *ctx);
bool_t PHY_graceful_idle_base(Phy_PhyLocalStruct_t *ctx);
#else
#define ctx_plme_msg_all(ctx, msg_type)
#define ProtectFromXcvrInterrupt_base(ctx) ProtectFromXcvrInterrupt()
#define UnprotectFromXcvrInterrupt_base(ctx) UnprotectFromXcvrInterrupt()
#define PhyPpGetState_base(ctx) PhyPpGetState()
#define PhyAbort_base(ctx) PhyAbort()
#define PHY_graceful_idle_base(ctx) PHY_graceful_idle()
#endif

uint16_t PHY_TransformArrayToUint16(uint8_t *pArray);

void update_rxf(uint8_t xcvseq, uint32_t irq_status);

void PHY_set_enh_ack_state(Phy_PhyLocalStruct_t *ctx, enh_ack_state_t state);
/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
extern uint8_t * const rxf;

#if gMWS_Enabled_d
static bool_t phy_is_active;
#endif

#if (gMWS_Enabled_d) || (gMWS_UseCoexistence_d)
/*
 * LDO ANT TRIM value to be applied at each XCVR Init or mode change.
 * This is init on NBU core based on gAppMaxTxPowerDbm_c define.
 * Valid value 0 - 15.
 * Set to invalid value 0xFF by default
 */
uint8_t g_ldo_ant_trim_15_4 = 0xFF;
#endif


static bool_t t1_less_t2(uint32_t tstp_1, uint32_t tstp_2)
{
    uint32_t neg_msk = 1 << (gPhyTimeShift_c - 1);

    tstp_1 &= gPhyTimeMask_c;
    tstp_2 &= gPhyTimeMask_c;

    return !!((tstp_1 - tstp_2) & neg_msk);
}

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief  This function creates the PHY task
*
********************************************************************************** */

/* API call order:
   Phy_Init() (=> PHY_Enable), PHY_Enable() / PHY_get_ctx()
   PHY_release_ctx() / PHY_Disable(), Phy_Deinit()

   All contexts must be released for PHY_Disable() to succeed */

static bool_t phy_init_done = FALSE;
static bool_t phy_enable_done = FALSE;

void Phy_Init(void)
{
    OSA_InterruptDisable();

    if (phy_init_done)
    {
        OSA_InterruptEnable();

        return;
    }

    phy_init_done = TRUE;

    OSA_InterruptEnable();

#if defined(CPU_KW45B41Z83AFPA_NBU) || defined(MCXW727C_cm33_core1_SERIES)
    int RNG_Init();
    RNG_Init();
#endif

#if (defined(HDI_MODE) && (HDI_MODE == 1L))
#if (defined(KW43B43ZC7_SERIES) || defined(KW43B43ZC7_NBU_SERIES))
    HDI_Init();
    HDI_Set_Mode_Zigbee();
#endif
#endif

    /* ISR registration. Must be done before IRQ enablement (PHY_Enable()).
       PhyHwInit() => PHY_Enable() */
    PHY_PhyIrqCreate();

    PhyHwInit();
    PhyTime_TimerInit(NULL);

    ctx_init();

#if gMWS_Enabled_d
    MWS_Register(gMWS_802_15_4_c, MWS_802_15_4_Callback);
#endif

#if gMWS_UseCoexistence_d
    MWS_CoexistenceInit(&gCoexistence_RfDeny, &gCoexistence_RfActive, &gCoexistence_RfStatus);
    MWS_CoexistenceRegister(gMWS_802_15_4_c, MWS_802_15_4_Callback);
#endif

#if defined(K32W1480_SERIES) || defined(CPU_KW45B41Z83AFPA_NBU) ||                \
    defined(MCXW727C_cm33_core0_SERIES) || defined(MCXW727C_cm33_core1_SERIES) || \
    defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES)
    PLATFORM_SetGenfskMaxTxPower(gAppMaxTxPowerDbm_c);
#endif

#if (gMWS_Enabled_d) || (gMWS_UseCoexistence_d)
    /* Store maintained value for LDO Ant Trim used by 15.4 Phy layer */
    g_ldo_ant_trim_15_4 = XCVR_getLdoAntTrim();
#endif

#ifdef CTX_SCHED
    sched_enable();
#endif
}

/*! *********************************************************************************
* \brief  This function Deinit PHY task
*
********************************************************************************** */
void Phy_Deinit(void)
{
    OSA_InterruptDisable();

    if (!phy_init_done || phy_enable_done)
    {
        OSA_InterruptEnable();

        return;
    }

    phy_init_done = FALSE;

    OSA_InterruptEnable();

    PhyTime_TimerDeinit();
}

/*! *********************************************************************************
* \brief  This function enables the Phy ISR
*
********************************************************************************** */
void PHY_Enable(void)
{
    OSA_InterruptDisable();

    if (!phy_init_done || phy_enable_done)
    {
        OSA_InterruptEnable();

        return;
    }

    phy_enable_done = TRUE;

    OSA_InterruptEnable();

    PHY_PhyIrqClearPending();
    PHY_PhyIrqEnable();
    PHY_PhyIrqSetPriority();
    UnprotectFromXcvrInterrupt();
}

/*! *********************************************************************************
* \brief  This function disables the Phy ISR and aborts current activity
*
********************************************************************************** */
void PHY_Disable(void)
{
    OSA_InterruptDisable();

    if (!phy_init_done || !phy_enable_done || !PHY_ctx_all_disabled())
    {
        OSA_InterruptEnable();

        return;
    }

    phy_enable_done = FALSE;

    OSA_InterruptEnable();

    PhyAbort();
    PHY_PhyIrqDisable();
    ProtectFromXcvrInterrupt();
}

/*! *********************************************************************************
* \brief  This function registers the MAC PD and PLME SAP handlers
*
* \param[in]  pPD_MAC_SapHandler   Pointer to the MAC PD handler function
* \param[in]  pPLME_MAC_SapHandler Pointer to the MAC PLME handler function
* \param[in]  instanceId           The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
void Phy_RegisterSapHandlers(PD_MAC_SapHandler_t pPD_MAC_SapHandler,
                             PLME_MAC_SapHandler_t pPLME_MAC_SapHandler,
                             instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);

    ctx->PD_MAC_SapHandler = pPD_MAC_SapHandler;
    ctx->PLME_MAC_SapHandler = pPLME_MAC_SapHandler;
}

/*! *********************************************************************************
* \brief  This function represents the PHY's task
*
* \param[in]  taskParam The instance of the PHY
*
********************************************************************************** */
static void Phy24Task(Phy_PhyLocalStruct_t *ctx)
{
    phyMessageHeader_t *pMsgIn = NULL;
    phyStatus_t status = gPhySuccess_c;

    if (!ctx)
    {
        return;
    }

    OSA_InterruptDisable();

    if (ctx->ps == PS_RX)
    {
        uint32_t t = PhyTime_ReadClock();
        uint32_t dt = (ctx->rx_poll_to - t) & gPhyTimeMask_c;

        if ((PhyPpGetState_base(ctx) == gIdle_c) &&
            t1_less_t2(t, ctx->rx_poll_to) && (dt > PHY_IMM_ACK_LENGTH))
        {
            ctx->flags &= ~gPhyFlagIdleRx_c;

            Phy_Handle_RxReq(ctx, NULL, dt);

            OSA_InterruptEnable();
            return;
        }
        else
        {
            ctx->ps = PS_NONE;
        }
    }

    /* Handling messages from upper layer */
    while (MSG_Pending(&ctx->macPhyInputQueue))
    {
        status = gPhySuccess_c;

        /* abort just rx in certain cases */
        if (!PHY_graceful_idle_base(ctx))
        {
            /* skip enter idle */
            status = gPhyBusy_c;
            break;
        }

        pMsgIn = MSG_DeQueue(&ctx->macPhyInputQueue);
        if (pMsgIn == NULL)
        {
            /* fail safe */
            continue;
        }

        /* PHY is idle */
        ctx->flags &= ~gPhyFlagIdleRx_c;

        switch (pMsgIn->msgType)
        {
        case gPdDataReq_c:
            status = Phy_Handle_PdDataReq(ctx, (macToPdDataMessage_t *)pMsgIn);
            break;

        case gPlmeRxReq_c:
            status = Phy_Handle_RxReq(ctx, (macToPlmeMessage_t *)pMsgIn, 0);
            break;

        case gPlmeCcaReq_c:
            status = Phy_Handle_PlmeCcaEdRequest(ctx, (macToPlmeMessage_t *)pMsgIn);
            break;

        case gPlmeEdReq_c:
            status = Phy_Handle_PlmeCcaEdRequest(ctx, (macToPlmeMessage_t *)pMsgIn);
            break;

        default:
            status = gPhyInvalidPrimitive_c;
            break;
        }

        MSG_Free(pMsgIn);

        if ((status == gPhySuccess_c) || (status == gPhyPendingOp))
        {
            /* Just started rx/tx/CCA */
            break;
        }
        else
        {
            PLME_SendMessage(ctx, gPlmeAbortInd_c);
            status = gPhySuccess_c;
        }
    }

    /* Check if PHY can enter Idle state (when a context switch is not in progress) */
    if ((status != gPhyPendingOp) && (gIdle_c == PhyPpGetState_base(ctx)))
    {
        Phy_EnterIdle(ctx);
    }

    OSA_InterruptEnable();
}

/*! *********************************************************************************
* \brief  This is the PD SAP message handler
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  instanceId The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PD_SapHandler(macToPdDataMessage_t *pMsg, instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    phyStatus_t result = gPhySuccess_c;
    uint32_t baseIndex = ctx->id * (gPhySAPSize_d / CTX_NO);
    macToPdDataMessage_t * pMacToPdMsg = NULL;
    uint32_t allocSize = 0;

    if (NULL == pMsg)
        return gPhyInvalidParameter_c;

    OSA_InterruptDisable();

    switch (pMsg->msgType)
    {
    case gPdIndQueueInsertReq_c:
        /* not used by ot-nxp, only by MAC */
        result = PhyPp_IndirectQueueInsert(baseIndex + pMsg->msgData.indQueueInsertReq.index,
                                           pMsg->msgData.indQueueInsertReq.checksum, ctx->id);
        break;

    case gPdIndQueueRemoveReq_c:
        /* not used by ot-nxp, only by MAC */
        result = PhyPp_RemoveFromIndirect(baseIndex + pMsg->msgData.indQueueRemoveReq.index, ctx->id);
        break;

    case gPdDataReq_c:
        allocSize = sizeof(macToPdDataMessage_t) + pMsg->msgData.dataReq.psduLength;
        pMacToPdMsg = (macToPdDataMessage_t *) MSG_Alloc(allocSize);

        if (NULL != pMacToPdMsg)
        {
            memcpy((uint8_t*) pMacToPdMsg, (uint8_t*)pMsg, allocSize);

            /* Point pPsdu to the new buffer which is immediately after
             * macToPdDataMessage_t structure (pPsdu is the last member)
             * where the buffer is located.
             */
            pMacToPdMsg->msgData.dataReq.pPsdu = (uint8_t *)pMacToPdMsg + sizeof(macToPdDataMessage_t);
            // The Phy queue is init as unlimited so there is no risc for error in adding msg to Phy queue regarding maximum size
            // if it will be a memory shortage it will remains without memory buffers first (i.e. pMacToPlmeMsg should be NULL)
            // so the case of MSG_Queue return error it is highly unlikely and even so the root would be from another place
            // It is ok the assumption it will be succes
            MSG_Queue(&ctx->macPhyInputQueue, pMacToPdMsg);

            ctx_set_pending(ctx);

            /* run the PHY state machine from PHY ISR context only */
            PHY_ForceIrqPending();
        }
        else
        {
            result = gPhyBusy_c;
        }
        break;

    default:
        result = gPhyInvalidPrimitive_c;
        break;
    }
    OSA_InterruptEnable();

    return result;
}

/*! *********************************************************************************
* \brief  This is the PLME SAP message handler
*
* \param[in]  pMsg Pointer to the PLME request message
* \param[in]  instanceId The instance of the PHY
*
* \return  phyStatus_t The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PLME_SapHandler(macToPlmeMessage_t *pMsg, instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    phyStatus_t result = gPhySuccess_c;
    macToPlmeMessage_t * pMacToPlmeMsg = NULL;

    if (NULL == pMsg) {
        return gPhyInvalidParameter_c;
    }

    OSA_InterruptDisable();

    switch (pMsg->msgType) {
    case gPlmeEdReq_c:
    case gPlmeCcaReq_c:
        pMacToPlmeMsg = (macToPlmeMessage_t *)MSG_Alloc(sizeof(macToPlmeMessage_t));

        if (NULL != pMacToPlmeMsg)
        {
            memcpy((uint8_t*) pMacToPlmeMsg, (uint8_t*)pMsg, sizeof(macToPlmeMessage_t));

            MSG_Queue(&ctx->macPhyInputQueue, pMacToPlmeMsg);

            ctx_set_pending(ctx);

            /* run the PHY state machine from PHY ISR context only */
            PHY_ForceIrqPending();
        }
        else
        {
            result = gPhyBusy_c;
        }
        break;

    case gPlmeSetReq_c:
        result = PhyPlmeSetPIBRequest(pMsg->msgData.setReq.PibAttribute,
                                      pMsg->msgData.setReq.PibAttributeValue,
                                      phyInstance);
        break;

    case gPlmeGetReq_c:
        result = PhyPlmeGetPIBRequest(pMsg->msgData.getReq.PibAttribute,
                                      (uint8_t *)&pMsg->msgData.getReq.PibAttributeValue,
                                      phyInstance);
        break;

    case gPlmeSetTRxStateReq_c:
        if (gPhySetRxOn_c == pMsg->msgData.setTRxStateReq.state)
        {
            pMacToPlmeMsg = (macToPlmeMessage_t *)MSG_Alloc(sizeof(macToPlmeMessage_t));

            if (NULL != pMacToPlmeMsg)
            {
                memcpy((uint8_t*) pMacToPlmeMsg, (uint8_t*)pMsg, sizeof(macToPlmeMessage_t));

                MSG_Queue(&ctx->macPhyInputQueue, pMacToPlmeMsg);

                ctx_set_pending(ctx);

                /* run the PHY state machine from PHY ISR context only */
                PHY_ForceIrqPending();
            }
            else
            {
                result = gPhyBusy_c;
            }
        }
        else if (gPhyForceTRxOff_c == pMsg->msgData.setTRxStateReq.state)
        {
            PhyAbort_base(ctx);

            /* disable rx when idle */
            ctx->flags &= ~(gPhyFlagRxOnWhenIdle_c | gPhyFlagIdleRx_c);
            ctx->ps = PS_NONE;

            ctx_set_pending(ctx);

            /* run the PHY state machine from PHY ISR context only */
            PHY_ForceIrqPending();
        }
        break;

    case gPlmeSetSAMState_c:
        PhyPpSetSAMState(phyInstance, pMsg->msgData.SAMState);
        break;

    case gPlmeAddToSapTable_c:
        PhyAddToSapTable(phyInstance, pMsg->msgData.deviceAddr.addr,
                         pMsg->msgData.deviceAddr.mode,
                         pMsg->msgData.deviceAddr.panId);
        break;

    case gPlmeEfpSet_c:
        PHY_SetEfp(phyInstance, pMsg->msgData.efpEnabled);
        break;

    case gPlmeNeighbourTblSet_c:
        PHY_SetNbt(phyInstance, pMsg->msgData.neighbourTblEnabled);
        break;

    case gPlmeNeighbourTblAdd_c:
        /* PHY Neighbour table only with short addr */
        if (pMsg->msgData.deviceAddr.mode == 2)
        {
            PhyNbTblAdd(phyInstance, pMsg->msgData.deviceAddr.addr,
                    pMsg->msgData.deviceAddr.mode,
                    pMsg->msgData.deviceAddr.panId);
        }

        break;

    case gPlmeNeighbourTblRemove_c:
        /* PHY Neighbour table only with short addr */
        if (pMsg->msgData.deviceAddr.mode == 2)
        {
            PhyNbTblRemove(phyInstance, pMsg->msgData.deviceAddr.addr,
                    pMsg->msgData.deviceAddr.mode,
                    pMsg->msgData.deviceAddr.panId);
        }

        break;

    case gPlmeRemoveFromSAMTable_c:
        PhyRemoveFromSamTable(phyInstance, pMsg->msgData.deviceAddr.addr,
                              pMsg->msgData.deviceAddr.mode,
                              pMsg->msgData.deviceAddr.panId);
        break;

    case gPlmeCslEnable_c:
        PHY_EnableCsl(phyInstance, pMsg->msgData.cslPeriod);
        break;

    case gPlmeCslSetSampleTime_c:
        PHY_SetCslSampleTime(phyInstance, pMsg->msgData.cslSampleTime);
        break;

    case gPlmeConfigureAckIeData_c:
        PHY_ConfigureAckIeData(phyInstance,
                               pMsg->msgData.AckIeData.data,
                               pMsg->msgData.AckIeData.param,
                               pMsg->msgData.AckIeData.shortAddr,
                               pMsg->msgData.AckIeData.extAddr);
        break;

    case gPlmeSetMacKey_c:
        PhySec_SetKeys(phyInstance,
                       pMsg->msgData.MacKeyData.keyId,
                       pMsg->msgData.MacKeyData.prevKey,
                       pMsg->msgData.MacKeyData.currKey,
                       pMsg->msgData.MacKeyData.nextKey);
        break;

    case gPlmeEnableEncryption_c:
        PhySec_Enable(phyInstance);
        break;

    case gPlmeSetMacFrameCounter_c:
        PhySec_SetFrameCounter(phyInstance, pMsg->msgData.MacFrameCounter);
        break;

    case gPlmeSetMacFrameCounterIfLarger_c:
        PhySec_SetFrameCounterIfLarger(phyInstance, pMsg->msgData.MacFrameCounter);
        break;

#if defined(FFU_DEVICE_LIMIT_VISIBILITY)
    case gPlmeAddVisibleExtAddr_c:
        PHY_addVisibleExtAddr(phyInstance, pMsg->msgData.filterAddr.extAddr);
        break;
    case gPlmeAddInvisibleLocalAddr_c:
        PHY_addInvisibleLocalAddr(phyInstance, pMsg->msgData.filterAddr.shortAddr);
        break;
    case gPlmeRemoveInvisibleLocalAddr_c:
        PHY_removeInvisibleLocalAddr(phyInstance, pMsg->msgData.filterAddr.shortAddr);
        break;
    case gPlmeClearVisibleFilters_c:
        PHY_clearVisibleFilters(phyInstance);
        break;
    case gPlmeSetBeaconFiltering_c:
        PHY_setBeaconFiltering(phyInstance, pMsg->msgData.filterAddr.block);
        break;
    case gPlmeUpdateLocalWithExtAddr_c:
        PHY_updateLocalWithExtAddr(phyInstance, pMsg->msgData.filterAddr.extAddr, pMsg->msgData.filterAddr.shortAddr);
        break;
#endif

    default:
        result = gPhyInvalidPrimitive_c;
        break;
    }
    OSA_InterruptEnable();

    return result;
}

static phyStatus_t Phy_Handle_RxReq(Phy_PhyLocalStruct_t *ctx, macToPlmeMessage_t *pMsg, uint32_t dt)
{
    phyStatus_t status = gPhySuccess_c;

    if (pMsg)
    {
        ctx->rxParams.startTime = pMsg->msgData.setTRxStateReq.startTime;
        ctx->rxParams.duration = pMsg->msgData.setTRxStateReq.rxDuration;
    }
    else
    {
        ctx->rxParams.startTime = gPhySeqStartAsap_c;
        ctx->rxParams.duration = dt;
    }

    OSA_InterruptDisable();
    ProtectFromXcvrInterrupt_base(ctx);

    ctx_set_rx(ctx);

    if (ctx_is_active(ctx))
    {
        status = PhyPlmeRxRequest(ctx);
    }
    else if (ctx_is_paused(ctx))
    {
        status = gPhyPendingOp;
    }

    if ((gPhySuccess_c != status) && (gPhyPendingOp != status))
    {
        PhyAbort_base(ctx);
    }

    UnprotectFromXcvrInterrupt_base(ctx);
    OSA_InterruptEnable();

    return status;
}

static phyStatus_t Phy_Handle_PdDataReq(Phy_PhyLocalStruct_t *ctx, macToPdDataMessage_t *pMsg)
{
    phyStatus_t status = gPhySuccess_c;

    if (!pMsg->msgData.dataReq.pPsdu)
    {
        return gPhyInvalidParameter_c;
    }

    ctx->txParams.dataReq = &pMsg->msgData.dataReq;

    ctx_set_tx(ctx, pMsg);

    if (ctx_is_active(ctx))
    {
        status = PhyPdDataRequest(ctx);
    }
    else if (ctx_is_paused(ctx))
    {
        status = gPhyPendingOp;
    }

    if ((gPhySuccess_c != status) && (gPhyPendingOp != status))
    {
        PhyAbort_base(ctx);
    }

    return status;
}

static phyStatus_t Phy_Handle_PlmeCcaEdRequest(Phy_PhyLocalStruct_t *ctx, macToPlmeMessage_t *pMsg)
{
    phyStatus_t status = gPhySuccess_c;
    uint32_t cca_time = gPhyRxWuTimeSym + gCCATime_c + gPhyRxWdTimeSym;

    ctx->channelParams.maxEnergyLeveldB = -127;     /* set maxEnergyLeveldB to minimum value */
    ctx->channelParams.energyLeveldB = 0;

    if (!pMsg)
    {
        return gPhyInvalidParameter_c;
    }

    switch (pMsg->msgType)
    {
    case gPlmeCcaReq_c:
        ctx->ccaParams.msgType = gPlmeCcaReq_c;

#if (CCA_MODE_SELECT_SUPPORT==1)
        ctx->ccaParams.ccaParam = pMsg->msgData.ccaReq.ccaType;
#else
        ctx->ccaParams.ccaParam = gPhyCCAMode1_c;
#endif
        ctx->ccaParams.cccaMode = gPhyContCcaDisabled;
        break;

    case gPlmeEdReq_c:
        ctx->ccaParams.msgType = gPlmeEdReq_c;
        ctx->ccaParams.ccaParam = gPhyEnergyDetectMode_c;
        ctx->ccaParams.cccaMode = gPhyContCcaDisabled;

        /* convert the ED time to CCA measurements */
        ctx->ccaParams.edScanMaxCnt = (pMsg->msgData.edReq.measureDurationSym + cca_time - 1) / cca_time;
        ctx->ccaParams.edScanCnt = 0;
        break;

    default:
        return gPhyInvalidParameter_c;
    }

    OSA_InterruptDisable();
    ProtectFromXcvrInterrupt_base(ctx);

    ctx_set_cca(ctx);

    if (ctx_is_active(ctx))
    {
        status = PhyPlmeCcaEdRequest(ctx);
    }
    else if (ctx_is_paused(ctx))
    {
        status = gPhyPendingOp;
    }

    if ((gPhySuccess_c != status) && (gPhyPendingOp != status))
    {
        PhyAbort_base(ctx);
    }

    UnprotectFromXcvrInterrupt_base(ctx);
    OSA_InterruptEnable();

    return status;
}

/*! *********************************************************************************
* \brief  This function sets the start time and the timeout value for a sequence.
*
* \param[in]  startTime The absolute start time for the sequence.
*             If startTime is gPhySeqStartAsap_c, the start timer is disabled.
* \param[in]  seqDuration The duration of the sequence.
*             If seqDuration is 0xFFFFFFFF, the timeout is disabled.
*
********************************************************************************** */
void Phy_SetSequenceTiming(phyTime_t startTime, uint32_t seqDuration, uint32_t overhead)
{
    phyTime_t endTime;
    uint32_t delta;

    OSA_InterruptDisable();

    /* Check if there is enough time for delayed operation (considering PHY_TMR_CMP_MIN) */
    if (startTime != gPhySeqStartAsap_c)
    {
        startTime &= gPhyTimeMask_c;

        /* 24bit timer. Do modulo operations */
        delta = (startTime - (PhyTime_ReadClock() & gPhyTimeMask_c)) & gPhyTimeMask_c;

        if ((delta  < (overhead + PHY_TMR_CMP_MIN)) || (delta > DELAYED_TX_RANGE))
        {
            startTime = gPhySeqStartAsap_c;
        }
        else
        {
            startTime -= overhead;
            startTime &= gPhyTimeMask_c;
        }
    }

    if (gPhySeqStartAsap_c == startTime)
    {
        endTime = PhyTime_ReadClock();

        PhyTimeDisableEventTrigger();
    }
    else
    {
        endTime = startTime;

        PhyTimeSetEventTrigger(startTime);
    }

    if (0xFFFFFFFFU != seqDuration)
    {
        endTime += (seqDuration + overhead);
        endTime &= gPhyTimeMask_c;

        PhyTimeSetEventTimeout(endTime);
    }
    else
    {
        PhyTimeDisableEventTimeout();
    }

    OSA_InterruptEnable();
}

int8_t PHY_handle_get_RSSI(Phy_PhyLocalStruct_t *ctx)
{
    int8_t ret = 127;     /* RSSI is invalid */

    OSA_InterruptDisable();

    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        ret = PhyPlmeGetRSSILevelRequest(ctx->id);
    }

    OSA_InterruptEnable();

    return ret;
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  ctx pointer to PHY data
*
********************************************************************************** */
static void Phy_EnterIdle(Phy_PhyLocalStruct_t *ctx)
{
    if (ctx->flags & gPhyFlagRxOnWhenIdle_c)
    {
        ctx->flags |= gPhyFlagIdleRx_c;
        Phy_Handle_RxReq(ctx, NULL, 0xFFFFFFFFU);
    }
    else
    {
        ctx->flags &= ~(gPhyFlagIdleRx_c);
        ctx_set_none(ctx);
    }
}

/*! *********************************************************************************
* \brief  This function sets the state of the PhyRxOnWhenIdle PIB
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  state The PhyRxOnWhenIdle value
*
********************************************************************************** */
void PhyPlmeSetRxOnWhenIdle(bool_t state, instanceId_t instanceId)
{
    OSA_InterruptDisable();

    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint8_t radioState = PhyPpGetState_base(ctx);

    if (state)
    {
        ctx->flags |= gPhyFlagRxOnWhenIdle_c;

        ctx_set_pending(ctx);

        /* run the PHY state machine from PHY ISR context only */
        PHY_ForceIrqPending();
    }
    else
    {
        ctx->flags &= ~gPhyFlagRxOnWhenIdle_c;

        if ((ctx->flags & gPhyFlagIdleRx_c) && (radioState == gRX_c))
        {
            ctx->flags &= ~gPhyFlagIdleRx_c;
            PhyAbort_base(ctx);

            ctx_set_pending(ctx);

            /* run the PHY state machine from PHY ISR context only */
            PHY_ForceIrqPending();
        }
    }

    OSA_InterruptEnable();
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
bool_t PhyIsIdleRx(instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    bool_t status = FALSE;
    uint8_t state = PhyPpGetState_base(ctx);

    if ((ctx->flags & gPhyFlagIdleRx_c) && (gRX_c == state))
    {
        status = TRUE;
    }

    return status;
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a TX operation completed successfully.
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  framePending The value of the framePending bit for the received ACK
*
********************************************************************************** */
void Radio_Phy_PdDataConfirm(Phy_PhyLocalStruct_t *ctx, bool_t framePending)
{
    if (!ctx)
    {
        return;
    }

    if (framePending)
    {
        /* start rx for POLL if not rx on idle */
        if (!(ctx->flags & gPhyFlagRxOnWhenIdle_c) && ctx->rx_time_poll && (ctx->ps == PS_DATA_REQ))
        {
            ctx->ps = PS_RX;
            ctx->rx_poll_to = (ctx->rx_time_poll + PhyTime_ReadClock()) & gPhyTimeMask_c;
        }
        else
        {
            ctx->ps = PS_NONE;
        }

        ctx->flags |= gPhyFlagRxFP_c;
    }
    else
    {
        ctx->flags &= ~gPhyFlagRxFP_c;
    }

    PD_SendMessage(ctx, gPdDataCnf_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that new data has been received
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PdDataIndication(Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return;
    }

    uint16_t fcf = PHY_TransformArrayToUint16(ctx->trx_buff);

    if (fcf & phyFcfAckRequest)
    {
        /* stop rx for POLL after a unicast packet is received */
        ctx->ps = PS_NONE;
    }

    PD_SendMessage(ctx, gPdDataInd_c);

    ctx_data_ind_all(ctx);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a CCA sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  phyChannelStatus The status of the channel: Idle/Busy
*
* \return  None.
*
********************************************************************************** */
void Radio_Phy_PlmeCcaConfirm(phyStatus_t phyChannelStatus, Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return;
    }

    ctx->channelParams.channelStatus = phyChannelStatus;

    PLME_SendMessage(ctx, gPlmeCcaCnf_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a ED sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  energyLevel The enetgy level on the channel.
* \param[in]  energyLeveldB The energy level in DB
*
********************************************************************************** */
void Radio_Phy_PlmeEdConfirm(Phy_PhyLocalStruct_t *ctx, int8_t energyLeveldB)
{
    if (!ctx)
    {
        return;
    }

    ctx->channelParams.energyLeveldB = energyLeveldB;

    if (energyLeveldB > ctx->channelParams.maxEnergyLeveldB)
    {
        ctx->channelParams.maxEnergyLeveldB = energyLeveldB;
    }

    ctx->ccaParams.edScanCnt++;
    if (ctx->ccaParams.edScanCnt >= ctx->ccaParams.edScanMaxCnt)
    {
        PLME_SendMessage(ctx, gPlmeEdCnf_c);
    }
    else if (PhyPlmeCcaEdRequest(ctx) != gPhySuccess_c)
    {
        PLME_SendMessage(ctx, gPlmeAbortInd_c);

    }
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has timed out
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_TimeRxTimeoutIndication(Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return;
    }

    if (!(ctx->flags & gPhyFlagIdleRx_c) && (ctx->ps != PS_RX))
    {
        PLME_SendMessage(ctx, gPlmeTimeoutInd_c);
    }

    ctx->ps = PS_NONE;

    ctx_plme_msg_all(ctx, gPlmeTimeoutInd_c);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has aborted
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_AbortIndication(Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return;
    }

    if (!(ctx->flags & gPhyFlagIdleRx_c) && (ctx->ps != PS_RX))
    {
        PLME_SendMessage(ctx, gPlmeAbortInd_c);
    }

    ctx->ps = PS_NONE;

    ctx_plme_msg_all(ctx, gPlmeAbortInd_c);
}

/* Update reception timeout */
void Radio_Phy_PlmeRxWatermark(uint32_t frameLength, uint16_t fcf)
{
    phyTime_t currentTime;
    uint32_t rx_time = frameLength * gPhySymbolsPerOctet_c + mPhyOverhead_d;     /* symbols */

    if (fcf & phyFcfAckRequest)
    {
        rx_time += gPhyTurnaroundTime_c + gPhySHRDuration_c + gPhyPHRDuration_c;

        if (FCF_VER_GET(fcf) == FCF_VER_MAX)
        {
            rx_time += PHY_ENH_ACK_LENGTH * gPhySymbolsPerOctet_c;
        }
        else
        {
            rx_time += PHY_IMM_ACK_LENGTH * gPhySymbolsPerOctet_c;
        }
    }

    /* update timeout */
    currentTime = PhyTime_ReadClock();
    currentTime = (currentTime + rx_time) & gPhyTimeMask_c;
    PhyTimeSetEventTimeout(currentTime);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Sync Loss occured (PLL unlock)
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeSyncLossIndication(Phy_PhyLocalStruct_t *ctx)
{
    Radio_Phy_TimeRxTimeoutIndication(ctx);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Filter Fail occured
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeFilterFailRx(Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return;
    }

#if defined(FFU_FMTM_APP)       // This is used for 15.4 FW testing with FMTM
    PLME_SendMessage(ctx, gPlme_FilterFailInd_c);
#endif
}

/*! *********************************************************************************
* \brief  Senf a PLME message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
void PLME_SendMessage(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msgType)
{
    plmeToMacMessage_t *pMsg;

    if (!ctx)
    {
        return;
    }

    if (!ctx->PLME_MAC_SapHandler)
    {
        return;
    }

    pMsg = MSG_Alloc(sizeof(plmeToMacMessage_t));

    if (NULL == pMsg)
    {
        phyTimeEvent_t ev;

        ctx->delayed_msg = msgType;

        ev.parameter = ctx->id;
        ev.callback = Phy_SendLatePLME;
        ev.timestamp = gPhyRxRetryInterval_c + PhyTime_GetTimestamp();

        PhyTime_ScheduleEvent(&ev);
        return;
    }

    pMsg->fc = ctx->frameCounter - 1;   /* frame counter is updated after it's used */

    pMsg->msgType = msgType;

    switch (msgType)
    {
    case gPlmeCcaCnf_c:
        pMsg->msgData.ccaCnf.status = ctx->channelParams.channelStatus;
        break;

    case gPlmeEdCnf_c:
        pMsg->msgData.edCnf.status           = gPhySuccess_c;
        pMsg->msgData.edCnf.energyLeveldB    = ctx->channelParams.energyLeveldB;
        pMsg->msgData.edCnf.maxEnergyLeveldB = ctx->channelParams.maxEnergyLeveldB;
        pMsg->msgData.edCnf.energyLevel      = Phy_GetEnergyLevel(ctx->channelParams.energyLeveldB);
        break;

    default:
        /* No aditional info needs to be filled */
        break;
    }

    ctx->PLME_MAC_SapHandler(pMsg, ctx->id);
}

/*! *********************************************************************************
* \brief  Senf a PD message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
static void PD_SendMessage(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msgType)
{
    pdDataToMacMessage_t *pMsg;

    if (!ctx || !ctx->PD_MAC_SapHandler)
    {
        return;
    }

    pMsg = MSG_Alloc(sizeof(pdDataToMacMessage_t) + ctx->rxParams.psduLength);

    if (NULL == pMsg)
    {
        phyTimeEvent_t ev;

        ctx->delayed_msg = msgType;

        ev.callback = Phy_SendLatePD;
        ev.parameter = ctx->id;
        ev.timestamp = gPhyRxRetryInterval_c + PhyTime_GetTimestamp();

        PhyTime_ScheduleEvent(&ev);
        return;
    }

    pMsg->fc = ctx->frameCounter - 1;   /* frame counter is updated after it's used */

    if (msgType == gPdDataInd_c)
    {
        pMsg->msgData.dataInd.pPsdu = (uint8_t *)pMsg + sizeof(pdDataToMacMessage_t);

        memcpy(pMsg->msgData.dataInd.pPsdu, ctx->trx_buff, ctx->rxParams.psduLength);

        pMsg->msgType = gPdDataInd_c;
        pMsg->msgData.dataInd.ppduLinkQuality = ctx->rxParams.linkQuality;
        pMsg->msgData.dataInd.ppduRssi = ctx->rxParams.rssi;
        pMsg->msgData.dataInd.psduLength = ctx->rxParams.psduLength;
        pMsg->msgData.dataInd.rxAckFp = !!(ctx->flags & gPhyFlagTxAckFP_c);
        if (ctx->flags & gPhyFlagNbTblRxAckFP_c)
        {
            pMsg->msgData.dataInd.rxAckFpReason = gPhyRxAckFpNoNeighbour;
        }
        else
        {
            pMsg->msgData.dataInd.rxAckFpReason = gPhyRxAckFpIndirect;
        }

        /* Passing FP bit to upper stack when using SW Enh Ack
           along with first RX Data (Poll) indication
           gPhyFlagTxAckFP_c and gPhyFlagNbTblRxAckFP_c must be
           cleared in ctx.flags */
        ctx->flags &= ~gPhyFlagTxAckFP_c;
        ctx->flags &= ~gPhyFlagNbTblRxAckFP_c;

        /* SFD detection timestamp */
        pMsg->msgData.dataInd.timeStamp = ctx->rxParams.timeStamp & gPhyTimeMask_c;

        pMsg->msgData.dataInd.ackedWithSecEnhAck = ctx->rxParams.ackedWithSecEnhAck;

        /* Reset value to prevent any unwanted scenario */
        ctx->rxParams.ackedWithSecEnhAck = FALSE;

        if (pMsg->msgData.dataInd.ackedWithSecEnhAck)
        {
            pMsg->msgData.dataInd.ackFrameCounter = ctx->frameCounter - 1;                                           /* frame counter was incremented */
            pMsg->msgData.dataInd.ackKeyId        = PhyPacket_GetKeyIndex(ctx->trx_buff, ctx->rxParams.psduLength);  /* security key index of received frame */
        }

        ctx->PD_MAC_SapHandler(pMsg, ctx->id);
    }
    else
    {
        phyStatus_t status = gPhySuccess_c;

        pMsg->msgData.dataCnf.ackData = (uint8_t *)pMsg + sizeof(pdDataToMacMessage_t);

        if ((ctx->flags & gPhyFlagRxFP_c) == gPhyFlagRxFP_c)
        {
            ctx->flags &= ~(gPhyFlagRxFP_c);
            status = gPhyFramePending_c;
        }

        pMsg->msgType = gPdDataCnf_c;
        pMsg->msgData.dataCnf.status = status;
        pMsg->msgData.dataCnf.ackLength = ctx->rxParams.psduLength;
        pMsg->msgData.dataCnf.ppduLinkQuality = ctx->rxParams.linkQuality;  /* valid if AR=1 */
        pMsg->msgData.dataCnf.ppduRssi = ctx->rxParams.rssi;                /* valid if AR=1 */

        /* SFD detection timestamp */
        pMsg->msgData.dataCnf.timeStamp = ctx->rxParams.timeStamp & gPhyTimeMask_c;

        memcpy(pMsg->msgData.dataCnf.ackData, ctx->trx_buff, ctx->rxParams.psduLength);

        ctx->PD_MAC_SapHandler(pMsg, ctx->id);
    }
}

static void Phy_SendLatePLME(uint32_t param)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(param);

    PLME_SendMessage(ctx, ctx->delayed_msg);
}

static void Phy_SendLatePD(uint32_t param)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(param);

    PD_SendMessage(ctx, ctx->delayed_msg);
}

void Radio_Phy_Notify(Phy_PhyLocalStruct_t *ctx)
{
    Phy24Task(ctx);
}

#if gMWS_Enabled_d || defined(CTX_SCHED)
/* TMR4 management */
/* switch timer - 0, scheduler timer - 1 */
#define T4_CNT 2

/* timer states */
#define T4_OFF 0    /* inactive */
#define T4_ON  1    /* active */
#define T4_RUN 2    /* running */
#define T4_EXP 3    /* expired */

static uint8_t t4_timer_state[T4_CNT];
static uint32_t t4_timer_tstp[T4_CNT];

static bool_t t1_near_t2(uint32_t tstp_1, uint32_t tstp_2)
{
    tstp_1 &= gPhyTimeMask_c;
    tstp_2 &= gPhyTimeMask_c;

    return ((tstp_1 - tstp_2 <= PHY_TMR_CMP_MIN) || (tstp_2 - tstp_1 <= PHY_TMR_CMP_MIN));
}

static void start_t4(uint32_t tstp)
{
    tstp &= gPhyTimeMask_c;

    /* is current tstp in the past or close to current time? */
    uint32_t phy_tstp = PhyTime_ReadClock() & gPhyTimeMask_c;

    if (t1_less_t2(tstp, phy_tstp) || t1_near_t2(tstp, phy_tstp))
    {
        tstp = (PhyTime_ReadClock() + PHY_TMR_CMP_MIN) & gPhyTimeMask_c;
    }

    TMR_UNMASK_AND_SET(4, tstp);
}

static void start_t4_timer(bool_t id, uint32_t dt)
{
    /* id - selects between switch timer (FALSE) and scheduler timer (TRUE) */

    if (t4_timer_state[id] != T4_OFF)
    {
        /* timer is already started */
        return;
    }

    /* start timer */
    t4_timer_state[id] = T4_ON;
    t4_timer_tstp[id] = (dt + PhyTime_ReadClock()) & gPhyTimeMask_c;

    /* no active timers yet */
    if ((t4_timer_state[!id] == T4_OFF) ||
        (t4_timer_state[!id] == T4_EXP))
    {
        start_t4(t4_timer_tstp[id]);

        t4_timer_state[id] = T4_RUN;
        return;
    }

    /* no running timers */
    if (t4_timer_state[!id] == T4_ON)
    {
        /* select closest timestamp */
        bool_t current = id;

        if (t1_less_t2(t4_timer_tstp[!id], t4_timer_tstp[id]))
        {
            current = !id;
        }

        t4_timer_state[current] = T4_RUN;

        /* is the other timestamp close? */
        if (t1_near_t2(t4_timer_tstp[current], t4_timer_tstp[!current]))
        {
            t4_timer_state[!current] = T4_RUN;;
        }

        start_t4(t4_timer_tstp[current]);
        return;
    }

    /* a timer is already running */
    if (t4_timer_state[!id] == T4_RUN)
    {
        /* are the timestamps close? */
        if (t1_near_t2(t4_timer_tstp[id], t4_timer_tstp[!id]))
        {
            t4_timer_state[id] = T4_RUN;
            return;
        }

        /* timestamp is far away */
        if (t1_less_t2(t4_timer_tstp[!id], t4_timer_tstp[id]))
        {
            return;
        }

        /* timestamp is closer but the other timer has already expired */
        if (ZLL->IRQSTS & ZLL_IRQSTS_TMR4IRQ_MASK)
        {
            t4_timer_state[id] = T4_RUN;
            return;
        }

        /* switch the running timer */
        start_t4(t4_timer_tstp[id]);

        t4_timer_state[id] = T4_RUN;
        t4_timer_state[!id] = T4_ON;
        return;
    }
}

static void stop_t4_timer(bool_t id)
{
    t4_timer_state[id] = T4_OFF;

    if (t4_timer_state[!id] == T4_ON)
    {
        start_t4(t4_timer_tstp[!id]);

        t4_timer_state[!id] = T4_RUN;
    }
    else if ((t4_timer_state[!id] == T4_OFF) ||
             (t4_timer_state[!id] == T4_EXP))
    {
        TMR_CLEAR(4);
    }
}

static bool_t t4_timer_expired(bool_t id)
{
    /* timer expired */
    if ((t4_timer_state[id] == T4_RUN) &&
        (ZLL->IRQSTS & ZLL_IRQSTS_TMR4IRQ_MASK))
    {
        t4_timer_state[id] = T4_EXP;

        if (t4_timer_state[!id] == T4_RUN)
        {
            t4_timer_state[!id] = T4_EXP;
        }
    }

    if (t4_timer_state[id] != T4_EXP)
    {
        return FALSE;
    }

    stop_t4_timer(id);

    return TRUE;
}
#endif

#if (gMWS_Enabled_d) || (gMWS_UseCoexistence_d)
static void mode_switch_ZB()
{
    const xcvr_config_t *xcvrConfig = &xcvr_oqpsk_802p15p4_250kbps_full_config;
    const xcvr_coding_config_t *rbmeConfig = &xcvr_ble_uncoded_config;

    if (XCVR_GetActiveLL() == XCVR_ACTIVE_LL_ZIGBEE_LL)
    {
        return;
    }

    XCVR_ChangeMode(&xcvrConfig, &rbmeConfig);
    XCVR_SetActiveLL(XCVR_ACTIVE_LL_ZIGBEE_LL);

    /*
     * Also restore the settings that need to be overridden (i.e. parameters
     * that are different from the default ones available in the OQPSK config)
     */
    PhyPlatformHwInit();

    /*
     * In the process of dynamically changing modes between BLE and 15.4 Phy,
     *  BLE may change LDO_ANT_TRIM value
     *  restore LDO ANT TRIM to value maintained and expected by 15.4
     */
    XCVR_setLdoAntTrim(g_ldo_ant_trim_15_4);

    /* Reset TX state machine -> MATTER-256. This is a software workaround to reset the TX done
    signal that remains asserted when the ZB LL starts a transmission during BLE active time.
    This transmission is triggered by TSM tx_dig_enable signal that is common for both BLE and 15.4.
    The first real TX that will happen will complete immediately because the TX done signal
    is already asserted. */
    uint32_t *resetReg = (uint32_t *)TX_DONE_RESET_REGISTER;
    *resetReg |= TX_DONE_RESET_REGISTER_MASK;
    *resetReg &= ~TX_DONE_RESET_REGISTER_MASK;

#if gMWS_UseCoexistence_d && gMWS_Enabled_d
    /* Restore Pin settings */
    MWS_CoexistenceInit(&gCoexistence_RfDeny, &gCoexistence_RfActive, &gCoexistence_RfStatus);
#endif
}

static void mode_switch_BLE()
{
    const xcvr_config_t           *xcvrConfigBLE = &xcvr_gfsk_bt_0p5_h_0p5_1mbps_full_config;
    const xcvr_coding_config_t    *rbmeConfigBLE = &xcvr_ble_coded_s8_config;

    if (XCVR_GetActiveLL() == XCVR_ACTIVE_LL_BTLE)
    {
        return;
    }

    XCVR_ChangeMode(&xcvrConfigBLE, &rbmeConfigBLE);
    XCVR_SetActiveLL(XCVR_ACTIVE_LL_BTLE);

    /*
     * In the process of dynamically changing modes between BLE and 15.4 Phy,
     * 15.4 Phy Layer may change LDO_ANT_TRIM value
     * restore LDO ANT TRIM to value maintained and expected by BLE
     */
    extern void Controller_RestoreLdoAntTrim(void);
    Controller_RestoreLdoAntTrim();
}

/*! *********************************************************************************
* \brief  This function represents the callback used by the MWS module to signal
*         events to the 802.15.4 PHY
*
* \param[in]  event - the MWS event
*
* \return  status
*
********************************************************************************** */
static uint32_t MWS_802_15_4_Callback(mwsEvents_t event)
{
    uint32_t status = gMWS_Success_c;

    OSA_InterruptDisable();

    switch (event)
    {
    case gMWS_Active_c:
        mode_switch_ZB();
        phy_is_active = TRUE;
        break;

    case gMWS_Idle_c:
        PHY_ForceIrqPending();  /* run the PHY state machine from PHY ISR context only */
        break;

    case gMWS_Abort_c:
        PHY_sw_abort();

    case gMWS_Release_c:
        mode_switch_BLE();

    case gMWS_Init_c:
        phy_is_active = FALSE;
        break;

    case gMWS_GetInactivityDuration_c:
        status = 0;     /* Default status is 0 (Busy)  */
        if (gIdle_c == PhyPpGetState())
        {
            status = 0xFFFFFFFFu;
        }
        break;

    default:
        status = gMWS_InvalidParameter_c;
        break;
    }

    OSA_InterruptEnable();
    return status;
}

static void stop_switch_timer()
{
    stop_t4_timer(FALSE);
}

static void start_switch_timer(uint32_t dt)
{
    start_t4_timer(FALSE, dt);
}

static bool_t switch_timer_expired()
{
    return t4_timer_expired(FALSE);
}

/* - Maximum frame size with preamble: 6 * 2 + 127 * 2 symbols
 * - AIFS: 12 symbols
 * - Maximum ACK size with preamble: 6 * 2 + 39 * 2 symbols
 * (destination PAN ID, extended destination/source address, CSL IE) */
#define SWITCH_MIN_TIME (uint32_t)370       /* symbols */
#define SWITCH_MAX_TIME (uint32_t)((1 << (gPhyTimeShift_c - 1)) - 1)    /* to allow signed comparisons of timestamps */

static bool_t delay_phy_release;

void do_ble_phy_coex()
{
    delay_phy_release = FALSE;

    if (!phy_is_active)
    {
        /* gMWS_Abort_c doesn't stop switch timer */
        stop_switch_timer();
    }
    else if (switch_timer_expired())
    {
        bool_t seq_end = !!(ZLL->IRQSTS & ZLL_IRQSTS_SEQIRQ_MASK);

        stop_switch_timer();

        /* let current trx finish and handle mode switch later */
        if (seq_end)
        {
            delay_phy_release = TRUE;
        }
        else
        {
            PHY_sw_abort();
            MWS_Release(gMWS_802_15_4_c);
        }
    }

    if (!phy_is_active)
    {
        uint32_t dt = MWS_GetInactivityDuration(gMWS_802_15_4_c) / PHY_SYMBOLS_US;

        /* if gMWS_Abort_c/gMWS_Release_c happens, there shouldn't be enough time left */
        if (dt >= SWITCH_MIN_TIME)
        {
            MWS_Acquire(gMWS_802_15_4_c, FALSE);

            if (phy_is_active)
            {
                /* if dt is very big, then the BLE LL is not active */
                if (dt < SWITCH_MAX_TIME)
                {
                    start_switch_timer(dt);
                }
                delay_phy_release = FALSE;
            }
        }
    }
}

bool_t PHY_is_active()
{
    if (delay_phy_release)
    {
        delay_phy_release = FALSE;
        PHY_sw_abort();     /* ED request restarts several times at seq_end */
        MWS_Release(gMWS_802_15_4_c);
    }
    return phy_is_active;
}

static void PHY_set_inactive()
{
    if (phy_is_active)
    {
        delay_phy_release = FALSE;
        PhyAbort();
        MWS_Release(gMWS_802_15_4_c);
    }
}
#endif

static bool_t PHY_ctx_can_sleep()
{
    uint8_t id;
    Phy_PhyLocalStruct_t *ctx;

    for (id = 0; id < CTX_NO; id++)
    {
        ctx = ctx_get(id);

        if (ctx->ps == PS_RX)
        {
            return FALSE;
        }

        if (ctx->flags & gPhyFlagRxOnWhenIdle_c)
        {
            return FALSE;
        }

        if (MSG_Pending(&ctx->macPhyInputQueue))
        {
            return FALSE;
        }

#ifdef CTX_SCHED
        if ((ctx->state != E_SCHED_PROTO_OFF) && ctx->op)
        {
            return FALSE;
        }
#endif
    }

    return TRUE;

}

/*! *********************************************************************************
* \brief  returns true if 15.4 PHY allows sleep, returns false others ways
*
********************************************************************************** */
bool PHY_XCVR_AllowLowPower(void)
{
    OSA_InterruptDisable();

#if gMWS_Enabled_d
    if (!phy_is_active && PhyTime_can_sleep())
    {
        OSA_InterruptEnable();
        return true;
    }
#endif

    if ((PhyPpGetState() == gIdle_c) && PHY_ctx_can_sleep() && PhyTime_can_sleep())
    {
#if gMWS_Enabled_d
        PHY_set_inactive();
#endif
#ifdef CTX_SCHED
        sched_reset();  /* disable scheduler */
#endif
        OSA_InterruptEnable();
        return true;
    }

    OSA_InterruptEnable();
    return false;
}



Phy_PhyLocalStruct_t ctxs[CTX_NO];


#ifdef CTX_SCHED

#define E_SCHED_MIN_PRIORITY 0

#define SCHED_DEFAULT_DUTY_CYCLE_TICK 3750  /* symbols (60 ms) */
#define SCHED_DEFAULT_PRIO_TIME_TICK 1000   /* symbols (16 ms) */
#define SCHED_RETRY_TICK 544                /* symbols (8.7 ms). 2 max length frames + AIFS */
#define SCHED_MIN_TICK PHY_TMR_CMP_MIN      /* symbols */

struct sched_ctx
{
    bool_t active;

    sched_policy policy;
    sched_policy next_policy;
    uint8_t lock_tag;

    Phy_PhyLocalStruct_t *current;
    Phy_PhyLocalStruct_t *next;

    bool_t wait_idle;       /* wait for idle state before switching contexts */
    bool_t switch_later;    /* context switch is delayed */

    bool_t auto_rx;         /* auto dual PAN allowed */
    bool_t rx_all;          /* all contexts are in rx (auto rx / dual PAN) */
    bool_t rxed_on_all;     /* auto dual PAN on single channel: reception successful on both PANs (broadcast frame) */
};

struct sched_ctx scheduler;
#endif

uint8_t PHY_get_ctx()
{
    uint8_t id;

#ifdef CTX_SCHED
    OSA_InterruptDisable();

    for (id = 0; id < CTX_NO; id++)
    {
        Phy_PhyLocalStruct_t *ctx = ctx_get(id);

        if (ctx->state == E_SCHED_PROTO_OFF)
        {
            ctx_init_single(id);
            ctx->state = E_SCHED_PROTO_INACTIVE;

            OSA_InterruptEnable();

            return id;
        }
    }

    OSA_InterruptEnable();
#endif

    id = 0;

    ctx_init_single(id);

    return id;
}

void ctx_init()
{
    uint8_t id = 0;

    for (id = 0; id < CTX_NO; id++)
    {
        ctx_init_single(id);
    }
}

void ctx_init_single(uint8_t id)
{
    Phy_PhyLocalStruct_t *ctx;

    OSA_InterruptDisable();

    ctx = ctx_get(id);

    ctx->id = id;
    ctx->flags = 0;

    /* Prepare input queues.*/
    MSG_QueueInit(&ctx->macPhyInputQueue);

    ctx->mPhyLastRxLQI = 0;
    ctx->mPhyLastRxRSSI = 0;

    /* header IE: CSL with length 4 */
    ctx->ackIeDataAndHdr[0] = 0x04;
    ctx->ackIeDataAndHdr[1] = 0x0d;

    /* Vendor Specific Header IE */
    memset(ctx->phyIeDataTable, 0, sizeof(ctx->phyIeDataTable));

    ctx->phySecurity = FALSE;
    ctx->keyId = 0;
    ctx->frameCounter = 0;
    ctx->updateFrameCounter = 0;

    ctx->efp = TRUE;

    ctx->filter_fail = 0;
    ctx->neighbourTblEnabled = FALSE;

    ctx->rx_time_poll = PHY_RX_TIME_POLL;
    ctx->ps = PS_NONE;

#ifdef CTX_SCHED
    ctx->state = E_SCHED_PROTO_OFF;
    ctx->op = NONE_OP;

    ctx->op_pending = FALSE;
    ctx->rx_ongoing = FALSE;

    ctx->priority = E_SCHED_MIN_PRIORITY;
    ctx->slice = SCHED_DEFAULT_DUTY_CYCLE_TICK;
    ctx->lock_timeout = 0;
#endif

    OSA_InterruptEnable();
}

void PHY_release_ctx(uint8_t id)
{
    Phy_PhyLocalStruct_t *ctx;
    phyMessageHeader_t *pMsgIn = NULL;

#ifdef CTX_SCHED
    bool_t was_current = 0;
#endif

    OSA_InterruptDisable();

    ctx = ctx_get(id);

#ifdef CTX_SCHED
    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        was_current = 1;
    }
#endif

    PhyAbort_base(ctx);

    /* Empty the macPhyInputQueue */
    while (MSG_Pending(&ctx->macPhyInputQueue))
    {
        pMsgIn = MSG_DeQueue(&ctx->macPhyInputQueue);

        if (pMsgIn != NULL)
        {
            MSG_Free(pMsgIn);
        }
    }

#ifdef CTX_SCHED
    sched_stop_timer();

    if (was_current)
    {
        scheduler.current = NULL;
    }

    ctx->state = E_SCHED_PROTO_OFF;
    sched_start_timer(SCHED_MIN_TICK);
#endif

    OSA_InterruptEnable();
}

Phy_PhyLocalStruct_t *ctx_get(instanceId_t id)
{
    /* return from ctx array */
    if (id >= CTX_NO)
    {
        id = 0;
    }

    return &ctxs[id];
}

Phy_PhyLocalStruct_t *ctx_get_current()
{
#ifdef CTX_SCHED
    return scheduler.current;
#else
    return ctx_get(0);
#endif
}

#ifdef CTX_SCHED

#if !gMWS_Enabled_d
#define do_ble_phy_coex()
#define PHY_is_active() TRUE
#endif  /* gMWS_Enabled_d */

void sched_start_timer(uint32_t ticks)
{
    start_t4_timer(TRUE, ticks);
}

void sched_stop_timer()
{
    stop_t4_timer(TRUE);
}

static bool_t sched_timer_expired()
{
    return t4_timer_expired(TRUE);
}

void sched_reschedule()
{
    sched_stop_timer();

    scheduler.switch_later = TRUE;

    sched_start_timer(SCHED_MIN_TICK);
}

bool_t sched_is_auto_rx()
{
    bool_t rx_one_ch = (ZLL->CHANNEL_NUM0 == ZLL->CHANNEL_NUM1);    /* implicit auto dual PAN */

    return (rx_one_ch || scheduler.auto_rx);
}

void ctx_set_pending(Phy_PhyLocalStruct_t *ctx)
{
    ctx->op_pending = TRUE;
}

bool_t ctx_is_active(Phy_PhyLocalStruct_t *ctx)
{
    return (ctx->state == E_SCHED_PROTO_ACTIVE);
}

bool_t ctx_is_paused(Phy_PhyLocalStruct_t *ctx)
{
    return (ctx->state == E_SCHED_PROTO_PAUSING);
}

static void ctx_plme_msg_all(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msg_type)
{
    if (!scheduler.rx_all || !ctx)
    {
        return;
    }

    Phy_PhyLocalStruct_t *ctx_2 = ctx_get((ctx->id + 1) % CTX_NO);

    if (!(ctx_2->flags & gPhyFlagIdleRx_c) && (ctx_2->ps != PS_RX))
    {
        PLME_SendMessage(ctx_2, msg_type);
    }

    ctx_2->ps = PS_NONE;
}

void ctx_data_ind_all(Phy_PhyLocalStruct_t *ctx)
{
    /* broadcast frames, auto dual PAN on single channel.
       Send two data indication */

    if (!scheduler.rxed_on_all || !ctx)
    {
        return;
    }

    Phy_PhyLocalStruct_t *ctx_2 = ctx_get((ctx->id + 1) % CTX_NO);

    ctx_2->rxParams = ctx->rxParams;
    memcpy(ctx_2->trx_buff, ctx->trx_buff, ctx->rxParams.psduLength);

    uint16_t fcf = PHY_TransformArrayToUint16(ctx_2->trx_buff);

    if (fcf & phyFcfAckRequest)
    {
        /* stop rx for POLL after a unicast packet is received */
        ctx_2->ps = PS_NONE;
    }

    /* Phy_GetRxInfo() */
    ctx_2->mPhyLastRxRSSI = ctx->mPhyLastRxRSSI;
    ctx_2->mPhyLastRxLQI = ctx->mPhyLastRxLQI;

    scheduler.rxed_on_all = FALSE;

    PD_SendMessage(ctx_2, gPdDataInd_c);
}

bool_t all_ctx_rx()
{
    uint8_t id;
    Phy_PhyLocalStruct_t *ctx, *ctx_2;

    for (id = 0; id < (CTX_NO - 1); id++)
    {
        ctx = ctx_get(id);
        ctx_2 = ctx_get(id + 1);

        if (!((ctx->state != E_SCHED_PROTO_OFF) && (ctx_2->state != E_SCHED_PROTO_OFF) &&
              (ctx->op == RX_OP) && (ctx_2->op == RX_OP) &&
              (ctx->rxParams.startTime == ctx_2->rxParams.startTime) &&
              (ctx->rxParams.duration == ctx_2->rxParams.duration)))
        {
            return FALSE;
        }
    }

    return TRUE;
}

void ctx_set_rx_ongoing(Phy_PhyLocalStruct_t *ctx, bool_t status)
{
    ctx->rx_ongoing = status;
}

void ctx_set_rx(Phy_PhyLocalStruct_t *ctx)
{
    ctx->op = RX_OP;
    ctx->tstp = (uint32_t)PhyTime_ReadClock();

    if (!scheduler.rx_all && sched_is_auto_rx() && all_ctx_rx())
    {
        /* force context switch */
        sched_stop_timer();

        scheduler.switch_later = TRUE;

        sched_start_timer(SCHED_MIN_TICK);
    }
}

void ctx_set_tx(Phy_PhyLocalStruct_t *ctx, macToPdDataMessage_t *pMsg)
{
    ctx->op = TX_OP;
    ctx->tstp = (uint32_t)PhyTime_ReadClock();

    memcpy(ctx->trx_buff, pMsg->msgData.dataReq.pPsdu, pMsg->msgData.dataReq.psduLength);

    ctx->tx_data_req = pMsg->msgData.dataReq;
    ctx->txParams.dataReq = &ctx->tx_data_req;
    ctx->txParams.dataReq->pPsdu = ctx->trx_buff;
}

void ctx_set_cca(Phy_PhyLocalStruct_t *ctx)
{
    ctx->op = CCA_OP;
    ctx->tstp = (uint32_t)PhyTime_ReadClock();
}

void ctx_set_none(Phy_PhyLocalStruct_t *ctx)
{
    ctx->op = NONE_OP;
}

bool_t start_rx_all()
{
    uint8_t id;
    Phy_PhyLocalStruct_t *ctx;

    if (!(sched_is_auto_rx() && all_ctx_rx()))
    {
        return FALSE;
    }

    for (id = 0; id < CTX_NO; id++)
    {
        ctx = ctx_get(id);
        ctx->state = E_SCHED_PROTO_INACTIVE;
        ctx->filter_fail = 0;
        ctx->rx_ongoing = FALSE;
        PHY_set_enh_ack_state(ctx, ENH_ACK_INVALID);
    }

    scheduler.rx_all = TRUE;
    scheduler.rxed_on_all = FALSE;

    scheduler.current = NULL;
    scheduler.next = NULL;

    PhyPpSetDualPanAuto(TRUE);
    PhyPpSetDualPanActiveNwk(0);
    PhyPpSetDualPanDwell(0x8);     /* 1.5ms dwell time - half of total tx time for data req */

    /* suspend the scheduler, rely on PHY events */
    sched_stop_timer();

    return TRUE;
}

void proto_restore(Phy_PhyLocalStruct_t *ctx)
{
    if (start_rx_all())
    {
        PhyPlmeRxRequest(ctx_get(0));   /* ctx could be NULL */
        return;
    }

    if (!ctx)
    {
        return;
    }

#ifdef PHY_WLAN_COEX
    if (ZLL->CHANNEL_NUM0 != ZLL->CHANNEL_NUM1)
    {
        PhyWlanCoex_ChannelUpdate(ctx->id ? ZLL->CHANNEL_NUM1 : ZLL->CHANNEL_NUM0);
    }
#endif

    scheduler.rx_all = FALSE;
    scheduler.rxed_on_all = FALSE;

    /* manual dual PAN */
    PhyPpSetDualPanAuto(FALSE);
    PhyPpSetDualPanActiveNwk(ctx->id);

    /* start rx / tx / CCA */
    switch (ctx->op)
    {
        case RX_OP:
            PhyPlmeRxRequest(ctx);
            break;

        case TX_OP:
            PhyPdDataRequest(ctx);
            break;

        case CCA_OP:
            PhyPlmeCcaEdRequest(ctx);
            break;

        default:
            break;
    }
}

void get_next_proto_rr(Phy_PhyLocalStruct_t **ctx)
{
    /* operation priority: tx, cca, rx. Select oldest when it's a tie */
    uint8_t id = 0;
    uint8_t cnt;
    Phy_PhyLocalStruct_t *t = NULL, *t2 = NULL;
    uint32_t neg_msk = (1 << (gPhyTimeShift_c - 1));    /* 24bit hw timer */

    if (scheduler.current)
    {
        id = scheduler.current->id;
    }

    for (cnt = 0; cnt < CTX_NO; cnt++)
    {
        id = (id + 1) % CTX_NO;

        t = ctx_get(id);

        if ((t->state != E_SCHED_PROTO_OFF) && t->slice && t->op)
        {
            if (!sched_is_auto_rx())
            {
                t2 = t;
                break;
            }
            else if (!t2 ||
                     ((t2->ps != PS_RX) &&
                      ((t->ps == PS_RX) || (t2->op < t->op) ||
                       ((t2->op == t->op) && ((t->tstp - t2->tstp) & neg_msk)))))
            {
                t2 = t;
            }
        }
    }

    *ctx = t2;
}

/* Advance scheduler protocol and policy */
void next_proto(void)
{
    /*
     * Next protocol is needed in multiple places.
     * Compute it only once and save it in scheduler ctx.
     * It is reset after scheduling (schedule()).
     */

    if (scheduler.next)
    {
        return;
    }

    /* Need to switch policy */
    if (E_SCHED_NO_POLICY != scheduler.next_policy)
    {
        scheduler.policy      = scheduler.next_policy;
        scheduler.next_policy = E_SCHED_NO_POLICY;
    }

    /* just Round Robin for now */
    get_next_proto_rr(&scheduler.next);

    if (!scheduler.next)
    {
        scheduler.next = scheduler.current;
    }

    return;
}

void sched_abort_current()
{
    PhyAbort();

    if (scheduler.current)
    {
        /* send notification to MAC */
        Radio_Phy_AbortIndication(scheduler.current);
    }

    sched_reset();
}

/* force_switch is used when wifi access was not granted for current protocol
   so we have to retry when there is no protocol switch */
void schedule(bool_t force_switch)
{
    uint32_t slice;

    /* Save context for current proto only if the next to schedule is not the same. */
    if ((NULL != scheduler.current) && (scheduler.next != scheduler.current))
    {
        /* Set the old proto state to inactive */
        scheduler.current->state = E_SCHED_PROTO_INACTIVE;
    }

    /* Restore context for next protocol */
    if ((scheduler.next != scheduler.current) || force_switch)
    {
        scheduler.current = scheduler.next;
        proto_restore(scheduler.next);
    }

    if (scheduler.rx_all)
    {
        /* suspend the scheduler, rely on PHY events */
        return;
    }

    scheduler.next = NULL;

    /* Fail safe check */
    if (!scheduler.current)
    {
        sched_start_timer(SCHED_DEFAULT_DUTY_CYCLE_TICK);
        return;
    }

    scheduler.current->state = E_SCHED_PROTO_ACTIVE;

    switch (scheduler.policy)
    {
        case E_SCHED_DUTY_CYCLE:
            slice = scheduler.current->slice;
            break;
        case E_SCHED_LOCKED:
            slice = scheduler.current->lock_timeout;
            break;
        case E_SCHED_PRIORITY:
            slice = SCHED_DEFAULT_PRIO_TIME_TICK;
            break;
        default:
            /* Error */
            sched_start_timer(SCHED_DEFAULT_DUTY_CYCLE_TICK);
            return;
    }

    /* Reschedule timer */
    sched_start_timer(slice);
}

static void sched_reset()
{
    uint8_t id;
    Phy_PhyLocalStruct_t *ctx;

    for (id = 0; id < CTX_NO; id++)
    {
        ctx = ctx_get(id);

        if (ctx->state != E_SCHED_PROTO_OFF)
        {
            ctx->state = E_SCHED_PROTO_INACTIVE;
            ctx->rx_ongoing = FALSE;
            ctx->filter_fail = 0;
            ctx->op_pending = TRUE;     /* select next operation when PHY is again active */

            if ((scheduler.current == ctx) || scheduler.rx_all)
            {
                /* context operation was canceled because PHY was deactivated */
                ctx->op = NONE_OP;
                ctx->flags &= ~gPhyFlagIdleRx_c;
                ctx->ps = PS_NONE;
            }
        }
    }

    scheduler.current = NULL;
    scheduler.next = NULL;

    scheduler.switch_later = FALSE;
    scheduler.rx_all = FALSE;
    scheduler.rxed_on_all = FALSE;

    sched_stop_timer();
}

void sched_enable()
{
    scheduler.active = TRUE;
    scheduler.policy = E_SCHED_NO_POLICY;
    scheduler.next_policy = E_SCHED_NO_POLICY;
    scheduler.wait_idle = TRUE;
    scheduler.auto_rx = gHwAutoDualPanMode_c;

    sched_reset();
}

static uint32_t ctx_match(uint8_t *p, uint8_t *a, uint8_t len)
{
    uint16_t pan = (p ? PHY_TransformArrayToUint16(p) : 0);
    uint64_t addr = 0, tmp;

    if (!a || ((len != sizeof(uint16_t)) && (len != sizeof(uint64_t))))
    {
        return INV_IDX;
    }

    memcpy((uint8_t *)&addr, a, len);

    /* PAN0 check */
    if (len == sizeof(uint16_t))
    {
        tmp = (ZLL->MACSHORTADDRS0 & ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK) >> ZLL_MACSHORTADDRS0_MACSHORTADDRS0_SHIFT;
    }
    else
    {
        tmp = ZLL->MACLONGADDRS0_MSB;
        tmp = (tmp << 32) | ZLL->MACLONGADDRS0_LSB;
    }

    if ((!p || (pan == BCST_PAN_ID) ||
         (pan == (ZLL->MACSHORTADDRS0 & ZLL_MACSHORTADDRS0_MACPANID0_MASK))) &&
        (tmp == addr))
    {
        return 0;
    }

    /* PAN1 check */
    if (len == sizeof(uint16_t))
    {
        tmp = (ZLL->MACSHORTADDRS1 & ZLL_MACSHORTADDRS1_MACSHORTADDRS1_MASK) >> ZLL_MACSHORTADDRS1_MACSHORTADDRS1_SHIFT;
    }
    else
    {
        tmp = ZLL->MACLONGADDRS1_MSB;
        tmp = (tmp << 32) | ZLL->MACLONGADDRS1_LSB;
    }

    if ((!p || (pan == BCST_PAN_ID) ||
         (pan == (ZLL->MACSHORTADDRS1 & ZLL_MACSHORTADDRS1_MACPANID1_MASK))) &&
        (tmp == addr))
    {
        return 1;
    }

    return INV_IDX;
}

/* if rx started / ended, try to select the context */
static void sched_set_current_ctx(uint32_t irq_status)
{
    uint32_t idx = INV_IDX;
    uint16_t fcf = 0;

    bool_t wtmrk = !!(irq_status & ZLL_IRQSTS_RXWTRMRKIRQ_MASK);
    bool_t seq_end = !!(irq_status & ZLL_IRQSTS_SEQIRQ_MASK);

    bool_t hdr_ok = (ZLL->RX_WTR_MARK != RX_WTMRK_START);
    bool_t rx_one_ch = (ZLL->CHANNEL_NUM0 == ZLL->CHANNEL_NUM1);

    uint8_t *addr = NULL, *pan = NULL, len = 0;

    if (!scheduler.rx_all)
    {
        return;
    }

    if (scheduler.current)
    {
        scheduler.rx_all = FALSE;
        scheduler.rxed_on_all = FALSE;
    }

    if (rx_one_ch)
    {
        if (wtmrk && hdr_ok)
        {
            fcf = PHY_TransformArrayToUint16(rxf);

            /* frame header is received, AR set */
            if (fcf & phyFcfAckRequest)
            {
                /* Select context from destination address in received frame (RX_PB) */
                PhyPacket_get_dest_pan_addr(rxf, &pan, &addr, &len);

                idx = ctx_match(pan, addr, len);
            }
        }
    }
    else if (wtmrk)
    {
        idx = !!(ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK_MASK);
    }

    if ((idx == INV_IDX) && seq_end)
    {
        if ((ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_MASK) &&
            (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_MASK))
        {
            idx = 0;
            scheduler.rxed_on_all = TRUE;
        }
        else if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_MASK)
        {
            idx = 0;
        }
        else if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_MASK)
        {
            idx = 1;
        }
    }

    if (idx != INV_IDX)
    {
        /* current context is valid. Reschedule at the end of reception */
        scheduler.rx_all = FALSE;

        scheduler.current = ctx_get(idx);
        scheduler.current->state = E_SCHED_PROTO_ACTIVE;

        scheduler.switch_later = TRUE;

#ifdef PHY_WLAN_COEX
        if (ZLL->CHANNEL_NUM0 != ZLL->CHANNEL_NUM1)
        {
            PhyWlanCoex_ChannelUpdate(idx ? ZLL->CHANNEL_NUM1 : ZLL->CHANNEL_NUM0);
        }
#endif
    }
}

static void sched_update_ctx_pending()
{
    uint8_t id;
    Phy_PhyLocalStruct_t *ctx;

    for (id = 0; id < CTX_NO; id++)
    {
        ctx = ctx_get(id);

        if ((ctx->state == E_SCHED_PROTO_INACTIVE) && ctx->op_pending)
        {
            ctx->op_pending = FALSE;

            Phy24Task(ctx);
        }
    }
}

/* both RPMSG/IMU and PHY IRQs have the same priority (4) */
void PHY_InterruptHandler()
{
    OSA_InterruptDisable();

    do_ble_phy_coex();

    uint8_t xcvseq = ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK;
    uint32_t irq_status = ZLL->IRQSTS;

    bool_t wtmrk = !!(irq_status & ZLL_IRQSTS_RXWTRMRKIRQ_MASK);
    bool_t seq_end = !!(irq_status & ZLL_IRQSTS_SEQIRQ_MASK);
    bool_t sched_switch = !!sched_timer_expired();
    bool_t proto_switch = FALSE; /* Need to change protocol */

    update_rxf(xcvseq, irq_status);

    sched_set_current_ctx(irq_status);

    sched_switch |= scheduler.switch_later;

    if (sched_switch)
    {
        sched_stop_timer();

        next_proto();

        if (scheduler.current != scheduler.next)
        {
            proto_switch = TRUE;
        }
    }

    proto_switch |= scheduler.switch_later;

    if (scheduler.current && (scheduler.current->op == RX_OP))
    {
        if (wtmrk && !scheduler.current->rx_ongoing)
        {
            scheduler.current->rx_ongoing = TRUE;
        }

        if (scheduler.current->rx_ongoing && seq_end)
        {
            scheduler.current->rx_ongoing = FALSE;
        }
    }

    if (scheduler.current && proto_switch)
    {
        /* Prevent a new tx/rx to be started if next != current */
        scheduler.current->state = E_SCHED_PROTO_PAUSING;
    }

    PHY_InterruptHandler_base(xcvseq,  irq_status);

    if (!PHY_is_active())
    {
        /* check again sw abort. Could happen because ED request restarts several times at seq_end  */
        bool_t sw_abort = !!(ZLL->SEQ_CTRL_STS & ZLL_SEQ_CTRL_STS_SW_ABORTED_MASK);
        seq_end = !!(ZLL->IRQSTS & ZLL_IRQSTS_SEQIRQ_MASK);

        if (!(seq_end && sw_abort))
        {
            sched_reset();
        }

        OSA_InterruptEnable();
        return;
    }

    /* No PAN selected, because of filter fail, timeout or abort but rx finished */
    if (scheduler.rx_all && (PhyPpGetState() == gIdle_c))
    {
        sched_reset();

        sched_switch = TRUE;
        proto_switch = TRUE;
    }

    if (proto_switch)
    {
        if (!scheduler.wait_idle)
        {
            sched_abort_current();
        }
        else if ((scheduler.current &&
                  ((scheduler.current->ps == PS_RX) ||
                   ((scheduler.current->op == RX_OP) && scheduler.current->rx_ongoing))) ||
                 !PHY_graceful_idle())
        {
            scheduler.switch_later = TRUE;
            sched_switch = FALSE;
        }
    }

    if (sched_switch)
    {
        if (proto_switch)
        {
            /* No context is active at this point, PHY is idle */
            if (scheduler.current)
            {
                /* switch from transitory state so Phy24Task() doesn't enter idle */
                scheduler.current->state = E_SCHED_PROTO_INACTIVE;
            }

            /* Check for pending op queue */
            sched_update_ctx_pending();
        }

        if (sched_is_auto_rx() && (proto_switch || !scheduler.current))
        {
            /* late context selection to account for
               tx just set on the current context while the
               next context has rx */
            scheduler.next = NULL;

            next_proto();
        }

        scheduler.switch_later = FALSE;

        scheduler.rx_all = FALSE;
        scheduler.rxed_on_all = FALSE;

        /* Schedule next */
        schedule(proto_switch);
    }

    /* fail safe: in case there is no sequence-complete interrupt later */
    if (scheduler.switch_later)
    {
        sched_start_timer(SCHED_RETRY_TICK);
    }

    OSA_InterruptEnable();
}

void ProtectFromXcvrInterrupt_base(Phy_PhyLocalStruct_t *ctx)
{
    OSA_InterruptDisable();

    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        ProtectFromXcvrInterrupt();
    }

    OSA_InterruptEnable();
}

void UnprotectFromXcvrInterrupt_base(Phy_PhyLocalStruct_t *ctx)
{
    OSA_InterruptDisable();

    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        UnprotectFromXcvrInterrupt();
    }

    OSA_InterruptEnable();
}

uint8_t PhyPpGetState_base(Phy_PhyLocalStruct_t *ctx)
{
    uint8_t state = gIdle_c;

    if (!ctx)
    {
        return state;
    }

    OSA_InterruptDisable();

    switch (ctx->op)
    {
        case RX_OP:
            state = gRX_c;
            break;

        case TX_OP:
            if (ctx->txParams.dataReq->ackRequired != gPhyNoAckRqd_c)
            {
                state = gTR_c;
            }
            else
            {
                state = gTX_c;
            }
            break;

        case CCA_OP:
            if (ctx->ccaParams.cccaMode == gPhyContCcaEnabled)
            {
                state = gCCCA_c;
            }
            else
            {
                state = gCCA_c;
            }
            break;

        default:
            break;
    }

    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        state = PhyPpGetState();
    }

    OSA_InterruptEnable();

    return state;
}

void PhyAbort_base(Phy_PhyLocalStruct_t *ctx)
{
    OSA_InterruptDisable();

    if (!ctx || scheduler.rx_all || ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        PhyAbort();

        if (!ctx || scheduler.rx_all)
        {
            /* force context switch */
            sched_stop_timer();

           scheduler.rx_all = FALSE;
           scheduler.rxed_on_all = FALSE;

           scheduler.switch_later = TRUE;

           sched_start_timer(SCHED_MIN_TICK);
        }
    }

    if (ctx)
    {
        ctx->op          = NONE_OP;
        ctx->filter_fail = 0;
        ctx->rx_ongoing  = FALSE;
        ctx->ps = PS_NONE;
    }

    OSA_InterruptEnable();
}

bool_t PHY_graceful_idle_base(Phy_PhyLocalStruct_t *ctx)
{
    bool_t status = FALSE;

    if (scheduler.rx_all)
    {
        status = PHY_graceful_idle();
    }
    else if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        if ((ctx->op != RX_OP) || !ctx->rx_ongoing)
        {
            status = PHY_graceful_idle();
        }
    }
    else if (ctx->op == RX_OP)
    {
        status = TRUE;
    }

    if (status)
    {
        ctx->op = NONE_OP;

        if (scheduler.rx_all)
        {
            /* force context switch */
            sched_stop_timer();

            scheduler.rx_all = FALSE;
            scheduler.rxed_on_all = FALSE;

            scheduler.switch_later = TRUE;

            sched_start_timer(SCHED_MIN_TICK);
        }
    }

    status = (ctx->op == NONE_OP);

    return status;
}


bool_t PHY_ctx_all_disabled()
{
    uint8_t id;

    OSA_InterruptDisable();

    for (id = 0; id < CTX_NO; id++)
    {
        if (ctx_get(id)->state != E_SCHED_PROTO_OFF)
        {
            OSA_InterruptEnable();

            return FALSE;
        }
    }

    OSA_InterruptEnable();

    return TRUE;

}
#endif /* CTX_SCHED */
