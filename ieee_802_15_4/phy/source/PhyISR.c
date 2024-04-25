/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2018, 2023-2024 NXP
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
#include "Phy.h"
#include "PhyInterface.h"
#include "PhyPacket.h"
#include "PhySec.h"
#include "PhyTypes.h"
#include "dbg_io.h"

#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#include "nxp2p4_xcvr.h"

#include "fsl_ltc.h"

#if defined(FFU_PHY_ONLY_OVER_IMU)
#include "os_if.h"
#endif

#if defined(FFU_DEVICE_LIMIT_VISIBILITY)
#include "PhyDeviceFiltering.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
uint8_t                         mPhyIrqDisableCnt = 1;

#if (WEIGHT_IN_LQI_CALCULATION == 1)
extern Phy_nbRssiCtrl_t nbRssiCtrlReg;
#endif

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void PhyIsrSeqCleanup(void);
static void PhyIsrTimeoutCleanup(void);
static void Phy_GetRxInfo(Phy_PhyLocalStruct_t *ctx);
static uint8_t Phy_LqiConvert(uint8_t hwLqi);

static bool_t PHY_IsV2Frame(uint16_t fcf);

static uint16_t PHY_GetCslPhase(Phy_PhyLocalStruct_t *ctx, uint32_t macHdrTimestampSym);
static bool_t PHY_CheckIfCslIsNeeded(Phy_PhyLocalStruct_t *ctx);

static uint32_t PHY_CalculateTxAckMhrTstampSym(uint32_t sfdDectecTimestampSym, uint32_t rxPktLen);

static void PHY_TransformUint16ToArray(volatile uint8_t *pArray, uint16_t value);

void PHY_set_enh_ack_state(Phy_PhyLocalStruct_t *ctx, enh_ack_state_t state);
enh_ack_state_t PHY_get_enh_ack_state(Phy_PhyLocalStruct_t *ctx);

static void PHY_GenerateSwEnhAck(Phy_PhyLocalStruct_t *ctx, uint32_t rxLength);
static void PHY_write_enh_ack(Phy_PhyLocalStruct_t *ctx);

static void PHY_SetImmAckFp(Phy_PhyLocalStruct_t *ctx, uint8_t *packet, uint8_t packetLength);

static phyIeData_t* PHY_GetAckIeData(Phy_PhyLocalStruct_t *ctx, uint16_t shortAddr, uint8_t *extAddr, uint32_t addrLen);
static uint32_t PHY_GenerateVendorIeData_OpenThread(Phy_PhyLocalStruct_t *ctx, phyIeData_t *pAckIeData, uint32_t txIndex, int nRssi, uint8_t ucLqi);

#ifdef CTX_SCHED
void PHY_InterruptHandler_base(uint8_t xcvseqCopy, uint32_t irqStatus);
void PhyAbort_base(Phy_PhyLocalStruct_t *ctx);
#else
#define PHY_InterruptHandler_base PHY_InterruptHandler
#define PhyAbort_base(ctx) PhyAbort()
#endif

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

#define MHR_LEN 31      /* FCF + SN + addressing fields + PHY_ASH_TOTAL_SIZE + 1 byte (CMD ID) */

uint8_t rxf_mhr[MHR_LEN];   /* copy from RX_PB of received MHR */
uint8_t * const rxf = &rxf_mhr[0];
uint8_t rxf_len = 0;

/* copy the FCF on first watermark (RX_WTMRK_START (rxf_len should be 0)
   copy the MHR on second watermark (rxf_len should be RX_WTMRK_START - 1) */
void update_rxf(uint8_t xcvseq, uint32_t irq_status)
{
    uint32_t wtrmk = ZLL->RX_WTR_MARK;
    bool_t wtmrk_irq = !!(irq_status & ZLL_IRQSTS_RXWTRMRKIRQ_MASK);

    if ((xcvseq != gRX_c) || !wtmrk_irq)
    {
        return;
    }

    if ((!rxf_len && (wtrmk == RX_WTMRK_START)) ||
        ((rxf_len < RX_WTMRK_START) && (wtrmk > RX_WTMRK_START)))
    {
        rxf_len = MIN(wtrmk - 1, MHR_LEN);

        PHY_MemCpy(rxf_mhr, RX_PB, rxf_len);
    }
}

void prepare_for_rx(Phy_PhyLocalStruct_t *ctx)
{
    rxf_len = 0;
    rxf_mhr[0] = 0;

    PHY_set_enh_ack_state(ctx, ENH_ACK_INVALID);

    if (ctx)
    {
        ctx->filter_fail = 0;
    }

    ZLL->RX_WTR_MARK = RX_WTMRK_START;

#ifndef gPhyUseHwSAMTable
    /* Disable Automatic FP set */
    ZLL->SAM_TABLE |= ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK;
#endif
}

/*! *********************************************************************************
* \brief  Disable the 802.15.4 radio IRQ
*
********************************************************************************** */
void ProtectFromXcvrInterrupt(void)
{
    OSA_InterruptDisable();
    // Handle XCVR interrupt mask outside the PHY interrupt context.
    if (!IsCpuInterruptContext())
    {
        if (mPhyIrqDisableCnt == 0)
        {
            // Mask XCVR interrupt -> disables ipi_int_zigbee interrupt
            SET_PHYCTRL_FIELD(ZLL_PHY_CTRL_TRCV_MSK);
        }

        mPhyIrqDisableCnt++;
    }
    OSA_InterruptEnable();
}

/*! *********************************************************************************
* \brief  Enable the 802.15.4 radio IRQ
*
********************************************************************************** */
void UnprotectFromXcvrInterrupt(void)
{
    OSA_InterruptDisable();
    // Handle XCVR interrupt unmask outside the PHY interrupt context.
    if (!IsCpuInterruptContext())
    {
        if (mPhyIrqDisableCnt)
        {
            mPhyIrqDisableCnt--;

            if (mPhyIrqDisableCnt == 0)
            {
                // Unmask XCVR interrupt -> enables ipi_int_zigbee interrupt
                CLR_PHYCTRL_FIELD(ZLL_PHY_CTRL_TRCV_MSK);
            }
        }
    }
    OSA_InterruptEnable();
}

/*! *********************************************************************************
* \brief  Clear IE data array
*
********************************************************************************** */
void PHY_ClearAckIeData(instanceId_t id)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(id);

    // set all the array to zero
    memset(ctx->phyIeDataTable, 0, sizeof(ctx->phyIeDataTable));
}

/*! *********************************************************************************
* \brief  Configure IE data that will be put in an Enhanced ack
*
* \param[in]  pIeData pointer to IE data
* \param[in]  ieDataLen IE data len, if > 0 add entry, if = 0 remove entry
* \param[in]  shortAddr short address of the peer device
* \param[in]  extAddr extended address of the peer device
*
********************************************************************************** */
void PHY_ConfigureAckIeData(instanceId_t phyInstance, uint8_t * pIeData, uint32_t ieDataParam, uint16_t shortAddr, uint8_t *extAddr)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    /* Returns entry corresponding to short address or the first empty one */
    phyIeData_t *pIeDataEntry = PHY_GetAckIeData(ctx, shortAddr, NULL, sizeof(uint16_t));

    uint32_t ieDataLen = 0;

    if ((IeData_MSB_VALID_DATA & ieDataParam) && (NULL != pIeData))
    { // Data length is valid in IE
       ieDataLen = (pIeData[1] << 8) | pIeData[0];
       ieDataLen &= 0x3FF; // Data len from IE Header
    }

    /* Add case */
    if (NULL != pIeDataEntry)
    {
        if ((ieDataLen > 0) && (NULL != pIeData))
        {
            /* Add/Update data */
            memcpy(pIeDataEntry->ieData, pIeData, ieDataLen);
            memcpy(pIeDataEntry->extAddr, extAddr, sizeof(uint64_t));
            pIeDataEntry->ieParam = ieDataParam;
            pIeDataEntry->shortAddr = shortAddr;
        }
        else
        {
            /* Remove case */
            memset(pIeDataEntry, 0, sizeof(phyIeData_t));
        }
    }
}

/*! *********************************************************************************
* \brief  Enable CSL IE inclusion in Enhanced ACK
*
********************************************************************************** */
void PHY_EnableCsl(instanceId_t phyInstance, uint32_t cslPeriod)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->phyCslPeriod = cslPeriod;

   if (cslPeriod > 0)
   {
       memcpy(ctx->ackIeDataAndHdr + sizeof(HdrIe_t) + sizeof(uint16_t), (uint8_t*)&cslPeriod, sizeof(uint16_t));
   }
}

/*! *********************************************************************************
* \brief  Set EFP (Enhanced Frame Pending)
*
********************************************************************************** */
void PHY_SetEfp(instanceId_t phyInstance, bool_t state)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->efp = state;
}

/*! *********************************************************************************
* \brief  Set Neighbour Table for FP in ACK generation
*
********************************************************************************** */
void PHY_SetNbt(instanceId_t phyInstance, bool_t state)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->neighbourTblEnabled = state;
}

/*! *********************************************************************************
* \brief  Set Csl sample time in us
*
********************************************************************************** */
void PHY_SetCslSampleTime(instanceId_t phyInstance, uint32_t cslSampleTimeUs)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->phyCslSampleTimeUs = cslSampleTimeUs;
}

/*! *********************************************************************************
* \brief  Copy data to/from packet ram in a safe mannner
*
********************************************************************************** */
void PHY_MemCpy(uint8_t *dst, volatile uint8_t *src, uint32_t len)
{
    /* The Packet Buffer, is accessible by the MCU in 16-, or 8-bit accesses */
    for (uint32_t i = 0; i < len; i++)
    {
        dst[i] = src[i];
    }
}

/* The write into tx PB doesn't always succeed the first time */
void PHY_MemCpyVerify(volatile uint8_t *dst, uint8_t *src, uint32_t len)
{
    uint8_t tmp;

    /* The Packet Buffer, is accessible by the MCU in 16-, or 8-bit accesses */
    for (uint32_t i = 0; i < len; i++)
    {
        for (uint32_t cnt = 0; cnt < TX_PB_TRY_CNT; cnt++)
        {
            dst[i] = src[i];

            tmp = dst[i];
            if (src[i] == tmp)
            {
                break;
            }
        }
    }
}

/*! *********************************************************************************
* \brief  Calculate CSL phase
*
********************************************************************************** */
static uint16_t PHY_GetCslPhase(Phy_PhyLocalStruct_t *ctx, uint32_t macHdrTimestampSym)
{
    uint32_t cslPeriodInUs = ctx->phyCslPeriod * PHY_TEN_SYMBOLS_US;

    /* convert the sfd timestamp to us */
    macHdrTimestampSym *= PHY_SYMBOLS_US;
    uint32_t diff = (cslPeriodInUs - (macHdrTimestampSym % cslPeriodInUs) + (ctx->phyCslSampleTimeUs % cslPeriodInUs)) % cslPeriodInUs;
    return (uint16_t)(diff / PHY_TEN_SYMBOLS_US + 1);
}

/*! *********************************************************************************
* \brief  PHY_GenerateVendorIeData_OpenThread
*
* \param[in]  pAckIeData = pointer to Vendor Specific IE
* \param[in]  txIndex = index in packet buffer
* \param[in]  nRssi
* \param[in]  ucLqi
*
* \return     Data Length written into packet buffer
********************************************************************************** */
static uint32_t PHY_GenerateVendorIeData_OpenThread(Phy_PhyLocalStruct_t *ctx, phyIeData_t *pAckIeData, uint32_t txIndex, int nRssi, uint8_t ucLqi)
{
    uint32_t ieDataLen = 0; // return value
    uint8_t ucLinkMargin = 0;
    uint32_t ieVendorOUI = 0;
    uint8_t ieLinkMetrics = 0;

    /* Open Thread 4.11.3.4.3 Link Metrics Data and Sub-TLV Formats */
    do
    {
        // check data pointer
        if (pAckIeData == NULL)
            break;

        // read Vendor Ie Header
        if (!(IeData_MSB_VALID_DATA & pAckIeData->ieParam))
            break;

        // Data Lenght from Header
        ieDataLen = (pAckIeData->ieData[1] << 8) | pAckIeData->ieData[0];
        ieDataLen &= 0x3FF; // Data len from IE Header
        ieDataLen += 2; // add header lenght 2 bytes

        // read Vendor OUI
        ieVendorOUI = (((uint32_t)(pAckIeData->ieData[2])) << 0) |
                      (((uint32_t)(pAckIeData->ieData[3])) << 8) |
                      (((uint32_t)(pAckIeData->ieData[4])) << 16);

        // check Vendor OUI
        if (IeVendorOuiThreadCompanyId != ieVendorOUI)
            break;

        // parameters
        ieLinkMetrics = (pAckIeData->ieParam & 0x7FFFFFFF);

        /* Link Margin - RSSI margin above noise floor in dB, having the range 0 dB to 130 dB scaled linearly from 0 to 255. */
        ucLinkMargin = (uint8_t) ( nRssi >= IeData_LinkMarginThreshold_c ? (nRssi - IeData_LinkMarginThreshold_c) : 0);
        /* Linear scale Link Margin from [0, 130] to [0, 255] */
        if (ucLinkMargin > 130)
        {
           ucLinkMargin = 130;
        }
        ucLinkMargin = ucLinkMargin * 255 / 130;

        /* RSSI in absolute dBm, having the range -130 dBm to 0 dBm scaled linearly 27 from 0 to 255. */
        if (nRssi > 0)
        { /* in case of positive rssi value keep it 0 (maximum) to be in requested scale */
            nRssi = 0;
        }
        /* Linear scale RSSI from [-130, 0] to [0, 255] as per OT specs */
        nRssi = ((nRssi + 130) * 255 / 130);

        switch (ieDataLen)
        {
        case 7:
            if (ieLinkMetrics & IeData_Lqi_c)
            {
                pAckIeData->ieData[6] = ucLqi;
            }
            if (ieLinkMetrics & IeData_LinkMargin_c)
            {
                pAckIeData->ieData[6] = ucLinkMargin;
            }
            if (ieLinkMetrics & IeData_Rssi_c)
            {
                pAckIeData->ieData[6] = (uint8_t) nRssi;
            }
            break;
        case 8:
            if ((ieLinkMetrics & IeData_Lqi_c) && (ieLinkMetrics & IeData_LinkMargin_c))
            {
                pAckIeData->ieData[6] = ucLqi;
                pAckIeData->ieData[7] = ucLinkMargin;
            }
            if ((ieLinkMetrics & IeData_Lqi_c) && (ieLinkMetrics & IeData_Rssi_c))
            {
                pAckIeData->ieData[6] = ucLqi;
                pAckIeData->ieData[7] = (uint8_t) nRssi;
            }
            if ((ieLinkMetrics & IeData_LinkMargin_c) && (ieLinkMetrics & IeData_Rssi_c))
            {
                pAckIeData->ieData[6] = ucLinkMargin;
                pAckIeData->ieData[7] = (uint8_t) nRssi;
            }
            break;
        }
        memcpy(ctx->trx_buff + txIndex, pAckIeData->ieData, ieDataLen);
    } while (0);

    return ieDataLen;
}

/*! *********************************************************************************
* \brief  Returns entry corresponding to short address or the first empty one
*
* \param[in]  shortAddr short address of the peer device
* \param[in]  extAddr extended address of the peer device
* \param[in]  addrLen identifies the type of address used short = 2, extended = 8
*
********************************************************************************** */
static phyIeData_t* PHY_GetAckIeData(Phy_PhyLocalStruct_t *ctx, uint16_t shortAddr, uint8_t *extAddr, uint32_t addrLen)
{
    phyIeData_t * pFoundEntry = NULL;

    if ((addrLen == sizeof(uint16_t)) || ((addrLen == sizeof(uint64_t)) && extAddr))
    {
        for (uint32_t i = 0; i < gPhyIeDataTableSize; i++)
        {
            if (IeData_MSB_VALID_DATA & ctx->phyIeDataTable[i].ieParam)
            {
                if (addrLen == sizeof(uint16_t))
                {
                    if (shortAddr == ctx->phyIeDataTable[i].shortAddr)
                    {
                        pFoundEntry = &ctx->phyIeDataTable[i];
                        break;
                    }
                }
                else
                {
                    if (memcmp(extAddr, ctx->phyIeDataTable[i].extAddr, addrLen))
                    {
                        pFoundEntry = &ctx->phyIeDataTable[i];
                        break;
                    }
                }
            }
            else if (NULL == pFoundEntry)
            {
                pFoundEntry = &ctx->phyIeDataTable[i];
            }
        }
    }

    return pFoundEntry;
}

void PHY_set_enh_ack_state(Phy_PhyLocalStruct_t *ctx, enh_ack_state_t state)
{
    if (!ctx)
    {
        return;
    }

    ctx->enh_ack_state = state;
}

enh_ack_state_t PHY_get_enh_ack_state(Phy_PhyLocalStruct_t *ctx)
{
    if (!ctx)
    {
        return ENH_ACK_INVALID;
    }

    return ctx->enh_ack_state;
}

static bool_t PHY_IsV2Frame(uint16_t fcf)
{
    bool_t bSendSwEnhAck = false;
    if ((fcf & phyFcfFrameVersionMask) == phyFcfFrameVersion2015)
    {
        bSendSwEnhAck = true;
    }
    return bSendSwEnhAck;
}

/*! *********************************************************************************
* \brief  Generate a Sw Enahced ACK
*
************************************************************************************/
static void PHY_GenerateSwEnhAck(Phy_PhyLocalStruct_t *ctx, uint32_t rxLength)
{
    uint16_t rxFcf = PHY_TransformArrayToUint16(rxf);
    uint32_t varHdrLen = PhyPacket_GetHdrLength(rxf);
    uint16_t txFcf = phyFcfFrameAck | phyFcfFrameVersion2015 | phyFcfSrcAddrNone;
    uint8_t *pSrcAddr = NULL;
    uint32_t srcAddrLen = 0;
    uint32_t txIndex = 0;
    uint32_t micSize = 0;
    uint8_t  pktLen = 0;
    bool_t bCslNeeded = false;
    bool_t bIeNeeded = false;
    uint16_t checksum = 0;
    uint16_t panId = ZLL->MACSHORTADDRS0 & ZLL_MACSHORTADDRS0_MACPANID0_MASK;
    uint16_t shortSrcAddr = 0;
    phyIeData_t *pAckIeData = NULL;
    int8_t nRssi = 0;
    uint8_t ucLqi = 0;

    if (PHY_get_enh_ack_state(ctx) == ENH_ACK_INVALID)
    {
        return;
    }

    if (varHdrLen + gPhyFCSSize_c > rxLength)
    {
        /* malformed frame */
        PHY_set_enh_ack_state(ctx, ENH_ACK_INVALID);

        return;
    }

    if (ctx->id)
    {
        panId = ZLL->MACSHORTADDRS1 & ZLL_MACSHORTADDRS1_MACPANID1_MASK;
    }

    /* Len field has to be put into tx PB */
    txIndex += sizeof(uint8_t);

    if (rxFcf & phyFcfPanidCompression)
    {
        txFcf |= phyFcfPanidCompression;
    }

    /* Destination address mode of ACK packet */
    if ((rxFcf & phyFcfSrcAddrMask) == phyFcfSrcAddrExt)
    {
        txFcf |= phyFcfDstAddrExt;
        srcAddrLen = sizeof(uint64_t);
        pSrcAddr = rxf + (varHdrLen - srcAddrLen);
    }
    else if ((rxFcf & phyFcfSrcAddrMask) == phyFcfSrcAddrShort)
    {
        txFcf |= phyFcfDstAddrShort;
        srcAddrLen = sizeof(uint16_t);
        pSrcAddr = rxf + (varHdrLen - srcAddrLen);
        shortSrcAddr = PHY_TransformArrayToUint16(pSrcAddr);
    }
    else
    {
        txFcf |= phyFcfDstAddrNone;
    }

    /* Calculate checksum based on source address and panid and search SAP table for a match */
    checksum = PhyGetChecksum(pSrcAddr, (srcAddrLen == 2) ? mShortAddr_d : mExtAddr_d, panId);
    if (((ctx->efp && (FCF_FT_GET(rxFcf) == phyFcfFrameData)) || (FCF_FT_GET(rxFcf) == phyFcfFrameMacCmd)) &&
        pSrcAddr && (PhyGetIndexOf(ctx, checksum) != INV_IDX))
    {
        /* Set FP bit if we have a checksum match in SAP table */
        txFcf |= phyFcfFramePending;
        /* Since SW Enh Ack is used we need to keep track of FP bit
           to be passed to upper stack along with next RX Data (Poll) indication */
        ctx->flags |= gPhyFlagTxAckFP_c;
    }

    /* Check if we need to add CSL information element to the packet */
    if (PHY_CheckIfCslIsNeeded(ctx))
    {
        txFcf |= phyFcfIePresent;
        bCslNeeded = true;
    }

    /* Check if we need to add generic information elements to the packet */
    pAckIeData = PHY_GetAckIeData(ctx, shortSrcAddr, pSrcAddr, srcAddrLen);
    if ((NULL != pAckIeData) && (pAckIeData->ieParam & IeData_MSB_VALID_DATA))
    {
        txFcf |= phyFcfIePresent;
        bIeNeeded = true;
    }

    /* Use security in ACK frame only if the frame has security and contains CSL or IE or frame payload.
       In other cases, the security is optional so we choose to not use it as it doesn't provide any benefit */
    if (rxFcf & phyFcfSecurityEnabled)
    {
        if (((varHdrLen + gPhyFCSSize_c + PHY_ASH_SEC_CTRL_SIZE) > rxLength) ||
            ((varHdrLen + gPhyFCSSize_c +
              PhyPacket_GetASHLength(rxf[varHdrLen]) +
              PhyPacket_GetMicLength(rxf[varHdrLen])) > rxLength))
        {
            /* malformed frame: send simple Enh-ACK: no security or IEs */
            bCslNeeded = FALSE;
            bIeNeeded = FALSE;

            txFcf &= ~(phyFcfSecurityEnabled | phyFcfIePresent);
        }
        else if (bCslNeeded || bIeNeeded)
        {
            txFcf |= phyFcfSecurityEnabled;
        }
    }

    if (txFcf & phyFcfSecurityEnabled)
    {
        uint8_t ash_ctl = rxf[varHdrLen];
        uint8_t key_id_pos = PhyPacket_GetKeyIndexPos(ash_ctl);
        uint8_t key_id = INV_ASH_KEY_ID;

        if (key_id_pos != INV_ASH_KEY_ID_POS)
        {
            key_id = rxf[varHdrLen + key_id_pos];
        }

        if (!PhySec_IsASHValid(ash_ctl, key_id))
        {
            /* send simple Enh-ACK: no security or IEs */
            bCslNeeded = FALSE;
            bIeNeeded = FALSE;

            txFcf &= ~(phyFcfSecurityEnabled | phyFcfIePresent);
        }
    }

    PHY_TransformUint16ToArray(ctx->trx_buff + txIndex, txFcf);
    txIndex += sizeof(txFcf);

    /* Set seq number */
    if (rxFcf & phyFcfSeqNbSuppression)
    {
        txFcf |= phyFcfSeqNbSuppression;
    }
    else
    {
        ctx->trx_buff[txIndex] = rxf[sizeof(rxFcf)];
        txIndex += sizeof(uint8_t);
    }

    /* Set destination panid if needed */
    if ((txFcf & phyFcfPanidCompression) == 0)
    {
        PHY_TransformUint16ToArray((ctx->trx_buff + txIndex), panId);
        txIndex += sizeof(panId);
    }

    /* Set destination address if needed */
    if (pSrcAddr)
    {
        /* pSrcAddr points into packet buffer */
        memcpy(ctx->trx_buff + txIndex, pSrcAddr, srcAddrLen);
        txIndex += srcAddrLen;
    }

    if (txFcf & phyFcfSecurityEnabled)
    {
        /* Copy ASH from RX packet */
        uint8_t ash_ctl = rxf[varHdrLen];
        uint8_t ash_len = PhyPacket_GetASHLength(ash_ctl);

        memcpy(ctx->trx_buff + txIndex, rxf + varHdrLen, ash_len);
        txIndex += ash_len;

        micSize = PhyPacket_GetMicLength(ash_ctl);

        PhyPacket_SetFrameCounter(ctx->trx_buff + 1, ctx->frameCounter++, txIndex + micSize + gPhyFCSSize_c);  /* frame counter is updated after it's used */
    }

    if (bCslNeeded)
    {
        /* Calculate the timestamp of the tx ack mac hdr from the rx sfd timestamp */
        uint32_t sfdDetectTstampSym = ZLL->TIMESTAMP >> ZLL_TIMESTAMP_TIMESTAMP_SHIFT;

        uint16_t cslPhase = PHY_GetCslPhase(ctx, PHY_CalculateTxAckMhrTstampSym(sfdDetectTstampSym, rxLength));

        memcpy(ctx->ackIeDataAndHdr + sizeof(HdrIe_t), (uint8_t*)&cslPhase, sizeof(uint16_t));
        memcpy(ctx->trx_buff + txIndex, ctx->ackIeDataAndHdr, PHY_CSL_TOTAL_SIZE);

        txIndex += PHY_CSL_TOTAL_SIZE;
    }

    /* Copy generic IE data to ACK packet */
    if (bIeNeeded)
    {
        nRssi = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_RSSI_MASK) >> ZLL_LQI_AND_RSSI_RSSI_SHIFT;
        ucLqi = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_LQI_VALUE_MASK) >> ZLL_LQI_AND_RSSI_LQI_VALUE_SHIFT;
        ucLqi = Phy_LqiConvert(ucLqi);

        txIndex += PHY_GenerateVendorIeData_OpenThread(ctx, pAckIeData, txIndex, nRssi, ucLqi);
    }

    /* Set size of frame in index 0 of the tx buffer - from txIndex subtract 1 byte
       for the len and add the FCS 2 bytes */
    pktLen = txIndex - 1 + gPhyFCSSize_c + micSize;
    ctx->trx_buff[0] = pktLen;

    if (txFcf & phyFcfSecurityEnabled)
    {
        PhySec_Encrypt(ctx, ctx->trx_buff + 1, pktLen);
        ctx->rxParams.ackedWithSecEnhAck = TRUE;
    }

    PHY_set_enh_ack_state(ctx, ENH_ACK_READY);
}

static void PHY_write_enh_ack(Phy_PhyLocalStruct_t *ctx)
{
    if (PHY_get_enh_ack_state(ctx) == ENH_ACK_INVALID)
    {
        return;
    }

    /* Load Enh-ACK into packet RAM as it's safe to transmit */
    PHY_MemCpyVerify(TX_PB, ctx->trx_buff, ctx->trx_buff[0] + 1 - gPhyFCSSize_c);

    /* Set ENH_ACK_LEN register based on calculated length */
    uint32_t tmp = ZLL->ENHACK_CTRL0 & (~ZLL_ENHACK_CTRL0_SW_MHR_LENGTH_MASK);

    tmp |= ZLL_ENHACK_CTRL0_SW_LEN_RDY(1) |
           ZLL_ENHACK_CTRL0_SW_MHR_LENGTH(ctx->trx_buff[0] - gPhyFCSSize_c) |
           ZLL_ENHACK_CTRL0_SW_HIE_RDY(1);

    ZLL->ENHACK_CTRL0 = tmp;

    PHY_set_enh_ack_state(ctx, ENH_ACK_INVALID);
}

static void PHY_SetImmAckFp(Phy_PhyLocalStruct_t *ctx, uint8_t *packet, uint8_t packetLength)
{
    uint16_t fcf = PHY_TransformArrayToUint16(packet);
    uint16_t srcAddrMode = fcf & phyFcfSrcAddrMask;
    uint16_t dstAddrMode = fcf & phyFcfDstAddrMask;
    macCmdId_t commId = phyMacCmdInvalid;
    uint8_t rxIndex = 0;
    uint16_t checksum = 0;
    uint16_t panId;

    /* Sane state, don't set FP by default */
    PhyPpSetFpManually(FALSE);

    if (FCF_VER_GET(fcf) >= FCF_VER_MAX)
    {
        return;
    }

    if (!ctx)
    {
        return;
    }

    /* Sane state, set FP = FALSE by default if CTX exists */
    ctx->flags &= ~gPhyFlagTxAckFP_c;

    /* Get the command ID, if EFP is enabled it will compute FP regardless of the frame type */
    if (!ctx->efp && (FCF_FT_GET(fcf) == phyFcfFrameMacCmd))
    {
        /* Set Imm Ack only for encrypted POLLs that pass the size assumption of ASH */
        if ((fcf & phyFcfSecurityEnabled) &&
            (PhyPacket_GetSecurityHeaderLength(packet, packetLength) > PHY_ASH_TOTAL_SIZE))
        {
            return;
        }

        commId = PhyPacket_GetMacV1CmdId(packet, packetLength);
    }

    if ((ctx->efp ||
        ((FCF_FT_GET(fcf) == phyFcfFrameMacCmd) && (commId == phyMacCmdDataReq))) &&
        ((srcAddrMode == phyFcfSrcAddrShort) || (srcAddrMode == phyFcfSrcAddrExt)))
    {
        /* Store dst PAN Id */
        rxIndex += sizeof(fcf) + !(fcf & phyFcfSeqNbSuppression);
        panId = PHY_TransformArrayToUint16(packet + rxIndex);
        rxIndex += sizeof(panId);

        /* Skip over dst addr fields */
        if ( dstAddrMode == phyFcfDstAddrShort )
        {
            rxIndex += sizeof(uint16_t);
        }
        else if ( dstAddrMode == phyFcfDstAddrExt )
        {
            rxIndex += sizeof(uint64_t);
        }
        else if ( dstAddrMode != mNoAddr_d )
        {
            return;
        }

        /* Store src PanId if present */
        if ((fcf & phyFcfPanidCompression) == 0)
        {
            panId  = PHY_TransformArrayToUint16(packet + rxIndex);
            rxIndex += sizeof(panId);
        }

        /* Get FP state */
        checksum = PhyGetChecksum(packet + rxIndex, srcAddrMode >> 14, panId);
        if (PhyGetIndexOf(ctx, checksum) != INV_IDX)
        {
            ctx->flags |= gPhyFlagTxAckFP_c;
            PhyPpSetFpManually(TRUE);
        }
        /* Neighbour Table is implemented only for Short Addresses */
        else if (ctx->neighbourTblEnabled &&
                (srcAddrMode == phyFcfSrcAddrShort) &&
                (PhyNbTblIndexOf(ctx, checksum) == INV_IDX))
        {
            ctx->flags |= gPhyFlagTxAckFP_c;
            ctx->flags |= gPhyFlagNbTblRxAckFP_c;
            PhyPpSetFpManually(TRUE);
        }
    }
}

/*! *********************************************************************************
* \brief  Check if we need to include CSL IE in the Sw Enahced ACK
*
************************************************************************************/
static bool_t PHY_CheckIfCslIsNeeded(Phy_PhyLocalStruct_t *ctx)
{
    bool_t bCslIsNeed = false;

    if (ctx->phyCslPeriod >  0)
    {
        bCslIsNeed = true;
    }

    return bCslIsNeed;
}

/*! *********************************************************************************
* \brief  Adjust timestamp so that it reflects start of TX eAck MHR starting from received SFD
*
************************************************************************************/
static uint32_t PHY_CalculateTxAckMhrTstampSym(uint32_t sfdDectecTimestampSym, uint32_t rxPktLen)
{
    /* Len(RX len + SFD + LEN field) of RX packet in symbols */
    uint32_t timestamp = sfdDectecTimestampSym + ((rxPktLen + 2) * gPhySymbolsPerOctet_c);

    /* Add turnaround time + TX Preamble, SFD and LEn */
    timestamp += gPhyTurnaroundTime_c + gPhySHRDuration_c + gPhyPHRDuration_c;

    return timestamp;

}
/*!*************************************************************************************************
\brief  Converts an big endian array to a 16 bit numeric value

 ***************************************************************************************************/
uint16_t PHY_TransformArrayToUint16(uint8_t *pArray)
{
    union uuint16_t
    {
        uint16_t    u16;     /*!< 16bit variable */
        uint8_t     u8[2];   /*!< 8bit array */
    };
/* For V18 DS5 compiler, variable need be given initial value */
    union uuint16_t out ={0};

    out.u8[0] = *pArray++;
    out.u8[1] = *pArray;

    return out.u16;
}

/*!*************************************************************************************************
\brief  Converts a 16 bit numeric value to array.

 ***************************************************************************************************/
static void PHY_TransformUint16ToArray(volatile uint8_t *pArray, uint16_t value)
{
    *pArray++ = (uint8_t)(value);
    *pArray   = (uint8_t)(value >> 8);
}

/*! *********************************************************************************
* \brief  Clear and mask PHY IRQ, set sequence to Idle
*
********************************************************************************** */
static void PhyIsrSeqCleanup(void)
{
    uint32_t irqStatus;

    /* Set the PHY sequencer back to IDLE */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_XCVSEQ_MASK;
    /* Mask SEQ, RX, TX and CCA interrupts */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCAMSK_MASK |
                     ZLL_PHY_CTRL_RXMSK_MASK  |
                     ZLL_PHY_CTRL_TXMSK_MASK  |
                     ZLL_PHY_CTRL_SEQMSK_MASK;

    while (ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK)
    {
    }

    irqStatus = ZLL->IRQSTS;
    /* Mask TMR3 interrupt */
    irqStatus |= ZLL_IRQSTS_TMR3MSK_MASK;
    /* Clear transceiver interrupts except TMRxIRQ */
    irqStatus &= ~( ZLL_IRQSTS_TMR1IRQ_MASK |
                    ZLL_IRQSTS_TMR2IRQ_MASK |
                    ZLL_IRQSTS_TMR3IRQ_MASK |
                    ZLL_IRQSTS_TMR4IRQ_MASK );
    ZLL->IRQSTS = irqStatus;

    ZLL->SAM_TABLE &= ~(ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK);
}

/*! *********************************************************************************
* \brief  Clear and mask PHY IRQ, disable timeout, set sequence to Idle
*
********************************************************************************** */
static void PhyIsrTimeoutCleanup(void)
{
    uint32_t irqStatus;

    /* Set the PHY sequencer back to IDLE and disable TMR3 comparator and timeout */
    ZLL->PHY_CTRL &= ~(ZLL_PHY_CTRL_TMR3CMP_EN_MASK |
                       ZLL_PHY_CTRL_TC3TMOUT_MASK   |
                       ZLL_PHY_CTRL_XCVSEQ_MASK);
    /* Mask SEQ, RX, TX and CCA interrupts */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCAMSK_MASK |
                     ZLL_PHY_CTRL_RXMSK_MASK  |
                     ZLL_PHY_CTRL_TXMSK_MASK  |
                     ZLL_PHY_CTRL_SEQMSK_MASK;

    while (ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK)
    {
    }

    irqStatus = ZLL->IRQSTS;
    /* Mask TMR3 interrupt */
    irqStatus |= ZLL_IRQSTS_TMR3MSK_MASK;
    /* Clear transceiver interrupts except TMR1IRQ and TMR4IRQ. */
    irqStatus &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    ZLL->IRQSTS = irqStatus;

    ZLL->SAM_TABLE &= ~(ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK);
}

/*! *********************************************************************************
* \brief  Scales energy level to 0-255
*
* \param[in]  energyLevel  the energy level reported by HW
*
* \return  uint8_t  the energy level scaled in 0x00-0xFF
*
********************************************************************************** */
uint8_t Phy_GetEnergyLevel(uint8_t energyLevel) /* [dbm] */
{
    int32_t temp = (int8_t)energyLevel;

    if (temp <= MIN_ENERGY_LEVEL)
    {
        temp = 0x00;
    }
    else if (temp >= MAX_ENERGY_LEVEL)
    {
        temp = 0xFF;
    }
    else
    {
        temp = CONVERT_ENERGY_LEVEL(temp);
    }

    return (uint8_t)temp;
}

/*! *********************************************************************************
* \brief  This function returns the LQI for the las received packet
*
* \return  uint8_t  LQI value
*
********************************************************************************** */
uint8_t PhyGetLastRxLqiValue(instanceId_t id)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(id);

    return ctx->mPhyLastRxLQI;
}

/*! *********************************************************************************
* \brief  This function returns the RSSI for the last received packet
*
* \return  uint8_t  RSSI value
*
********************************************************************************** */
uint8_t PhyGetLastRxRssiValue(instanceId_t id)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(id);

#if (LAST_RSSI_USING_LQI == 1)
    /* This doesn't work with dual PAN */
    /*RSSI*/
    /*
    **       LQI
    **RSSI = ---  - LQI_COMP
    **       2.25
    */
    uint8_t LQI_COMP = (ZLL->CCA_LQI_CTRL & ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_MASK) >> ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_SHIFT;
    int32_t lqi_to_rssi = ((ctx->mPhyLastRxRSSI * 456u) >> 10u) + 5 - LQI_COMP;

    return (uint8_t)lqi_to_rssi;
#else
    return ctx->mPhyLastRxRSSI;
#endif
}

/*! *********************************************************************************
* \brief  This function converts the LQI reported by the PHY into an signed RSSI value
*
* \param[in]  LQI  the LQI reported by the PHY
*
* \return  the RSSI value in dbm
*
********************************************************************************** */
int8_t PhyConvertLQIToRSSI(uint8_t lqi)
{
    int32_t rssi = (36 * lqi - 9836) / 109;

    return (int8_t)rssi;
}


/*! *********************************************************************************
* \brief  PHY ISR
*
********************************************************************************** */
/* No vector table in RAM for now in OpenThread build
   So we need to implement directly the handler defined in the startup code */
#if defined(CPU_KW45B41Z83AFPA_NBU)
void ZIGBEE_INT_IRQHandler(void)
{
    /* wrap 802.15.4 interrupt */
    PHY_InterruptHandler();
}
#elif defined(RW610N_BT_CM3_SERIES)
#if defined(FFU_PHY_ONLY_OVER_IMU)
UINT32 FFU_ZIGBEE_INT_IRQHandler(UINT32 vector, UINT32 data)
#else
void FFU_ZIGBEE_INT_IRQHandler(void)
#endif
{
    /* wrap 802.15.4 interrupt */
    PHY_InterruptHandler();
#if defined(FFU_PHY_ONLY_OVER_IMU)
    return 0;
#endif
}
#elif defined(K32W1480_SERIES)
void RF_802_15_4_IRQHandler(void)
{
    /* wrap 802.15.4 interrupt */
    PHY_InterruptHandler();
}
#elif defined(MCXW72BD_cm33_core0_SERIES)
void Reserved69_IRQHandler(void)
{
    PHY_InterruptHandler();
}
#elif defined(MCXW72BD_cm33_core1_SERIES)
void ZIGBEE_INT_IRQHandler(void)
{
    /* wrap 802.15.4 interrupt */
    PHY_InterruptHandler();
}
#endif

void PHY_InterruptHandler_base(
#ifdef CTX_SCHED
        uint8_t xcvseqCopy, uint32_t irqStatus
#endif
)
{
#ifndef CTX_SCHED
    uint8_t xcvseqCopy;
    uint32_t irqStatus;
#endif
    uint32_t length;
    uint32_t crc_valid;
    uint16_t rxFcf;
    Phy_PhyLocalStruct_t *ctx = ctx_get_current();

#if (ARB_GRANT_DEASSERTION_SUPPORT == 1)
#if defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1)
    uint32_t seqCtrlStatus;
    seqCtrlStatus = ZLL->SEQ_CTRL_STS;
#endif /* defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1) */
#endif //(ARB_GRANT_DEASSERTION_SUPPORT == 1)

#ifndef CTX_SCHED
    /* Read current XCVRSEQ and interrup status */
    xcvseqCopy  = ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK;
    irqStatus   = ZLL->IRQSTS;

    update_rxf(xcvseqCopy, irqStatus);
#endif

    /* Clear ZLL interrupts. The TX, RX, CCA and TC3 IRQs will be handled on SEQIRQ and must not be cleared. */
    ZLL->IRQSTS = irqStatus & ~(ZLL_IRQSTS_TMR3IRQ_MASK | ZLL_IRQSTS_CCAIRQ_MASK | ZLL_IRQSTS_RXIRQ_MASK | ZLL_IRQSTS_TXIRQ_MASK);

#if !defined(RW610N_BT_CM3_SERIES)
#if defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1)
    if(irqStatus & ZLL_IRQSTS_ARB_GRANT_DEASSERTION_IRQ_MASK)
    {
        // TODO: software should probably react to an external RF NOT ALLOWED signal
        // See Jira ticket: KFOURWONE-382
    }
#endif
#endif /* !defined(RW610N_BT_CM3_SERIES) */

    /* WAKE IRQ */
    if (irqStatus & ZLL_IRQSTS_WAKE_IRQ_MASK)
    {
#if defined(DEEP_SLEEP_MODE_FEATURE) && (DEEP_SLEEP_MODE_FEATURE == 1)
        PhyResetDSM();
#endif

#if 0
        // TODO: Update DSM handling using RFMC (RSIM not existing)
        uint32_t timeAdjust = RSIM->MAN_WAKE;
        /* Adjust the 802.15.4 EVENT_TMR */
        timeAdjust = (uint64_t)(timeAdjust - RSIM->MAN_SLEEP) * 1000000U / 32768; /* [us] */
        ZLL->EVENT_TMR = ((timeAdjust >> 0x4) << ZLL_EVENT_TMR_EVENT_TMR_SHIFT)      | /* [symbols]: divide by 16 */
                         ((timeAdjust & 0x0F) << ZLL_EVENT_TMR_EVENT_TMR_FRAC_SHIFT) | /* [us]: modulo 16 */
                         ZLL_EVENT_TMR_EVENT_TMR_ADD_MASK;
        ZLL->DSM_CTRL = 0;
#endif
    }

    /* Flter Fail IRQ */
    if (irqStatus & ZLL_IRQSTS_FILTERFAIL_IRQ_MASK)
    {
#if gMWS_UseCoexistence_d
        MWS_CoexistenceReleaseAccess();
#endif
        if (!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK))
        {
            Radio_Phy_PlmeFilterFailRx(ctx);
        }

        if (ctx)
        {
            ctx->filter_fail = 1;
        }

        ZLL->RX_WTR_MARK = RX_WTMRK_START;
        ZLL->SAM_TABLE &= ~(ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK);
    }
    /* Rx Watermark IRQ */
    else
    {
        if ((!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_RX_WMRK_MSK_MASK)) && (irqStatus & ZLL_IRQSTS_RXWTRMRKIRQ_MASK) && (xcvseqCopy == gRX_c))
        {
            uint8_t curr_wtrmrk = ZLL->RX_WTR_MARK;
            length = (irqStatus & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK) >> ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT;

            rxFcf = PHY_TransformArrayToUint16(rxf);

            if (curr_wtrmrk == RX_WTMRK_START)
            {
                Radio_Phy_PlmeRxWatermark(length, rxFcf);

                uint32_t hdr_len = PhyPacket_GetHdrLength(rxf) - sizeof(rxFcf);

                /* advance watermark to the end of frame header or ASH if security is enabled.
                 * Used for FP and PAN detection based on destination address.
                 * Also wait for CMD ID in case of POLLS to be used in IMM ACK FP
                 */
                ZLL->RX_WTR_MARK = RX_WTMRK_START + hdr_len +
                        (rxFcf & phyFcfSecurityEnabled ? PHY_ASH_TOTAL_SIZE : 0) + PHY_MAC_CMD_ID_SIZE;

                PHY_set_enh_ack_state(ctx, ENH_ACK_RESET);
            }
            else
            {
#if defined(FFU_DEVICE_LIMIT_VISIBILITY)
                if( !PHY_isFrameVisible(ctx, rxFcf) )
                {
                    /* Filtering Feature is enabled - abort sequencer to drop current frame + ACK.
                       Cleanup curent context. There is no sequence end event */
                    PhyAbort_base(ctx);
                    Radio_Phy_AbortIndication(ctx);

                    // Default Rx Watermark Level
                    ZLL->RX_WTR_MARK = RX_WTMRK_START;
                }
                else
#endif
                {
                    if (rxFcf & phyFcfAckRequest)      /* ACK generation */
                    {
                        if (PHY_IsV2Frame(rxFcf))
                        {
                            /* Enh-ACK */
                            if (PHY_get_enh_ack_state(ctx) == ENH_ACK_RESET)
                            {
                                /* set watermark at frame end */
                                ZLL->RX_WTR_MARK = length - 1;

                                PHY_GenerateSwEnhAck(ctx, length);      /* sets ENH_ACK_READY */
                            }
                            else if (PHY_get_enh_ack_state(ctx) == ENH_ACK_READY)
                            {
                                PHY_write_enh_ack(ctx);     /* sets ENH_ACK_INVALID */
                            }
                        }
#ifndef gPhyUseHwSAMTable
                        else
                        {
                            /* Imm-ACK */
                            PHY_SetImmAckFp(ctx, rxf, length);
                        }
#endif
                    }
                }
            }

#if gMWS_UseCoexistence_d
            if ( (xcvseqCopy == gRX_c) && (gMWS_Success_c != MWS_CoexistenceRequestAccess(gMWS_RxState_c)) )
            {
                PhyAbort();
                Radio_Phy_TimeRxTimeoutIndication(ctx);
            }
#endif
        }
        else if ((!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_RX_WMRK_MSK_MASK)) && (irqStatus & ZLL_IRQSTS_RXWTRMRKIRQ_MASK) && (xcvseqCopy == gIdle_c))
        {
            /*
             * WSW-18719 : This else if is necessary in case the Watermark interrupt happened right at the moment
             * when the sequencer is being put in IDLE in PhyAbort()
             * In this case the interrupt cannot be cleared because the sequencer is in IDLE, and in this state
             * the clock is stopped. To avoid infinite loops in ISR handler due to this race condition, we activate
             * the clock then clear the interrupts and deactivate the clock for minimum power consumption.
             */
            /* Enable clock to clear status register */
            ZLL->SEQ_CTRL_STS |= ZLL_SEQ_CTRL_STS_FORCE_CLK_ON_MASK;
            /* Clear ZLL interrupts. The TX, RX, CCA and TC3 IRQs will be handled on SEQIRQ and must not be cleared. */
            ZLL->IRQSTS = irqStatus & ~(ZLL_IRQSTS_TMR3IRQ_MASK | ZLL_IRQSTS_CCAIRQ_MASK | ZLL_IRQSTS_RXIRQ_MASK | ZLL_IRQSTS_TXIRQ_MASK);
            /* Disable clock to clear status register */
            ZLL->SEQ_CTRL_STS &= ~ZLL_SEQ_CTRL_STS_FORCE_CLK_ON_MASK;
        }
    }

    /* Timer 1 Compare Match */
    if ((irqStatus & ZLL_IRQSTS_TMR1IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR1MSK_MASK)))
    {
        PhyTimeDisableWaitTimeout();
        PhyTime_ISR();
    }

    /* Sequencer interrupt, the autosequence has completed */
    if ((!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_SEQMSK_MASK)) && (irqStatus & ZLL_IRQSTS_SEQIRQ_MASK))
    {
        /* PLL unlock, the autosequence has been aborted due to PLL unlock */
        if (irqStatus & ZLL_IRQSTS_PLL_UNLOCK_IRQ_MASK)
        {
            PhyIsrSeqCleanup();
#if gMWS_UseCoexistence_d
            MWS_CoexistenceReleaseAccess();
#endif
            Radio_Phy_PlmeSyncLossIndication(ctx);
        }
#if (ARB_GRANT_DEASSERTION_SUPPORT == 1)
#if defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1)
        /* Arbitration Grant Loss, the autosequence has been aborted due to ARB_GRANT_DEASSERTION */
        else if((seqCtrlStatus & ZLL_SEQ_CTRL_STS_ARB_GRANT_DEASSERTION_ABORTED_MASK) ||
                (irqStatus & ZLL_IRQSTS_ARB_GRANT_DEASSERTION_IRQ_MASK))
        {
            PhyIsrSeqCleanup();
            PhyIsrTimeoutCleanup();

#if gMWS_UseCoexistence_d
            MWS_CoexistenceReleaseAccess();
#endif
            // Clear up the RF_NOT_ALLOWED_TX_ABORT/RF_NOT_ALLOWED_RX_ABORT by clearing
            // RF_NOT_ALLOWED_ASSERTED bit in 0xA9066008[4] COEXISTENCE CONTROL (COEX_CTRL) Register
            if(RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED_MASK)
            {
                RADIO_CTRL->COEX_CTRL |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED(1);
            }

            if(irqStatus & ZLL_IRQSTS_ARB_GRANT_DEASSERTION_IRQ_MASK)
            {
                // ARB_GRANT_DEASSERTION interrupt is already cleared, nothing to do!
                // TODO: software should probably react to an external RF NOT ALLOWED signal
                // See Jira ticket: KFOURWONE-382
                // Debug_IO_Set(2, 1);
                // Debug_IO_Set(2, 0);
            }

            Radio_Phy_AbortIndication(ctx);
        }
        /* COEX Timeout, the autosequence has been aborted due to COEX Timeout */
        else if (ZLL->COEX_CTRL & ZLL_COEX_CTRL_COEX_TIMEOUT_IRQ_MASK)
        {
            PhyIsrSeqCleanup();
            PhyIsrTimeoutCleanup();
#if gMWS_UseCoexistence_d
            MWS_CoexistenceReleaseAccess();
#endif
            // Clear COEX_TIMEOUT_IRQ flag
            ZLL->COEX_CTRL |= ZLL_COEX_CTRL_COEX_TIMEOUT_IRQ(1);

            // Clear up the RF_NOT_ALLOWED_TX_ABORT/RF_NOT_ALLOWED_RX_ABORT by clearing
            // RF_NOT_ALLOWED_ASSERTED bit in 0xA9066008[4] COEXISTENCE CONTROL (COEX_CTRL) Register
            if(RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED_MASK)
            {
                RADIO_CTRL->COEX_CTRL |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED(1);
            }
            Radio_Phy_AbortIndication(ctx);
        }
#endif /* defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1) */
#endif /* #if (ARB_GRANT_DEASSERTION_SUPPORT == 1) */
        /* TMR3 timeout, the autosequence has been aborted due to TMR3 timeout */
        else if ((irqStatus & ZLL_IRQSTS_TMR3IRQ_MASK) &&
                 (!(irqStatus & ZLL_IRQSTS_RXIRQ_MASK)) &&
                 (gTX_c != xcvseqCopy))
        {
            PhyIsrTimeoutCleanup();
#if gMWS_UseCoexistence_d
            MWS_CoexistenceReleaseAccess();
#endif
            Radio_Phy_TimeRxTimeoutIndication(ctx);
        }
        else
        {
            PhyIsrSeqCleanup();

#if gMWS_UseCoexistence_d
            MWS_CoexistenceReleaseAccess();
#endif
            switch (xcvseqCopy)
            {
            case gTX_c:
                if ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCABFRTX_MASK) && (irqStatus & ZLL_IRQSTS_CCA_MASK))
                {
                    Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, ctx);
                }
                else
                {
                    if (ctx)
                    {
                        ctx->rxParams.psduLength = 0;
                    }

                    Radio_Phy_PdDataConfirm(ctx, FALSE);
                }
                break;

            case gTR_c:
                if ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCABFRTX_MASK) && (irqStatus & ZLL_IRQSTS_CCA_MASK))
                {
                    Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, ctx);
                }
                else if (irqStatus & ZLL_IRQSTS_RXIRQ_MASK)
                {
                    /* Avoid to process unsupported enh ACK in TR sequence.
                       Filter fail interrupt occurred if unsupported packet received */
                    if (ctx && !ctx->filter_fail)
                    {
                        Phy_GetRxInfo(ctx);  /* timestamp, length */

                        if (!(irqStatus & ZLL_IRQSTS_ENH_PKT_STATUS_MASK))
                        {
                            /* Create Imm-ACK because it's not received in PB */
                            RX_PB[1] = 0;   /* FCF */
                            RX_PB[0] = phyFcfFrameAck;

                            if (PhyPpIsRxAckDataPending())
                            {
                                RX_PB[0] |= phyFcfFramePending;
                            }
                            RX_PB[2] = TX_PB[3];    /* SN */
                            ctx->rxParams.psduLength = PHY_IMM_ACK_LENGTH - gPhyFCSSize_c;
                        }

                        /* Copy the received ACK */
                        PHY_MemCpy(ctx->trx_buff, RX_PB, ctx->rxParams.psduLength);

                        Radio_Phy_PdDataConfirm(ctx, (irqStatus & ZLL_IRQSTS_RX_FRM_PEND_MASK) > 0);
                    }
                    else
                    {
                        Radio_Phy_AbortIndication(ctx);
                    }
                }
                else
                {
                    // if SEQIRQ is raised but RxIRQ not, then an error happened
                    // Do Cleanup before abort
                    PhyIsrSeqCleanup();
                    PhyIsrTimeoutCleanup();

#if gMWS_UseCoexistence_d
                    MWS_CoexistenceReleaseAccess();
#endif
                    Radio_Phy_AbortIndication(ctx);
                }
                break;

            case gRX_c:
                /* Frame pending bit can be set for both data and cmd frame for both FV1 and FV2 */
                /* FP is set in PHY_SetImmAckFp() because hw mechanism doesn't work for PAN1
                if (ctx && ((rxFcf & phyFcfFrameVersionMask) != phyFcfFrameVersion2015))
                {
                    if (PhyPpIsTxAckDataPending())
                    {
                        ctx->flags |= gPhyFlagTxAckFP_c;
                    }
                    else
                    {
                        ctx->flags &= ~gPhyFlagTxAckFP_c;
                    }
                } */

                /* Clear EnhAck after Rx (and Ack Tx) finishes */
                ZLL->ENHACK_CTRL0 &= ~ZLL_ENHACK_CTRL0_SW_LEN_RDY_MASK;
                ZLL->ENHACK_CTRL0 &= ~ZLL_ENHACK_CTRL0_SW_HIE_RDY_MASK;

                ZLL->SAM_TABLE &= ~(ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK);

                crc_valid = (ZLL->SEQ_STATE & ZLL_SEQ_STATE_CRCVALID_MASK) >> ZLL_SEQ_STATE_CRCVALID_SHIFT;

                if (ctx && !ctx->filter_fail && crc_valid)
                {
                    Phy_GetRxInfo(ctx);

                    /* Copy the received packet */
                    PHY_MemCpy(ctx->trx_buff, RX_PB, ctx->rxParams.psduLength);


                    Radio_Phy_PdDataIndication(ctx);
                }
                break;

            case gCCA_c:
                if (gCcaED_c == ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCATYPE_MASK) >> ZLL_PHY_CTRL_CCATYPE_SHIFT))
                {
                    Radio_Phy_PlmeEdConfirm(ctx, (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK) >> ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT);

                    if (ctx && (ctx->ccaParams.edScanDurationSym != 0))
                    {
                        /* Restart the energy scan here when scan duration is different from 0 */
                        phyStatus_t status = PhyPlmeCcaEdRequest(ctx);

                        if (ctx && (gPhySuccess_c != status))
                        {
                            ctx->ccaParams.edScanDurationSym = 0;
                            /* Don't disable timeout timer, wait for it to expire */
                        }
                    }
                }
                else /* CCA */
                {
                    if (irqStatus & ZLL_IRQSTS_CCA_MASK)
                    {
                        Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, ctx);
                    }
                    else
                    {
                        Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, ctx);
                    }
                }
                break;

            case gCCCA_c:
                Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, ctx);
                break;

            default:
                Radio_Phy_PlmeSyncLossIndication(ctx);
                break;
            }
        }

        /* NO_RX_RECYCLE bit is set in PhyHwInit().
           The Sequence Manager returns to idle state, and issue a SEQIRQ,
           after a FilterFail or CRC failure */
        if (ctx)
        {
            ctx->filter_fail = 0;
        }

        ZLL->RX_WTR_MARK = RX_WTMRK_START;
    }

    /* Timers interrupt */
    else
    {
        /* Timer 2 Compare Match */
        if ((irqStatus & ZLL_IRQSTS_TMR2IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR2MSK_MASK)))
        {
            PhyTimeDisableEventTrigger();

            if (gIdle_c != xcvseqCopy)
            {
                Radio_Phy_TimeStartEventIndication(ctx);
            }
        }

        /* Timer 3 Compare Match */
        if ((irqStatus & ZLL_IRQSTS_TMR3IRQ_MASK) && (!(irqStatus & ZLL_IRQSTS_TMR3MSK_MASK)))
        {
            PhyTimeDisableEventTimeout();

            /* Ensure that we're not issuing TimeoutIndication while the Automated sequence is still in progress */
            /* TMR3 can expire during R-T turnaround for example, case in which the sequence is not interrupted */
            if (gIdle_c == xcvseqCopy)
            {
                Radio_Phy_TimeRxTimeoutIndication(ctx);
            }
        }
    }

    Radio_Phy_Notify(ctx);
}

/*! *********************************************************************************
* \brief  This function enables the Phy ISR
*
********************************************************************************** */
void PHY_Enable(void)
{
    static bool handler_registered = false;
    if (handler_registered == false)
    {
        handler_registered = true;
        PHY_PhyIrqCreate();
    }
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
    PhyAbort();
    PHY_PhyIrqDisable();
    ProtectFromXcvrInterrupt();
}

/*! *********************************************************************************
* \brief  This function forces the PHY IRQ to be triggered to run the ISR
*
********************************************************************************** */
void PHY_ForceIrqPending( void )
{
    PHY_PhyIrqSetPending();

    sched_reschedule();
}

/* This doesn't work with dual PAN.
   To be replaced with disabling Rx in idle */
#if (AUTO_ACK_DISABLE_SUPPORT == 1)
/*! *********************************************************************************
* \brief  Enable the 802.15.4 radio Auto Acknowledge
*
* Applies only to Sequence R and Sequence TR, ignored during other sequences
* sequence manager will follow a receive frame with an automatic hardware-generated Tx Ack
* frame, assuming other necessary conditions are met.
********************************************************************************** */
void PhyEnableAutoAck(void)
{
    if ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_AUTOACK_MASK) == 0){
        SET_PHYCTRL_FIELD(ZLL_PHY_CTRL_AUTOACK);
    }
}

/*! *********************************************************************************
* \brief  Disable the 802.15.4 radio Auto Acknowledge
*
* Applies only to Sequence R and Sequence TR, ignored during other sequences
* sequence manager will not follow a receive frame with a Tx Ack frame, under any conditions;
* the autosequence will terminate after the receive frame.
********************************************************************************** */
void PhyDisableAutoAck(void)
{
    if ((ZLL->PHY_CTRL & ZLL_PHY_CTRL_AUTOACK_MASK) != 0){
        CLR_PHYCTRL_FIELD(ZLL_PHY_CTRL_AUTOACK);
    }
}
#endif

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */

/*! *********************************************************************************
* \brief  Fill the Rx parameters: RSSI, LQI, Timestamp and PSDU length
*
********************************************************************************** */
static void Phy_GetRxInfo(Phy_PhyLocalStruct_t *ctx)
{
    ctx->mPhyLastRxRSSI       = PhyPlmeGetRSSILevelRequest(ctx->id);
    ctx->mPhyLastRxLQI        = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_LQI_VALUE_MASK) >> ZLL_LQI_AND_RSSI_LQI_VALUE_SHIFT;
    ctx->mPhyLastRxLQI        = Phy_LqiConvert(ctx->mPhyLastRxLQI);

    ctx->rxParams.rssi        = ctx->mPhyLastRxRSSI;
    ctx->rxParams.linkQuality = ctx->mPhyLastRxLQI;

    ctx->rxParams.timeStamp   = ZLL->TIMESTAMP >> ZLL_TIMESTAMP_TIMESTAMP_SHIFT;
    ctx->rxParams.psduLength  = (ZLL->IRQSTS & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK) >> ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT;  /* Including FCS (2 bytes) */
}

/*! *********************************************************************************
* \brief  Scales LQI to 0-255 because HW LQI is not reported over the entire range.
*         To reach values close to 255 the reported RSSI should have a positive value but
*         this doesn't happen in real conditions. In our case, we consider RSSI values closer to 0
*         as being very good so we want the LQI values to be larger than what is being reported.
*
* \param[in]  hwLqi  the LQI reported by HW
*
* \return  uint8_t  the LQI scaled in 0x00-0xFF
*
********************************************************************************** */
static uint8_t Phy_LqiConvert(uint8_t hwLqi)
{
#if (WEIGHT_IN_LQI_CALCULATION == 1)

    int16_t lqi_final_tmp = 0;
    uint8_t lqi_final = 0;
    uint8_t snr_rd = 0;
    int16_t rssi = 0;

    if (hwLqi < LQI_REG_MAX_VAL)
    {
        lqi_final = hwLqi;
    }
    else
    {
        rssi = (XCVR_RX_DIG->NB_RSSI_RES0 & XCVR_RX_DIG_NB_RSSI_RES0_RSSI_NB_MASK) >> XCVR_RX_DIG_NB_RSSI_RES0_RSSI_NB_SHIFT;
        /* As the RSSI register value is only 9bit signed value, need to extend the sign value for 16bit */
        rssi = (rssi & 0x0100) ? (rssi | 0xFF00):(rssi & 0x00FF);

        if(rssi > CLIP_LQI_VAL)
        {
            lqi_final = LQI_MAX_VAL;
        }
        else
        {
            snr_rd = (XCVR_RX_DIG->NB_RSSI_RES1 & XCVR_RX_DIG_NB_RSSI_RES1_SNR_NB_MASK) >> XCVR_RX_DIG_NB_RSSI_RES1_SNR_NB_SHIFT;

            /* Statement implements following formula as a SW WAR for A1 ECO_18 fix [GLC8993-450, GLC8993-450] *
            * LQI = (RSSI - (-103 + LQI_RSSI_SENS))*wRSSI + SNR*wSNR + (-36 + 2*LQI_BIAS)         *
            * RSSI Weight For LQI Calulation - Ref: d_ip_15p4_radio_syn_bg doc                    *
            * RSSI result weight in LQI calculation. Actual weight is: 2 + 0.125 * LQI_SNR_WEIGHT *
            * SNR Weight For LQI Calulation - Ref: d_ip_15p4_radio_syn_bg doc                     *
            * SNR result weight in LQI calculation. When LQI_SNR_WEIGHT!=0, the actual            *
            * weight value is: 1 + 0.125 * LQI_SNR_WEIGHT.                                        *
            * When LQI_SNR_WEIGHT=0, the actual weight value is 0.                                */

            lqi_final_tmp = (103 - nbRssiCtrlReg.lqi_rssi_sens + rssi * 1.0) * (2 + 0.125 * nbRssiCtrlReg.wt_rssi) \
                             + snr_rd / 2 * (1 + nbRssiCtrlReg.wt_snr * 0.125) * (nbRssiCtrlReg.wt_snr != 0 ? 1.0 : 0.0) \
                             + ( -36 + 2 * nbRssiCtrlReg.lqi_bias);
            /* Saturate LQI value to 1Byte */
            lqi_final = (lqi_final_tmp & LQI_MAX_VAL);
        }
    }

    return lqi_final;
/* Code retained for K4 */
#else
    if (hwLqi >= 195)
    {
        hwLqi = 255;
    }
    else
    {
        /* using the same scaling value as KW41 */
        hwLqi = (51 * hwLqi) / 44;
    }

    return hwLqi;
#endif
}
