/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2018-2021, 2023 NXP
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
#include "Phy.h"
#include "PhyInterface.h"
#include "PhySec.h"
#include "PhyPacket.h"
#ifdef PHY_WLAN_COEX
#include "PhyWlanCoex.h"
#endif
#include "dbg_io.h"
#include "EmbeddedTypes.h"

#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#ifndef gMWS_UseCoexistence_d
#define gMWS_UseCoexistence_d 0
#endif

#if gMWS_UseCoexistence_d
#include "MWS.h"
#endif

#if defined(HDI_MODE) && (HDI_MODE == 1) &&                                   \
    (defined(K32W1480_SERIES) ||                                              \
     defined(MCXW727C_cm33_core0_SERIES) ||                                   \
     defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES))
#include "hdi.h"
#endif

#if defined(K32W1480_SERIES) || defined(CPU_KW45B41Z83AFPA_NBU) || \
    defined(MCXW727C_cm33_core0_SERIES)                         || \
        defined(MCXW727C_cm33_core1_SERIES)                     || \
     defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES)
#include "nxp2p4_xcvr.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define PHY_PARAMETERS_VALIDATION 1

#if (TX_POWER_LIMIT_FEATURE == 1)
#define MAX_TX_POWER_BACKOFF gPhyMaxTxPowerLevel_d
#define MIN_TX_POWER_BACKOFF 1
#endif

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
uint8_t gPhyChannelTxPowerLimits[] = gChannelTxPowerLimit_c;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
#ifdef CTX_SCHED
uint8_t PhyPpGetState_base(Phy_PhyLocalStruct_t *ctx);
void PhyAbort_base(Phy_PhyLocalStruct_t *ctx);
#else
#define PhyPpGetState_base(ctx) PhyPpGetState()
#define PhyAbort_base(ctx) PhyAbort()
#endif

void prepare_for_rx(Phy_PhyLocalStruct_t *ctx);

void PLME_SendMessage(Phy_PhyLocalStruct_t *ctx, phyMessageId_t msgType);

#if (TX_POWER_LIMIT_FEATURE == 1)
/*! *********************************************************************************
 * \brief This function will set max tx power limit as per txPowerLimit byte.
 *
 * \param txPowerLimit
 * txPowerLimit (0 or default value), No power backoff is applied
 * txPowerLimit = 1 to 44, force TX power back off to txPowerLimit
 * (txPowerLimit = 0.5dBm step, TX power back off : 0.5dBm step )
 * If > 44 : gPhyMaxTxPowerLevel_d is used.
 * \return txPowerLimit really stored in gPhyChannelTxPowerLimits
 */
uint8_t PhySetTxPowerLimit(uint8_t txPowerLimit)
{
    uint8_t channel_index = 0; // loop index
    uint8_t tmp_limit = 0;

    if ((txPowerLimit <= MAX_TX_POWER_BACKOFF) && (txPowerLimit >= MIN_TX_POWER_BACKOFF))
    {
        tmp_limit = txPowerLimit;
    }
    else
    {
        tmp_limit = gPhyMaxTxPowerLevel_d;
    }

    // Apply TX power limit for each channel
    for (channel_index = 0 ; channel_index < 15; channel_index++)
    {
        gPhyChannelTxPowerLimits[channel_index] = tmp_limit;
    }

    // WSW-24399 - txpowerlimit is capped at 0dBm for channel 26
    gPhyChannelTxPowerLimits[15] = 0;

    return tmp_limit;
}

/*! *********************************************************************************
 * \brief This function will get tx power limit
 *
 * \return txPowerLimit stored in gPhyChannelTxPowerLimits
 */
uint8_t PhyGetTxPowerLimit(void)
{
    // The limit is supposed to be equal for all channels so return only the value for the first channel
    return gPhyChannelTxPowerLimits[0];
}
#endif

/*! *********************************************************************************
 * \brief This function will update tx power limit
 *
 * \return update txPowerLimit stored in gPhyChannelTxPowerLimits
 */
uint8_t PhyUpdateTxPowerLimit(void)
{
    uint8_t channel_cpt = 0;

    /* The channel 26 is not updated because it is clamped to zero dBm => channel_cpt < CHANNEL_NUMBER - 1 */
    for (channel_cpt = 0; channel_cpt < CHANNEL_NUMBER - 1; channel_cpt++)
    {
        gPhyChannelTxPowerLimits[channel_cpt] = gPhyMaxTxPowerLevel_With_FEM_PA_enable_d;
    }

    // The limit is supposed to be equal for all channels so return only the value for the first channel
    return gPhyChannelTxPowerLimits[0];
}

/*! *********************************************************************************
 * \brief This function will get CCA Configuration Values
 * \param[in]  aCca1Threshold       pointer to CCA1 Threshold value
 * \param[in]  aCca2CorrThreshold   pointer to CCA2 Correlation Threshold value
 * \param[in]  aCca2MinNumOfCorrTh  pointer to CCA2 Minimum number of Correlation peaks
 *
 * \return void
 */
void PhyGetCcaConfig(uint8_t *aCca1Threshold, uint8_t *aCca2CorrThreshold, uint8_t *aCca2MinNumOfCorrTh)
{
    *aCca1Threshold      = (uint8_t)(ZLL->CCA_LQI_CTRL & ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK);
    *aCca2CorrThreshold  = (uint8_t)((ZLL->CCA2_CTRL & ZLL_CCA2_CTRL_CCA2_CORR_THRESH_MASK) >> ZLL_CCA2_CTRL_CCA2_CORR_THRESH_SHIFT);
    *aCca2MinNumOfCorrTh = (uint8_t)((ZLL->CCA2_CTRL & ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_MASK) >> ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_SHIFT);
}

/*! *********************************************************************************
 * \brief This function will set CCA Configuration Values
 * \param[in]  aCca1Threshold       CCA1 Threshold value
 * \param[in]  aCca2CorrThreshold   CCA2 Correlation Threshold value
 * \param[in]  aCca2MinNumOfCorrTh  CCA2 Minimum number of Correlation peaks
 *
 * \return void
 */
void PhySetCcaConfig(uint8_t aCca1Threshold, uint8_t aCca2CorrThreshold, uint8_t aCca2MinNumOfCorrTh)
{
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_CCA1_THRESH(aCca1Threshold);

    ZLL->CCA2_CTRL &= ~ZLL_CCA2_CTRL_CCA2_CORR_THRESH_MASK;
    ZLL->CCA2_CTRL |= ZLL_CCA2_CTRL_CCA2_CORR_THRESH(aCca2CorrThreshold);

    ZLL->CCA2_CTRL &= ~ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_MASK;
    ZLL->CCA2_CTRL |= ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH(aCca2MinNumOfCorrTh);
}

/*! *********************************************************************************
* \brief  This function will start a TX sequence. The packet will be sent OTA
*
* \param[in]  pTxPacket   pointer to the TX packet structure
* \param[in]  pRxParams   pointer to RX parameters
* \param[in]  pTxParams   pointer to TX parameters
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPdDataRequest(Phy_PhyLocalStruct_t *ctx)
{
    phyFcf_t *fcf;
    uint32_t irqSts;
    uint8_t xcvseq;
    pdDataReq_t *pTxPacket;
    uint32_t ccaOverheadSym = gPhyTxWuTimeSym;

    if (NULL == ctx)
    {
        return gPhyInvalidParameter_c;
    }

    pTxPacket = ctx->txParams.dataReq;

    if (pTxPacket->CCABeforeTx < gPhyNoCCABeforeTx_c)
    {
        ccaOverheadSym += gPhyRxWuTimeSym + gCCATime_c + gPhyRxWdTimeSym;
    }

    /* if CCA required ... */
    if ((pTxPacket->CCABeforeTx > gPhyNoCCABeforeTx_c) || (pTxPacket->CCABeforeTx == gPhyEnergyDetectMode_c))
    {
        return gPhyInvalidParameter_c;
    }

    if (gIdle_c != PhyPpGetState())
    {
        return gPhyBusy_c;
    }

    ctx_set_rx_ongoing(ctx, FALSE);

    /* Load data into Packet Buffer byte by byte to avoid memory access issues.
       psduLength should include the 2 FCS bytes (gPhyFCSSize_c) */
    PHY_MemCpyVerify(TX_PB, &pTxPacket->psduLength, sizeof(pTxPacket->psduLength));

    /*
     * If this packet contains security, read and store the MacFrameCounter so that it can
     * be used when sending an EnhAck packet reply
     */
    fcf = (phyFcf_t *)(pTxPacket->pPsdu);

    if (fcf->securityEnabled && (pTxPacket->flags & gPhyEncFrame))
    {
        if (pTxPacket->flags & gPhyUpdHDr)
        {
            PhyPacket_SetFrameCounter(pTxPacket->pPsdu, ctx->frameCounter++, pTxPacket->psduLength);
        }

        PhySec_Encrypt(ctx, pTxPacket->pPsdu, pTxPacket->psduLength);
    }

    PHY_MemCpyVerify(TX_PB + sizeof(pTxPacket->psduLength), pTxPacket->pPsdu, pTxPacket->psduLength);

    /* Perform CCA before TX if required */
    if (pTxPacket->CCABeforeTx != gPhyNoCCABeforeTx_c)
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCABFRTX_MASK;
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_CCATYPE_MASK;
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCATYPE(pTxPacket->CCABeforeTx);
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_CCABFRTX_MASK;
    }

    /* Slotted operation */
    if (pTxPacket->slottedTx == gPhySlottedMode_c)
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_SLOTTED_MASK;
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SLOTTED_MASK;
    }

    /* Perform TxRxAck sequence if required by phyTxMode */
    if ((pTxPacket->ackRequired == gPhyRxAckRqd_c) || (pTxPacket->ackRequired == gPhyEnhancedAckReq))
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_RXACKRQD_MASK;

        /* Permit the reception of ACK frames during TR sequence */
        ZLL->RX_FRAME_FILTER |= ZLL_RX_FRAME_FILTER_ACK_FT_MASK;
        xcvseq = gTR_c;
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_RXACKRQD_MASK;
        xcvseq = gTX_c;
    }

    /* Ensure that no spurious interrupts are raised(do not change TMR1 and TMR4 IRQ status) */
    irqSts = ZLL->IRQSTS;
    irqSts &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    irqSts |= ZLL_IRQSTS_TMR3MSK_MASK;
    ZLL->IRQSTS = irqSts;

#if (RADIO_COEX_METRICS_ENABLE==1)
    PhyUpdateCoexSwMetricsTx();
#endif

    Phy_SetSequenceTiming(&pTxPacket->startTime, pTxPacket->txDuration, ccaOverheadSym);

    /* Start the TX / TRX sequence */
    ZLL->PHY_CTRL |= xcvseq;

    /* Unmask SEQ interrupt */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SEQMSK_MASK;

#if gMWS_UseCoexistence_d
    if (gMWS_Success_c != MWS_CoexistenceRequestAccess(gMWS_TxState_c))
    {
        PhyAbort();
        return gPhyBusy_c;
    }
#endif

    PHY_disallow_sleep();

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will start a RX sequence
*
* \param[in]  phyRxMode   slotted/unslotted
* \param[in]  pRxParams   pointer to RX parameters
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeRxRequest(Phy_PhyLocalStruct_t *ctx)
{
    uint32_t irqSts;

    if (NULL == ctx)
    {
        return gPhyInvalidParameter_c;
    }

    if (gIdle_c != PhyPpGetState())
    {
        return gPhyBusy_c;
    }

    ctx->rxParams.ackedWithSecEnhAck = FALSE;

    ctx_set_rx_ongoing(ctx, FALSE);

    Phy_SetSequenceTiming(&ctx->rxParams.startTime, ctx->rxParams.duration, 0);

	/* Slotted operation is not suppported */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SLOTTED_MASK;

    /* Ensure that no spurious interrupts are raised, but do not change TMR1 and TMR4 IRQ status */
    irqSts = ZLL->IRQSTS;
    irqSts &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    irqSts |= ZLL_IRQSTS_TMR3MSK_MASK;
    ZLL->IRQSTS = irqSts;

    /* Filter ACK frames during RX sequence */
    ZLL->RX_FRAME_FILTER &= ~(ZLL_RX_FRAME_FILTER_ACK_FT_MASK);

#if (RADIO_COEX_METRICS_ENABLE==1)
    PhyUpdateCoexSwMetricsRx();
#endif

    prepare_for_rx(ctx);

	/* Start the RX sequence */
	ZLL->PHY_CTRL |= gRX_c;

	/* unmask SEQ interrupt */
	ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SEQMSK_MASK;

	PHY_disallow_sleep();

    return gPhySuccess_c;
}

static void Phy_EdScanTimeoutHandler(uint32_t param)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(param);

    PhyAbort_base(ctx);

    ctx->ccaParams.edScanDurationSym = 0;
    ctx->ccaParams.timer = gInvalidTimerId_c;

    PLME_SendMessage(ctx, gPlmeEdCnf_c);
}

/*! *********************************************************************************
* \brief  This function will start a CCA / CCCA sequence
*
* \param[in]  ccaParam   the type of CCA
* \param[in]  cccaMode   continuous or single CCA
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeCcaEdRequest(Phy_PhyLocalStruct_t *ctx)
{
    uint32_t irqSts;

    if (!ctx)
    {
        return gPhyBusy_c;
    }

#ifdef PHY_PARAMETERS_VALIDATION
    /* Check for illegal CCA type */
    if ((ctx->ccaParams.ccaParam != gPhyCCAMode1_c) &&
        (ctx->ccaParams.ccaParam != gPhyCCAMode2_c) &&
        (ctx->ccaParams.ccaParam != gPhyCCAMode3_c) &&
        (ctx->ccaParams.ccaParam != gPhyEnergyDetectMode_c))
    {
        return gPhyInvalidParameter_c;
    }

    /* Cannot perform Continuous CCA using ED type */
    if ((ctx->ccaParams.ccaParam == gPhyEnergyDetectMode_c) &&
        (ctx->ccaParams.cccaMode == gPhyContCcaEnabled))
    {
        return gPhyInvalidParameter_c;
    }

#endif /* PHY_PARAMETERS_VALIDATION */

    if (gIdle_c != PhyPpGetState())
    {
        return gPhyBusy_c;
    }

    ctx_set_rx_ongoing(ctx, FALSE);

    /* Write in PHY CTRL the desired type of CCA */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_CCATYPE_MASK;
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_CCATYPE(ctx->ccaParams.ccaParam);

    /* Ensure that no spurious interrupts are raised(do not change TMR1 and TMR4 IRQ status) */
    irqSts = ZLL->IRQSTS;
    irqSts &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK);
    irqSts |= ZLL_IRQSTS_TMR3MSK_MASK;
    ZLL->IRQSTS = irqSts;
    /* Unmask SEQ interrupt */
    ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_SEQMSK_MASK;

    if (ctx->ccaParams.cccaMode == gPhyContCcaEnabled) /* continuous CCA */
    {
        /* start the continuous CCA sequence immediately or by TC2', depending on a previous PhyTimeSetEventTrigger() call) */
        ZLL->PHY_CTRL |= gCCCA_c;
    }
    else /* normal CCA */
    {
        /* start the CCA or ED sequence (this depends on CcaType used) immediately or by TC2', depending on a previous PhyTimeSetEventTrigger() call) */
        ZLL->PHY_CTRL |= gCCA_c;
    }

    /* At the end of the scheduled sequence, an interrupt will occur: CCA , SEQ or TMR3 */

    if ((ctx->ccaParams.msgType == gPlmeEdReq_c) &&
        (ctx->ccaParams.timer == gInvalidTimerId_c) &&
        (ctx->ccaParams.edScanDurationSym != 0))
    {
        phyTimeEvent_t ev;
        ev.parameter = ctx->id;
        ev.callback = Phy_EdScanTimeoutHandler;
        ev.timestamp = PhyTime_GetTimestamp() + ctx->ccaParams.edScanDurationSym;
        ctx->ccaParams.timer = PhyTime_ScheduleEvent(&ev);

        if (ctx->ccaParams.timer == gInvalidTimerId_c)
        {
            ctx->ccaParams.edScanDurationSym = 0;
        }
    }

    PHY_disallow_sleep();

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will set the channel number for the specified PAN
*
* \param[in]   channel   new channel number
* \param[in]   pan       the PAN registers (0/1)
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetCurrentChannelRequest(uint8_t channel, uint8_t pan)
{
    uint8_t txPwrDbm;
#if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1)
    uint8_t channelTxPowerLimit = 0;
    uint8_t minTxPower = 0;
#endif

#ifdef PHY_PARAMETERS_VALIDATION
    if ((channel < 11) || (channel > 26))
    {
        return gPhyInvalidParameter_c;
    }
#endif /* PHY_PARAMETERS_VALIDATION */

    txPwrDbm = PhyPlmeGetPwrLevelRequest();

    if (!pan)
    {
        ZLL->CHANNEL_NUM0 = channel;
    }
    else
    {
        ZLL->CHANNEL_NUM1 = channel;
    }

#if defined(HDI_MODE) && (HDI_MODE == 1) &&                                    \
    (defined(K32W1480_SERIES) ||                                               \
     defined(MCXW727C_cm33_core0_SERIES) ||                                    \
     defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES))
    HDI_SendChannelSwitchCmd((uint32_t)channel);
#endif

#if defined(FFU_FPGA_INTF) && (FFU_FPGA_INTF == 1)
    FPGA_SendChannelSwitchCmd((uint32_t)channel);
#endif

#if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1)
    // txPwrDbm unit is dBm and gPhyChannelTxPowerLimits and gPhyMinTxPowerLevel_d unit is half dBm
    // a conversion is needed to have the same unit for the following comparison
    channelTxPowerLimit = gPhyChannelTxPowerLimits[channel - 11]/2;
    minTxPower = (gPhyMinTxPowerLevel_d >> 1) + 0x80; // gPhyMinTxPowerLevel_d is signed but encoded on a uint8_t

    /* Make sure the current Tx power doesn't exceed the Tx power limit for the new channel */
    if ((txPwrDbm > channelTxPowerLimit) && (txPwrDbm < minTxPower))
    {
        PhyPlmeSetPwrLevelRequest(channelTxPowerLimit);
    }
#else
    /* Make sure the current Tx power doesn't exceed the Tx power limit for the new channel */
    if (txPwrDbm > gPhyChannelTxPowerLimits[channel - 11])
    {
        PhyPlmeSetPwrLevelRequest(gPhyChannelTxPowerLimits[channel - 11]);
    }
#endif

#if defined(K32W1480_SERIES) || defined(CPU_KW45B41Z83AFPA_NBU) ||             \
    defined(MCXW727C_cm33_core0_SERIES) ||                                     \
    defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES)
    if (channel == 26)
    {
        XCVR_802p15p4_TxRegulatory(3);
    }
    else
    {
        XCVR_802p15p4_TxRegulatory(0);
    }
#endif

#ifdef PHY_WLAN_COEX
    /* Inform WLAN of the channel update */
    PhyWlanCoex_ChannelUpdate(channel);
#endif

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will return the current channel for a specified PAN
*
* \param[in]   pan   the PAN registers (0/1)
*
* \return  uint8_t  current channel number
*
********************************************************************************** */
uint8_t PhyPlmeGetCurrentChannelRequest(uint8_t pan)
{
    uint8_t channel;

    if (!pan)
    {
        channel = ZLL->CHANNEL_NUM0;
    }
    else
    {
        channel = ZLL->CHANNEL_NUM1;
    }

    return channel;
}

/*! *********************************************************************************
* \brief  This function will set the radio Tx power
*
* \param[in]   pwrStep   the Tx power (dBm)
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetPwrLevelRequest(int8_t pwrStep)
{
    phyStatus_t status = gPhySuccess_c;

#if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1)
    uint8_t phyMinTxPowerLevel = gPhyMinTxPowerLevel_d;
    uint8_t phyMaxTxPowerLevel = gPhyMaxTxPowerLevel_d;
    uint8_t phyTxPowerLevel = 0;
    uint8_t t_pwrStep = pwrStep;

#ifdef PHY_PARAMETERS_VALIDATION
    /* Max Tx power value supported for int8 conversion is (-64 to 63) */
    if ((pwrStep > gPhyMaxTxPowerLevelInt8_d) || (pwrStep < gPhyMinTxPowerLevelInt8_d))
    {
        return gPhyInvalidParameter_c; // avoid setting an invalid value
    }
#endif

#ifndef FFU_SMAC_APP
    /* For ESMAC app, tx power is already in 0.5dB steps, but for others its in dBm */
    t_pwrStep = t_pwrStep << 1;
#endif

    /*
       The PA_PWR register is in half dBm
       The FE_LOSS in the ANNEX55 is already in half dB : not multiplied by 2
    */
    phyTxPowerLevel = t_pwrStep + FE_LOSS_VALUE;

    /* Range modification if an external power amplifier is present in the RX/TX path */
    if (FE_POWER_AMPLIFIER_ENABLE)
    {
        /* The FEM Power amplifier in the ANNEX100 is in dB : multiplied by 2 */
        phyTxPowerLevel -= 2 * FE_POWER_AMPLIFIER_GAIN;
        phyMaxTxPowerLevel = PhyUpdateTxPowerLimit() + FE_LOSS_VALUE - 2 * FE_POWER_AMPLIFIER_GAIN;
        if (ZLL->CHANNEL_NUM0 == 26)
            phyMinTxPowerLevel = 0xD4;  /* -22dBm */
    }

#ifdef PHY_PARAMETERS_VALIDATION
    if ((FE_POWER_AMPLIFIER_ENABLE == 0)
        &&
        ((phyTxPowerLevel < phyMinTxPowerLevel) && (phyTxPowerLevel > phyMaxTxPowerLevel)))
    {
        status = gPhyInvalidParameter_c;
    }
    else if ((FE_POWER_AMPLIFIER_ENABLE==1)
             &&
             ((phyTxPowerLevel < phyMinTxPowerLevel) || (phyTxPowerLevel > phyMaxTxPowerLevel)))
    {
        status = gPhyInvalidParameter_c;
    }
    else
#endif /* PHY_PARAMETERS_VALIDATION */
    {
        /* Do not exceed the Tx power limit for the current channel */
        /* As the TX power is encoded on a uint8_t and represents a int8_t, then the min value
        (which is negative) is superior to the max value.
        Then an incorrect value is superior to the max but inferior to the min value.
         |--------|----------------------|--------|
         0------max-----error value------min----255 */
        if (t_pwrStep > gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM0 - 11]
            && t_pwrStep < phyMinTxPowerLevel)
        {
            return gPhyInvalidParameter_c; // avoid setting an invalid value
        }
#ifdef CTX_SCHED
        if (t_pwrStep > gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM1 - 11]
            && t_pwrStep < phyMinTxPowerLevel)
        {
            return gPhyInvalidParameter_c; // avoid setting an invalid value
        }
#endif

        ZLL->PA_PWR = phyTxPowerLevel & 0x7F; //ZLL->PA_PWR is of 7bits [6:0]
    }
#else /* #if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1) */

#ifdef PHY_PARAMETERS_VALIDATION
    if (pwrStep > gPhyMaxTxPowerLevel_d)
    {
        status = gPhyInvalidParameter_c;
    }
    else
#endif /* PHY_PARAMETERS_VALIDATION */
    {
        /* Do not exceed the Tx power limit for the current channel */
        if (pwrStep > gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM0 - 11])
        {
            pwrStep = gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM0 - 11];
        }

#ifdef CTX_SCHED
        if (pwrStep > gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM1 - 11])
        {
            pwrStep = gPhyChannelTxPowerLimits[ZLL->CHANNEL_NUM1 - 11];
        }
#endif

#if defined(HDI_MODE) && (HDI_MODE == 1) &&                                    \
     (defined(K32W1480_SERIES) ||                                              \
      defined(MCXW727C_cm33_core0_SERIES) ||                                   \
      defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES))
        HDI_SendPowerSwitchCmd((uint32_t)pwrStep);
#endif

        if (pwrStep > 2)
        {
            pwrStep = (pwrStep << 1) - 2;
        }
        else if ( pwrStep < 0 )
        {
            pwrStep = pwrStep << 1;
        }

        // for FPGA, set power in half dB
#if defined(FFU_FPGA_INTF) && (FFU_FPGA_INTF == 1)
        FPGA_SendSetPwrLevelCmd((uint8_t)pwrStep);
#endif

        ZLL->PA_PWR = pwrStep;
    }
#endif
    return status;
}

/*! *********************************************************************************
* \brief  This function will return the radio Tx power
*
* \return  Power level (dBm)
*
********************************************************************************** */
uint8_t PhyPlmeGetPwrLevelRequest(void)
{
    uint8_t pwrStep = (uint8_t)ZLL->PA_PWR;

#if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1)
    int8_t pwrStep_signed = pwrStep;
    // ZLL->PA_PWR is a uint8_t that represents a signed byte
    if(pwrStep_signed > 0x40){  // If negative number
        pwrStep_signed |= 0x80; // Add sign bit
    }

     /*
       The PA_PWR register is in half dBm
       The FE_LOSS in the ANNEX55 is already in half dB : not multiplied by 2
    */
    if (FE_POWER_AMPLIFIER_ENABLE)
    {
        /* The FEM Power amplifier in the ANNEX100 is in dB : multiplied by 2 */
        pwrStep_signed = pwrStep_signed - FE_LOSS_VALUE + 2 * FE_POWER_AMPLIFIER_GAIN;
    }
    else
    {
        pwrStep_signed = pwrStep_signed - FE_LOSS_VALUE;
    }

    /* For ESMAC app, tx power is already in 0.5dB steps, but for others its in dBm */
#ifdef FFU_SMAC_APP
    pwrStep = pwrStep_signed;
#else /*FFU_SMAC_APP*/
    // ZLL->PA_PWR is 0.5dBm step and return value should be in dBm
    pwrStep = pwrStep_signed >> 1; // Division by 2
#endif

#else /*#if (FFU_CNS_TX_PWR_TABLE_CALIBRATION == 1)*/
    if (pwrStep > 2)
    {
        pwrStep = (pwrStep + 2) >> 1;
    }
#endif

    return pwrStep;
}

/*! *********************************************************************************
* \brief  This function will set the value of PHY PIBs
*
* \param[in]   pibId            the Id of the PIB
* \param[in]   pibValue         the new value of the PIB
* \param[in]   phyRegistrySet   the PAN registers (0/1)
* \param[in]   instanceId       the instance of the PHY
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetPIBRequest(phyPibId_t pibId, uint64_t pibValue, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    phyStatus_t result = gPhySuccess_c;

    switch (pibId)
    {
    case gPhyPibCurrentChannel_c:
        {
            bool_t value = !!(ctx->flags & gPhyFlagRxOnWhenIdle_c);

            if (gRX_c == PhyPpGetState_base(ctx))
            {
                PhyAbort_base(ctx);
            }
            result = PhyPlmeSetCurrentChannelRequest((uint8_t) pibValue, instanceId);
            PhyPlmeSetRxOnWhenIdle(value, instanceId);
        }
        break;
    case gPhyPibTransmitPower_c:
        {
            result = PhyPlmeSetPwrLevelRequest((int8_t) pibValue);
        }
        break;
    case gPhyPibLongAddress_c:
        {
            uint64_t longAddr = pibValue;
            result = PhyPpSetLongAddr((uint8_t *) &longAddr, instanceId);
        }
        break;
    case gPhyPibShortAddress_c:
        {
            uint16_t shortAddr = (uint16_t) pibValue;
            result = PhyPpSetShortAddr((uint8_t *) &shortAddr, instanceId);
        }
        break;
    case gPhyPibPanId_c:
        {
            uint16_t panId = (uint16_t) pibValue;
            result = PhyPpSetPanId((uint8_t *) &panId, instanceId);
        }
        break;
    case gPhyPibPanCoordinator_c:
        {
            bool_t macRole = (bool_t) pibValue;
            result = PhyPpSetMacRole(macRole, instanceId);
        }
        break;
    case gPhyPibCurrentPage_c:
        {
            /* Nothinh to do... */
        }
        break;
    case gPhyPibPromiscuousMode_c:
        {
            PhyPpSetPromiscuous((uint8_t)pibValue);
        }
        break;
    case gPhyPibRxOnWhenIdle:
        {
            PhyPlmeSetRxOnWhenIdle( (bool_t)pibValue, instanceId );
        }
        break;
    case gPhyPibFrameWaitTime_c:
        break;
    case gPhyPibDeferTxIfRxBusy_c:
        {
            if (pibValue)
            {
                ctx->flags |= gPhyFlagDeferTx_c;
            }
            else
            {
                ctx->flags &= ~gPhyFlagDeferTx_c;
            }
        }
        break;
    case gPhyPibLastTxAckFP_c:
        {
            result = gPhyReadOnly_c;
        }
        break;
    case gPhyPibCCA3Mode_c:
        {
            result = PhyPlmeSetCCA3ModeRequest((phyCCA3Mode_t) pibValue);
        }
        break;
    default:
        {
            result = gPhyUnsupportedAttribute_c;
        }
        break;
    }

    return result;
}

/*! *********************************************************************************
* \brief  This function will return the value of PHY PIBs
*
* \param[in]   pibId            the Id of the PIB
* \param[out]  pibValue         pointer to a location where the value will be stored
* \param[in]   phyRegistrySet   the PAN registers (0/1)
* \param[in]   instanceId       the instance of the PHY
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeGetPIBRequest(phyPibId_t pibId, uint8_t *pibValue, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    phyStatus_t result = gPhySuccess_c;
    uint64_t value = 0;
    uint8_t size = 1;

    switch (pibId)
    {
    case gPhyPibCurrentChannel_c:
        {
            value = (uint64_t) PhyPlmeGetCurrentChannelRequest(instanceId);
        }
        break;
    case gPhyPibTransmitPower_c:
        {
            value = PhyPlmeGetPwrLevelRequest();
        }
        break;
    case gPhyPibLongAddress_c:
        {
            size = 8;
            PhyPpGetLongAddr((uint8_t *)&value, instanceId);
        }
        break;
    case gPhyPibShortAddress_c:
        {
            size = 2;
            if (!instanceId)
            {
                value = (ZLL->MACSHORTADDRS0 & ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK) >> ZLL_MACSHORTADDRS0_MACSHORTADDRS0_SHIFT;
            }
            else
            {
                value = (ZLL->MACSHORTADDRS1 & ZLL_MACSHORTADDRS1_MACSHORTADDRS1_MASK) >> ZLL_MACSHORTADDRS1_MACSHORTADDRS1_SHIFT;
            }
        }
        break;
    case gPhyPibPanId_c:
        {
            size = 2;
            if (!instanceId)
            {
                value = (ZLL->MACSHORTADDRS0 & ZLL_MACSHORTADDRS0_MACPANID0_MASK) >> ZLL_MACSHORTADDRS0_MACPANID0_SHIFT;
            }
            else
            {
                value = (ZLL->MACSHORTADDRS1 & ZLL_MACSHORTADDRS1_MACPANID1_MASK) >> ZLL_MACSHORTADDRS1_MACPANID1_SHIFT;
            }
        }
        break;
    case gPhyPibPanCoordinator_c:
        {
            if (!instanceId)
            {
                value = !!(ZLL->PHY_CTRL & ZLL_PHY_CTRL_PANCORDNTR0_MASK);
            }
            else
            {
                value = !!(ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_PANCORDNTR1_MASK);
            }
        }
        break;
    case gPhyPibRxOnWhenIdle:
        {
            value = !!(ctx->flags & gPhyFlagRxOnWhenIdle_c);
        }
        break;
    case gPhyPibFrameWaitTime_c:
        break;
    case gPhyPibDeferTxIfRxBusy_c:
        {
            value = !!(ctx->flags & gPhyFlagDeferTx_c);
        }
        break;
    case gPhyPibLastTxAckFP_c:
        {
            value = !!(ctx->flags & gPhyFlagTxAckFP_c);
        }
        break;
    case gPhyPibCCAType_c:
        {
            value = (uint64_t)PhyPlmeGetCCATypeRequest();
        }
        break;
    case gPhyPibCCA3Mode_c:
        {
            value = (uint64_t)PhyPlmeGetCCA3ModeRequest();
        }
        break;

    case gPhyGetRSSILevel_c:
        value = PHY_handle_get_RSSI(ctx);
        break;

    case gPhyGetCtxId:
        value = PHY_get_ctx();
        break;

    default:
        {
            size = 0;
            result = gPhyUnsupportedAttribute_c;
        }
        break;
    }

    /* Avoid unaligned memory access issues */
    memcpy(pibValue, &value, size);
    return result;
}
