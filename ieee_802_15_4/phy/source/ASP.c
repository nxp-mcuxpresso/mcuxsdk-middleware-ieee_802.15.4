/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2021, 2023-2024 NXP
* All rights reserved.
*
* \file
*
* This is the source file for the ASP module.
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
#include "nxp2p4_xcvr.h"
//#include "xcvr_test_fsk.h"

#include "Phy.h"
#include "PhyInterface.h"
#include "PhySec.h"
#include "AspInterface.h"
#include "fsl_component_mem_manager.h"
#include "FunctionLib.h"
#include "fsl_adapter_rpmsg.h"

#if MFG_ENABLE
#include "fsl_adapter_rfimu.h"
#endif

#if !defined(gAspNoHWParameters_c) || (gAspNoHWParameters_c == 0)
#include "HWParameter.h"
#endif

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciAspCommands.h"
#endif

#ifdef gSmacSupported
#include "SMAC_Interface.h"
#endif

#define CH2FREQ(x)             \
    (x == 11) ? 2405000000UL : \
    (x == 12) ? 2410000000UL : \
    (x == 13) ? 2415000000UL : \
    (x == 14) ? 2420000000UL : \
    (x == 15) ? 2425000000UL : \
    (x == 16) ? 2430000000UL : \
    (x == 17) ? 2435000000UL : \
    (x == 18) ? 2440000000UL : \
    (x == 19) ? 2445000000UL : \
    (x == 20) ? 2450000000UL : \
    (x == 21) ? 2455000000UL : \
    (x == 22) ? 2460000000UL : \
    (x == 23) ? 2465000000UL : \
    (x == 24) ? 2470000000UL : \
    (x == 25) ? 2475000000UL : \
    (x == 26) ? 2480000000UL : 0

#if gAspCapability_d

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define mFAD_THR_ResetValue         0x82
#define mANT_AGC_CTRL_ResetValue    0x40
#define mASP_MinTxIntervalMS_d      (5)


/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
enum
{
    gDftNormal_c          = 0,
    gDftTxNoMod_Carrier_c = 1,
    gDftTxPattern_c       = 2,
    gDftTxRandom_c        = 7,
    gDftTxPnChipData_c    = 8,
    gDftTxExternalSrc_c   = 9
};

/*! *********************************************************************************
*************************************************************************************
* Private functions prototype
*************************************************************************************
********************************************************************************** */
static void ASP_PRBS9_Load(void);
static void ASP_TxInterval(uint32_t param);

static void ASP_clearDftfeatures(void);
static void ASP_setDftMode(uint8_t mode);

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#ifdef MFG_ENABLE
static RPMSG_HANDLE_DEFINE(aspRpmsgHandle);
#endif
static uint32_t mAsp_TxIntervalMs = mASP_MinTxIntervalMS_d;
static phyTimeTimerId_t mAsp_TxTimer = gInvalidTimerId_c;
static const uint8_t mAsp_Prbs9Packet[65] =
{
    0x42,
    0xff,0xc1,0xfb,0xe8,0x4c,0x90,0x72,0x8b,0xe7,0xb3,0x51,0x89,0x63,0xab,0x23,0x23,
    0x02,0x84,0x18,0x72,0xaa,0x61,0x2f,0x3b,0x51,0xa8,0xe5,0x37,0x49,0xfb,0xc9,0xca,
    0x0c,0x18,0x53,0x2c,0xfd,0x45,0xe3,0x9a,0xe6,0xf1,0x5d,0xb0,0xb6,0x1b,0xb4,0xbe,
    0x2a,0x50,0xea,0xe9,0x0e,0x9c,0x4b,0x5e,0x57,0x24,0xcc,0xa1,0xb7,0x59,0xb8,0x87
};

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief  Initialize the ASP module
*
* \param[in]  phyInstance The instance of the PHY
* \param[in]  interfaceId The FSCI interface used
*
********************************************************************************** */
void ASP_Init(instanceId_t phyInstance)
{
}

/*! *********************************************************************************
* \brief  ASP SAP handler.
*
* \param[in]  pMsg        Pointer to the request message
* \param[in]  instanceId  The instance of the PHY
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t phyInstance)
{
    AspStatus_t status = gAspSuccess_c;
    uint64_t tmpTstamp = 0;
#if !defined(gAspNoHWParameters_c) || (gAspNoHWParameters_c == 0)
    hardwareParameters_t *pHWParams = NULL;
#endif

#if gFsciIncluded_c
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, NULL, fsciGetAspInterfaceId(phyInstance));
#endif
    switch (pMsg->msgType)
    {
    case aspMsgTypeGetTimeReq_c:
        Asp_GetTimeReq(&tmpTstamp);
        pMsg->msgData.aspGetTimeReq.time = tmpTstamp;  /* remove compiler warning */
        break;
    case aspMsgTypeSetXtalTrimReq_c:
        if(gXcvrSuccess_c == XCVR_SetXtalTrim(pMsg->msgData.aspXtalTrim.trim))
        {
#if !defined(gAspNoHWParameters_c) || (gAspNoHWParameters_c == 0)
            (void)NV_ReadHWParameters(&pHWParams);
            if(pHWParams->xtalTrim != pMsg->msgData.aspXtalTrim.trim)
            {
              pHWParams->xtalTrim = pMsg->msgData.aspXtalTrim.trim;
              (void)NV_WriteHWParameters();
            }
#endif
        }
        else
        {
            status = gAspInvalidParameter_c;
        }
        break;
    case aspMsgTypeGetXtalTrimReq_c:
        *((uint8_t*)&status) = XCVR_GetXtalTrim(); /* remove compiler warning */
        break;
    case aspMsgTypeXcvrWriteReq_c:
        status = Asp_XcvrWriteReq( pMsg->msgData.aspXcvrData.mode,
                                   pMsg->msgData.aspXcvrData.addr,
                                   pMsg->msgData.aspXcvrData.len,
                                   pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeXcvrReadReq_c:
        status = Asp_XcvrReadReq( pMsg->msgData.aspXcvrData.mode,
                                  pMsg->msgData.aspXcvrData.addr,
                                  pMsg->msgData.aspXcvrData.len,
                                  pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeSetFADState_c:
        status = Asp_SetFADState(pMsg->msgData.aspFADState);
        break;
    case aspMsgTypeSetFADThreshold_c:
        status = Asp_SetFADThreshold(pMsg->msgData.aspFADThreshold);
        break;
    case aspMsgTypeSetANTXState_c:
        status = Asp_SetANTXState(pMsg->msgData.aspANTXState);
        break;
    case aspMsgTypeGetANTXState_c:
        *((uint8_t*)&status) = Asp_GetANTXState();
        break;
    case aspMsgTypeSetPowerLevel_c:
        status = Asp_SetPowerLevel(pMsg->msgData.aspSetPowerLevelReq.powerLevel);
        break;
    case aspMsgTypeGetPowerLevel_c:
        *((uint8_t*)&status) = Asp_GetPowerLevel(); /* remove compiler warning */
        break;
    case aspMsgTypeTelecSetFreq_c:
        status = ASP_TelecSetFreq(pMsg->msgData.aspTelecsetFreq.channel);
        break;
    case aspMsgTypeTelecSendRawData_c:
        status = ASP_TelecSendRawData((uint8_t*)&pMsg->msgData.aspTelecSendRawData);
        break;
    case aspMsgTypeTelecTest_c:
        status = ASP_TelecTest(pMsg->msgData.aspTelecTest.mode);
        break;
    case aspMsgTypeSetCCA3Mode_c:
        status = Asp_SetCCA3Mode(pMsg->msgData.aspCCA3Mode);
        break;

    case aspMsgTypeGetRSSILevel_c:
        *((uint8_t*)&status) = Asp_GetRSSILevel(phyInstance); /* remove compiler warning */
        break;

    case aspMsgTypeSetTxInterval_c:
        if( pMsg->msgData.aspSetTxInterval.intervalMs >= mASP_MinTxIntervalMS_d )
        {
            mAsp_TxIntervalMs = pMsg->msgData.aspSetTxInterval.intervalMs;
        }
        else
        {
            status = gAspInvalidParameter_c;
        }
        break;

#ifdef MFG_ENABLE
    case aspMsgTypeReadRegisterReq_c:
        pMsg->msgData.aspReadRegisterRsp.value = *(uint32_t *)(pMsg->msgData.aspReadRegisterCmd.addr);
        pMsg->msgData.aspReadRegisterRsp.status = gAspSuccess_c;

        PLATFORM_RemoteActiveReq();
        if (HAL_RpmsgSend((hal_rpmsg_handle_t)aspRpmsgHandle, (uint8_t *)pMsg, sizeof(AppToAspMessage_t)) != kStatus_HAL_RpmsgSuccess)
        {
            assert(0);
        }
        PLATFORM_RemoteActiveRel();
        break;
	case aspMsgTypeWriteRegisterReq_c:
	    *(uint32_t *)(pMsg->msgData.aspWriteRegisterCmd.addr) = (pMsg->msgData.aspWriteRegisterCmd.value);
        pMsg->msgData.aspWriteRegisterRsp.status = gAspSuccess_c;

        PLATFORM_RemoteActiveReq();
        if (HAL_RpmsgSend((hal_rpmsg_handle_t)aspRpmsgHandle, (uint8_t *)pMsg, sizeof(AppToAspMessage_t)) != kStatus_HAL_RpmsgSuccess)
        {
            assert(0);
        }
        PLATFORM_RemoteActiveRel();
        break;
#endif /* MFG_ENABLE */
    default:
        status = gAspInvalidRequest_c; /* OR gAspInvalidParameter_c */
        break;
    }
#if gFsciIncluded_c
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, (void *)&status, fsciGetAspInterfaceId(phyInstance));
#endif
    return status;
}

/*! *********************************************************************************
* \brief  Returns the current PHY time
*
* \param[in]  time  location where the PHY time will be stored
*
********************************************************************************** */
void Asp_GetTimeReq(uint64_t *time)
{
    *time = PhyTime_ReadClock();
}

/*! *********************************************************************************
* \brief  Write XCVR registers
*
* \param[in]  mode   ZLL/XCVR access
* \param[in]  addr   address
* \param[in]  len    number of bytes to write
* \param[in]  pData  data o be written
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_XcvrWriteReq(uint8_t mode, uint16_t addr, uint8_t len, uint8_t *pData)
{
    if (mode)
    {
        /* Indirect access: XCVR regs */
        FLib_MemCpy((uint8_t *)XCVR_RX_DIG + addr, pData, len);
    }
    else
    {
        /* Direct access: ZLL regs */
        FLib_MemCpy((uint8_t *)ZLL + addr, pData, len);
    }

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Read XCVR registers
*
* \param[in]  mode   Direct/Indirect access
* \param[in]  addr   XCVR address
* \param[in]  len    number of bytes to read
* \param[in]  pData  location where data will be stored
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_XcvrReadReq(uint8_t mode, uint16_t addr, uint8_t len, uint8_t *pData)
{
    if (mode)
    {
        /* Indirect access: XCVR regs */
        FLib_MemCpy(pData, (uint8_t *)XCVR_RX_DIG + addr, len);
    }
    else
    {
        /* Direct access: ZLL regs */
        FLib_MemCpy(pData, (uint8_t *)ZLL + addr, len);
    }

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set Tx output power level
*
* \param[in]  powerLevel   The new power level: 0x03-0x1F (see documentation for details)
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetPowerLevel(uint8_t powerLevel)
{
    if (powerLevel > gPhyMaxTxPowerLevel_d)
    {
        return gAspInvalidParameter_c;
    }

    ZLL->PA_PWR = ZLL_PA_PWR_PA_PWR(powerLevel);

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Read the current Tx power level
*
* \return  power level
*
********************************************************************************** */
uint8_t Asp_GetPowerLevel(void)
{
    return PhyPlmeGetPwrLevelRequest();
}

/*! *********************************************************************************
* \brief  Set the state of Active Promiscuous functionality
*
* \param[in]  state  new state
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetActivePromState(bool_t state)
{
    PhySetActivePromiscuous(state);
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set the state of Fast Antenna Diversity functionality
*
* \param[in]  state  new state
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetFADState(bool_t state)
{
    AspStatus_t status = gAspSuccess_c;

    if (gPhySuccess_c != PhyPlmeSetFADStateRequest(state))
    {
        status = gAspDenied_c;
    }

    return status;
}

/*! *********************************************************************************
* \brief  Set the Fast Antenna Diversity threshold
*
* \param[in]  threshold
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetFADThreshold(uint8_t thresholdFAD)
{
    AspStatus_t status = gAspSuccess_c;

    if (gPhySuccess_c != PhyPlmeSetFADThresholdRequest(thresholdFAD))
    {
        status = gAspDenied_c;
    }
    return status;
}

/*! *********************************************************************************
* \brief  Set the ANTX functionality
*
* \param[in]  state
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetANTXState(bool_t state)
{
    AspStatus_t status = gAspSuccess_c;

    if (gPhySuccess_c != PhyPlmeSetANTXStateRequest(state))
    {
        status = gAspDenied_c;
    }
    return status;
}

/*! *********************************************************************************
* \brief  Get the ANTX functionality
*
* \return  current state
*
********************************************************************************** */
uint8_t Asp_GetANTXState(void)
{
    return PhyPlmeGetANTXStateRequest();
}

/*! *********************************************************************************
* \brief  Set the ANTX pad state
*
* \param[in]  antAB_on
* \param[in]  rxtxSwitch_on
*
* \return  status
*
********************************************************************************** */
uint8_t Asp_SetANTPadStateRequest(bool_t antAB_on, bool_t rxtxSwitch_on)
{
    return PhyPlmeSetANTPadStateRequest(antAB_on, rxtxSwitch_on);
}

/*! *********************************************************************************
* \brief  Set the ANTX pad strength
*
* \param[in]  hiStrength
*
* \return  status
*
********************************************************************************** */
uint8_t Asp_SetANTPadStrengthRequest(bool_t hiStrength)
{
    return 0;
}

/*! *********************************************************************************
* \brief  Set the ANTX inverted pads
*
* \param[in]  invAntA  invert Ant_A pad
* \param[in]  invAntB  invert Ant_B pad
* \param[in]  invTx    invert Tx pad
* \param[in]  invRx    invert Rx pad
*
* \return  status
*
********************************************************************************** */
uint8_t Asp_SetANTPadInvertedRequest(bool_t invAntA, bool_t invAntB, bool_t invTx, bool_t invRx)
{
    return PhyPlmeSetANTPadInvertedRequest(invAntA, invAntB, invTx, invRx);
}

/*! *********************************************************************************
* \brief  Set the CCA3 mode
*
* \param[in]  mode
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t Asp_SetCCA3Mode(phyCCA3Mode_t cca3mode)
{
    AspStatus_t status = gAspSuccess_c;

    if (gPhySuccess_c != PhyPlmeSetCCA3ModeRequest(cca3mode))
    {
        status = gAspDenied_c;
    }
    return status;
}

/*! *********************************************************************************
* \brief  Get the last RSSI level
*
* \param[in]  instanceId  The instance of the PHY
*
* \return  RSSI
*
********************************************************************************** */
uint8_t Asp_GetRSSILevel(instanceId_t phyInstance)
{
    return PhyPlmeGetRSSILevelRequest(phyInstance);
}

/*! *********************************************************************************
* \brief  Set current channel
*
* \param[in]  channel  channel number (11-26)
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t ASP_TelecSetFreq(uint8_t channel)
{
    AspStatus_t status = gAspSuccess_c;

    PhyAbort();

    if (gPhySuccess_c != PhyPlmeSetCurrentChannelRequest(channel, 0))
    {
        status = gAspInvalidParameter_c;
    }

    return status;
}

/*! *********************************************************************************
* \brief  Send a raw data frame OTA
*
* \param[in]  dataPtr  raw data
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t ASP_TelecSendRawData(uint8_t *dataPtr)
{
    AspStatus_t status = gAspSuccess_c;
    uint8_t *pTxBuffer = (uint8_t*)TX_PACKET_RAM;
    uint32_t len;

    /* Validate the length */
    if ((dataPtr[0] + 2) > gMaxPHYPacketSize_c)
    {
        status = gAspTooLong_c;
    }
    else
    {
        /* Force Idle */
        PhyAbort();
#if (XCVR_MISC_SUPPORT == 1)
        XCVR_MISC->RF_DFT_CTRL &= ~XCVR_MISC_RF_DFT_CTRL_RADIO_DFT_MODE_MASK;
#endif
        ZLL->SEQ_CTRL_STS &= ~ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK;

        len = *dataPtr++;
        /* Add FCS length to PSDU Length*/
        *pTxBuffer++ = len + 2;
        /* Load the TX PB: load the PSDU Length byte but not the FCS bytes */
        while (len--)
        {
            *pTxBuffer++ = *dataPtr++;
        }
        /* Program a Tx sequence */
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_XCVSEQ(gTX_c);
    }

    return status;
}

/*! *********************************************************************************
* \brief  Set Telec test mode
*
* \param[in]  mode  Telec test mode
*
* \return  AspStatus_t
*
********************************************************************************** */
AspStatus_t ASP_TelecTest(uint8_t mode)
{
    uint8_t channel;
    AspStatus_t status = gAspSuccess_c;
    static bool_t fracSet = FALSE;
    static uint32_t pad_dly;

    /* Get current channel number */
    channel = PhyPlmeGetCurrentChannelRequest(0);

    if (fracSet)
    {
        ASP_TelecSetFreq(channel);
        fracSet = FALSE;
    }

    switch (mode)
    {
    case gTestForceIdle_c:  /* ForceIdle() */
        XCVR_DftTxOff();
        /* CONNRF-1310 */
        XCVR_TX_DIG->DATA_PADDING_CTRL = pad_dly;

        /* Stop Tx interval timer (if started) */
        PhyTime_CancelEvent(mAsp_TxTimer);
        PhyAbort();

        ASP_setDftMode(0x0U);
        ASP_clearDftfeatures();
        XCVR_TSM->CTRL &= ~XCVR_TSM_CTRL_FORCE_TX_EN_MASK;
        ZLL->SEQ_CTRL_STS &= ~ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK;
        XCVR_TX_DIG->RF_DFT_PATTERN = 0x00000000;
        break;

    case gTestPulseTxPrbs9_c:  /* Continuously transmit a PRBS9 pattern. */
#if (XCVR_MISC_SUPPORT == 1)
        XCVR_MISC->RF_DFT_CTRL &= ~XCVR_MISC_RF_DFT_CTRL_RADIO_DFT_MODE_MASK;
#endif
        ASP_PRBS9_Load(); /* Load the TX RAM */
        /* Enable continuous TX mode */
        ZLL->SEQ_CTRL_STS |= ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK;
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_XCVSEQ(gTX_c);
        break;

    case gTestContinuousRx_c:  /* Sets the device into continuous RX mode */
#if (XCVR_MISC_SUPPORT == 1)
        XCVR_MISC->RF_DFT_CTRL &= ~XCVR_MISC_RF_DFT_CTRL_RADIO_DFT_MODE_MASK;
#endif
        /* Set length of data in DUAL_PAN_DWELL register */
        ZLL->DUAL_PAN_CTRL &= ~ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_MASK;
        ZLL->DUAL_PAN_CTRL |= ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL(127);
        /* Enable continuous RX mode */
        ZLL->SEQ_CTRL_STS |= ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK;
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_XCVSEQ(gRX_c);
        break;

    case gTestContinuousTxMod_c:  /* Sets the device to continuously transmit a 10101010 pattern */
        ASP_setDftMode(0x0U);
        ASP_clearDftfeatures();
        ASP_setDftMode(0x2U);
        /* leaves LFSR mode disabled but sets clock field which is common */
        XCVR_TX_DIG->RF_DFT_TX_CTRL1 = XCVR_TX_DIG_RF_DFT_TX_CTRL1_LFSR_CLK_SEL(0x2U);
        XCVR_TX_DIG->RF_DFT_PATTERN = 0xAAAAAAAAU;
        /* PAD_DLY_EN=0 is required for TX DFT to be operational */
        XCVR_TX_DIG->DATA_PADDING_CTRL = XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(0);
        /* Warm-up the Radio */
        XCVR_ForceTxWu();
        XCVR_TX_DIG->RF_DFT_TX_CTRL2 = XCVR_TX_DIG_RF_DFT_TX_CTRL2_DFT_PATTERN_EN(1); /* now enable the PATTERN mode */
        break;

    case gTestContinuousTxNoMod_c: /* Sets the device to continuously transmit an unmodulated CW */
        /*
         * As per CONNRF-1310, there's a HW issue, that's
         * present on both KW45 as well as KW47. The short version
         * is that the data padding is incorrectly being added as
         * an offset to the PLL numerator in this DFT mode.
         * As a W/A save it and restore it when exiting DFT.
         */
        pad_dly = XCVR_TX_DIG->DATA_PADDING_CTRL;
        XCVR_TX_DIG->DATA_PADDING_CTRL = 0;

        XCVR_DftTxCW(CH2FREQ(channel));

        fracSet = TRUE;
        break;

    case gTestContinuousTx1MbpsPRBS9_c:
#if 0
        XCVR_TX_DIG->CTRL &= ~(XCVR_TX_DIG_CTRL_DFT_CLK_SEL_MASK |
                               XCVR_TX_DIG_CTRL_RADIO_DFT_MODE_MASK |
                               XCVR_TX_DIG_CTRL_LFSR_LENGTH_MASK);
        XCVR_TX_DIG->CTRL |= XCVR_TX_DIG_CTRL_RADIO_DFT_MODE(gDftTxPnChipData_c) |
                             XCVR_TX_DIG_CTRL_LFSR_LENGTH(0) | /* length 9 */
                             XCVR_TX_DIG_CTRL_LFSR_EN_MASK;
        *((volatile uint32_t *)(XCVR_MISC_BASE+0x10)) |= XCVR_CTRL_DTEST_CTRL_DTEST_EN_MASK;
        XCVR_TSM->CTRL |= XCVR_TSM_CTRL_FORCE_TX_EN_MASK;
#endif
        break;

    case gTestContinuousTxExternalSrc_c:
#if 0
        XCVR_TX_DIG->CTRL &= ~(XCVR_TX_DIG_CTRL_DFT_CLK_SEL_MASK |
                               XCVR_TX_DIG_CTRL_RADIO_DFT_MODE_MASK);
        XCVR_TX_DIG->CTRL |= XCVR_TX_DIG_CTRL_RADIO_DFT_MODE(gDftTxExternalSrc_c);
        *((volatile uint32_t *)(XCVR_MISC_BASE+0x10)) |= XCVR_CTRL_DTEST_CTRL_DTEST_EN_MASK;
        XCVR_TSM->CTRL |= XCVR_TSM_CTRL_FORCE_TX_EN_MASK;
#endif
        break;

    case gTestContinuousTxModZero_c:
        ASP_setDftMode(0x0U);
        ASP_clearDftfeatures();
        ASP_setDftMode(0x2U);
        /* leaves LFSR mode disabled but sets clock field which is common */
        XCVR_TX_DIG->RF_DFT_TX_CTRL1 = XCVR_TX_DIG_RF_DFT_TX_CTRL1_LFSR_CLK_SEL(0x2U);
        XCVR_TX_DIG->RF_DFT_PATTERN = 0x00000000U;
        /* PAD_DLY_EN=0 is required for TX DFT to be operational */
        XCVR_TX_DIG->DATA_PADDING_CTRL = XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(0);
        /* Warm-up the Radio */
        XCVR_ForceTxWu();
        XCVR_TX_DIG->RF_DFT_TX_CTRL2 = XCVR_TX_DIG_RF_DFT_TX_CTRL2_DFT_PATTERN_EN(1); /* now enable the PATTERN mode */
        break;

    case gTestContinuousTxModOne_c:
        ASP_setDftMode(0x0U);
        ASP_clearDftfeatures();
        ASP_setDftMode(0x2U);
        /* leaves LFSR mode disabled but sets clock field which is common */
        XCVR_TX_DIG->RF_DFT_TX_CTRL1 = XCVR_TX_DIG_RF_DFT_TX_CTRL1_LFSR_CLK_SEL(0x2U);
        XCVR_TX_DIG->RF_DFT_PATTERN = 0xFFFFFFFFU;
        /* PAD_DLY_EN=0 is required for TX DFT to be operational */
        XCVR_TX_DIG->DATA_PADDING_CTRL = XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(0);
        /* Warm-up the Radio */
        XCVR_ForceTxWu();
        XCVR_TX_DIG->RF_DFT_TX_CTRL2 = XCVR_TX_DIG_RF_DFT_TX_CTRL2_DFT_PATTERN_EN(1); /* now enable the PATTERN mode */
        break;

    case gTestTxPacketPRBS9_c:
        ASP_TxInterval( (uint32_t)mAsp_Prbs9Packet );
        break;

    default:
        status = gAspInvalidParameter_c;
        break;
    }

    return status;
}

/*! *********************************************************************************
* \brief  Transmit a raw data packet at a specific interval
*
* \param[in]  address of the raw data packet
*
********************************************************************************** */
static void ASP_TxInterval(uint32_t param)
{
    phyTimeEvent_t ev;

    /* convert interval to symbols */
    ev.timestamp = ((uint64_t)mAsp_TxIntervalMs * 1000) / 16;
    ev.timestamp += PhyTime_GetTimestamp();
    ev.callback = ASP_TxInterval;
    ev.parameter = param;
    mAsp_TxTimer = PhyTime_ScheduleEvent(&ev);

    ASP_TelecSendRawData((uint8_t *)param);
}

/*! *********************************************************************************
* \brief  Generate and load a PRBS9 pattern into Tx buffer
*
********************************************************************************** */
static void ASP_PRBS9_Load(void)
{
#if 1
    uint32_t i;

    for (i = 0; i < sizeof(mAsp_Prbs9Packet); i++)
    {
        ((uint8_t*)TX_PACKET_RAM)[i] = mAsp_Prbs9Packet[i];
    }
#else
    uint8_t c1;  /* Byte counter */
    uint8_t c2;  /* Bit counter */
    uint16_t t1; /* LFSR */
    uint16_t t2; /* LFSR output */
    uint16_t t3; /* LFSR feedback tap */
    uint8_t t4;  /* Assembled transmit byte */
    uint8_t *pTxBuffer = (uint8_t *)ZLL->PKT_BUFFER_TX;

    pTxBuffer[0] = 64;
    t1 = 0x01FF;                 /* Initialize the LFSR */
    for (c1 = 1; c1 <= 64; c1++) /* Byte counter */
    {
        t4 = 0x00;                 /* Initialize the byte */
        for (c2 = 0; c2 < 8; c2++) /* Bit counter */
        {
            t2 = (t1 & 0x0001); /* LFSR output */
            if (t2 == 0x0001)
            {
                t4 = t4 | 0x80; /* Set/Clear byte based on LFSR output */
            }
            if (c2 != 7)
            {
                t4 = t4 >> 1; /* LSBit will be first bit out of LFSR */
            }
            t3 = ((t1 & 0x0010u) >> 4u); /* LFSR tap */
            t1 = t1 >> 1;                /* Now shift the LFSR */
            if (t2 == t3)                /* Set/Clr the LFSR MSBit */
            {
                t1 = t1 & 0xFEFF;
            }
            else
            {
                t1 = t1 | 0x0100;
            }
        }
        pTxBuffer[c1] = t4;
    }
#endif
}

static void ASP_clearDftfeatures(void)
{
    /* Clear DFT_RAM_EN */
    XCVR_TX_DIG->RF_DFT_TX_CTRL0 &= ~(XCVR_TX_DIG_RF_DFT_TX_CTRL0_DFT_RAM_EN_MASK);
    /* Clear LFSR_EN */
    XCVR_TX_DIG->RF_DFT_TX_CTRL1 &= ~(XCVR_TX_DIG_RF_DFT_TX_CTRL1_LFSR_EN_MASK);
    /* Clear DFT_PATTERN_EN */
    XCVR_TX_DIG->RF_DFT_TX_CTRL2 &= ~(XCVR_TX_DIG_RF_DFT_TX_CTRL2_DFT_PATTERN_EN_MASK);
}

static void ASP_setDftMode(uint8_t mode)
{
#if !RW610_FPGA /* No definition for XCVR_MISC->RADIO_DFT */
#if defined(RADIO_IS_GEN_3P5)
    /* Clear RADIO_DFT_MODE */
    XCVR_MISC->RADIO_DFT &= ~XCVR_MISC_RADIO_DFT_RADIO_DFT_MODE_MASK;
    /* Set RADIO_DFT_MODE */
    XCVR_MISC->RADIO_DFT |= XCVR_MISC_RADIO_DFT_RADIO_DFT_MODE(mode);

#elif defined(RADIO_IS_GEN_4P0) || defined(RADIO_IS_GEN_4P5) || defined(RADIO_IS_GEN_4P7)
#if (XCVR_MISC_SUPPORT == 1)
    /* Clear RADIO_DFT_MODE */
    XCVR_MISC->RF_DFT_CTRL &= ~XCVR_MISC_RF_DFT_CTRL_RADIO_DFT_MODE_MASK;
    /* Set RADIO_DFT_MODE */
    XCVR_MISC->RF_DFT_CTRL |= XCVR_MISC_RF_DFT_CTRL_RADIO_DFT_MODE(mode);
#endif
#else
   #error "Unsupported radio version selected in compile flags."
#endif /* defined(RADIO_IS_GEN_3P5) */
#endif
}

#endif /* gAspCapability_d */
