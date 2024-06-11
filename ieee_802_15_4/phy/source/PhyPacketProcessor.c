/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2018-2024 NXP
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
#include "PhyPlatform.h"
#include "dbg_io.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_oqpsk_802p15p4_config.h"
#include "nxp_xcvr_coding_config.h"
#include "nxp_xcvr_ext_ctrl.h"

#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#include <stdint.h>
#include <limits.h>
#include <assert.h>
#if (defined(HWINIT_DEBUG_DTEST) && (HWINIT_DEBUG_DTEST == 1L))
#include "dtest.h"
#endif

#if gMWS_UseCoexistence_d
#include "MWS.h"
#endif


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define mPhyDSM_GuardTime_d  (5) /* DSM_TIME ticks (32.768KHz) */

#ifndef gPhyUseHwSAMTable
/*
 * SAM bitmask is used as an optmization for cases where the indirect queue
 * is sparse containing a lot of unused entries. This makes the search quicker, very
 * needed in case search is happening while in a time sensitive context (eg. ISR).
 * TODO: This is mainly used as a workaround because the SAP table is not working
 * correctly for multipan contexts and thus SAP table is just a plain table and
 * searching is performed in SW.
 *
 * Using uint32_t as bitmask base chunk and SHFT value needs to correlate with
 * the number of bits (for 32 bit, 5 is the correct shift as 2^5 = 32). There is
 * a static assert to ensure that this has been properly configure as the
 * underlying chunk representation can be any power of 2 type (u8, u16, u32, u64).
 * Controlled by bitmask_chnk_t typedef.
 * Note that indexes are stored in reversed order in the bitmap (in order to make
 * use of CLZ instruction for ARM and not use CTZ which using an additional RBIT).
 * So index(bit[1]) < index(bit[0]).
 */

typedef uint32_t bitmask_chnk_t;

typedef struct {
    bitmask_chnk_t *bitmask;
    uint8_t        no_chunks;
    uint16_t       *table;
} sw_sam_tbl_t;

#define SW_SAM_BITMASK_SHFT       (5)
#define SW_SAM_BITMASK_CHNK_NBITS (sizeof(bitmask_chnk_t) * CHAR_BIT)
#define SW_SAM_BITMASK_REMD       (SW_SAM_BITMASK_CHNK_NBITS - 1)
#define SW_SAM_BITMASK_POS_SHIFT  (1ULL << SW_SAM_BITMASK_REMD)

#define SW_SAM_BITMASK_CHNK_NO(tbl_size) (((tbl_size) + SW_SAM_BITMASK_CHNK_NBITS - 1) >> SW_SAM_BITMASK_SHFT)

#define SW_SAM_BITMASK_TEST(bitmask, idx) ((bitmask)[((uint8_t)(idx)) >> SW_SAM_BITMASK_SHFT] & (SW_SAM_BITMASK_POS_SHIFT >> ((uint8_t)(idx) & SW_SAM_BITMASK_REMD)))
#define SW_SAM_BITMASK_SET(bitmask, idx) ((bitmask)[((uint8_t)(idx)) >> SW_SAM_BITMASK_SHFT] |= (SW_SAM_BITMASK_POS_SHIFT >> ((uint8_t)(idx) & SW_SAM_BITMASK_REMD)))
#define SW_SAM_BITMASK_CLEAR(bitmask, idx) ((bitmask)[((uint8_t)(idx)) >> SW_SAM_BITMASK_SHFT] &= ~(SW_SAM_BITMASK_POS_SHIFT >> ((uint8_t)(idx) & SW_SAM_BITMASK_REMD)))

bitmask_chnk_t sw_sap_bitmask[SW_SAM_BITMASK_CHNK_NO(gPhySAPSize_d)];

/* Table to hold indirect checksum when SW variant is used */
uint16_t sw_sap_tbl[gPhySAPSize_d];

sw_sam_tbl_t sw_sap =
{
    .bitmask = sw_sap_bitmask,
    .no_chunks = SW_SAM_BITMASK_CHNK_NO(gPhySAPSize_d),
    .table = sw_sap_tbl,
};

/* Make sure that SW SAP bitmask is properly configured */
_Static_assert((1ULL << SW_SAM_BITMASK_SHFT) == (sizeof(sw_sap_bitmask[0]) * CHAR_BIT), "Error in configuring the SW SAP bitmask shift");

bitmask_chnk_t sw_saa_bitmask[SW_SAM_BITMASK_CHNK_NO(gPhySAASize_d)];

/* Table to hold indirect checksum when SW variant is used */
uint16_t sw_saa_tbl[gPhySAASize_d];

sw_sam_tbl_t sw_saa =
{
    .bitmask = sw_saa_bitmask,
    .no_chunks = SW_SAM_BITMASK_CHNK_NO(gPhySAASize_d),
    .table = sw_saa_tbl,
};

/* Make sure that SW SAA bitmask is properly configured */
_Static_assert((1ULL << SW_SAM_BITMASK_SHFT) == (sizeof(sw_saa_bitmask[0]) * CHAR_BIT), "Error in configuring the SW SAA bitmask shift");
#endif

/*! *********************************************************************************
*************************************************************************************
* Private declaration
*************************************************************************************
********************************************************************************** */
#define FIRST_CHANNEL_OFFSET    (11)
static uint8_t sChannelRssiOffset[] = gChannelRssiOffset_c;

/*! *********************************************************************************
*************************************************************************************
* Private functions prototype
*************************************************************************************
********************************************************************************** */
uint32_t PhyTime_GetNextEvent(void);

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
const uint8_t gPhyIdlePwrState = gPhyDefaultIdlePwrMode_c;
const uint8_t gPhyActivePwrState = gPhyPwrIdle_c; /* Do not change! */
#if 0
static uint32_t mPhyDSMDuration = 0xFFFFF0;
#endif

/* Accept FrameVersion 0, 1 and 2 packets, reject all others (FRM_VER[11:8] = b0111)
 * Accept Beacon, Data and MAC command frame types. */
const uint32_t mDefaultRxFiltering = ZLL_RX_FRAME_FILTER_FRM_VER_FILTER(7) |
                                     ZLL_RX_FRAME_FILTER_CMD_FT_MASK       |
                                     ZLL_RX_FRAME_FILTER_DATA_FT_MASK      |
                                     ZLL_RX_FRAME_FILTER_BEACON_FT_MASK;

uint8_t gPhyTxWuTimeSym;  /*!< TSM TX warmup time in us */
uint8_t gPhyTxWdTimeSym;  /*!< TSM TX warmdown time in us */
uint8_t gPhyRxWuTimeSym;  /*!< TSM RX warmup time in us */
uint8_t gPhyRxWdTimeSym;  /*!< TSM RX warmdown time in us */

#if (WEIGHT_IN_LQI_CALCULATION == 1)
Phy_nbRssiCtrl_t nbRssiCtrlReg;
#endif

#ifndef gPhyUseHwSAMTable
static uint32_t sw_sam_search_idx(sw_sam_tbl_t *tbl, uint32_t start, uint32_t stop, uint16_t checksum, bool free)
{
    uint8_t chnk_start = start >> SW_SAM_BITMASK_SHFT;
    uint8_t chnk_end = (stop + SW_SAM_BITMASK_CHNK_NBITS - 1) >> SW_SAM_BITMASK_SHFT;
    uint32_t entry_idx;

    /* Optimized search using indirect queue bitmask */
    for (uint8_t chnk = chnk_start; chnk < chnk_end; chnk++)
    {
        bitmask_chnk_t sw_bitmask_copy = tbl->bitmask[chnk];

        /* Searching for free spot */
        if (free)
        {
            sw_bitmask_copy = ~sw_bitmask_copy;
        }

        while (sw_bitmask_copy)
        {
            /* Find first set bit */
            entry_idx = __builtin_clz(sw_bitmask_copy);

            /* Bits order are reversed with respect to the index number */
            sw_bitmask_copy &= ~(SW_SAM_BITMASK_POS_SHIFT >> entry_idx);

            /* Adjust to absolute indexes */
            entry_idx += chnk * SW_SAM_BITMASK_CHNK_NBITS;
            if ((entry_idx >= start) && (entry_idx < stop))
            {
                if (free || (tbl->table[entry_idx] == checksum))
                {
                    return entry_idx;
                }
            }
        }
    }
    return INV_IDX;
}
#endif

static void PhyRadioInit(void)
{
    const xcvr_config_t *xcvrConfig         = &xcvr_oqpsk_802p15p4_250kbps_full_config;
    const xcvr_coding_config_t *rbmeConfig  = &xcvr_ble_uncoded_config;

    XCVR_Init(&xcvrConfig, &rbmeConfig);

    PhyPlatformHwInit();
}

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief  Initialize the 802.15.4 Radio registers
*
********************************************************************************** */
void PhyHwInit(void)
{
    PhyRadioInit();

    /* Set demodulator to legacy 802.15.4 */
    XCVR_MISC->XCVR_CTRL &= ~XCVR_MISC_XCVR_CTRL_DEMOD_SEL_MASK;
    XCVR_MISC->XCVR_CTRL |= XCVR_MISC_XCVR_CTRL_DEMOD_SEL(0x2U);

#if (HWINIT_CONFIGURE_LEGACY_DATA_RATE == 1)
    /* Select Data Rate to legacy 802.15.4 */
    XCVR_MISC->XCVR_CTRL &= ~XCVR_MISC_XCVR_CTRL_DATA_RATE_MASK;
    XCVR_MISC->XCVR_CTRL |= XCVR_MISC_XCVR_CTRL_DATA_RATE(0x0U);
#endif

#if (HWINIT_USE_CCA1_FROM_RX_DIGITAL == 1)
    /* Use the CCA1 information computed by the RX Digital */
    XCVR_ZBDEMOD_REGS->CCA_LQI_SRC |= XCVR_ZBDEM_CCA_LQI_SRC_CCA1_FROM_RX_DIG(1);
#endif

#if (HWINIT_USE_CCA2_DECOUPLED_FROM_DEMOD == 1)
    /* Reset to 0b0 - Use standalone (new) CCA Mode 2 Engine, decoupled from demodulator */
    XCVR_ZBDEMOD_REGS->CCA2_CTRL &= ~(XCVR_ZBDEM_CCA2_CTRL_USE_DEMOD_CCA2_MASK);
#endif

#if (HWINIT_CONFIGURE_CCA2_THRESHOLD == 1)
    /* Set CCA Mode2 Threshold Values */
    XCVR_ZBDEMOD_REGS->CCA2_THRESH &= ~(XCVR_ZBDEM_CCA2_THRESH_CCA2_CNT_THRESH_MASK |\
                                        XCVR_ZBDEM_CCA2_THRESH_CCA2_SYM_THRESH_MASK);
    XCVR_ZBDEMOD_REGS->CCA2_THRESH |= XCVR_ZBDEM_CCA2_THRESH_CCA2_CNT_THRESH(0xAA) |\
                                      XCVR_ZBDEM_CCA2_THRESH_CCA2_SYM_THRESH(0x40);
    ZLL->CCA2_CTRL &= ~ZLL_CCA2_CTRL_CCA2_CORR_THRESH_MASK;
    ZLL->CCA2_CTRL |= ZLL_CCA2_CTRL_CCA2_CORR_THRESH(0xB2);
#endif

    /* The CCA RSSI must come from RX DIG otherwise the CCA will not work correctly and will report
       last received packet RSSI */
    HWINIT_CCA1_FROM_RX_DIG();

#if (HWINIT_SET_RSSI_ADJUSTEMENT == 1)
    /* Update finetuned RSSI RSSI_ADJ_NB Offset value */
    XCVR_SetRssiAdjustment(HWINIT_RSSI_ADJ_NB);
#endif

    /* Prevent SEQ_MGR to automatically restart RX in case of filterfail interrupt */
    ZLL->SEQ_CTRL_STS |= ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE(0x1U);

#if defined(gPhyUseExternalCoexistence_d) && (gPhyUseExternalCoexistence_d == 1)
#if 0
    PORT_SetPinMux(PORTA, 18U, kPORT_MuxAlt6); /* RF_GPO 0 */
    PORT_SetPinMux(PORTA, 19U, kPORT_MuxAlt6); /* RF_GPO 1 */
    PORT_SetPinMux(PORTA, 20U, kPORT_MuxAlt6); /* RF_GPO 2 */
    PORT_SetPinMux(PORTA, 21U, kPORT_MuxAlt6); /* RF_GPO 3 */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt4); /* RF_GPO 4 */
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt4); /* RF_GPO 5 */
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt4); /* RF_GPO 6 */
#else
    // ON FPGA : RF_GPO[7:0] are directly connected to IO's (No Muxing)
    // TODO: update the pinmux once silicon is out
#endif

#if (HWINIT_USE_RFMC_COEX == 1)
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt5); /* RF_NOT_ALLOWED instead of RF_GPO 7 */

    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_EN(0x1U);

#if (HWINIT_BCA_BYPASS_SUPPORT == 1)
    /* Need to enable the special treatment for 15.4 grant signal if COEX_REQ_DELAY_EN = 1 in 15.4 PHY
     * ZLL COEX_CTRL Register, so that 15.4 NB gets grant in delay mode */
    if (ZLL->COEX_CTRL & ZLL_COEX_CTRL_COEX_REQ_DELAY_EN_MASK)
    {
        *(volatile uint32_t *)BCA_BYPASS_CONTROL_STATUS_REG |= BCA_BYPASS_NBGRANT_SPECIAL_EN;
    }
#endif

    // 0b1..RF_ACTIVE will deassert when TSM is idle
    RFMC->RF2P4GHZ_COEXT    |= RFMC_RF2P4GHZ_COEXT_RFACT_IDIS(0x1U);
    // 01b - RF_ACTIVE is driven by the TSM/LL
    RFMC->RF2P4GHZ_COEXT    |= RFMC_RF2P4GHZ_COEXT_RFACT_SRC(0x1U);
    // 101b - RF_NOT_ALLOWED input pin uses PTD6 (On FPGA : J14 header-pin 19)
    RFMC->RF2P4GHZ_COEXT    |= RFMC_RF2P4GHZ_COEXT_RFNA_IBE(0x5U);
    // 001b - RF_GPO[7:0] = {fem_ctrl[3:0], coext[3:0]}
    RFMC->RF2P4GHZ_COEXT    |= RFMC_RF2P4GHZ_COEXT_RFGPO_SRC(0x1U);
    // RF_GPO Output Buffer Enable
    RFMC->RF2P4GHZ_COEXT    |= RFMC_RF2P4GHZ_COEXT_RFGPO_OBE(0x1U);
    // RF_NOT_ALLOWED connected to ZLL
    RADIO_CTRL->COEX_CTRL   |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_EN(0x4U);
#else
    // Enable 15.4 Co-Existence operation with BT/BLE/Other Wireless Technologies
    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_EN(0x1U);
    // Disable Coexistence Timeout Interrupt generation is disabled,
    // But a COEX_TIMEOUT_IRQ flag can be set
    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_TIMEOUT_MSK(0x1U);
    // RF_NOT_ALLOWED connected to ZLL
    RADIO_CTRL->COEX_CTRL   |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_EN(0x4U);
    // Set COEX_TIMEOUT Value in steps of 32usec. [Timeout value=187, [187*32 = 6000us=6ms]]
    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_TIMEOUT(187);
    // Set COEX_REQ_DELAY_EN for arb_request is delayed until preamble is detected during R sequence.
    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_REQ_DELAY_EN(HWINIT_COEX_REQ_DELAY_EN_VALUE);
#if (HWINIT_BCA_BYPASS_SUPPORT == 1)
    /* Need to enable the special treatment for 15.4 grant signal if COEX_REQ_DELAY_EN = 1 in 15.4 PHY
     * ZLL COEX_CTRL Register, so that 15.4 NB gets grant in delay mode */
    if (ZLL->COEX_CTRL & ZLL_COEX_CTRL_COEX_REQ_DELAY_EN_MASK)
    {
        *(volatile uint32_t *)BCA_BYPASS_CONTROL_STATUS_REG |= BCA_BYPASS_NBGRANT_SPECIAL_EN;
    }
#endif
    // Set COEX_REQ_ON_PD for arb_request is delayed until preamble is detected during R sequence.
    ZLL->COEX_CTRL          |= ZLL_COEX_CTRL_COEX_REQ_ON_PD(1);
    // COEX_PRIORITY Value Mappings:= 0x01[High], 0x3[Medium High], 0x2[Medium Low], 0x0[Low]
    // Set PRIORITY_T
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_T(0x1);
    // Set PRIORITY_R_PRE
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_R_PRE(0x0);
    // Set PRIORITY_R_PKT
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_R_PKT(0x1);
    // Set PRIORITY_TACK
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_TACK(0x1);
    // Set PRIORITY_CCA
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_CCA(0x1);
    // Set PRIORITY_CCCA
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_CCCA(0x0);
    // Set PRIORITY_CTX
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_CTX(0x1);
    // Set PRIORITY_RACK_PRE
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_RACK_PRE(0x1);
    // Set PRIORITY_RACK_PKT
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_RACK_PKT(0x1);
    // Set PRIORITY_OVRD
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_OVRD(0x1);
    // Set PRIORITY_OVRD_EN
    ZLL->COEX_PRIORITY      |= ZLL_COEX_PRIORITY_PRIORITY_OVRD_EN(0x0);
#endif
#endif /* gPhyUseExternalCoexistence_d */

    /* Configure DTEST signals for debug */
#if (defined(HWINIT_DEBUG_DTEST) && (HWINIT_DEBUG_DTEST == 1L))
    dtest_init(DTEST0 | DTEST1 | DTEST2 | DTEST3 | DTEST4 | DTEST5 | DTEST6 | DTEST7 | DTEST8 | DTEST9 | DTEST10 | DTEST11 | DTEST12 | DTEST13 );
    dtest_select(0x00); /* Place selftest waveform onto DTEST ports */
#endif

    /* Disable all timers, enable AUTOACK, mask all interrupts */
    ZLL->PHY_CTRL = (gCcaCCA_MODE1_c << ZLL_PHY_CTRL_CCATYPE_SHIFT) |
#if (HWINIT_MASK_TSM_ZLL == 1)
                    ZLL_PHY_CTRL_TSM_MSK_MASK                       |
#endif
                    ZLL_PHY_CTRL_WAKE_MSK_MASK                      |
                    ZLL_PHY_CTRL_CRC_MSK_MASK                       |
                    ZLL_PHY_CTRL_PLL_UNLOCK_MSK_MASK                |
                    ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK                |
                    ZLL_PHY_CTRL_RX_WMRK_MSK_MASK                   |
                    ZLL_PHY_CTRL_CCAMSK_MASK                        |
                    ZLL_PHY_CTRL_RXMSK_MASK                         |
                    ZLL_PHY_CTRL_TXMSK_MASK                         |
                    ZLL_PHY_CTRL_SEQMSK_MASK                        |
                    ZLL_PHY_CTRL_AUTOACK_MASK                       |
#if !defined(gPhyUseExternalCoexistence_d) || (gPhyUseExternalCoexistence_d == 0) || (ARB_GRANT_DEASSERTION_SUPPORT == 1)
                    ZLL_PHY_CTRL_ARB_GRANT_DEASSERTION_MSK_MASK     |
#endif
                    ZLL_PHY_CTRL_TRCV_MSK_MASK;

    /* Clear all PP IRQ bits to avoid unexpected interrupts immediately after init
       disable all timer interrupts */
    ZLL->IRQSTS = ZLL->IRQSTS;

    /* Source Address Management. Use SAM tables */
    ZLL->SAM_CTRL = 0;

#ifdef gPhyUseHwSAMTable
    ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAA0_START(gPhyHwSAMTableSize_d / CTX_NO);
    ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAP1_START(gPhyHwSAMTableSize_d / CTX_NO);
    ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAA1_START(gPhyHwSAMTableSize_d);

#if defined(MCXW72BD_cm33_core0_SERIES)
    ZLL->SAM_CTRL |= ZLL_SAM_CTRL_ENABLE_FV1_DATA_PKT_IND(1);
#endif

    PhyPpSetSAMState(0, TRUE);

#ifdef CTX_SCHED
    PhyPpSetSAMState(1, TRUE);
#endif

    /* Clear HW SAM TABLE (Indirect queue && PHY neighbour table) */
    ZLL->SAM_TABLE |= ZLL_SAM_TABLE_INVALIDATE_ALL_MASK;
#endif

    /* Frame Filtering */
    ZLL->RX_FRAME_FILTER = mDefaultRxFiltering;
    ZLL->LENIENCY_MSB = ZLL_LENIENCY_MSB_LENIENCY_MSB(gPhyLeniency_c);

    /* Set prescaller to obtain 1 symbol (16us) timebase */
    ZLL->TMR_PRESCALE = 0x05;

    /* Set CCA threshold to -75 dBm */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_CCA1_THRESH(-75);

    /* Adjust ACK delay to fulfill the 802.15.4 turnaround requirements */
#if (HWINIT_TXDELAY_VALUE == 0) && (HWINIT_RXDELAY_VALUE == 0)
    ZLL->ACKDELAY &= ~ZLL_ACKDELAY_ACKDELAY_MASK;
    ZLL->ACKDELAY |= ZLL_ACKDELAY_ACKDELAY(HWINIT_ACKDELAY_VALUE);
#else
    ZLL->ACKDELAY &= ~( ZLL_ACKDELAY_ACKDELAY_MASK |
                        ZLL_ACKDELAY_TXDELAY_MASK |
                        ZLL_ACKDELAY_RXDELAY_MASK );

    ZLL->ACKDELAY |= ZLL_ACKDELAY_ACKDELAY(HWINIT_ACKDELAY_VALUE);
    ZLL->ACKDELAY |= ZLL_ACKDELAY_TXDELAY(HWINIT_TXDELAY_VALUE);
    ZLL->ACKDELAY |= ZLL_ACKDELAY_RXDELAY(HWINIT_RXDELAY_VALUE);
#endif

#if (HWINIT_RSSI_STARTS_AT_PREAMBLE == 1)
    /* Configure corectly LQI reporting by WH -> RSSI calculation starts when preamble is detected */
    XCVR_RX_DIG->RSSI_GLOBAL_CTRL &= ~XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_PA_AA_MATCH_SEL_MASK;
#endif

    /* Adjust LQI compensation */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP(gPhyLqiOffsetCompValue_d);

    /* Adjust RSSI compensation */
    //XCVR_RX_DIG->RSSI_CTRL_0 &= ~XCVR_RX_DIG_RSSI_CTRL_0_RSSI_ADJ_MASK;
    //XCVR_RX_DIG->RSSI_CTRL_0 |= XCVR_RX_DIG_RSSI_CTRL_0_RSSI_ADJ(0xD8);

    /* Enable the RxWatermark IRQ and FilterFail IRQ */
    ZLL->PHY_CTRL &= ~(ZLL_PHY_CTRL_RX_WMRK_MSK_MASK | ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK);

    /* Set default Rx watermark level */
    ZLL->RX_WTR_MARK = RX_WTMRK_START;

    /* Enable Enhanced ACK. Disable RECYC, EMPTY_ACK, ACK_ABORT IRQs */
    ZLL->ENHACK_CTRL0 = ZLL_ENHACK_CTRL0_RECYC_IRQ_MASK | ZLL_ENHACK_CTRL0_RECYC_MSK_MASK |
                        ZLL_ENHACK_CTRL0_EMPTY_ACK_IRQ_MASK | ZLL_ENHACK_CTRL0_EMPTY_ACK_MSK_MASK |
                        ZLL_ENHACK_CTRL0_ACK_ABORT_IRQ_MASK | ZLL_ENHACK_CTRL0_ACK_ABORT_MSK_MASK |
                        ZLL_ENHACK_CTRL0_EMPTY_SRC_ADDR_MODE(2) |
                        ZLL_ENHACK_CTRL0_ENHACK_EN_MASK;

#if (WEIGHT_IN_LQI_CALCULATION == 1)
    /* Get NB RSSI control values */
    XCVR_RX_DIG->NB_RSSI_CTRL1 = XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_WEIGHT(0x4) |\
                                 XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_SNR_WEIGHT(0x5) |\
                                 XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_BIAS(0x6) |\
                                 XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_SENS_ADJ(0x0);

    nbRssiCtrlReg.wt_snr = (XCVR_RX_DIG->NB_RSSI_CTRL1 & XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_SNR_WEIGHT_MASK) \
                             >> XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_SNR_WEIGHT_SHIFT;
    nbRssiCtrlReg.wt_rssi = (XCVR_RX_DIG->NB_RSSI_CTRL1 & XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_WEIGHT_MASK) \
                              >> XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_WEIGHT_SHIFT;
    nbRssiCtrlReg.lqi_bias = (XCVR_RX_DIG->NB_RSSI_CTRL1 & XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_BIAS_MASK) \
                               >> XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_BIAS_SHIFT;
    nbRssiCtrlReg.lqi_rssi_sens = (XCVR_RX_DIG->NB_RSSI_CTRL1 & XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_SENS_ADJ_MASK) \
                                    >> XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_SENS_ADJ_SHIFT;
#endif

    /* Set default channels */
    PhyPlmeSetCurrentChannelRequest(0x0B, 0); /* 2405 MHz */
    PhyPlmeSetCurrentChannelRequest(0x0B, 1); /* 2405 MHz */

    /* Set the default power level */
    PhyPlmeSetPwrLevelRequest(gPhyDefaultTxPowerLevel_d);

    /* Save TX/RX WU/WD times for quick access - devide the reg value by 16 to get symbol time (16us)*/
    gPhyTxWuTimeSym = ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT) >> 4;
    gPhyTxWdTimeSym = ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_SHIFT) >> 4;
    gPhyTxWdTimeSym -= gPhyTxWuTimeSym;

    gPhyRxWuTimeSym = ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT) >> 4;
    gPhyRxWdTimeSym = ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_SHIFT) >> 4;
    gPhyRxWdTimeSym -= gPhyRxWuTimeSym;

#if (HWINIT_FRONT_END_SWITCHING_SUPPORT == 1)
    /* SET FESW Mode - Front End Switching Control Mode & Delay - These settings used only for Single Antenna & NA for Dual Ant cases */
    FFU_FESW->MODE = FFU_FESW_MODE_FESW_MODE(0x1) | FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY(0);
    FFU_FESW->DELAY = FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE(0x0C) |
                      FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE(0x40)  |
                      FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE(0x01) |
                      FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE(0x80);
#endif

    /* Enable Phy */
    PHY_Enable();
}

/*! *********************************************************************************
* \brief  Aborts the current sequence and force the radio to IDLE
*
********************************************************************************** */
void PhyAbort(void)
{
    OSA_InterruptDisable();

    /* Disable all 15.4 IRQ sources before starting abort sequence.
     * This is a software workaround to an hardware issue on KW45 A0.
     * The issue happens when we perform a SW abort during a sequence and a RF_NOT_ALLOWED
     * signal is issued by an external device.
     * this leads to ARB_GRANT_DEASSERTION_IRQ being raised before sequence is in idle state
     * To workaround this, we disable all IRQs sources, we wait the sequence to be idle, and then
     * we release the interrupts. This makes sure the sequence is IDLE before processing interrupts */
    ProtectFromXcvrInterrupt();

    /* Mask SEQ interrupt */
    ZLL->PHY_CTRL |= ZLL_PHY_CTRL_SEQMSK_MASK;

    /* Disable timer trigger (for scheduled XCVSEQ) */
    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMRTRIGEN_MASK)
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_TMRTRIGEN_MASK;
        /* give the FSM enough time to start if it was triggered */
        while ((XCVR_MISC->XCVR_CTRL & XCVR_MISC_XCVR_STATUS_TSM_COUNT_MASK) == 0)
        {
        }
    }

    /* If XCVR is not idle, abort current SEQ */
    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK)
    {
        /*
         * When aborting the RX sequence wait for the RW_WU to be finished.
         * On RW610 when setting the board into IDLE mode during the RW_WU state
         * the device blocks.
         */
        while ((ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK) == g_ZSM_RX_WU)
        {
        }

        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_XCVSEQ_MASK;

        /* wait for Sequence Idle (if not already) */
        while (ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK)
        {
        }
    }

#if gMWS_UseCoexistence_d
    MWS_CoexistenceReleaseAccess();
#endif

    /* Stop timers */
    ZLL->PHY_CTRL &= ~(ZLL_PHY_CTRL_TMR2CMP_EN_MASK |
                       ZLL_PHY_CTRL_TMR3CMP_EN_MASK |
                       ZLL_PHY_CTRL_TC3TMOUT_MASK );
    /* clear all PP IRQ bits to avoid unexpected interrupts( do not change TMR1 and TMR4 IRQ status )
     * also avoid clearing ARB_GRANT_DEASSERTION_IRQ status in case external coexistence is used (gPhyUseExternalCoexistence_d) */
    ZLL->IRQSTS &= ~(ZLL_IRQSTS_TMR1IRQ_MASK | ZLL_IRQSTS_TMR4IRQ_MASK | ZLL_IRQSTS_ARB_GRANT_DEASSERTION_IRQ_MASK);

    ZLL->RX_WTR_MARK = RX_WTMRK_START;

    PHY_allow_sleep();

    UnprotectFromXcvrInterrupt();
    OSA_InterruptEnable();
}

bool_t PHY_graceful_idle()
{
    /* When tx op is pending and seq end IRQ is not fired yet,
       but the SEQ_state allows graceful idle (after rx finished),
       current rx is discarded */
    bool_t status = FALSE;
    uint8_t state, seq_state;

    OSA_InterruptDisable();

    state = PhyPpGetState();
    seq_state = ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK;

    if (state == gIdle_c)
    {
        status = TRUE;
    }
    else if (state == gRX_c)
    {
        /* ZSM STATE
           SEQ_IDLE  0x00
           RX_WU     0x10
           RX_PRE    0x14
           RX_PKT    0x15
           RX2ACK    0x16
           RX_CYC    0x17
           RX_PAN1   0x1D
           RX_PAN2   0x1E
           RX_CCA1   0x18
           CCA2_WAIT 0x19
           RX_CCA2   0x1A
           RX_CCCA   0x1C
           RX2TX     0x1B
           TX_WU     0x01
           TX_PKT    0x05
           TX_ACK    0x06
           TX_SLOT   0x07
           TX_WD     0x08
           TX_PA_OFF 0x09
           XTRA_CLK1 0x0A
           TX2RX     0x0B
           TSM_WD    0x1F
         */
        if ((seq_state == g_ZSM_SEQ_IDLE) ||
            ((seq_state > g_ZSM_TX_ACK) && (seq_state != g_ZSM_RX_PKT) && (seq_state != g_ZSM_RX2ACK)))
        {
            /* idle or not trx. Do PHY abort */
            PhyAbort();

            status = TRUE;
        }
    }

    OSA_InterruptEnable();

    return status;
}

/*! *********************************************************************************
* \brief  Get the state of the ZLL
*
* \return  uint8_t state
*
********************************************************************************** */
uint8_t PhyPpGetState(void)
{
    return (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK) >> ZLL_PHY_CTRL_XCVSEQ_SHIFT;
}

/*! *********************************************************************************
* \brief  Set the value of the MAC PanId
*
* \param[in]  pPanId
* \param[in]  pan
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPpSetPanId(uint8_t *pPanId, uint8_t pan)
{
    uint16_t value;

#ifdef PHY_PARAMETERS_VALIDATION
    if (NULL == pPanId)
    {
        return gPhyInvalidParameter_c;
    }
#endif /* PHY_PARAMETERS_VALIDATION */

    /* Avoid unaligned memory access issues */
    memcpy(&value, pPanId, sizeof(value));

    if (0 == pan)
    {
        ZLL->MACSHORTADDRS0 &= ~ZLL_MACSHORTADDRS0_MACPANID0_MASK;
        ZLL->MACSHORTADDRS0 |= ZLL_MACSHORTADDRS0_MACPANID0(value);
    }
    else
    {
        ZLL->MACSHORTADDRS1 &= ~ZLL_MACSHORTADDRS1_MACPANID1_MASK;
        ZLL->MACSHORTADDRS1 |= ZLL_MACSHORTADDRS1_MACPANID1(value);
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Set the value of the MAC Short Address
*
* \param[in]  pShortAddr
* \param[in]  pan
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPpSetShortAddr(uint8_t *pShortAddr, uint8_t pan)
{
    uint16_t value;

#ifdef PHY_PARAMETERS_VALIDATION
    if (NULL == pShortAddr)
    {
        return gPhyInvalidParameter_c;
    }
#endif /* PHY_PARAMETERS_VALIDATION */

    /* Avoid unaligned memory access issues */
    memcpy(&value, pShortAddr, sizeof(value));

    if (pan == 0)
    {
        ZLL->MACSHORTADDRS0 &= ~ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK;
        ZLL->MACSHORTADDRS0 |= ZLL_MACSHORTADDRS0_MACSHORTADDRS0(value);
    }
    else
    {
        ZLL->MACSHORTADDRS1 &= ~ZLL_MACSHORTADDRS1_MACSHORTADDRS1_MASK;
        ZLL->MACSHORTADDRS1 |= ZLL_MACSHORTADDRS1_MACSHORTADDRS1(value);
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Set the value of the MAC extended address
*
* \param[in]  pLongAddr
* \param[in]  pan
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPpSetLongAddr(uint8_t *pLongAddr, uint8_t pan)
{
    uint32_t addrLo;
    uint32_t addrHi;

#ifdef PHY_PARAMETERS_VALIDATION
    if (NULL == pLongAddr)
    {
        return gPhyInvalidParameter_c;
    }
#endif /* PHY_PARAMETERS_VALIDATION */

    /* Avoid unaligned memory access issues */
    memcpy(&addrLo, pLongAddr, sizeof(addrLo));

    pLongAddr += sizeof(addrLo);
    memcpy(&addrHi, pLongAddr, sizeof(addrHi));

    if (0 == pan)
    {
        ZLL->MACLONGADDRS0_LSB = addrLo;
        ZLL->MACLONGADDRS0_MSB = addrHi;
    }
    else
    {
        ZLL->MACLONGADDRS1_LSB = addrLo;
        ZLL->MACLONGADDRS1_MSB = addrHi;
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
 * \brief Get the device's IEEE Address
 *
 * \param
 *
 * \return status
 *
 ********************************************************************************** */
phyStatus_t PhyPpGetLongAddr(uint8_t *pLongAddr, uint8_t pan)
{
    uint32_t reg1, reg2;

    if (pan == 0)
    {
        reg1 = ZLL->MACLONGADDRS0_MSB;
        reg2 = ZLL->MACLONGADDRS0_LSB;
    }
    else
    {
        reg1 = ZLL->MACLONGADDRS1_MSB;
        reg2 = ZLL->MACLONGADDRS1_LSB;
    }

    pLongAddr[0] = (reg1 >> 24) & 0xff;
    pLongAddr[1] = (reg1 >> 16) & 0xff;
    pLongAddr[2] = (reg1 >> 8)  & 0xff;
    pLongAddr[3] = (reg1 >> 0)  & 0xff;

    pLongAddr[4] = (reg2 >> 24) & 0xff;
    pLongAddr[5] = (reg2 >> 16) & 0xff;
    pLongAddr[6] = (reg2 >> 8)  & 0xff;
    pLongAddr[7] = (reg2 >> 0)  & 0xff;

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Set the MAC PanCoordinator role
*
* \param[in]  macRole
* \param[in]  pan
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPpSetMacRole(bool_t macRole, uint8_t pan)
{
    uint8_t panCoord;

    if (gMacRole_PanCoord_c == macRole)
    {
        panCoord = 1;
    }
    else
    {
        panCoord = 0;
    }

    if (0 == pan)
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_PANCORDNTR0_MASK;
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_PANCORDNTR0(panCoord);
    }
    else
    {
        ZLL->DUAL_PAN_CTRL &= ~ZLL_DUAL_PAN_CTRL_PANCORDNTR1_MASK;
        ZLL->DUAL_PAN_CTRL |= ZLL_DUAL_PAN_CTRL_PANCORDNTR1(panCoord);
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Set the PHY in Promiscuous mode
*
* \param[in]  mode
*
********************************************************************************** */
void PhyPpSetPromiscuous(bool_t mode)
{
    /* This doesn't work with dual PAN */
    if (mode)
    {
        ZLL->PHY_CTRL |= ZLL_PHY_CTRL_PROMISCUOUS_MASK;
        /* FRM_VER[11:8] = b1111. Any Frame version/type accepted */
        ZLL->RX_FRAME_FILTER |= (ZLL_RX_FRAME_FILTER_FRM_VER_FILTER_MASK  |
                                 ZLL_RX_FRAME_FILTER_EXTENDED_FT_MASK     |
                                 ZLL_RX_FRAME_FILTER_MULTIPURPOSE_FT_MASK |
                                 ZLL_RX_FRAME_FILTER_LLDN_FT_MASK         |
                                 ZLL_RX_FRAME_FILTER_CMD_FT_MASK          |
                                 ZLL_RX_FRAME_FILTER_DATA_FT_MASK         |
                                 ZLL_RX_FRAME_FILTER_BEACON_FT_MASK       |
                                 ZLL_RX_FRAME_FILTER_ACK_FT_MASK          |
                                 ZLL_RX_FRAME_FILTER_NS_FT_MASK);
    }
    else
    {
        ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_PROMISCUOUS_MASK;
        /* FRM_VER[11:8] = b0011. Accept FrameVersion 0 and 1 packets, reject all others */
        /* Beacon, Data and MAC command frame types accepted */
        ZLL->RX_FRAME_FILTER = mDefaultRxFiltering;
    }
}

/*! *********************************************************************************
* \brief  Set the PHY in ActivePromiscuous mode
*
* \param[in]  state
*
********************************************************************************** */
void PhySetActivePromiscuous(bool_t state)
{
    /* This doesn't work with dual PAN */
    if (state)
    {
        if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_PROMISCUOUS_MASK)
        {
            /* Disable Promiscuous mode */
            ZLL->PHY_CTRL &= ~ZLL_PHY_CTRL_PROMISCUOUS_MASK;
            ZLL->RX_FRAME_FILTER |= ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK;
        }
    }
    else
    {
        if (ZLL->RX_FRAME_FILTER & ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK)
        {
            ZLL->RX_FRAME_FILTER &= ~ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK;
            /* Enable Promiscuous mode */
            /* Doesn't look right */
            ZLL->PHY_CTRL |= ZLL_PHY_CTRL_PROMISCUOUS_MASK;
        }
    }
}

/*! *********************************************************************************
* \brief  Get the state of the ActivePromiscuous mode
*
* \return  bool_t state
*
********************************************************************************** */
bool_t PhyGetActivePromiscuous(void)
{
    return !!(ZLL->RX_FRAME_FILTER & ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK);
}

/*! *********************************************************************************
* \brief  Set the state of the SAM HW module
*
* \param[in]  state
*
********************************************************************************** */
void PhyPpSetSAMState(instanceId_t instanceId, bool_t state)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);

    if (ctx->id)
    {
        ZLL->SAM_CTRL &= ~ZLL_SAM_CTRL_SAP1_EN_MASK;
        ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAP1_EN(state);
    }
    else
    {
        ZLL->SAM_CTRL &= ~ZLL_SAM_CTRL_SAP0_EN_MASK;
        ZLL->SAM_CTRL |= ZLL_SAM_CTRL_SAP0_EN(state);
    }
}

/*! *********************************************************************************
* \brief  This function adds an 802.15.4 device to the SAP partition of the SAP table.
*         If a polling device is not in the SAP partition, the ACK will have FP=1
*
* \param[in]  pAddr     Pointer to an 802.15.4 address
* \param[in]  addrMode  The 802.15.4 addressing mode
* \param[in]  PanId     The 802.15.2 PAN Id
*
********************************************************************************** */
uint8_t PhyAddToSapTable(instanceId_t instanceId, uint8_t *pAddr, uint8_t addrMode, uint16_t PanId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t index;
    uint16_t checksum = PhyGetChecksum(pAddr, addrMode, PanId);
    uint32_t min = ctx->id * (gPhySAPSize_d / CTX_NO);
    uint32_t max = min + (gPhySAPSize_d / CTX_NO);

    if (PhyGetIndexOf(ctx, checksum) != INV_IDX)
    {
        /* Device is already in the table */
        return 0;
    }

#ifdef gPhyUseHwSAMTable
    uint32_t phyReg;
    /* Find first free index */
    phyReg = ZLL->SAM_TABLE;
    phyReg &= ~(ZLL_SAM_TABLE_SAM_INDEX_WR_MASK  |
                ZLL_SAM_TABLE_SAM_INDEX_INV_MASK |
                ZLL_SAM_TABLE_SAM_INDEX_EN_MASK  |
                ZLL_SAM_TABLE_FIND_FREE_IDX_MASK |
                ZLL_SAM_TABLE_INVALIDATE_ALL_MASK );

    ZLL->SAM_TABLE = phyReg | ZLL_SAM_TABLE_FIND_FREE_IDX_MASK;

    while (ZLL->SAM_TABLE & ZLL_SAM_TABLE_SAM_BUSY_MASK)
    {
    }

    if (ctx->id)
    {
        index = (ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_MASK) >> ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_SHIFT;
    }
    else
    {
        index = (ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_MASK) >> ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_SHIFT;
    }
#else
    /* Search empty index */
    index = sw_sam_search_idx(&sw_sap, min, max, 0xFFFF, TRUE);
#endif

    if ((index != INV_IDX) && (index >= min) && (index < max))
    {
        PhyPp_IndirectQueueInsert(index, checksum, ctx->id);
        return 0;
    }
    return 1;
}

/*! *********************************************************************************
* \brief  This function adds an 802.15.4 device to the SAP partition of the SAP table.
*         If a polling device is not in the SAP partition, the ACK will have FP=1
*
* \param[in]  pAddr     Pointer to an 802.15.4 address
* \param[in]  addrMode  The 802.15.4 addressing mode
* \param[in]  PanId     The 802.15.2 PAN Id
*
********************************************************************************** */
uint8_t PhyNbTblAdd(instanceId_t instanceId, uint8_t *pAddr, uint8_t addrMode, uint16_t PanId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t index;
    uint16_t checksum = PhyGetChecksum(pAddr, addrMode, PanId);
    uint32_t min = ctx->id * (gPhySAASize_d / CTX_NO);
    uint32_t max = min + (gPhySAASize_d / CTX_NO);

    if (PhyNbTblIndexOf(ctx, checksum) != INV_IDX)
    {
        /* Device is already in the table */
        return 0;
    }

    /* Search empty index */
    index = sw_sam_search_idx(&sw_saa, min, max, 0xFFFF, TRUE);

    if ((index != INV_IDX) && (index >= min) && (index < max))
    {
        PhyPp_AddNeighbour(index, checksum, ctx->id);
        return 0;
    }
    return 1;
}

/*! *********************************************************************************
* \brief  Add a new element to the PHY indirect queue
*
* \param[in]  index
* \param[in]  checkSum
* \param[in]  instanceId
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPp_IndirectQueueInsert(uint32_t index, uint16_t checkSum, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t min = ctx->id * (gPhySAPSize_d / CTX_NO);
    uint32_t max = min + (gPhySAPSize_d / CTX_NO);
    phyStatus_t status = gPhySuccess_c;

    if ((index < min) || (index >= max))
    {
        status = gPhyInvalidParameter_c;
    }
    else
    {
#ifdef gPhyUseHwSAMTable
        uint32_t temp;
        temp = ZLL->SAM_TABLE;
        temp &= ~(ZLL_SAM_TABLE_SAM_INDEX_MASK | ZLL_SAM_TABLE_SAM_CHECKSUM_MASK);

        temp |= (index << ZLL_SAM_TABLE_SAM_INDEX_SHIFT) |
                ((uint32_t)checkSum << ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT) |
                 ZLL_SAM_TABLE_SAM_INDEX_WR_MASK |
                 ZLL_SAM_TABLE_SAM_INDEX_EN_MASK;
        ZLL->SAM_TABLE = temp;
#else
        sw_sap.table[index] = checkSum;
        SW_SAM_BITMASK_SET(sw_sap.bitmask, index);
#endif
    }

    return status;
}

/*! *********************************************************************************
* \brief  Add a new element to the neighbour table
*
* \param[in]  index
* \param[in]  checkSum
* \param[in]  instanceId
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPp_AddNeighbour(uint32_t index, uint16_t checkSum, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t min = ctx->id * (gPhySAASize_d / CTX_NO);
    uint32_t max = min + (gPhySAASize_d / CTX_NO);
    phyStatus_t status = gPhySuccess_c;

    if ((index < min) || (index >= max))
    {
        status = gPhyInvalidParameter_c;
    }
    else
    {
        sw_saa.table[index] = checkSum;
        SW_SAM_BITMASK_SET(sw_saa.bitmask, index);
    }

    return status;
}

/*! *********************************************************************************
* \brief  Remove an eleent from the PHY indirect queue
*
* \param[in]  index
* \param[in]  instanceId
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPp_RemoveFromIndirect(uint32_t index, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t min = ctx->id * (gPhySAPSize_d / CTX_NO);
    uint32_t max = min + (gPhySAPSize_d / CTX_NO);
    phyStatus_t status = gPhySuccess_c;

    if ((index < min) || (index >= max))
    {
        status = gPhyInvalidParameter_c;
    }
    else
    {
#ifdef gPhyUseHwSAMTable
        uint32_t temp;
        temp = ZLL->SAM_TABLE & ~(ZLL_SAM_TABLE_SAM_CHECKSUM_MASK | ZLL_SAM_TABLE_SAM_INDEX_MASK);
        temp |= ((uint32_t)0xFFFF << ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT) |
                (index << ZLL_SAM_TABLE_SAM_INDEX_SHIFT) |
                ZLL_SAM_TABLE_SAM_INDEX_INV_MASK |
                ZLL_SAM_TABLE_SAM_INDEX_WR_MASK;
        ZLL->SAM_TABLE = temp;
#else
        sw_sap.table[index] = 0xFFFF;
        SW_SAM_BITMASK_CLEAR(sw_sap.bitmask, index);
#endif
    }

    return status;
}

/*! *********************************************************************************
* \brief  Remove neighbour
*
* \param[in]  index
* \param[in]  instanceId
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPp_RemoveNeighbour(uint32_t index, instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t min = ctx->id * (gPhySAASize_d / CTX_NO);
    uint32_t max = min + (gPhySAASize_d / CTX_NO);
    phyStatus_t status = gPhySuccess_c;

    if ((index < min) || (index >= max))
    {
        status = gPhyInvalidParameter_c;
    }
    else
    {
        sw_saa.table[index] = 0xFFFF;
        SW_SAM_BITMASK_CLEAR(sw_saa.bitmask, index);
    }

    return status;
}

/*! *********************************************************************************
* \brief  Clear all neighbours
*
* \param[in]  instanceId
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPp_ClearNeighbourTbl(instanceId_t instanceId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint32_t size = (gPhySAASize_d / CTX_NO);
    uint32_t min = ctx->id * size;
    uint32_t max = min + size;
    uint32_t index = 0;

    // Clear the entries in the neighbor table
    for (index = min; index < max; index ++)
    {
        sw_saa.table[index] = 0xFFFF;
        SW_SAM_BITMASK_CLEAR(sw_saa.bitmask, index);
    }

    return gPhySuccess_c;
}


/*! *********************************************************************************
* \brief  Return TRUE if the received packet is a PollRequest
*
* \return  bool_t
*
********************************************************************************** */
bool_t PhyPpIsPollIndication(void)
{
    return !!(ZLL->IRQSTS & ZLL_IRQSTS_PI_MASK);
}

/*! *********************************************************************************
* \brief  Return the state of the FP bit of the received ACK
*
* \return  bool_t
*
********************************************************************************** */
bool_t PhyPpIsRxAckDataPending(void)
{
    return !!(ZLL->IRQSTS & ZLL_IRQSTS_RX_FRM_PEND_MASK);
}

/*! *********************************************************************************
* \brief  Return TRUE if there is data pending for the Poling Device
*
* \return  bool_t
*
********************************************************************************** */
bool_t PhyPpIsTxAckDataPending(void)
{
    bool_t status;

    if (ZLL->SAM_CTRL & (ZLL_SAM_CTRL_SAP0_EN_MASK | ZLL_SAM_CTRL_SAP1_EN_MASK))
    {
        status = !!(ZLL->IRQSTS & ZLL_IRQSTS_SRCADDR_MASK);
    }
    else
    {
        status = !!(ZLL->SAM_TABLE & ZLL_SAM_TABLE_ACK_FRM_PND_MASK);
    }

    return status;
}

/*! *********************************************************************************
* \brief  Set the state of the FP bit of an outgoing ACK frame
*
* \param[in]  FP  the state of the FramePending bit
*
********************************************************************************** */
void PhyPpSetFpManually(bool_t FP)
{
    /* Disable the Source Address Matching feature and set FP manually */
    ZLL->SAM_TABLE |= ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK;
    ZLL->SAM_TABLE &= ~ZLL_SAM_TABLE_ACK_FRM_PND_MASK;
    ZLL->SAM_TABLE |= ZLL_SAM_TABLE_ACK_FRM_PND(FP);
}

/*! *********************************************************************************
* \brief  Set the value of the CCA threshold
*
* \param[in]  ccaThreshold
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPpSetCcaThreshold(uint8_t ccaThreshold)
{
    /* This doesn't work with dual PAN */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_CCA1_THRESH(ccaThreshold);
    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will set the value for the FAD threshold
*
* \param[in]  FADThreshold   the FAD threshold
*
* \return  phyStatus_t
*
********************************************************************************** */
uint8_t PhyPlmeSetFADThresholdRequest(uint8_t FADThreshold)
{
#if defined(K32W1480_SERIES) || defined(MCXW72BD_cm33_core0_SERIES) || defined(MCXW72BD_cm33_core1_SERIES) || defined(MCXW716A_SERIES) || defined(MCXW716C_SERIES)
    XCVR_ZBDEMOD->FAD_LPPS_THR &= ~XCVR_ZBDEMOD_FAD_LPPS_THR_FAD_THR_MASK;
    XCVR_ZBDEMOD->FAD_LPPS_THR |= XCVR_ZBDEMOD_FAD_LPPS_THR_FAD_THR(FADThreshold);
#endif

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will enable/disable the FAD
*
* \param[in]  state   the state of the FAD
*
* \return  phyStatus_t
*
********************************************************************************** */
uint8_t PhyPlmeSetFADStateRequest(bool_t state)
{
    XCVR_MISC->FAD_CTRL &= ~(XCVR_MISC_FAD_CTRL_FAD_EN_MASK | XCVR_MISC_FAD_CTRL_FAD_NOT_GPIO_MASK);
    XCVR_TSM->TIMING07  &= ~(XCVR_TSM_TIMING07_GPIO2_TRIG_EN_TX_HI_MASK);
    XCVR_TSM->TIMING08  &= ~(XCVR_TSM_TIMING08_GPIO3_TRIG_EN_RX_HI_MASK);

    if (state)
    {
        XCVR_MISC->FAD_CTRL |= XCVR_MISC_FAD_CTRL_FAD_EN(1) | XCVR_MISC_FAD_CTRL_FAD_NOT_GPIO(0x0F);
        XCVR_TSM->TIMING07 |= XCVR_TSM_TIMING07_GPIO2_TRIG_EN_TX_HI(1);
        XCVR_TSM->TIMING07 |= XCVR_TSM_TIMING08_GPIO3_TRIG_EN_RX_HI(1);
    }
    else
    {
        XCVR_MISC->FAD_CTRL |= XCVR_MISC_FAD_CTRL_FAD_EN(0) | XCVR_MISC_FAD_CTRL_FAD_NOT_GPIO(0);
        XCVR_TSM->TIMING07 |= XCVR_TSM_TIMING07_GPIO2_TRIG_EN_TX_HI(0xFF);
        XCVR_TSM->TIMING07 |= XCVR_TSM_TIMING08_GPIO3_TRIG_EN_RX_HI(0xFF);
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will return the CCA Type
*
* \return  phyCCAType_t
*
********************************************************************************** */
phyCCAType_t PhyPlmeGetCCATypeRequest(void)
{
    /* Read in CCA PHY CTRL register, the CCA type */
    return (phyCCAType_t)((ZLL->PHY_CTRL & ZLL_PHY_CTRL_CCATYPE_MASK) >> ZLL_PHY_CTRL_CCATYPE_SHIFT);
}

/*! *********************************************************************************
* \brief  This function will set the CCA3 mode
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetCCA3ModeRequest(phyCCA3Mode_t cca3Mode)
{
    /* Write in CCA LQI CTRL register, the desired type of CCA3 mode */
    /* This doesn't work with dual PAN */
    ZLL->CCA_LQI_CTRL &= ~ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_MASK;
    ZLL->CCA_LQI_CTRL |= ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR((uint8_t)cca3Mode);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will return the CCA3 mode
*
* \return  phyCCA3Mode_t
*
********************************************************************************** */
phyCCA3Mode_t PhyPlmeGetCCA3ModeRequest(void)
{
    /* Read in CCA LQI CTRL register, the CCA3 mode */
    if ((ZLL->CCA_LQI_CTRL & ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_MASK) >>  ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_SHIFT)
        return gPhyCCAMode3_AND_c;
    else
        return gPhyCCAMode3_OR_c;
}

/*! *********************************************************************************
* \brief  This function will return the RSSI level
*
* \param[in]  instanceId
*
* \return  uint8_t
*
********************************************************************************** */
uint8_t PhyPlmeGetRSSILevelRequest(instanceId_t instanceId)
{
    uint8_t rssi, channel;
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);

    if (ctx_is_active(ctx) || ctx_is_paused(ctx))
    {
        rssi = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_RSSI_MASK) >> ZLL_LQI_AND_RSSI_RSSI_SHIFT;
        channel = PhyPlmeGetCurrentChannelRequest(ctx->id);

        if (FE_LNA_ENABLE)
        {
            return (rssi + sChannelRssiOffset[channel - FIRST_CHANNEL_OFFSET] + RSSI_WITH_FELOSS - FE_LNA_GAIN);
        }
        else
        {
            return (rssi + sChannelRssiOffset[channel-FIRST_CHANNEL_OFFSET] + RSSI_WITH_FELOSS);
        }
    }
    else
    {
        /* The RSSI value is invalid due to an inactive context */
        return 127;
    }
}

/*! *********************************************************************************
* \brief  This function will return the promiscuous state
*
* \return  bool_t
*
********************************************************************************** */
bool_t PhyPlmeGetPromiscuousRequest(void)
{
    return (ZLL->PHY_CTRL & ZLL_PHY_CTRL_PROMISCUOUS_MASK) == ZLL_PHY_CTRL_PROMISCUOUS_MASK;
}

/*! *********************************************************************************
* \brief  This function controls the antenna selection
*
* \param[in]  state   the state of the ANTX
*
* \return  phyStatus_t
*
********************************************************************************** */
uint8_t PhyPlmeSetANTXStateRequest(bool_t state)
{
    XCVR_MISC->FAD_CTRL &= ~XCVR_MISC_FAD_CTRL_ANTX_MASK;
    XCVR_MISC->FAD_CTRL |= XCVR_MISC_FAD_CTRL_ANTX(state);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief Invert the logic of the ANT pads
*
* \param[in] invAntA - invert the ANT_A pad
* \param[in] invAntB - invert the ANT_A pad
* \param[in] invTx   - invert the ANT_TX pad
* \param[in] invRx   - invert the ANT_RX pad
*
* \return gPhySuccess
*
********************************************************************************** */
uint8_t PhyPlmeSetANTPadInvertedRequest(bool_t invAntA, bool_t invAntB, bool_t invTx, bool_t invRx)
{
    uint32_t settings = 0;

    XCVR_MISC->FAD_CTRL &= ~XCVR_MISC_FAD_CTRL_ANTX_POL_MASK;

    if (invAntA)
    {
        settings |= (1 << 0);
    }
    if (invAntB)
    {
        settings |= (1 << 1);
    }
    if (invTx)
    {
        settings |= (1 << 2);
    }
    if (invRx)
    {
        settings |= (1 << 3);
    }

    XCVR_MISC->FAD_CTRL |= XCVR_MISC_FAD_CTRL_ANTX_POL(settings);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief Enable the ANT pads
*
* \param[in] antAB_on -
* \param[in] rxtxSwitch_on -
*
* \return gPhySuccess
*
********************************************************************************** */
uint8_t PhyPlmeSetANTPadStateRequest(bool_t antAB_on, bool_t rxtxSwitch_on)
{
    uint32_t settings = 0;

    XCVR_MISC->FAD_CTRL &= ~XCVR_MISC_FAD_CTRL_ANTX_EN_MASK;

    if (antAB_on)
    {
        settings |= (1 << 1);
    }

    if (rxtxSwitch_on)
    {
        settings |= (1 << 0);
    }

    XCVR_MISC->FAD_CTRL |= XCVR_MISC_FAD_CTRL_ANTX_EN(settings);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will retrn the state of the ANTX
*
* \return  uint8_t
*
********************************************************************************** */
uint8_t PhyPlmeGetANTXStateRequest(void)
{
    return (XCVR_MISC->FAD_CTRL & XCVR_MISC_FAD_CTRL_ANTX_MASK) >> XCVR_MISC_FAD_CTRL_ANTX_SHIFT;
}

/*! *********************************************************************************
* \brief  Set the state of the Dual Pan Auto mode
*
* \param[in]  mode TRUE/FALSE
*
********************************************************************************** */
void PhyPpSetDualPanAuto(bool_t mode)
{
    ZLL->DUAL_PAN_CTRL &= ~ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_MASK;
    ZLL->DUAL_PAN_CTRL |= ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO(mode);
}

/*! *********************************************************************************
* \brief  Get the state of the Dual Pan Auto mode
*
* \return  bool_t state
*
********************************************************************************** */
bool_t PhyPpGetDualPanAuto(void)
{
    return !!(ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_MASK);
}

/*! *********************************************************************************
* \brief  Set the dwell for the Dual Pan Auto mode
*
* \param[in]  dwell
*
********************************************************************************** */
void PhyPpSetDualPanDwell(uint8_t dwell)
{
    ZLL->DUAL_PAN_CTRL &= ~ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_MASK;
    ZLL->DUAL_PAN_CTRL |= ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL(dwell);
}

/*! *********************************************************************************
* \brief  Get the dwell for the Dual Pan Auto mode
*
* \return  uint8_t PAN dwell
*
********************************************************************************** */
uint8_t PhyPpGetDualPanDwell(void)
{
    return (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_MASK) >> ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_SHIFT;
}

/*! *********************************************************************************
* \brief  Get the remeining time before a PAN switch occures
*
* \return  uint8_t remaining time
*
********************************************************************************** */
uint8_t PhyPpGetDualPanRemain(void)
{
    return (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_MASK) >> ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_SHIFT;
}

/*! *********************************************************************************
* \brief  Set the current active Nwk
*
* \param[in]  nwk index of the nwk
*
********************************************************************************** */
void PhyPpSetDualPanActiveNwk(uint8_t nwk)
{
    ZLL->DUAL_PAN_CTRL &= ~ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_MASK;
    ZLL->DUAL_PAN_CTRL |= ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK(nwk);
}

/*! *********************************************************************************
* \brief  Return the index of the Acive PAN
*
* \return  uint8_t index
*
********************************************************************************** */
uint8_t PhyPpGetDualPanActiveNwk(void)
{
    return !!(ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_MASK);
}

/*! *********************************************************************************
* \brief  Returns the PAN bitmask for the last Rx packet.
*         A packet can be received on multiple PANs
*
* \return  uint8_t bitmask
*
********************************************************************************** */
uint8_t PhyPpGetPanOfRxPacket(void)
{
    uint8_t PanBitMask = 0;

    if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_MASK)
    {
        if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_MASK)
        {
            PanBitMask |= (1 << 0);
        }

        if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_MASK)
        {
            PanBitMask |= (1 << 1);
        }
    }
    else
    {
        if (ZLL->DUAL_PAN_CTRL & ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_MASK)
        {
            PanBitMask |= (1 << 1);
        }
        else
        {
            PanBitMask |= (1 << 0);
        }
    }

    return PanBitMask;
}

/*! *********************************************************************************
* \brief  This function compute the hash code for an 802.15.4 device
*
* \param[in]  pAddr     Pointer to an 802.15.4 address
* \param[in]  addrMode  The 802.15.4 addressing mode
* \param[in]  PanId     The 802.15.2 PAN Id
*
* \return  hash code
*
********************************************************************************** */
uint16_t PhyGetChecksum(uint8_t *pAddr, uint8_t addrMode, uint16_t PanId)
{
    uint16_t checksum;

    if (!pAddr)
    {
        return 0;
    }

    /* Short address */
    checksum = PanId;
    checksum += *pAddr++;
    checksum += (uint16_t)((uint16_t)(*pAddr++) << 8);

    if (addrMode == 3)
    {
        /* Extended address */
        checksum += *pAddr++;
        checksum += (uint16_t)((uint16_t)(*pAddr++) << 8);
        checksum += *pAddr++;
        checksum += (uint16_t)((uint16_t)(*pAddr++) << 8);
        checksum += *pAddr++;
        checksum += (uint16_t)((uint16_t)(*pAddr++) << 8);
    }

    return checksum;
}

/*! *********************************************************************************
* \brief  This function returns the table index of the specified checksum.
*
* \param[in]  checksum     hash code generated by PhyGetChecksum()
*
* \return  The table index where the checksum was found or
*          -1 if no entry was found with the specified chacksum
*
*
********************************************************************************** */
uint32_t PhyGetIndexOf(Phy_PhyLocalStruct_t *ctx, uint16_t checksum)
{
    uint32_t start = ctx->id * (gPhySAPSize_d / CTX_NO);
    uint32_t stop = start + (gPhySAPSize_d / CTX_NO);
    uint32_t index = INV_IDX;

#ifdef gPhyUseHwSAMTable
    uint32_t phyReg;

    for (uint32_t i = start; i < stop; i++)
    {
        /* Set the index value */
        phyReg = ZLL->SAM_TABLE;
        phyReg &= ~(ZLL_SAM_TABLE_SAM_INDEX_MASK     |
                    ZLL_SAM_TABLE_SAM_INDEX_WR_MASK  |
                    ZLL_SAM_TABLE_SAM_INDEX_INV_MASK |
                    ZLL_SAM_TABLE_SAM_INDEX_EN_MASK  |
                    ZLL_SAM_TABLE_FIND_FREE_IDX_MASK |
                    ZLL_SAM_TABLE_INVALIDATE_ALL_MASK);

        ZLL->SAM_TABLE = phyReg | (i << ZLL_SAM_TABLE_SAM_INDEX_SHIFT);

        /* Read checksum located at the specified index */
        phyReg = ZLL->SAM_TABLE;
        phyReg = (phyReg & ZLL_SAM_TABLE_SAM_CHECKSUM_MASK) >> ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT;

        if (phyReg == checksum)
        {
            index = i;
            break;
        }
    }
#else
    index = sw_sam_search_idx(&sw_sap, start, stop, checksum, FALSE);
#endif

    return index;
}

/*! *********************************************************************************
* \brief  This function returns the table index of the specified checksum.
*
* \param[in]  checksum     hash code generated by PhyGetChecksum()
*
* \return  The table index where the checksum was found or
*          -1 if no entry was found with the specified chacksum
*
*
********************************************************************************** */
uint32_t PhyNbTblIndexOf(Phy_PhyLocalStruct_t *ctx, uint16_t checksum)
{
    uint32_t start = ctx->id * (gPhySAASize_d / CTX_NO);
    uint32_t stop = start + (gPhySAASize_d / CTX_NO);
    uint32_t index = INV_IDX;

    index = sw_sam_search_idx(&sw_saa, start, stop, checksum, FALSE);
    return index;
}

/*! *********************************************************************************
* \brief  This function removes an 802.15.4 device from the SAM table table.
*         This affects both SAP or SAA partitions
*
* \param[in]  pAddr     Pointer to an 802.15.4 address
* \param[in]  addrMode  The 802.15.4 addressing mode
* \param[in]  PanId     The 802.15.2 PAN Id
*
********************************************************************************** */
uint8_t PhyNbTblRemove(instanceId_t instanceId, uint8_t *pAddr, uint8_t addrMode, uint16_t PanId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint16_t checksum;
    uint32_t  index;

    checksum = PhyGetChecksum(pAddr, addrMode, PanId);
    index    = PhyNbTblIndexOf(ctx, checksum);

    if (index != INV_IDX)
    {
        /* Invalidate current index and checksum */
        PhyPp_RemoveNeighbour(index, ctx->id);
        return 0;
    }
    return 1;
}
/*! *********************************************************************************
* \brief  This function removes an 802.15.4 device from the SAM table table.
*         This affects both SAP or SAA partitions
*
* \param[in]  pAddr     Pointer to an 802.15.4 address
* \param[in]  addrMode  The 802.15.4 addressing mode
* \param[in]  PanId     The 802.15.2 PAN Id
*
********************************************************************************** */
uint8_t PhyRemoveFromSamTable(instanceId_t instanceId, uint8_t *pAddr, uint8_t addrMode, uint16_t PanId)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(instanceId);
    uint16_t checksum;
    uint32_t  index;

    checksum = PhyGetChecksum(pAddr, addrMode, PanId);
    index    = PhyGetIndexOf(ctx, checksum);

    if (index != INV_IDX)
    {
        /* Invalidate current index and checksum */
        PhyPp_RemoveFromIndirect(index, ctx->id);
        return 0;
    }
    return 1;
}

/*! *********************************************************************************
* \brief  Change the XCVR DSM duration
*
* \param[in]  duration  the new XCVR sleep duration
*
********************************************************************************** */
void PhyPlmeSetDSMDuration(uint32_t duration)
{
#if 0
    /* Minimum DSM duration */
    mPhyDSMDuration = (RSIM->DSM_WAKEUP & RSIM_DSM_WAKEUP_DSM_POWER_OFFSET_TIME_MASK) >> RSIM_DSM_WAKEUP_DSM_POWER_OFFSET_TIME_SHIFT;
    mPhyDSMDuration += mPhyDSM_GuardTime_d;

    if (duration > mPhyDSMDuration)
    {
        mPhyDSMDuration = duration;
    }
#endif
}

#if 0
/*! *********************************************************************************
* \brief  Get remaining time until the next ZLL event
*
* \return  time in symbols until the next programmed event
*
********************************************************************************** */
uint32_t PhyTime_GetNextEvent(void)
{
    uint32_t currentTime = ZLL->EVENT_TMR >> ZLL_EVENT_TMR_EVENT_TMR_SHIFT;
    uint32_t minTime = 0x00FFFFFF;
    uint32_t t;

    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMR1CMP_EN_MASK)
    {
        t = (ZLL->T1CMP - currentTime) & ZLL_T1CMP_T1CMP_MASK;

        if (t < minTime)
        {
            minTime = t;
        }
    }

    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMR2CMP_EN_MASK)
    {
        t = (ZLL->T2CMP - currentTime) & ZLL_T2CMP_T2CMP_MASK;

        if (t < minTime)
        {
            minTime = t;
        }
    }

    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMR3CMP_EN_MASK)
    {
        t = (ZLL->T3CMP - currentTime) & ZLL_T3CMP_T3CMP_MASK;

        if (t < minTime)
        {
            minTime = t;
        }
    }

    if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_TMR4CMP_EN_MASK)
    {
        t = (ZLL->T4CMP - currentTime) & ZLL_T4CMP_T4CMP_MASK;

        if (t < minTime)
        {
            minTime = t;
        }
    }

    return minTime - 10;
}
#endif

/*! *********************************************************************************
* \brief  Change the XCVR power state
*
* \param[in]  state  the new XCVR power state
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetPwrState( uint8_t state )
{
    static uint8_t mPhyPwrState = gPhyPwrIdle_c;
    phyStatus_t status = gPhySuccess_c;
#if 0
    uint32_t offset;
#endif

    /* Parameter validation */
    if (state > gPhyPwrReset_c)
    {
        status = gPhyInvalidParameter_c;
    }
    /* Check if the new power state = old power state */
    else if (state == mPhyPwrState)
    {
        status = gPhyBusy_c;
    }
    else
    {
#if 0
        // TODO: Update DSM handling using RFMC (RSIM not existing)
        offset = (RSIM->DSM_WAKEUP & RSIM_DSM_WAKEUP_DSM_POWER_OFFSET_TIME_MASK) >> RSIM_DSM_WAKEUP_DSM_POWER_OFFSET_TIME_SHIFT;

        switch (state)
        {
        case gPhyPwrIdle_c:
            /* Check if XCVR is preparing to enter DSM, and wait for confirm. */
            if (RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_DSM_MAN_READY_MASK)
            {
                while (!(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_MAN_DEEP_SLEEP_STATUS_MASK));
            }
            /* Set XCVR in run mode if not allready */
            if (RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_MAN_DEEP_SLEEP_STATUS_MASK)
            {
                /* Force DSM wake-up. The WAKE_IRQ will trigger and update the EVENT_TMR register */
                RSIM->MAN_WAKE = (RSIM->DSM_TIMER + offset + 1);
                while (RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_MAN_DEEP_SLEEP_STATUS_MASK);
                ZLL->DSM_CTRL = 0;
            }
            break;

        case gPhyPwrDSM_c:
            /* Set XCVR in low power mode if not allready */
            if (!(RSIM->DSM_CONTROL & RSIM_DSM_CONTROL_MAN_DEEP_SLEEP_STATUS_MASK))
            {
                /* Convert from symbols to microseconds (<< 4), and then to 32768 (<< 15) ticks. */
                uint32_t dsm_time = ((uint64_t)(PhyTime_GetNextEvent() << 4) << 15) / 1000000;

                if (dsm_time > (offset + mPhyDSM_GuardTime_d))
                {
                    RSIM->MAN_SLEEP = RSIM->DSM_TIMER + mPhyDSM_GuardTime_d;
                    RSIM->MAN_WAKE = RSIM->DSM_TIMER + dsm_time;
                    ZLL->DSM_CTRL = ZLL_DSM_CTRL_ZIGBEE_SLEEP_REQUEST_MASK;
                }
            }
            break;

        default:
            status = gPhyInvalidPrimitive_c;
            /* do not change current state */
            state = mPhyPwrState;
            break;
        }
#endif

        mPhyPwrState = state;
    }

    return status;
}
