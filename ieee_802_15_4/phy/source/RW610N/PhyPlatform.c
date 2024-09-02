/*
 * Copyright 2023-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PhyPlatform.h"
#include "Phy.h"

void PhyPlatformHwInit(void)
{
    /* Following config is used by TX Design testing */
    /* Set FSK modulator and correct data path */
    XCVR_TX_DIG->TXDIG_CTRL &= ~(XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL_MASK |\
                                 XCVR_TX_DIG_TXDIG_CTRL_PFC_EN_MASK |\
                                 XCVR_TX_DIG_TXDIG_CTRL_DATA_STREAM_SEL_MASK);
    XCVR_TX_DIG->TXDIG_CTRL |= XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(0x1U) |\
                               XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(0x1U) |\
                               XCVR_TX_DIG_TXDIG_CTRL_DATA_STREAM_SEL(0x1U);

    /* Configure data padding */
    XCVR_TX_DIG->DATA_PADDING_CTRL &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_CTE_DATA_MASK |\
                                        XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_MASK |\
                                        XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN_MASK);
    XCVR_TX_DIG->DATA_PADDING_CTRL |= XCVR_TX_DIG_DATA_PADDING_CTRL_CTE_DATA(0x1U) |\
                                      XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY(0x1U) |\
                                      XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(0x0U);

    XCVR_TX_DIG->DATA_PADDING_CTRL_1 &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_1_PA_PUP_ADJ_MASK |\
                                          XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK |\
                                          XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY_MASK);
    XCVR_TX_DIG->DATA_PADDING_CTRL_1 |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_PA_PUP_ADJ(0x1U) |\
                                        XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(0x6U) |\
                                        XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY(0x0U);

    /* Configure PowerAmplifier */
    XCVR_TX_DIG->PA_CTRL &= ~(XCVR_TX_DIG_PA_CTRL_TGT_PWR_SRC_MASK |\
                              XCVR_TX_DIG_PA_CTRL_PA_TGT_POWER_MASK |\
                              XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL_MASK);
    XCVR_TX_DIG->PA_CTRL |= XCVR_TX_DIG_PA_CTRL_TGT_PWR_SRC(0x1U) |\
                            XCVR_TX_DIG_PA_CTRL_PA_TGT_POWER(0x3CU) |\
                            XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(0x0U);

    XCVR_TX_DIG->DATARATE_CONFIG_FSK_CTRL &= ~(XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0_MASK |\
                                               XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1_MASK);
    XCVR_TX_DIG->DATARATE_CONFIG_FSK_CTRL |= XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1800U) |\
                                            XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x800U);

    /* Enable NB-RSSI computation during CCA sequence */
    XCVR_RX_DIG->RSSI_GLOBAL_CTRL |= XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CCA1_ED_EN(0x1U);
}

#if (PHY_WLAN_COEX == 1)
void PhyWlanCoex_ChannelUpdate(uint8_t channel)
{
    UINT8 Sharedchannel = 0;
    assert(initialized == TRUE);

    Sharedchannel = coex_wlan_readOTStat_helper(ICC_LINK0,offsetof(coex_OTToWlan_t, OTChanUsed));

    /* Update the channel used in the shared memory (only if it changed) */
    if ((channel != Sharedchannel)&&(Sharedchannel != 0xFF))
    {
        /* Indicate WLAN CPU the channel has been updated */
        coex_ot_set_channel(channel);
    }
}
#endif

