/* Copyright 2023-2024 NXP
 *
 * NXP CONFIDENTIAL
 * The source code contained or described herein and all documents related to
 * the source code ("Materials") are owned by NXP ( NXP ), its
 * suppliers and/or its licensors. Title to the Materials remains with NXP,
 * its suppliers and/or its licensors. The Materials contain
 * trade secrets and proprietary and confidential information of NXP, its
 * suppliers and/or its licensors. The Materials are protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Materials may be
 * used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without NXP's prior
 * express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by NXP in writing.
 *
 * \file Platform specific definitions.
 *
 */

#ifndef __PHY_PLATFORM_RW610N_H__
#define __PHY_PLATFORM_RW610N_H__

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "os_if.h"
#include "hal_pstore_rom.h"
#include "wl_macros.h"
#include "PhyFesw.h"
#include "PhyInterface.h"

#if defined (WLAN_OT_CHANNEL_SHARE) && (WLAN_OT_CHANNEL_SHARE == 1)
#include "coex_inform.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
extern void CPU_CM3_NVIC_ClearPendingISR(uint8 param);

#if defined(FFU_PHY_ONLY_OVER_IMU)
UINT32 FFU_ZIGBEE_INT_IRQHandler(UINT32 vector, UINT32 data);
#else
void FFU_ZIGBEE_INT_IRQHandler(void);
#endif

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */

/** Peripheral FFU base address */
#define FFU_REG_BASE                    (0x4012A000u)

#define gPhyIrqPriority_c     (1 << 5)
#define FFU_ZIGBEE_INT_IRQn   (13)

#define mPhySymTime_d                   (16)
#define mUStoTimeStamp_d(us)            (us/mPhySymTime_d)

#define gPhyIrqNo_d                     ((IRQn_Type)FFU_ZIGBEE_INT_IRQn)

#define MIN_ENERGY_LEVEL                (-94)
#define MAX_ENERGY_LEVEL                (-20)
#define CONVERT_ENERGY_LEVEL(energy)    (3.446 * (energy + 94)) /* multiplying factor = (255-0)/(-20 -(-94)) = 3.446 */

/* Front End loss value used for TX power calculation */
#define FE_LOSS_VALUE                   (int8_t)(_tBtPsAnnex55.ucFELoss)

/* External Front End module Power Amplifier gain enable */
#define FE_POWER_AMPLIFIER_ENABLE       (uint8_t)(_tBlePsAnnex100.eRF_CNTL_TX_Granted)

/* External TX Power Amplifier gain for a Front End module */
#define FE_POWER_AMPLIFIER_GAIN         (int8_t)(_tBlePsAnnex100.eRF_CNTL_TX_Gain)

/* External Front End module LNA (Low Noise Attenuator) enable */
#define FE_LNA_ENABLE                   (uint8_t)(_tBlePsAnnex100.eRF_CNTL_RX_Granted)

/* External RX gain for a LNA (Low Noise Attenuator) Front End module */
#define FE_LNA_GAIN                     (int8_t)(_tBlePsAnnex100.eRF_CNTL_RX_Gain)

/*
 * FEATURES SUPPORT
 */
#define AUTO_ACK_DISABLE_SUPPORT        (1)
#define CCA_MODE_SELECT_SUPPORT         (1)
#define ARB_GRANT_DEASSERTION_SUPPORT   (1)
#define TX_POWER_LIMIT_FEATURE          (1)
#define WLAN_COEX_TYPE_DEFINITION       (0)
#define LAST_RSSI_USING_LQI             (0) // calculate RSSI based on LQI value
#define WEIGHT_IN_LQI_CALCULATION       (1)
#define XCVR_MISC_SUPPORT               (0)
#define RSSI_WITH_FELOSS                (FE_LOSS_VALUE / 2)

#if defined (WLAN_OT_CHANNEL_SHARE) && (WLAN_OT_CHANNEL_SHARE == 1)
#define PHY_WLAN_COEX                   (1)
#endif

#define HWINIT_USE_CCA1_FROM_RX_DIGITAL         (1)
#define HWINIT_USE_CCA2_DECOUPLED_FROM_DEMOD    (1)
#define HWINIT_SET_RSSI_ADJUSTEMENT             (1)
#define HWINIT_RSSI_ADJ_NB                      (0xED)
#define HWINIT_MASK_TSM_ZLL                     (1)
#define HWINIT_ACKDELAY_VALUE                   (0x04U)
#define HWINIT_TXDELAY_VALUE                    (0x01U)
#define HWINIT_RXDELAY_VALUE                    (0x24U)
#define HWINIT_CONFIGURE_LEGACY_DATA_RATE       (1)
#define HWINIT_CONFIGURE_CCA2_THRESHOLD         (1)
#define HWINIT_USE_RFMC_COEX                    (0)
#define HWINIT_CCA1_FROM_RX_DIG()               XCVR_ZBDEMOD_REGS->CCA_LQI_SRC |= XCVR_ZBDEM_CCA_LQI_SRC_LQI_FROM_RX_DIG(0x1U)
#define HWINIT_FRONT_END_SWITCHING_SUPPORT      (1)
#define HWINIT_BCA_BYPASS_SUPPORT               (0)

/*
 * These functions are used to only mask/unmask 8bits of ZLL->PHY_CTRL to avoid side effects.
 * The 32bits rewriting was causing HW non expecting behavior because it was rewriting
 * status bit while HW was on idle mode.
 * Please refer to jira GLC8993-509
 * bitshift is the number of the bit that mask will modify,
 * for example bitshift = 31, mask = 0x80000000 -> bitshift/8 = 3 (the mask is on byte number 3)
 */
static uint32_t ZLL_PHY_CTRL_ADDR =     (uint32_t)&(ZLL->PHY_CTRL); // Address of PHY_CTRL register
#define FIELD_MASK(field)               (field ## _MASK)
#define FIELD_BITSHIFT(field)           (field ## _SHIFT) / 8
#define SHIFT_MASK(field)               FIELD_MASK(field) >> (FIELD_BITSHIFT(field)*8)
#define SET_PHYCTRL_FIELD(field)        WL_REGS8_SETBITS(ZLL_PHY_CTRL_ADDR+FIELD_BITSHIFT(field), SHIFT_MASK(field))
#define CLR_PHYCTRL_FIELD(field)        WL_REGS8_CLRBITS(ZLL_PHY_CTRL_ADDR+FIELD_BITSHIFT(field), SHIFT_MASK(field))

/*
 * Phy IRQ handling functions needed for easier integration in common code.
*/
#define PHY_PhyIrqCreate()              os_InterruptCreate(gPhyIrqNo_d, FFU_ZIGBEE_INT_IRQHandler)
#define PHY_PhyIrqClearPending()        CPU_CM3_NVIC_ClearPendingISR(gPhyIrqNo_d)
#define PHY_PhyIrqSetPending()          os_InterruptMaskPendingISR(gPhyIrqNo_d)
#define PHY_PhyIrqEnable()              os_InterruptMaskSet(gPhyIrqNo_d)
#define PHY_PhyIrqDisable()             os_InterruptMaskClear(gPhyIrqNo_d)
#define PHY_PhyIrqSetPriority()         os_InterruptSetPriority(gPhyIrqNo_d, gPhyIrqPriority_c)

/* IsCpuInterruptContext : returns 1 if CPU is in interrupt context and 0 if not. */
#define IsCpuInterruptContext()         ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk)!=0)

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
/* Redfinch RSSI offset definition */
#define gChannelRssiOffset_c { 0,   /* 11 */ \
                               0,   /* 12 */ \
                               0,   /* 13 */ \
                               0,   /* 14 */ \
                               0,   /* 15 */ \
                               0,   /* 16 */ \
                               0,   /* 17 */ \
                               0,   /* 18 */ \
                               0,   /* 19 */ \
                               0,   /* 20 */ \
                               0,   /* 21 */ \
                               0,   /* 22 */ \
                               2,   /* 23 */ \
                               2,   /* 24 */ \
                               2,   /* 25 */ \
                               2 }  /* 26 */

#endif /* __PHY_PLATFORM_RW610N_H__ */
