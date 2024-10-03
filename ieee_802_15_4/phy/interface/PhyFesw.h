/*! *********************************************************************************
* Copyright 2023 NXP
* All rights reserved.
*
* \file FFU Front End Switching definitions.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_FESW_H__
#define __PHY_FESW_H__

#include "fsl_os_abstraction.h"

/*
 * ** Start of section using anonymous unions
 * */

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
 * -- FFU Front End Switching Control Registers Access Layer
 * ---------------------------------------------------------------------------- */

/** FFU FESW - Register Layout Typedef */
typedef struct {
  __IO uint32_t DELAY;                         /**< FESW DELAY CONTROL, offset: 0x0 */
  __IO uint32_t MODE;                          /**< FESW MODE, offset: 0x4 */
} FFU_FESW_Type;

/*! @name FESW DELAY - FESW DELAY CONTROL */
/*! @{ */

#define FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE_MASK          (0xFFU)
#define FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE_SHIFT         (0U)
/*! RX_FESW_ON_DELAY - RX FESW ON Delay in usec Value
 */
#define FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE_SHIFT)) & FFU_FESW_DELAY_RX_FESW_ON_DELAY_VALUE_MASK)


#define FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE_MASK          (0xFF00U)
#define FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE_SHIFT         (8U)
/*! RX_FESW_OFF_DELAY - RX FESW OFF Delay in usec Value
 */
#define FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE_SHIFT)) & FFU_FESW_DELAY_RX_FESW_OFF_DELAY_VALUE_MASK)


#define FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE_MASK          (0xFF0000U)
#define FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE_SHIFT         (16U)
/*! TX_FESW_ON_DELAY - TX FESW ON Delay in usec Value
 */
#define FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE_SHIFT)) & FFU_FESW_DELAY_TX_FESW_ON_DELAY_VALUE_MASK)


#define FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE_MASK          (0xFF000000U)
#define FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE_SHIFT         (24U)
/*! TX_FESW_OFF_DELAY - TX FESW OFF Delay in usec Value
 */
#define FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE_SHIFT)) & FFU_FESW_DELAY_TX_FESW_OFF_DELAY_VALUE_MASK)
/*! @} */

/*! @name FESW MODE - FESW MODE CONTROL */
/*! @{ */
#define FFU_FESW_MODE_FESW_MODE_MASK                   (0x1U)
#define FFU_FESW_MODE_FESW_MODE_SHIFT                  (0U)
/*! FESW_MODE - Selects mode for front end switch
 *  0b0..uses ipp_do_tx_switch and ipp_do_rx_switch from 15p4 IP
 *  0b1..uses programmable delayed signal from zigbee_tx_en and zigbee_tx_en
 */
#define FFU_FESW_MODE_FESW_MODE(x)                     (((uint32_t)(((uint32_t)(x)) << FFU_FESW_MODE_FESW_MODE_SHIFT)) & FFU_FESW_MODE_FESW_MODE_MASK)

#define FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY_MASK                   (0x2U)
#define FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY_SHIFT                  (1U)
/*! AGC_GAIN_GATEODD_DELAY - Selects delay after which agc_gain_change input from BRF is gated off to 15.4 IP
 *  0b0..Gate off agc_gain_change after 32 us in CCA mode
 *  0b1..Gate off agc_gain_change after 64 us in CCA mode
 */
#define FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY(x)                     (((uint32_t)(((uint32_t)(x)) << FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY_SHIFT)) & FFU_FESW_MODE_AGC_GAIN_GATEOFF_DELAY_MASK)
/*! @} */

/* ----------------------------------------------------------------------------
 * -- FFU Register Masks
 * ---------------------------------------------------------------------------- */

/* FFU Registers - These are Register description for AHB domain registers in FFU wrapper.
 *                 These control the DSM mode, testbus and front end antenna switching logic. */

/** Peripheral FFU FESW(Front End Switching Control) Offset */
#define FFU_FESW_OFFSET                              (0x300u)

/** Peripheral FFU FESW(Front End Switching Control) base pointer */
#define FFU_FESW                                     ((FFU_FESW_Type *)(FFU_REG_BASE+FFU_FESW_OFFSET))

/*
 * ** End of section using anonymous unions
 * */

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

#endif /* __PHY_FESW_H__ */