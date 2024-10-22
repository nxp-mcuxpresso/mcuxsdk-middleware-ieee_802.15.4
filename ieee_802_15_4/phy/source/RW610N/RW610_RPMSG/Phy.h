/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2018-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_H__
#define __PHY_H__

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include <PhyInterface.h>

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

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef struct Phy_PhyLocalStruct_tag
{
    PD_MAC_SapHandler_t         PD_MAC_SapHandler;
    PLME_MAC_SapHandler_t       PLME_MAC_SapHandler;
    messaging_t                  macPhyInputQueue;
    uint32_t                    maxFrameWaitTime;
    phyTxParams_t               txParams;
    phyRxParams_t               rxParams;
    phyChannelParams_t          channelParams;
    volatile uint8_t            flags;
    uint8_t                     currentMacInstance;
}Phy_PhyLocalStruct_t;

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */

#ifdef __cplusplus
}
#endif

#endif /* __PHY_H__ */
