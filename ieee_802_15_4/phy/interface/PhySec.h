/*! *********************************************************************************
* Copyright 2022-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_SEC_H__
#define __PHY_SEC_H__

#include "EmbeddedTypes.h"


void   PhySec_Enable(instanceId_t phyInstance);
bool_t PhySec_IsEnabled(Phy_PhyLocalStruct_t *ctx);

void   PhySec_SetFrameCounter(instanceId_t phyInstance, uint32_t frameCounter);
void   PhySec_SetFrameCounterIfLarger(instanceId_t phyInstance, uint32_t frameCounter);

void   PhySec_SetKeys(instanceId_t phyInstance, uint8_t keyId, uint8_t *prevKey, uint8_t *currKey, uint8_t *nextKey);
void   PhySec_Encrypt(Phy_PhyLocalStruct_t *ctx, uint8_t *packet, uint8_t packetLength);

bool_t PhySec_IsASHValid(uint8_t ash_ctl, uint8_t key_id);

#endif /* __PHY_SEC_H__ */
