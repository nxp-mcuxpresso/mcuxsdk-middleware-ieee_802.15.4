/*! *********************************************************************************
* Copyright 2022-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
***********************************************************************************/

#include "EmbeddedTypes.h"
#include "PhyPacket.h"
#include "Phy.h"
#include <string.h>
#include <fsl_ltc.h>


/* Support only for security level 5, key id mode 1, no frame counter suppression (Thread specification).
   Expected security control field: 0x5 | (0x1 << 3) */
#define EXP_ASH_CTL 0xd


void PhySec_Enable(instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->phySecurity = TRUE;
}

bool_t PhySec_IsEnabled(Phy_PhyLocalStruct_t *ctx)
{
    return ctx->phySecurity;
}

void PhySec_SetFrameCounter(instanceId_t phyInstance, uint32_t frameCounter)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->frameCounter = frameCounter;
}

void PhySec_SetFrameCounterIfLarger(instanceId_t phyInstance, uint32_t frameCounter)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    if (frameCounter > ctx->frameCounter)
    {
        ctx->frameCounter = frameCounter;
    }
}

void PhySec_SetKeys(instanceId_t phyInstance, uint8_t keyId, uint8_t *prevKey, uint8_t *currKey, uint8_t *nextKey)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->keyId = keyId;
    memcpy(ctx->prevKey, prevKey, PHY_SEC_KEY_SIZE);
    memcpy(ctx->currKey, currKey, PHY_SEC_KEY_SIZE);
    memcpy(ctx->nextKey, nextKey, PHY_SEC_KEY_SIZE);
}

static void PhySec_GenerateNonce(Phy_PhyLocalStruct_t *ctx, uint32_t frameCounter, uint8_t secLevel, uint8_t *nonce)
{
    (void)PhyPpGetLongAddr(nonce, ctx->id);
    nonce += sizeof(uint64_t);

    /* Big-Endian Format */
    nonce[0] = (frameCounter >> 24) & 0xff;
    nonce[1] = (frameCounter >> 16) & 0xff;
    nonce[2] = (frameCounter >>  8) & 0xff;
    nonce[3] = (frameCounter >>  0) & 0xff;
    nonce += sizeof(uint32_t);

    nonce[0] = secLevel;
}

bool_t PhySec_IsASHValid(uint8_t ash_ctl, uint8_t key_id)
{
    if ((ash_ctl != EXP_ASH_CTL) || (key_id == INV_ASH_KEY_ID))
    {
        return FALSE;
    }

    return TRUE;
}

void PhySec_Encrypt(Phy_PhyLocalStruct_t *ctx, uint8_t *packet, uint8_t packetLength)
{
    phyFcf_t *fcf = (phyFcf_t *)(packet);

    if (!fcf || !fcf->securityEnabled)
    {
        return;
    }

    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);
    if (!ash)
    {
        return;
    }

    uint8_t keyId = PhyPacket_GetKeyIndex(packet, packetLength);

    if (!PhySec_IsASHValid(*ash, keyId))
    {
        return;
    }

    uint8_t *key = NULL;

    if (keyId == ctx->keyId)
    {
        key = ctx->currKey;
    }
    else if (keyId == ctx->keyId - 1)
    {
        key = ctx->prevKey;
    }
    else if (keyId == ctx->keyId + 1)
    {
        key = ctx->nextKey;
    }
    else
    {
        return;
    }

    uint8_t mhrLength = PhyPacket_GetMacHdrLength(packet, packetLength);

    if (mhrLength == INV_LENGTH)
    {
        return;
    }

    uint8_t nonce[sizeof(uint64_t) + sizeof(uint32_t) + sizeof(uint8_t)];
    uint8_t mic[sizeof(uint64_t) + sizeof(uint64_t)];

    uint8_t secLevel   = PhyPacket_GetSecurityLevel(*ash);
    uint8_t micLength  = PhyPacket_GetMicLength(*ash);
    uint8_t dataLength = packetLength - mhrLength - micLength - gPhyFCSSize_c;
    uint8_t *encData   = packet + mhrLength;

    uint32_t frameCounter = 0;

    PhyPacket_GetFrameCounter(packet, &frameCounter, packetLength);
    PhySec_GenerateNonce(ctx, frameCounter, secLevel, nonce);

    LTC_AES_EncryptTagCcm(LTC0, encData, encData, dataLength, nonce, sizeof(nonce), packet, mhrLength, key, PHY_SEC_KEY_SIZE, mic, micLength);

    memcpy(packet + packetLength - micLength - gPhyFCSSize_c, mic, micLength);
}
