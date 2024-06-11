/*! *********************************************************************************
* Copyright 2021-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */
#ifndef __PHY_PACKET_H__
#define __PHY_PACKET_H__

#include "EmbeddedTypes.h"

enum
{
    phyFcfFrameBeacon      = 0 << 0,
    phyFcfFrameData        = 1 << 0,
    phyFcfFrameAck         = 2 << 0,
    phyFcfFrameMacCmd      = 3 << 0,
    phyFcfFrameTypeMask    = 7 << 0,
    phyFcfSecurityEnabled  = 1 << 3,
    phyFcfFramePending     = 1 << 4,
    phyFcfAckRequest       = 1 << 5,
    phyFcfPanidCompression = 1 << 6,
    phyFcfSeqNbSuppression = 1 << 8,
    phyFcfIePresent        = 1 << 9,
    phyFcfDstAddrNone      = 0 << 10,
    phyFcfDstAddrShort     = 2 << 10,
    phyFcfDstAddrExt       = 3 << 10,
    phyFcfDstAddrMask      = 3 << 10,
    phyFcfFrameVersion2006 = 1 << 12,
    phyFcfFrameVersion2015 = 2 << 12,
    phyFcfFrameVersionMask = 3 << 12,
    phyFcfSrcAddrNone      = 0 << 14,
    phyFcfSrcAddrShort     = 2 << 14,
    phyFcfSrcAddrExt       = 3 << 14,
    phyFcfSrcAddrMask      = 3 << 14,
    phyAshFmCntSupression  = 1 << 5,
    phyAshKeyIdMode0       = 0 << 3,
    phyAshKeyIdMode1       = 1 << 3,
    phyAshKeyIdMode2       = 2 << 3,
    phyAshKeyIdMode3       = 3 << 3,
    phyAshKeyIdModeMask    = 3 << 3,
    phyAshSecLevelMask     = 7 << 0,
    phyAshSecLevel0        = 0 << 0,
    phyAshSecLevel1        = 1 << 0,
    phyAshSecLevel2        = 2 << 0,
    phyAshSecLevel3        = 3 << 0,
    phyAshSecLevel4        = 4 << 0,
    phyAshSecLevel5        = 5 << 0,
    phyAshSecLevel6        = 6 << 0,
    phyAshSecLevel7        = 7 << 0,
};

typedef enum {
    phyMacCmdInvalid      = 0x00,
    phyMacCmdAssocReq     = 0x01,
    phyMacCmdAssocResp    = 0x02,
    phyMacCmdDisassoc     = 0x03,
    phyMacCmdDataReq      = 0x04,
    /* To be completed */
} macCmdId_t;

#define MAC_FRAME_TYPE_ACK          0x2
#define MAC_FRAME_TYPE_CMD          0x3

#define HDR_IE_ID_HT1               0x7e
#define HDR_IE_ID_HT2               0x7f

#define BCST_PAN_ID 0xffffu

#define FCF_SIZE sizeof(uint16_t)       /* Frame control: 2 bytes */
#define SN_SIZE sizeof(uint8_t)         /* Sequence Number: 1 byte */
#define PAN_SIZE sizeof(uint16_t)       /* PAN ID: 2 bytes */

#define FCF_FT_GET(x) ((x) & phyFcfFrameTypeMask)

#define FCF_VER_MAX 2
#define FCF_VER_GET(x) (((x) >> 12) & 0x3)

#define FCF_DAP (1 << 11)       /* destination address is present */
#define FCF_IS_DAS(x) ((((x) >> 10) & 0x3) == 2)    /* dst address is short */
#define FCF_IS_DAE(x) ((((x) >> 10) & 0x3) == 3)    /* dst addr is extended */

#define FCF_SAP (1 << 15)       /* source address is present */
#define FCF_IS_SAE(x) ((((x) >> 14) & 0x3) == 3)    /* src addr is extended */
#define FCF_IS_SAS(x) ((((x) >> 14) & 0x3) == 2)    /* src addr is short */

#define INV_ASH_KEY_ID 0    /* IEEE Std 802.15.4, security chapter: key indices are all different from 0x0 */
#define INV_ASH_KEY_ID_POS ((uint8_t)(-1))

#define INV_LENGTH ((uint8_t)(-1))

typedef PACKED_STRUCT {
    uint16_t frameType:3;
    uint16_t securityEnabled:1;
    uint16_t framePending:1;
    uint16_t ackRequest:1;
    uint16_t panIdCompression:1;
    uint16_t reserved:1;
    uint16_t snSupression:1;
    uint16_t iePresent:1;
    uint16_t dstAddressingMode:2;
    uint16_t frameVersion:2;
    uint16_t srcAddressingMode:2;
} phyFcf_t;

typedef PACKED_STRUCT {
    uint16_t length: 7;
    uint16_t id: 8;
    uint16_t type: 1;
} HdrIe_t;

uint8_t  PhyPacket_GetHdrLength(const uint8_t *packet);

uint8_t  PhyPacket_GetASHLength(uint8_t ash_ctl);

uint8_t *PhyPacket_GetSecurityHeader(const uint8_t *packet, uint8_t packetLength);

bool_t   PhyPacket_GetFCSuppression(uint8_t ash_ctl);
uint8_t  PhyPacket_GetKeyIdMode(uint8_t ash_ctl);
uint8_t  PhyPacket_GetSecurityLevel(uint8_t ash_ctl);
uint8_t  PhyPacket_GetKeyIdentifierLength(uint8_t ash_ctl);
uint8_t  PhyPacket_GetMicLength(uint8_t ash_ctl);
uint8_t  PhyPacket_GetKeyIndexPos(uint8_t ash_ctl);

uint8_t  PhyPacket_GetKeyIndex(const uint8_t *packet, uint8_t packetLength);
void     PhyPacket_SetKeyIndex(uint8_t *packet, uint8_t keyIndex, uint8_t packetLength);

void     PhyPacket_SetFrameCounter(uint8_t *packet, uint32_t frameCounter, uint8_t packetLength);
void     PhyPacket_GetFrameCounter(uint8_t *packet, uint32_t *frameCounter, uint8_t packetLength);

uint8_t  PhyPacket_GetSecurityHeaderLength(uint8_t *packet, uint8_t packetLength);
uint8_t  PhyPacket_GetMicLen(const uint8_t *packet, uint8_t packetLength);
uint8_t  PhyPacket_GetMacHdrLength(uint8_t *packet, uint8_t packetLength);
macCmdId_t PhyPacket_GetMacV1CmdId(uint8_t *packet, uint8_t packetLength);

void     PhyPacket_get_dest_pan_addr(uint8_t *f, uint8_t **pan, uint8_t **addr, uint8_t *len);

#endif /* __PHY_PACKET_H__ */
