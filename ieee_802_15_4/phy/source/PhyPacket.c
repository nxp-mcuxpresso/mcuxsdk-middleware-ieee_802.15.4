/*! *********************************************************************************
* Copyright 2022-2023 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */
#include "PhyPacket.h"
#include "Phy.h"


uint16_t PHY_TransformArrayToUint16(uint8_t *pArray);


#define PHY_PACKET_ADDR_LENGTH(__addr_mode)		((__addr_mode) ? (((__addr_mode) == 3) ? sizeof(uint64_t) : sizeof(uint16_t)) : 0)


static uint8_t PhyPacket_GetPanIdLength(const phyFcf_t* fcf)
{
    /*
     * This Look-Up Table (LUT) contains the sizes of the PanID (Src + Dst) fields
     * based on the Addressing Modes and PanId Compression fields as described in
     * the table 7-2 from the 2015 standard.
     * The index in the table is computed by concatinating the mentioned fields.
     */
    static uint8_t panidSizeLut[] = {
                /* Dst Addr Mode   | Src Addr Mode   | PanId Compression */
        0,      /* 0 (not present) | 0 (not present) |       0           */
        2,      /* 0 (not present) | 0 (not present) |       1           */
        2,      /* 0 (not present) | 2 (short)       |       0           */
        0,      /* 0 (not present) | 2 (short)       |       1           */
        2,      /* 0 (not present) | 3 (extended)    |       0           */
        0,      /* 0 (not present) | 3 (extended)    |       1           */
        2,      /* 2 (short)       | 0 (not present) |       0           */
        0,      /* 2 (short)       | 0 (not present) |       1           */
        4,      /* 2 (short)       | 2 (short)       |       0           */
        2,      /* 2 (short)       | 2 (short)       |       1           */
        4,      /* 2 (short)       | 2 (extended)    |       0           */
        2,      /* 2 (short)       | 2 (extended)    |       1           */
        2,      /* 2 (extended)    | 0 (not present) |       0           */
        0,      /* 2 (extended)    | 0 (not present) |       1           */
        4,      /* 2 (extended)    | 2 (short)       |       0           */
        2,      /* 2 (extended)    | 2 (short)       |       1           */
        2,      /* 2 (extended)    | 3 (extended)    |       0           */
        0,      /* 2 (extended)    | 3 (extended)    |       1           */
    };

    /* According to 7.2.1.5 PanId field specifications of the 2015 standard */
    if ((fcf->frameVersion == 0) || (fcf->frameVersion == 1))
    {
        if (fcf->panIdCompression == 1)
        {
            return 2;
        }
        else
        {
            return 4;
        }
    }
    else /* fcf->frameVersion == 2 */
    {
    uint8_t index = (fcf->dstAddressingMode ? fcf->dstAddressingMode - 1 : fcf->dstAddressingMode) * 6 +
                    (fcf->srcAddressingMode ? fcf->srcAddressingMode - 1 : fcf->srcAddressingMode) * 2 +
                     fcf->panIdCompression;
    return panidSizeLut[index];
    }
}

uint8_t PhyPacket_GetHdrLength(const uint8_t *packet)
{
    phyFcf_t *fcf = (phyFcf_t *)packet;
    uint8_t length = sizeof(phyFcf_t);

    if (!packet)
    {
        return 0;
    }

    if (fcf->snSupression == 0)
    {
        length += sizeof(uint8_t);
    }

    length += PhyPacket_GetPanIdLength(fcf);
    length += PHY_PACKET_ADDR_LENGTH(fcf->dstAddressingMode);
    length += PHY_PACKET_ADDR_LENGTH(fcf->srcAddressingMode);

    return length;
}

uint8_t PhyPacket_GetASHLength(uint8_t ash_ctl)
{
    uint8_t len = PHY_ASH_SEC_CTRL_SIZE;

    if (!PhyPacket_GetFCSuppression(ash_ctl))
    {
        len += PHY_ASH_FRAME_CNT_SIZE;
    }

    len += PhyPacket_GetKeyIdentifierLength(ash_ctl);

    return len;
}

uint8_t * PhyPacket_GetSecurityHeader(const uint8_t *packet, uint8_t packetLength)
{
    if (!packet)
    {
        return NULL;
    }

    if (!((phyFcf_t *)packet)->securityEnabled)
    {
        return NULL;
    }

    uint8_t secHeaderIndex = PhyPacket_GetHdrLength(packet);

    if (secHeaderIndex + gPhyFCSSize_c + PHY_ASH_SEC_CTRL_SIZE > packetLength)
    {
        /* malformed frame: no space for ASH CTL */
        return NULL;
    }

    uint8_t ash_ctl = packet[secHeaderIndex];

    if (secHeaderIndex + gPhyFCSSize_c + PhyPacket_GetASHLength(ash_ctl) +
        PhyPacket_GetMicLength(ash_ctl) > packetLength)
    {
        /* malformed frame */
        return NULL;
    }

    uint8_t *ash = (uint8_t *)packet + secHeaderIndex;
    return ash;
}

uint8_t PhyPacket_GetSecurityLevel(uint8_t ash_ctl)
{
    return (ash_ctl & phyAshSecLevelMask);
}

/* Returns the MIC length based on security level */
uint8_t PhyPacket_GetMicLength(uint8_t ash_ctl)
{
    uint8_t micSize = 0;
    uint8_t securityLevel = PhyPacket_GetSecurityLevel(ash_ctl);

    switch (securityLevel)
    {
    case phyAshSecLevel0:
    case phyAshSecLevel4:
        micSize = 0;
        break;

    case phyAshSecLevel1:
    case phyAshSecLevel5:
        micSize = 4;
        break;

    case phyAshSecLevel2:
    case phyAshSecLevel6:
        micSize = 8;
        break;

    case phyAshSecLevel3:
    case phyAshSecLevel7:
        micSize = 16;
        break;
    }
    return micSize;
}

uint8_t PhyPacket_GetMicLen(const uint8_t *packet, uint8_t packetLength)
{
    uint8_t ash_ctl = 0;
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (ash)
    {
        ash_ctl = *ash;
    }

    return PhyPacket_GetMicLength(ash_ctl);
}

uint8_t PhyPacket_GetKeyIdMode(uint8_t ash_ctl)
{
    return (ash_ctl & phyAshKeyIdModeMask);
}

bool_t PhyPacket_GetFCSuppression(uint8_t ash_ctl)
{
    return (ash_ctl & phyAshFmCntSupression);
}

/* Returns the key Identifier field length */
uint8_t PhyPacket_GetKeyIdentifierLength(uint8_t ash_ctl)
{
    uint8_t len = 0;
    uint8_t keyIdMode = PhyPacket_GetKeyIdMode(ash_ctl);

    switch (keyIdMode)
    {
    case phyAshKeyIdMode0:
        len = 0;
        break;

    case phyAshKeyIdMode1:
        len = 1;
        break;

    case phyAshKeyIdMode2:
        len = 5;
        break;

    case phyAshKeyIdMode3:
        len = 9;
        break;
    }

    return len;
}

uint8_t PhyPacket_GetKeyIndexPos(uint8_t ash_ctl)
{
    if (!PhyPacket_GetKeyIdentifierLength(ash_ctl))
    {
        return INV_ASH_KEY_ID_POS;
    }

    return PhyPacket_GetASHLength(ash_ctl) - PHY_ASH_KEY_IDX_SIZE;
}

uint8_t PhyPacket_GetKeyIndex(const uint8_t *packet, uint8_t packetLength)
{
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (!ash)
    {
        return INV_ASH_KEY_ID;
    }

    uint8_t pos = PhyPacket_GetKeyIndexPos(*ash);

    if (pos == INV_ASH_KEY_ID_POS)
    {
        return INV_ASH_KEY_ID;
    }

    return ash[pos];
}

void PhyPacket_SetKeyIndex(uint8_t *packet, uint8_t keyIndex, uint8_t packetLength)
{
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (!ash)
    {
        return;
    }

    uint8_t pos = PhyPacket_GetKeyIndexPos(*ash);

    if (pos == INV_ASH_KEY_ID_POS)
    {
        return;
    }

    ash[pos] = keyIndex;
}

void PhyPacket_SetFrameCounter(uint8_t *packet, uint32_t frameCounter, uint8_t packetLength)
{
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (!ash)
    {
        return;
    }

    if (PhyPacket_GetFCSuppression(*ash))
    {
        return;
    }

    /* Little-Endian Format */
    ash[PHY_ASH_SEC_CTRL_SIZE] = (frameCounter >>  0) & 0xff;
    ash[PHY_ASH_SEC_CTRL_SIZE + 1] = (frameCounter >>  8) & 0xff;
    ash[PHY_ASH_SEC_CTRL_SIZE + 2] = (frameCounter >> 16) & 0xff;
    ash[PHY_ASH_SEC_CTRL_SIZE + 3] = (frameCounter >> 24) & 0xff;
}

void PhyPacket_GetFrameCounter(uint8_t *packet, uint32_t *frameCounter, uint8_t packetLength)
{
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (!ash)
    {
        return;
    }

    if (PhyPacket_GetFCSuppression(*ash))
    {
        return;
    }

    /* Little-Endian Format */
    memcpy((uint8_t *)frameCounter, &ash[PHY_ASH_SEC_CTRL_SIZE], sizeof(uint32_t));
}

uint8_t PhyPacket_GetSecurityHeaderLength(uint8_t *packet, uint8_t packetLength)
{
    uint8_t *ash = PhyPacket_GetSecurityHeader(packet, packetLength);

    if (!ash)
    {
        return 0;
    }

    return PhyPacket_GetASHLength(*ash);
}

uint8_t PhyPacket_GetMacHdrLength(uint8_t *packet, uint8_t packetLength)
{
    if (!packet)
    {
        return INV_LENGTH;
    }

    phyFcf_t *fcf = (phyFcf_t *)packet;

    uint16_t mhrLength = PhyPacket_GetHdrLength(packet) + PhyPacket_GetSecurityHeaderLength(packet, packetLength);
    uint8_t mic_len = PhyPacket_GetMicLen(packet, packetLength);

    if (mhrLength + mic_len + gPhyFCSSize_c > packetLength)
    {
        /* malformed frame */
        return INV_LENGTH;
    }

    packetLength -= (gPhyFCSSize_c + mic_len);
    packet += mhrLength;

    if (fcf->iePresent)
    {
        HdrIe_t *ie;

        while (mhrLength < packetLength)
        {
            ie         = (HdrIe_t *)packet;
            packet    += (sizeof(HdrIe_t) + ie->length);
            mhrLength += (sizeof(HdrIe_t) + ie->length);

            if ((ie->id == HDR_IE_ID_HT1) || (ie->id == HDR_IE_ID_HT2))
            {
                break;
            }
        }
    }

    if ((fcf->frameVersion != 2) && (fcf->frameType == MAC_FRAME_TYPE_CMD))
    {
        /* beacon encryption not supported */
        mhrLength++;
    }

    if (mhrLength > packetLength)
    {
        /* malformed frame */
        return INV_LENGTH;
    }

    return mhrLength;
}

/* This will return the CMD ID for a V0/1 packet frame */
macCmdId_t PhyPacket_GetMacV1CmdId(uint8_t *packet, uint8_t packetLength)
{
    phyFcf_t *fcf     = (phyFcf_t *)packet;
    uint8_t hdr_len;

    if ((fcf->frameType != MAC_FRAME_TYPE_CMD) ||
       (fcf->frameVersion != 0 && fcf->frameVersion != 1))
    {
        return phyMacCmdInvalid;
    }

    /* Parse the packet and get HDR length (which includes the CMD ID) */
    hdr_len = PhyPacket_GetMacHdrLength(packet, packetLength);
    if (hdr_len == INV_LENGTH)
    {
        return phyMacCmdInvalid;
    }

    return (macCmdId_t)packet[hdr_len - 1];
}

void PhyPacket_get_dest_pan_addr(uint8_t *f, uint8_t **pan, uint8_t **addr, uint8_t *len)
{
    uint8_t tmp_pos = FCF_SIZE;      /* current position starts after FCF */
    uint16_t fcf = PHY_TransformArrayToUint16(f);

    if (!(fcf & FCF_DAP))
    {
        /* no destination address */
        *addr = NULL;
        return;
    }

    *pan = NULL;

    if (FCF_IS_DAS(fcf))
    {
        *len = sizeof(uint16_t);
    }
    else if (FCF_IS_DAE(fcf))
    {
        *len = sizeof(uint64_t);
    }

    if ((FCF_VER_GET(fcf) < FCF_VER_MAX) ||
        ((FCF_VER_GET(fcf) == FCF_VER_MAX) && !(fcf & phyFcfSeqNbSuppression)))
    {
        tmp_pos += SN_SIZE;
    }

    if (FCF_VER_GET(fcf) < FCF_VER_MAX)
    {
        /* destination address is present, so is destination PAN ID */
        *pan = f + tmp_pos;
        tmp_pos += PAN_SIZE;
    }
    else if (FCF_VER_GET(fcf) == FCF_VER_MAX)
    {
        if (!(fcf & FCF_SAP))
        {
            if (!(fcf & phyFcfPanidCompression))
            {
                *pan = f + tmp_pos;
                tmp_pos += PAN_SIZE;
            }
        }
        else
        {
            if (FCF_IS_DAE(fcf) && FCF_IS_SAE(fcf))
            {
                if (!(fcf & phyFcfPanidCompression))
                {
                    *pan = f + tmp_pos;
                    tmp_pos += PAN_SIZE;
                }
            }
            else
            {
                *pan = f + tmp_pos;
                tmp_pos += PAN_SIZE;
            }
        }
    }

    *addr = f + tmp_pos;
}
