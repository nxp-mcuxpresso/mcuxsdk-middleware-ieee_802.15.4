/*
 *
 * Copyright 2024 NXP
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
 */

/*
 * \file PhyDeviceFiltering.c
 * \brief This file contains an implementation of visibility device filtering.
 * It's mainly use for  ZigBee Certification Test Suite.
 */

#if defined(FFU_DEVICE_LIMIT_VISIBILITY)

/*********************************************************************************************************************/
/*   INCLUDES                                                                                                        */
/*********************************************************************************************************************/
#include "PhyDeviceFiltering.h"
#include "FunctionLib.h"
#include "PhyPacket.h"

/*********************************************************************************************************************/
/*   LOCAL VARIABLES                                                                                                 */
/*********************************************************************************************************************/
extern uint8_t * const rxf;

/*********************************************************************************************************************/
/*   LOCAL FUNCTIONS DEFINITIONS                                                                                     */
/*********************************************************************************************************************/
static void   Phy_initVisibleFilters(Phy_PhyLocalStruct_t *ctx);
static bool_t Phy_isVisibleFiltersEnabled(Phy_PhyLocalStruct_t *ctx);
static bool_t Phy_isExtAddrVisible(Phy_PhyLocalStruct_t *ctx, ext_addr_t extAddr);
static bool_t Phy_isLocalAddrVisible(Phy_PhyLocalStruct_t *ctx, uint16_t localAddr);
static int    Phy_getExtAddr(Phy_PhyLocalStruct_t *ctx, uint16_t localAddr, ext_addr_t extAddr);

/*********************************************************************************************************************/
/*   LOCAL FUNCTIONS IMPLEMENTATION                                                                                  */
/*********************************************************************************************************************/
static void Phy_initVisibleFilters(Phy_PhyLocalStruct_t *ctx)
{
    FLib_MemSet(&ctx->sFilter, 0, sizeof(visible_filter_t));
}

static bool_t Phy_isVisibleFiltersEnabled(Phy_PhyLocalStruct_t *ctx)
{
    return ctx->sFilter.nVisibleDevice ? true : false;
}

static bool_t Phy_isExtAddrVisible(Phy_PhyLocalStruct_t *ctx, ext_addr_t extAddr)
{
    bool_t bRet = true;

    if( ctx->sFilter.nVisibleDevice )
    {
        uint16_t i;

        bRet = false;
        for( i = 0; i < ctx->sFilter.nVisibleDevice; i++ )
        {
            if( FLib_MemCmp(extAddr, ctx->sFilter.aVisibleExtAddr[i], sizeof(ext_addr_t)) )
            {
                bRet = true;
                break;
            }
        }
    }

    return bRet;
}

static bool_t Phy_isLocalAddrVisible(Phy_PhyLocalStruct_t *ctx, uint16_t localAddr)
{
    uint16_t i;
    bool_t   bRet = true;

    for( i = 0; i < ctx->sFilter.nInvisibleDevice; i++ )
    {
        if( localAddr == ctx->sFilter.aInvisibleLocalAddr[i] )
        {
            bRet = false;
            break;
        }
    }

    return bRet;
}

static int Phy_getExtAddr(Phy_PhyLocalStruct_t *ctx, uint16_t localAddr, ext_addr_t extAddr)
{
    uint16_t i;
    int      ret = -1;

    for( i = 0; i < N_FILTER_DEVICES; i++ )
    {
        if( localAddr == ctx->sFilter.aVisibleLocalAddr[i] )
        {
            FLib_MemCpy(extAddr, ctx->sFilter.aVisibleExtAddr[i], sizeof(ext_addr_t));
            ret = 0;
            break;
        }
    }

    return ret;
}

/*********************************************************************************************************************/
/*   API FUNCTIONS                                                                                                   */
/*********************************************************************************************************************/
bool_t PHY_isFrameVisible(Phy_PhyLocalStruct_t *ctx, uint16_t rxfcf)
{
    bool_t bRet = TRUE;

    if( ctx != NULL )
    {
        if( Phy_isVisibleFiltersEnabled(ctx) )
        {
            ext_addr_t extAddr;
            uint32_t   varHdrLen = PhyPacket_GetHdrLength(rxf);
            uint16_t   srcAddrMode = rxfcf & phyFcfSrcAddrMask;
            uint32_t   srcAddrLen = 0;
            uint8_t   *pSrcAddr = NULL;

            // 64bits src address
            if( srcAddrMode == phyFcfSrcAddrExt )
            {
                srcAddrLen = sizeof(uint64_t);
                pSrcAddr   = rxf + (varHdrLen - srcAddrLen);
                FLib_MemCpy(extAddr, pSrcAddr, srcAddrLen);
                bRet = Phy_isExtAddrVisible(ctx, extAddr);
            }
            // 16bits or broadcast src address
            else if( srcAddrMode == phyFcfSrcAddrShort )
            {
                uint16_t shortSrcAddr = 0;

                srcAddrLen   = sizeof(uint16_t);
                pSrcAddr     = rxf + (varHdrLen - srcAddrLen);
                shortSrcAddr = PHY_TransformArrayToUint16(pSrcAddr);

                bRet = Phy_isLocalAddrVisible(ctx, shortSrcAddr);
                if( bRet )
                {
                    if( Phy_getExtAddr(ctx, shortSrcAddr, extAddr) == 0 )
                    {
                        bRet = Phy_isExtAddrVisible(ctx, extAddr);
                    }
                    else
                    {
                        if( (rxfcf & phyFcfFrameTypeMask) != phyFcfFrameBeacon )
                        {
                            bRet = false;
                        }
                    }
                }
            }
            else if( srcAddrMode == phyFcfSrcAddrNone )
            {
                if( (rxfcf & phyFcfFrameTypeMask) == phyFcfFrameAck )
                {
                    bRet = true;
                }
                else
                {
                    bRet = (ctx->sFilter.bDisableBeaconFrame == false);
                }
            }
        }
    }

    return bRet;
}

phyStatus_t PHY_addVisibleExtAddr(instanceId_t phyInstance, ext_addr_t extAddr)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    phyStatus_t           status = gPhySuccess_c;

    if( ctx->sFilter.nVisibleDevice < N_FILTER_DEVICES )
    {
        FLib_MemCpy(ctx->sFilter.aVisibleExtAddr[ctx->sFilter.nVisibleDevice++], extAddr, sizeof(ext_addr_t));
    }
    else
    {
        status = gPhyInvalidParameter_c;
    }

    return status;
}

phyStatus_t PHY_addInvisibleLocalAddr(instanceId_t phyInstance, uint16_t localAddr)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    phyStatus_t           status = gPhySuccess_c;

    if( ctx->sFilter.nInvisibleDevice < N_FILTER_DEVICES )
    {
        ctx->sFilter.aInvisibleLocalAddr[ctx->sFilter.nInvisibleDevice++] = localAddr;
    }
    else
    {
        status = gPhyInvalidParameter_c;
    }

    return status;
}

phyStatus_t PHY_removeInvisibleLocalAddr(instanceId_t phyInstance, uint16_t localAddr)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    uint16_t              i;
    phyStatus_t           status = gPhyInvalidParameter_c;

    for( i = 0; i < ctx->sFilter.nInvisibleDevice; i++ )
    {
        if( ctx->sFilter.aInvisibleLocalAddr[i] == localAddr )
        {
            memmove(&ctx->sFilter.aInvisibleLocalAddr[i], &ctx->sFilter.aInvisibleLocalAddr[i+1], ctx->sFilter.nInvisibleDevice - i - 1);
            ctx->sFilter.nInvisibleDevice--;
            status = gPhySuccess_c;
            break;
        }
    }

    return status;
}

void PHY_clearVisibleFilters(instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    Phy_initVisibleFilters(ctx);
}

phyStatus_t PHY_updateLocalWithExtAddr(instanceId_t phyInstance, ext_addr_t extAddr, uint16_t localAddr)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);
    uint16_t              i;
    phyStatus_t           status = gPhyInvalidParameter_c;

    for( i = 0; i < ctx->sFilter.nVisibleDevice; i++ )
    {
        if( FLib_MemCmp(extAddr, ctx->sFilter.aVisibleExtAddr[i], sizeof(ext_addr_t)) )
        {
            ctx->sFilter.aVisibleLocalAddr[i] = localAddr;
            status = gPhySuccess_c;
            break;
        }
    }

    return status;
}

void PHY_setBeaconFiltering(instanceId_t phyInstance, bool_t block)
{
    Phy_PhyLocalStruct_t *ctx = ctx_get(phyInstance);

    ctx->sFilter.bDisableBeaconFrame = block;
}
#endif // FFU_DEVICE_LIMIT_VISIBILITY