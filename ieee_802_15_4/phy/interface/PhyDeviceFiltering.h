/*
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
#ifndef _PHY_DEVICE_FILTERING_H_
#define _PHY_DEVICE_FILTERING_H_

#include "Phy.h"

#if defined(FFU_DEVICE_LIMIT_VISIBILITY)

/*********************************************************************************************************************/
/*   LOCAL DEFINITIONS                                                                                               */
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*   TYPE DEFINITIONS                                                                                               */
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*   API FUNCTIONS DEFINITIONS                                                                                     */
/*********************************************************************************************************************/
/*! *********************************************************************************
 * \brief  parse header to drop or not current frame (used at PHY layer)
 *
 * \param param Frame Header (2 bytes)
 *
 * \return true: continue / false : drop frame
 *
 ********************************************************************************** */
bool_t PHY_isFrameVisible(Phy_PhyLocalStruct_t *ctx, uint16_t rxfcf);

/*! *********************************************************************************
 * \brief  Insert long address from neighbor(s) which are visible by DUT
 *
 * \param param 64bits external address
 *
 * \return status
 *
 ********************************************************************************** */
phyStatus_t PHY_addVisibleExtAddr(instanceId_t phyInstance, ext_addr_t extAddr);

/*! *********************************************************************************
 * \brief  Insert short address from neighbor(s) which are __NOT__ visible by DUT
 *
 * \param param 16bits short address
 *
 * \return status
 *
 ********************************************************************************** */
phyStatus_t PHY_addInvisibleLocalAddr(instanceId_t phyInstance, uint16_t localAddr);

/*! *********************************************************************************
 * \brief  Remove short address from neighbor(s) which are __NOT__ visible by DUT
 *
 * \param param 16bits short address
 *
 * \return status
 *
 ********************************************************************************** */
phyStatus_t PHY_removeInvisibleLocalAddr(instanceId_t phyInstance, uint16_t localAddr);

/*! *********************************************************************************
 * \brief  disable/clear filter data
 *
 ********************************************************************************** */
void PHY_clearVisibleFilters(instanceId_t phyInstance);

/*! *********************************************************************************
 * \brief   enable/disable beacon filtering
 *
 * \param param boolean to enable/disable
 *
 *
 ********************************************************************************** */
void PHY_setBeaconFiltering(instanceId_t phyInstance, bool_t block);

/*! *********************************************************************************
 * \brief  update cross-table short address with external address
 *
 * \param param 64bits external address, 16bits short address
 *
 * \return status
 *
 ********************************************************************************** */
phyStatus_t PHY_updateLocalWithExtAddr(instanceId_t phyInstance, ext_addr_t extAddr, uint16_t localAddr);

#endif

#endif // _PHY_DEVICE_FILTERING_H_