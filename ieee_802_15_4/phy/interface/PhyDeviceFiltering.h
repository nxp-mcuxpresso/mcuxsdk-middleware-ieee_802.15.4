/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
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