/*! *********************************************************************************
* \defgroup MacGlobals Mac Globals
* @{
********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2025 NXP
* All rights reserved.
*
* \file
*
* This is the header file for the MacGlobals.c
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


#ifndef _MAC_GLOBALS_H_
#define _MAC_GLOBALS_H_

#include "MacInterface.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/*! The maximum number of pending MAC requests. 0 = no limit */
#ifndef gMacInpuQueueLimit_d
#define gMacInpuQueueLimit_d  (10)
#endif

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
/*! \cond DOXY_SKIP_TAG */
extern uint32_t gMacData[gMacInstancesCnt_c][(gMacInternalDataSize_c + 3)/sizeof(uint32_t)];
extern uint8_t gMacMaxIndirectTransactions;
extern uint8_t gMacMaxPendingReq;
extern const uint8_t gMacNoOfInstances;
extern const uint8_t gMacPoolId;
/*! \endcond */

#endif
/*! *********************************************************************************
* @}
********************************************************************************** */
