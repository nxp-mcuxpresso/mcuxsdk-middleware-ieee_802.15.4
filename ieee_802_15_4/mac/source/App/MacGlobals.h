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

/********************************/
/*** MAC Security Tables sizes **/
/********************************/

/*! The number of keys used by a device. */
#ifndef gNumKeyTableEntries_c
#define gNumKeyTableEntries_c                       2
#endif

/*! Configure one entry for each KeyIdMode used for a specific key. */
#ifndef gNumKeyIdLookupListEntries_c
#define gNumKeyIdLookupListEntries_c                2
#endif

/*! Configure one entry for every device from which secured frames must be received. */
#ifndef gMAC2011_d
    #ifndef gNumKeyDeviceListEntries_c
    #define gNumKeyDeviceListEntries_c              2
    #endif
#else /* gMAC2011_d */
    #ifndef gNumDeviceDescriptorHandleListEntries_c
    #define gNumDeviceDescriptorHandleListEntries_c 2
    #endif
#endif /* gMAC2011_d */

/*! Configure one entry for every secured frame type. */
#ifndef gNumKeyUsageListEntries_c
#define gNumKeyUsageListEntries_c                   2
#endif

/*! Configure one entry for every device from which secured frames must be received. */
#ifndef gNumDeviceTableEntries_c
#define gNumDeviceTableEntries_c                    2
#endif

/*! For MAC internal use only.
 *  Do not change the following macro definition! */
#ifndef gNumDeviceAddrTableEntries_c
    #ifndef gMAC2011_d
        #if (gNumDeviceTableEntries_c > gNumKeyDeviceListEntries_c)
        #define gNumDeviceAddrTableEntries_c (gNumKeyDeviceListEntries_c + 2)
        #else
        #define gNumDeviceAddrTableEntries_c (gNumDeviceTableEntries_c)
        #endif
    #else
        #if (gNumDeviceTableEntries_c > gNumDeviceDescriptorHandleListEntries_c)
        #define gNumDeviceAddrTableEntries_c (gNumDeviceDescriptorHandleListEntries_c + 2)
        #else
        #define gNumDeviceAddrTableEntries_c (gNumDeviceTableEntries_c)
        #endif
    #endif
#endif

/*! Configure one entry for every secured frame type received. */
#ifndef gNumSecurityLevelTableEntries_c
#define gNumSecurityLevelTableEntries_c             2
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
