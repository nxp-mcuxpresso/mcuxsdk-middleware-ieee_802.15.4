/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2025 NXP
* All rights reserved.
*
* \file
*
* This file contains various global variables definitions needed by the 802.15.4 MAC
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"

#include "MacGlobals.h"
#include "MacInterface.h"

#if !gFsciHost_802_15_4_c
#include "PhyInterface.h"
#endif

#include "ModuleInfo.h"
#include "fsl_component_panic.h"
#if gFsciIncluded_c && gFsciHost_802_15_4_c
  #include "FsciMacCommands.h"
#endif

/************************************************************************************
*************************************************************************************
* Public memory definitions
*************************************************************************************
************************************************************************************/
extern void Mac_Task( osa_task_param_t argument );
extern void Mac_InitializeData(osa_event_handle_t eventHandle, bool_t fast_init);

#if !gFsciHost_802_15_4_c
/* The following definitions are required by the VERSION_TAGS. DO NOT MODIFY or REMOVE */
extern const moduleInfo_t MAC_VERSION;
#if defined ( __IAR_SYSTEMS_ICC__ )
#pragma required=MAC_VERSION /* force the linker to keep the symbol in the current compilation unit */
uint8_t mac_dummy; /* symbol suppressed by the linker as it is unused in the compilation unit, but necessary because 
                             to avoid warnings related to #pragma required */
#elif defined(__GNUC__)
static const moduleInfo_t *const dummy __attribute__((__used__)) = &MAC_VERSION;
#endif /* __IAR_SYSTEMS_ICC__ */
#endif /* !gFsciHost_802_15_4_c */

#if gFsciIncluded_c && gFsciHost_802_15_4_c
extern FsciHostMacInterface_t fsciHostMacInterfaces[gMacInstancesCnt_c];
extern uint8_t                fsciToMacBinding[gMacInstancesCnt_c];

extern resultType_t Dumy_MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
extern resultType_t Dumy_MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
#endif

#if !defined(NO_MAC_TASK) && !defined(USE_ZBOSS_STACK)
/* MAC RTOS objects */
static OSA_TASK_HANDLE_DEFINE(gMacTaskHandler);
OSA_TASK_DEFINE( Mac_Task, gMacTaskPriority_c, 1, gMacTaskStackSize_c, FALSE );
#endif

/* The maximum number of Indirect transactions */
uint8_t gMacMaxIndirectTransactions;

/* The maximum number of pending MAC requests */
uint8_t gMacMaxPendingReq = gMacInpuQueueLimit_d;

/* The ID of the MEM pools used by the MAC */
const uint8_t gMacPoolId = gMacPoolId_d;

/* The maximum number MAC instances */
const uint8_t gMacNoOfInstances = gMacInstancesCnt_c;

/* Storage for MAC's internal data */
uint32_t gMacData[gMacInstancesCnt_c][(gMacInternalDataSize_c + 3)/sizeof(uint32_t)];

/* MAC internal data size. Used for sanity check */
extern const uint16_t gMacLocalDataSize;

/*! Table used to determine the addressing field length based on addressing mode */
const uint8_t gAddrModeFieldLengthTable[4] =
{
    0,  /*!< PAN identifier and address filds are not present */
    0,  /*!< Reserved */
    4,  /*!< Address field contains a 16-bit short address + 16-bit PAN Id */
    10  /*!< Address field contains a 64-bit short address + 16-bit PAn Id */
};

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This function will create the MAC task(s)
*
********************************************************************************** */
void MAC_init_ext(bool_t fast_init)
{
    static bool_t init = FALSE;

    OSA_InterruptDisable();

    if (init)
    {
        OSA_InterruptEnable();

        return;
    }

    init = TRUE;

    OSA_InterruptEnable();

#if gFsciIncluded_c && gFsciHost_802_15_4_c
    for( uint32_t i=0; i<gMacInstancesCnt_c; i++)
    {
        fsciHostMacInterfaces[i].upperLayerId = gInvalidInstanceId_c;
        fsciHostMacInterfaces[i].pfMCPS_NWK_SapHandler = Dumy_MCPS_NWK_SapHandler;
        fsciHostMacInterfaces[i].pfMLME_NWK_SapHandler = Dumy_MLME_NWK_SapHandler;
        fsciHostMacInterfaces[i].fsciInterfaceId = mFsciInvalidInterface_c;
        fsciToMacBinding[i] = mFsciInvalidInterface_c;
    }
   
    /* Nothing else to do if this is a FSCI host */
    return;
#elif !defined(USE_ZBOSS_STACK)

    if( gMacLocalDataSize > gMacInternalDataSize_c )
    {
        /* The value of gMacInternalDataSize_c define must be increased */
        panic(0,0,0,0);
    }
    else
    {
        Mac_InitializeData(NULL, fast_init);

#ifndef NO_MAC_TASK
        /* The instance of the MAC is passed at task creaton */
        if (KOSA_StatusSuccess != OSA_TaskCreate((osa_task_handle_t)gMacTaskHandler, OSA_TASK(Mac_Task), NULL))
        {
            panic(0,0,0,0);
        }
#endif
    }
#endif
}

void MAC_Init( void )
{
    MAC_init_ext(FALSE);
}


/*! *********************************************************************************
* \brief  This function determines tmaximum length of the MSDU of a MAC Data frame
*         given a set of Data Request parameters
*         MAC Frame: FC[2] | SN[1] | ADDR[0-20] | ASH[0-14] | MSDU[] | MIC[0-16] | FCS[2]
*
* \param[in]  pParams Pointer to the MAC Data Request Structure
*
* \return  Returns the maximum length of the Data Frame MSDU for the given
*          set of parameters
* 
* \remarks MSDU length for Data Frames = 127 - FrameControl[2] - SequenceNumber[1] -
*                                        AddressingFields[0 - 20] - 
*                                        ASH[0 - 14] - MIC[0 - 16] - FCS[2]
*                                      = minimum 72
*
********************************************************************************** */
uint16_t Mac_GetMaxMsduLength (mcpsDataReq_t* pParams)
{
    uint16_t maxDataMsduLen;
    
    /* Verify input parameters */
    if ((pParams->dstAddrMode > gAddrModeExtendedAddress_c) ||
        (pParams->srcAddrMode > gAddrModeExtendedAddress_c))
    {
        /* Return 0, some input parameters are invalid */
        maxDataMsduLen = 0;
    }
    else
    {
        /* Substract constant fields length: Frame Control, Sequence Number and CRC */
        maxDataMsduLen = gMaxPHYPacketSize_c - 2 - 1 -  gPhyFCSSize_c;
        
        /* Substract Addressing fields length */
        maxDataMsduLen -= gAddrModeFieldLengthTable[pParams->dstAddrMode];
        maxDataMsduLen -= gAddrModeFieldLengthTable[pParams->srcAddrMode];
        
        /* Check if PanId Compression will be used */
        if ((pParams->srcAddrMode != gAddrModeNoAddress_c) &&
            (pParams->dstAddrMode != gAddrModeNoAddress_c) &&
                (pParams->dstPanId == pParams->srcPanId))
        {
            maxDataMsduLen += 2; /* The source PanId is not present */
        }
    }

    return maxDataMsduLen;
}
