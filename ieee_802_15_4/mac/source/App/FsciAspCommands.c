/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017, 2023 NXP
* All rights reserved.
*
* \file
*
* This is a source file which implements the FSCI commands received from the host.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "FunctionLib.h"
#include "fsl_component_mem_manager.h"
#include "AspInterface.h"
#include "PhyInterface.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciAspCommands.h"
#include "FsciCommunication.h"
#endif

#include "fsl_os_abstraction.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define mFsciHost_WaitForStatusTimeout_c     500 /* milliseconds */
#define mFsciHost_WaitForStatusTimeout_us_c  (mFsciHost_WaitForStatusTimeout_c * 1000)

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
#if gFsciIncluded_c && gAspCapability_d
static uint32_t getAspInstance(uint32_t fsciInterfaceId);
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId);
#if gFsciHost_802_15_4_c
static void fsciAspCnfHandler(void *pData, void* param, uint32_t interfaceId);
static uint32_t fsciHostGetAspInterfaceId(instanceId_t phyInstance);
#endif
#endif


/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
#if gFsciIncluded_c
uint8_t mAspFsciBinding = mFsciInvalidInterface_c;

#if gFsciHost_802_15_4_c
FsciHostAspInterface_t fsciHostAspInterfaces = 
{
    .fsciInterfaceId = mFsciInvalidInterface_c
};

#if gFsciHostSupport_c
  extern clientPacket_t *pFsciHostSyncRsp;

  #if gFsciHostSyncUseEvent_c
    extern osaEventId_t         gFsciHostSyncRspEventId;
  #endif
#else
  #error Please enable also gFsciHostSupport_c to allow Host ASP module to work.
#endif
#endif
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief   This function registers the MAC SAP monitors and commands handlers
*
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciRegisterAsp( instanceId_t aspInterface, uint32_t fsciInterfaceId )
{
#if gFsciIncluded_c && gAspCapability_d
    
    if( aspInterface < gPhyInstancesCnt_c )
    {
        /* Bind ASP instance to FSCI interfaceId */    
        mAspFsciBinding = fsciInterfaceId;
        /* Register Handler for requests coming from the serial interface */
        FSCI_RegisterOpGroup( gFSCI_AppAspOpcodeGroup_c, 
                             gFsciMonitorMode_c, 
                             fsciAspReqHandler, 
                             NULL, 
                             fsciInterfaceId);
        /* Register SAP Monitor Handler */
        FSCI_RegisterOpGroup( gFSCI_AspSapId_c,          
                             gFsciMonitorMode_c, 
                             AspSapMonitor,     
                             NULL, 
                             fsciInterfaceId);
    }
#endif /* gFsciIncluded_c && gFSCI_IncludeMacCommands_c */
}

/*! *********************************************************************************
* \brief   This function registers the MAC to Host SAP monitors and commands handlers
*
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciRegisterAspToHost( instanceId_t aspInterface, uint32_t fsciInterfaceId )
{
#if gFsciIncluded_c && gAspCapability_d && gFsciHost_802_15_4_c
    
    if( aspInterface < gPhyInstancesCnt_c )
    {
        /* Bind ASP instance to FSCI interfaceId */
        fsciHostAspInterfaces.fsciInterfaceId = fsciInterfaceId;
        /* Register Handler for async confirms and indications coming from the serial interface */
        FSCI_RegisterOpGroup( gFSCI_AspAppOpcodeGroup_c, 
                             gFsciMonitorMode_c, 
                             fsciAspCnfHandler, 
                             NULL, 
                             fsciInterfaceId );
        
        /* Register SAP Monitor Handler */
        FSCI_RegisterOpGroup( gFSCI_AspSapId_c,
                             gFsciMonitorMode_c,
                             AspSapMonitor,
                             NULL,
                             fsciInterfaceId );
    }
#endif
}


/*! *********************************************************************************
* \brief  ASP SAP handler.
*
* \param[in]  pMsg        Pointer to the request message
* \param[in]  instanceId  The instance of the PHY
*
* \return  AspStatus_t
*
********************************************************************************** */
#if gFsciIncluded_c && gAspCapability_d && gFsciHost_802_15_4_c
AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t phyInstance)
{
    AspStatus_t status = gAspSuccess_c;
    uint32_t      fsciInterface;
    uint32_t      fsciHostInterface;
#if gFsciHostSyncUseEvent_c    
    osaEventFlags_t fsciHostSyncRspEventFlags;
#else
    uint64_t tStamp = 0;    
#endif

    fsciInterface = fsciGetAspInterfaceId(phyInstance);
    fsciHostInterface = fsciHostGetAspInterfaceId(phyInstance);

#if gFsciHostSupport_c    
    FSCI_HostSyncLock(fsciHostInterface, gFSCI_AspAppOpcodeGroup_c, pMsg->msgType);
    
    /* Monitor message from fsci interface */
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, NULL, fsciInterface);
    
    /* Parse message and send request to fsci host interface */
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, NULL, fsciHostInterface);
    
    /* Wait for result from the serial interface */
#if gFsciHostSyncUseEvent_c
     OSA_EventWait(gFsciHostSyncRspEventId, 0x00FFFFFF, FALSE, gFsciHost_WaitForStatusTimeout_c ,&fsciHostSyncRspEventFlags);
#else
    tStamp = TMR_GetTimestamp();
    while( !pFsciHostSyncRsp )
    {
        FSCI_receivePacket((void*)fsciHostInterface);
        if(TMR_GetTimestamp() - tStamp > mFsciHost_WaitForStatusTimeout_us_c)
        {
            break;
        }    
    }
#endif
    }
    
    if( pFsciHostSyncRsp )
    {
        MEM_BufferFree(pFsciHostSyncRsp);
    }
    else
    {
        status = gAspDenied_c;
    }
    
    FSCI_HostSyncUnlock(fsciHostInterface);
#endif
    
    return status;
}
#endif

#if gFsciIncluded_c && gAspCapability_d && gFsciHost_802_15_4_c
static void fsciAspCnfHandler(void *pData, void* param, uint32_t interfaceId)
{
    /* All ASP request are synchronous. No async indication/confirm should arrive! */
    MEM_BufferFree(pData);
}
#endif

/*! *********************************************************************************
* \brief  Monitor the ASP Requests and Responses
*
* \param[in]  pData        monitored message
* \param[in]  param        
* \param[in]  interfaceId  FSCI interface 
*
* \return  AspStatus_t
*
********************************************************************************** */
void AspSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
#if gFsciIncluded_c && gAspCapability_d
    clientPacket_t *pFsciPacket = MEM_BufferAlloc( sizeof(clientPacket_t) );
    AppToAspMessage_t *pReq = (AppToAspMessage_t*)pData;
    uint8_t *p;

    if( NULL == pFsciPacket )
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
    }
    else
    {
        p = pFsciPacket->structured.payload;
        
        if( NULL == param ) /* Requests */
        {
            pFsciPacket->structured.header.opGroup = gFSCI_AppAspOpcodeGroup_c;
            pFsciPacket->structured.header.opCode = pReq->msgType;
            
            switch( pReq->msgType )
            {
            case aspMsgTypeSetXtalTrimReq_c:
                *p++ = pReq->msgData.aspXtalTrim.trim;
                break;
            case aspMsgTypeXcvrWriteReq_c:
            case aspMsgTypeXcvrReadReq_c:
                *p++ = pReq->msgData.aspXcvrData.mode;
                FLib_MemCpy(p, (void *)(&pReq->msgData.aspXcvrData.addr), sizeof(uint16_t));
                p += sizeof(uint16_t);
                *p++ = pReq->msgData.aspXcvrData.len;
                if( pReq->msgType == aspMsgTypeXcvrWriteReq_c )
                {
                    FLib_MemCpy( p, pReq->msgData.aspXcvrData.data,
                                pReq->msgData.aspXcvrData.len );
                    p += pReq->msgData.aspXcvrData.len;
                }
                break;
            case aspMsgTypeSetFADState_c:
                FLib_MemCpy( p, &pReq->msgData.aspFADState, sizeof(pReq->msgData.aspFADState) );
                p += sizeof(pReq->msgData.aspFADState);
                break;
            case aspMsgTypeSetFADThreshold_c:
                FLib_MemCpy( p, &pReq->msgData.aspFADThreshold, sizeof(pReq->msgData.aspFADThreshold) );
                p += sizeof(pReq->msgData.aspFADThreshold);
                break;
            case aspMsgTypeSetANTXState_c:
                FLib_MemCpy( p, &pReq->msgData.aspANTXState, sizeof(pReq->msgData.aspANTXState) );
                p += sizeof(pReq->msgData.aspANTXState);
                break;
            case aspMsgTypeSetPowerLevel_c:
                FLib_MemCpy( p, &pReq->msgData.aspSetPowerLevelReq, sizeof(pReq->msgData.aspSetPowerLevelReq) );
                p += sizeof(pReq->msgData.aspSetPowerLevelReq);
                break;
            case aspMsgTypeTelecSetFreq_c:
                FLib_MemCpy( p, &pReq->msgData.aspTelecsetFreq, sizeof(pReq->msgData.aspTelecsetFreq) );
                p += sizeof(pReq->msgData.aspTelecsetFreq);
                break;
            case aspMsgTypeTelecSendRawData_c:
                FLib_MemCpy( p, &pReq->msgData.aspTelecSendRawData, sizeof(pReq->msgData.aspTelecSendRawData) );
                p += sizeof(pReq->msgData.aspTelecSendRawData);
                break;
            case aspMsgTypeTelecTest_c:
                FLib_MemCpy( p, &pReq->msgData.aspTelecTest, sizeof(pReq->msgData.aspTelecTest) );
                p += sizeof(pReq->msgData.aspTelecTest);
                break;
            case aspMsgTypeSetCCA3Mode_c:
                FLib_MemCpy(p, &pReq->msgData.aspCCA3Mode, sizeof(pReq->msgData.aspCCA3Mode) );
                p += sizeof(pReq->msgData.aspCCA3Mode);
                break;
            case aspMsgTypeGetTimeReq_c:
            case aspMsgTypeGetRSSILevel_c:
            case aspMsgTypeGetANTXState_c:
            case aspMsgTypeGetPowerLevel_c:
            case aspMsgTypeGetXtalTrimReq_c:
            default:
                /* Nothing to do here */
                break;
                
            }
        }
        else /* Confirms / Indications */
        {
            pFsciPacket->structured.header.opGroup = gFSCI_AspAppOpcodeGroup_c;
            pFsciPacket->structured.header.opCode = pReq->msgType;
            
            *p++ = *((uint8_t*)param);/* copy status */
            
            switch( pReq->msgType )
            {
            case aspMsgTypeGetTimeReq_c:
                FLib_MemCpy( p, (void *)(&pReq->msgData.aspGetTimeReq.time) , sizeof(aspGetTimeReq_t) );
                p += sizeof(aspGetTimeReq_t);
                break;
            case aspMsgTypeXcvrReadReq_c:
                *p++ = pReq->msgData.aspXcvrData.len; /* copy length */
                FLib_MemCpy( p, pReq->msgData.aspXcvrData.data, pReq->msgData.aspXcvrData.len );
                p += pReq->msgData.aspXcvrData.len;
                break;
            default:
                break;
            }
        }
        
        /* Send data over the serial interface */
        pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);
        
        if ( pFsciPacket->structured.header.len )
        {
            FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
        }
        else
        {
            MEM_BufferFree( pFsciPacket );
        }
    }
#endif
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Returns the instance of the PHY associated with the fsci interface
*
* \param[in]  interfaceId  FSCI interface 
*
* \return the instance of the PHY
*
********************************************************************************** */
#if gFsciIncluded_c && gAspCapability_d
static uint32_t getAspInstance( uint32_t fsciInterfaceId )
{
    return 0;
}
#endif


/*! *********************************************************************************
* \brief  Returns the id of the FSCI interface associated with the PHY instance
*
* \param[in]  phyInstance the instance of the PHY layer   
*
* \return the FSCI interface id
*
********************************************************************************** */
uint32_t fsciGetAspInterfaceId( instanceId_t phyInstance )
{
#if gFsciIncluded_c && gAspCapability_d
    return mAspFsciBinding;
#else
    return 0;
#endif
}


/*! *********************************************************************************
* \brief  Returns the id of the FSCI Host interface associated with the PHY instance
*
* \param[in]  phyInstance the instance of the PHY layer   
*
* \return the FSCI interface id
*
********************************************************************************** */
#if gFsciIncluded_c && gAspCapability_d && gFsciHost_802_15_4_c
uint32_t fsciHostGetAspInterfaceId( instanceId_t phyInstance )
{
    return fsciHostAspInterfaces.fsciInterfaceId;
}
#endif

/*! *********************************************************************************
* \brief  Handle ASP requests received from FSCI
*
* \param[in]  pData        monitored message
* \param[in]  param        
* \param[in]  interfaceId  FSCI interface 
*
********************************************************************************** */
#if gFsciIncluded_c && gAspCapability_d
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId)
{
    clientPacket_t *pClientPacket = ((clientPacket_t*)pData);
    uint8_t *p = &pClientPacket->structured.payload[0];

    AppToAspMessage_t *pMsg = MSG_Alloc(sizeof(AppToAspMessage_t));

    // Set type of ASP message
    pMsg->msgType = (AppAspMsgType_t)pClientPacket->structured.header.opCode;
    memcpy(&pMsg->msgData, p, pClientPacket->headerAndStatus.header.len);

    APP_ASP_SapHandler( (AppToAspMessage_t*)pMsg, getAspInstance( interfaceId ) );

    // Make sure that pMsg is freed (APP_ASP_SapHandler doesn't free it)
    MSG_Free(pMsg);
    MEM_BufferFree(pData);
}
#endif

/************************************************************************************/
