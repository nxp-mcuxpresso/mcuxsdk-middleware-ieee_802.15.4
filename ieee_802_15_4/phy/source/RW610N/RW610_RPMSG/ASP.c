/*! *********************************************************************************
* Copyright 2021 NXP
* All rights reserved.
*
* \file
*
* This is the source file for the ASP module.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */

#include "EmbeddedTypes.h"
#include "AspInterface.h"
#include "PhyInterface.h"
#include "fsl_component_messaging.h"
#include "fsl_adapter_rpmsg.h"
#include "fsl_adapter_rfimu.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciAspCommands.h"
#endif

#ifdef gSmacSupported
#include "SMAC_Interface.h"
#endif

/* To be removed */
#include "fsl_component_serial_manager.h"
#include "fsl_format.h"

#if gAspCapability_d

#ifndef ASP_TASK_PRIORITY
#define ASP_TASK_PRIORITY 4U
#endif

#ifndef ASP_TASK_STACK_SIZE
#define ASP_TASK_STACK_SIZE 1024U
#endif

static RPMSG_HANDLE_DEFINE(aspRpmsgHandle);
static hal_rpmsg_config_t aspRpmsgConfig = {
    .local_addr = 11,
    .remote_addr = 21,
	.imuLink       = kIMU_LinkCpu2Cpu3
};
messaging_t aspMsgQueue;
OSA_EVENT_HANDLE_DEFINE(aspEventHandle);

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define mFAD_THR_ResetValue         0x82
#define mANT_AGC_CTRL_ResetValue    0x40
#define mASP_MinTxIntervalMS_d      (5)


/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
enum
{
    gDftNormal_c          = 0,
    gDftTxNoMod_Carrier_c = 1,
    gDftTxPattern_c       = 2,
    gDftTxRandom_c        = 7,
    gDftTxPnChipData_c    = 8,
    gDftTxExternalSrc_c   = 9
};

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

static hal_rpmsg_return_status_t AspRpmsgRxCallback(void *param, uint8_t *data, uint32_t len)
{
	(void)param;
	AppToAspMessage_t *pMsg = (AppToAspMessage_t *)MSG_Alloc(len);
	
	memcpy(pMsg, data, len);
	
	switch(pMsg->msgType) {
	case aspMsgTypeGetXtalTrimReq_c:
	case aspMsgTypeTelecTest_c:
	case aspMsgTypeGetRSSILevel_c:
#if MFG_ENABLE
	case aspMsgTypeReadRegisterReq_c:
	case aspMsgTypeWriteRegisterReq_c:
#endif
		MSG_QueueAddTail(&aspMsgQueue, pMsg);
		OSA_EventSet(aspEventHandle, 1);
		break;
		
	default:
		assert(0);
	}
	
	return kStatus_HAL_RL_RELEASE;
}

/*! *********************************************************************************
* \brief  Initialize the ASP module
*
* \param[in]  phyInstance The instance of the PHY
* \param[in]  interfaceId The FSCI interface used
*
********************************************************************************** */
void ASP_Init(instanceId_t phyInstance)
{
	if (HAL_RpmsgInit((hal_rpmsg_handle_t)aspRpmsgHandle, &aspRpmsgConfig) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
        return;
    }
    
    if (HAL_RpmsgInstallRxCallback((hal_rpmsg_handle_t)aspRpmsgHandle, AspRpmsgRxCallback, NULL) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
        return;
    }

	MSG_QueueInit(&aspMsgQueue);
	OSA_EventCreate((osa_event_handle_t)aspEventHandle, 1);
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
AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t phyInstance)
{
    AspStatus_t status = gAspSuccess_c;
	osa_event_flags_t flags = 0;
#if gFsciIncluded_c
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, NULL, fsciGetAspInterfaceId(phyInstance));
#endif
	
	if (HAL_RpmsgSend((hal_rpmsg_handle_t)aspRpmsgHandle, (uint8_t *)pMsg, sizeof(AppToAspMessage_t)) != kStatus_HAL_RpmsgSuccess)
	{
		 assert(0);
    }
	
	/* wait for the response */
	switch(pMsg->msgType) {
	case aspMsgTypeGetXtalTrimReq_c:
	case aspMsgTypeTelecTest_c:
	case aspMsgTypeGetRSSILevel_c:
#if MFG_ENABLE
	case aspMsgTypeReadRegisterReq_c:
	case aspMsgTypeWriteRegisterReq_c:
#endif
	{
		AppToAspMessage_t *response;
		
		while (1)
		{
			(void)HAL_ImuReceive(aspRpmsgConfig.imuLink);
			OSA_EventWait(aspEventHandle, 1, 1, osaWaitForever_c, &flags);
			if (flags)
				break;
		}

		response = MSG_QueueRemoveHead(&aspMsgQueue);

		if (response == NULL || response->msgType != pMsg->msgType)
		{
			assert(0);
		}
		
		pMsg->msgData = response->msgData;
		MSG_Free(response);
	}
		break;
	
	/* These don't require a response */
	case aspMsgTypeSetXtalTrimReq_c:
	case aspMsgTypeSetSAMState_c:
	case aspMsgTypeAddToSapTable_c:
	case aspMsgTypeRemoveFromSAMTable_c:
	case aspMsgTypeCslEnable_c:
	case aspMsgTypeCslSetSampleTime_c:
	case aspMsgTypeConfigureAckIeData_c:
	case aspMsgTypeSetMacKey_c:	
	case aspMsgTypeEnableEncryption_c:
	case aspMsgTypeSetMacFrameCounter_c:
		break;
		
	default:
		assert(0);
	}
	
#if gFsciIncluded_c
    FSCI_Monitor(gFSCI_AspSapId_c, pMsg, (void *)&status, fsciGetAspInterfaceId(phyInstance));
#endif

    if (pMsg->msgType == aspMsgTypeGetRSSILevel_c)
    {
        *((uint8_t *)&status) = pMsg->msgData.aspRssiValue;
    }

    return status;
}

#endif /* gAspCapability_d */
