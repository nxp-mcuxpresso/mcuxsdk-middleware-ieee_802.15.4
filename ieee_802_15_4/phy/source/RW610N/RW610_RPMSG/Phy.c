/*! *********************************************************************************
* Copyright 2021 NXP
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include "EmbeddedTypes.h"
#include "PhyInterface.h"
#include "Phy.h"
#include "fsl_component_serial_manager.h"
#include "fsl_format.h"
#include "fsl_adapter_rpmsg.h"
#include "fsl_adapter_rfimu.h"
#include "fsl_os_abstraction.h"

#define PHY_MAC_PLME_MSG_RDY (1 << 0)
#define PHY_MAC_ALL_EVENTS   ( PHY_MAC_PLME_MSG_RDY )

#ifndef PLME_MAC_TASK_PRIORITY
#define PLME_MAC_TASK_PRIORITY 4U
#endif

#ifndef PLME_MAC_TASK_STACK_SIZE
#define PLME_MAC_TASK_STACK_SIZE 1024U
#endif


void plmeMacSapHandlerTask(void *argument);

Phy_PhyLocalStruct_t phyLocal;

static RPMSG_HANDLE_DEFINE(phyRpmsgHandle);
static hal_rpmsg_config_t phyRpmsgConfig = {
    .local_addr  = 10,
    .remote_addr = 20,
    .imuLink     = kIMU_LinkCpu2Cpu3
};

messaging_t plmeMsgQueue;
messaging_t plmeMacSapMsgQueue;
OSA_EVENT_HANDLE_DEFINE(plmeEventHandle);
OSA_EVENT_HANDLE_DEFINE(plmeMacSapEventHandle);
static OSA_TASK_DEFINE(plmeMacSapHandlerTask, 1, 1, 1024, 0);
static OSA_TASK_HANDLE_DEFINE(s_plmeMacSapTaskHandle);

/* Task used to call PLME to MAC Sap Handler from Thread context to allow any
 * any calls to the Phy (and their responses via RPMSGs).
 */
void plmeMacSapHandlerTask(void *argument)
{
	(void)argument;
	osa_event_flags_t flags;
	macToPlmeMessage_t *response;

	while (1) {
		if (OSA_EventWait(plmeMacSapEventHandle, 1, 1, osaWaitForever_c, &flags) == KOSA_StatusIdle)
		{
			break;
		}

		response = MSG_QueueRemoveHead(&plmeMacSapMsgQueue);
		phyLocal.PLME_MAC_SapHandler((plmeToMacMessage_t *)response, 0);
	}
}

static hal_rpmsg_return_status_t PhyRpmsgRxCallback(void *param, uint8_t *data, uint32_t len)
{
    (void)param;
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();

	phyMessageHeader_t *pMsg = (phyMessageHeader_t *)MSG_Alloc(len);
	memcpy(pMsg, data, len);

	OSA_EXIT_CRITICAL();

	switch(pMsg->msgType) {
	case gPdDataInd_c:
		((pdDataToMacMessage_t *)pMsg)->msgData.dataInd.pPsdu = (uint8_t *)&((pdDataToMacMessage_t *)pMsg)->msgData.dataInd.pPsdu + sizeof(uint8_t *);
		phyLocal.PD_MAC_SapHandler((pdDataToMacMessage_t *)pMsg, 0);
        break;
	case gPdDataCnf_c:
		((pdDataToMacMessage_t *)pMsg)->msgData.dataCnf.ackData = (uint8_t *)&((pdDataToMacMessage_t *)pMsg)->msgData.dataCnf + sizeof(((pdDataToMacMessage_t *)pMsg)->msgData.dataCnf);
		phyLocal.PD_MAC_SapHandler((pdDataToMacMessage_t *)pMsg, 0);
		break;

	case gPlmeCcaCnf_c:
	case gPlmeEdCnf_c:
	case gPlmeSetTRxStateCnf_c:
	case gPlmeSetCnf_c:
	case gPlmeTimeoutInd_c:
    case gPlmeAbortInd_c:
		MSG_QueueAddTail(&plmeMacSapMsgQueue, pMsg);
		OSA_EventSet(plmeMacSapEventHandle, PHY_MAC_PLME_MSG_RDY);
		break;

	case gPlmeGetReq_c:
		MSG_QueueAddTail(&plmeMsgQueue, pMsg);
		OSA_EventSet(plmeEventHandle, 1);
		break;

	default:
		assert(0);
	}

    return kStatus_HAL_RL_RELEASE;
}

void Phy_Init(void)
{
    if (HAL_RpmsgInit((hal_rpmsg_handle_t)phyRpmsgHandle, &phyRpmsgConfig) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

    while(HAL_ImuLinkIsUp(phyRpmsgConfig.imuLink) != kStatus_HAL_RpmsgSuccess)
	{
		(void)HAL_ImuReceive(phyRpmsgConfig.imuLink);
	}

    if (HAL_RpmsgInstallRxCallback((hal_rpmsg_handle_t)phyRpmsgHandle, PhyRpmsgRxCallback, NULL) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

	MSG_QueueInit(&plmeMsgQueue);
	MSG_QueueInit(&plmeMacSapMsgQueue);
	OSA_EventCreate((osa_event_handle_t)plmeEventHandle, 1);
	OSA_EventCreate((osa_event_handle_t)plmeMacSapEventHandle, 1);
	OSA_TaskCreate((osa_task_handle_t)s_plmeMacSapTaskHandle, OSA_TASK(plmeMacSapHandlerTask), NULL);
}

/*! *********************************************************************************
* \brief  This function registers the MAC PD and PLME SAP handlers
*
* \param[in]  pPD_MAC_SapHandler   Pointer to the MAC PD handler function
* \param[in]  pPLME_MAC_SapHandler Pointer to the MAC PLME handler function
* \param[in]  instanceId           The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
void Phy_RegisterSapHandlers(PD_MAC_SapHandler_t pPD_MAC_SapHandler,
                             PLME_MAC_SapHandler_t pPLME_MAC_SapHandler,
                             instanceId_t instanceId)
{
    //instanceId = instanceId;
    phyLocal.PD_MAC_SapHandler   = pPD_MAC_SapHandler;
    phyLocal.PLME_MAC_SapHandler = pPLME_MAC_SapHandler;
}

/*! *********************************************************************************
* \brief  This is the PLME SAP message handler
*
* \param[in]  pMsg Pointer to the PLME request message
* \param[in]  instanceId The instance of the PHY
*
* \return  phyStatus_t The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PLME_SapHandler(macToPlmeMessage_t *pMsg, instanceId_t phyInstance)
{
    phyStatus_t result = gPhySuccess_c;
	osa_event_flags_t flags;
    uint8_t* rpmsgBuffer;
    uint32_t len;
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    
	len = sizeof(macToPlmeMessage_t);
    rpmsgBuffer = (uint8_t*)HAL_RpmsgAllocTxBuffer((hal_rpmsg_handle_t)phyRpmsgHandle, len);
    memcpy(rpmsgBuffer, pMsg, len);
    
	OSA_EXIT_CRITICAL();

	if (HAL_RpmsgNoCopySend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)rpmsgBuffer, sizeof(macToPlmeMessage_t)) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

	switch (pMsg->msgType) {
	case gPlmeGetReq_c:
	{
		macToPlmeMessage_t *response;
	
		while (1)
		{
			(void)HAL_ImuReceive(phyRpmsgConfig.imuLink);
			OSA_EventWait(plmeEventHandle, 1, 1, osaWaitForever_c, &flags);
			if (flags)
				break;
		}
		
		response = MSG_QueueRemoveHead(&plmeMsgQueue);

		if (response == NULL || response->msgType != pMsg->msgType)
		{
			assert(0);
		}

		pMsg->msgData.getReq.PibAttributeValue = response->msgData.getReq.PibAttributeValue;
		MSG_Free(response);
	}
		break;

	case gPlmeSetReq_c:
	case gPlmeSetTRxStateReq_c:
	case gPlmeEdReq_c:
	case gPlmeCcaReq_c:
		break;

	default:
		assert(0);
	}

	return result;
}

/*! *********************************************************************************
* \brief  This is the PD SAP message handler
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  instanceId The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PD_SapHandler(macToPdDataMessage_t *pMsg, instanceId_t phyInstance)
{
	uint32_t len;
    phyStatus_t result = gPhySuccess_c;
    uint8_t* rpmsgBuffer;
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
	len = sizeof(macToPdDataMessage_t);
	if (pMsg->msgType == gPdDataReq_c) {
        /* allocate message in RPMSG TX buffer and add size of psdu */
        rpmsgBuffer = (uint8_t*)HAL_RpmsgAllocTxBuffer((hal_rpmsg_handle_t)phyRpmsgHandle, len + pMsg->msgData.dataReq.psduLength);
        /* copy original message data */
        memcpy(rpmsgBuffer, pMsg, len);
        /* copy psdu in buffer tail */
        memcpy(rpmsgBuffer + len, pMsg->msgData.dataReq.pPsdu, pMsg->msgData.dataReq.psduLength);
        ((macToPdDataMessage_t *)rpmsgBuffer)->msgData.dataReq.pPsdu = rpmsgBuffer + len;
        len += pMsg->msgData.dataReq.psduLength;
	}
    else
    {
        rpmsgBuffer = (uint8_t*)HAL_RpmsgAllocTxBuffer((hal_rpmsg_handle_t)phyRpmsgHandle, len);
        memcpy(rpmsgBuffer, pMsg, len);
    }
    OSA_EXIT_CRITICAL();

	if (HAL_RpmsgNoCopySend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)rpmsgBuffer, len) != kStatus_HAL_RpmsgSuccess)
	{
		 assert(0);
    }

	return result;
}


/*! *********************************************************************************
* \brief  This function returns the RSSI for the las received packet
*
* \return  uint8_t  RSSI value
*
********************************************************************************** */
uint8_t PhyGetLastRxRssiValue(void)
{
    int32_t lqi_to_rssi = 0;

    return (uint8_t)lqi_to_rssi;
}
