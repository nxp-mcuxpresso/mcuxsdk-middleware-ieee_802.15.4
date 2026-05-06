/*
 * Copyright 2023-2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "EmbeddedTypes.h"
#include "fsl_adapter_rpmsg.h"
#include "fwk_platform.h"
#include "Phy.h"
#include "AspInterface.h"
#ifdef MAC_ENABLED
#include "RNG_Interface.h"
#endif

static RPMSG_HANDLE_DEFINE(phyRpmsgHandle);
static hal_rpmsg_config_t phyRpmsgConfig = {
    .local_addr = 20,
    .remote_addr = 10,
};

static RPMSG_HANDLE_DEFINE(aspRpmsgHandle);
static hal_rpmsg_config_t aspRpmsgConfig = {
    .local_addr = 21,
    .remote_addr = 11,
};


#define WAIT_TICKS 12u   /* 192 us */


static void delay_phy_get_rsp(void *p);

static void send_phy_get_rsp(uint32_t param)
{
    uint8_t *response = (uint8_t *)param;

    PLATFORM_RemoteActiveReq();

    if (HAL_RpmsgSend((hal_rpmsg_handle_t)phyRpmsgHandle, response, sizeof(macToPlmeMessage_t)) != kStatus_HAL_RpmsgSuccess)
    {
        delay_phy_get_rsp(response);
    }
    else
    {
        MSG_Free(response);
    }

    PLATFORM_RemoteActiveRel();
}

static void delay_phy_get_rsp(void *p)
{
    phyTimeEvent_t event;

    event.callback = send_phy_get_rsp;
    event.timestamp = WAIT_TICKS + PhyTime_GetTimestamp();
    event.parameter = (uint32_t)p;

    if (PhyTime_ScheduleEvent(&event) == ((phyTimeTimerId_t)gInvalidTimerId_c))
    {
        /* add timeout on the main core side */
        MSG_Free(p);
    }
}


static hal_rpmsg_return_status_t phy_ext_cmd_callback(void *param, uint8_t *data, uint32_t len)
{
    (void)param;
    (void)len;

    phyMessageHeader_t *hdr = (phyMessageHeader_t *)data;

    hdr->ctx_id &= CTX_ID_MASK;

    PHY_ext_cmd(hdr, hdr->ctx_id);

    return kStatus_HAL_RL_RELEASE;
}

static hal_rpmsg_return_status_t PhyRpmsgRxCallback(void *param, uint8_t *data, uint32_t len)
{
    (void)param;

    phyMessageHeader_t *hdr = (phyMessageHeader_t *)data;

    uint8_t msg_type = (hdr->ctx_id >> CTX_ID_SIZE) & CTX_CMD_MASK;

    if (msg_type == CTX_EXT_CMD)
    {
        return phy_ext_cmd_callback(param, data, len);
    }

    if (msg_type != CTX_CMD)
    {
        return kStatus_HAL_RL_RELEASE;
    }

    phyMessageHeader_t *pMsg = (phyMessageHeader_t *)data;

    switch (pMsg->msgType)
    {
        case gGetApiVersion_c:
            HAL_RpmsgSend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)phy_get_api_version(), sizeof(api_version_t));
            break;

        case gPdDataReq_c:
            ((macToPdDataMessage_t *)pMsg)->msgData.dataReq.pPsdu = (uint8_t *)pMsg + sizeof(macToPdDataMessage_t);

        case gPdIndQueueInsertReq_c:
        case gPdIndQueueRemoveReq_c:
            MAC_PD_SapHandler((macToPdDataMessage_t *)pMsg, pMsg->ctx_id);
            break;

        case gPlmeGetReq_c:
        case gPlmeGetTxPowerCapabilities:
        {
            macToPlmeMessage_t *response = MSG_Alloc(sizeof(macToPlmeMessage_t));

            if (!response)
            {
                /* add timeout on the main core side */
                break;
            }

            memcpy(response, pMsg, MIN(len, sizeof(macToPlmeMessage_t)));

            MAC_PLME_SapHandler(response, pMsg->ctx_id);

            /* send response from PHY ISR context */
            delay_phy_get_rsp(response);
        }
            break;

        default:
            MAC_PLME_SapHandler((macToPlmeMessage_t *)pMsg, pMsg->ctx_id);
            break;
    }

    return kStatus_HAL_RL_RELEASE;
}

static hal_rpmsg_return_status_t AspRpmsgRxCallback(void *param, uint8_t *data, uint32_t len)
{
    (void)param;
    AppToAspMessage_t *pMsg = (AppToAspMessage_t *)MSG_Alloc(len);

    memcpy(pMsg, data, len);

    switch (pMsg->msgType) {
    case aspMsgTypeGetXtalTrimReq_c:
        pMsg->msgData.aspXtalTrim.trim = APP_ASP_SapHandler(pMsg, 0);
        PLATFORM_RemoteActiveReq();
        if (HAL_RpmsgSend((hal_rpmsg_handle_t)aspRpmsgHandle, (uint8_t *)pMsg, sizeof(AppToAspMessage_t)) != kStatus_HAL_RpmsgSuccess)
        {
            assert(0);
        }
        PLATFORM_RemoteActiveRel();
        MSG_Free(pMsg);
        break;

    case aspMsgTypeTelecTest_c:
        pMsg->msgData.aspTelecTest.mode = APP_ASP_SapHandler(pMsg, 0);
        PLATFORM_RemoteActiveReq();
        if (HAL_RpmsgSend((hal_rpmsg_handle_t)aspRpmsgHandle, (uint8_t *)pMsg, sizeof(AppToAspMessage_t)) != kStatus_HAL_RpmsgSuccess)
        {
            assert(0);
        }
        PLATFORM_RemoteActiveRel();
        MSG_Free(pMsg);
        break;

    default:
        APP_ASP_SapHandler(pMsg, 0);
        MSG_Free(pMsg);
    }

    return kStatus_HAL_RL_RELEASE;
}

phyStatus_t Pd_Mac_SapHandler(pdDataToMacMessage_t *pMsg, instanceId_t instanceId)
{
    uint32_t len = sizeof(pdDataToMacMessage_t);

    pMsg->ctx_id = instanceId;

    if (pMsg->msgType == gPdDataInd_c)
    {
        len += pMsg->msgData.dataInd.psduLength;
    }
    else if (pMsg->msgType == gPdDataCnf_c)
    {
        len += pMsg->msgData.dataCnf.ackLength;
    }
    else
    {
        /* This should never happen */
        assert(0);
    }

    PLATFORM_RemoteActiveReq();

    if (HAL_RpmsgSend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)pMsg, len) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

    PLATFORM_RemoteActiveRel();

    MSG_Free(pMsg);

    return gPhySuccess_c;
}

phyStatus_t Plme_Mac_SapHandler(plmeToMacMessage_t *pMsg, instanceId_t instanceId)
{
    pMsg->ctx_id = instanceId;

    PLATFORM_RemoteActiveReq();

    if (HAL_RpmsgSend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)pMsg, sizeof(plmeToMacMessage_t)) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

    PLATFORM_RemoteActiveRel();

    MSG_Free(pMsg);

    return gPhySuccess_c;
}

void PHY_ext_cmd_handler(phyMessageHeader_t *pMsg, instanceId_t instanceId)
{
    ext_phy_cmd_t *m = (ext_phy_cmd_t *)pMsg;
    uint32_t len = sizeof(ext_phy_cmd_t) + m->io.out.cnf.ackLength + m->io.out.rx_ind.psduLength;

    pMsg->ctx_id = (instanceId & CTX_ID_MASK) | (CTX_EXT_CMD << CTX_ID_SIZE);

    PLATFORM_RemoteActiveReq();

    if (HAL_RpmsgSend((hal_rpmsg_handle_t)phyRpmsgHandle, (uint8_t *)pMsg, len) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
    }

    PLATFORM_RemoteActiveRel();

    if (!m->do_not_free)
    {
        MSG_Free(pMsg);
    }

    return;
}

#ifdef MAC_ENABLED
void panic(uint32_t id, uint32_t location, uint32_t extra1, uint32_t extra2)
{
}

void RNG_GetRandomNo(uint32_t *pRandomNo)
{
    RNG_GetPseudoRandomData((uint8_t *)pRandomNo, sizeof(uint32_t), NULL);
}
#endif

void init_15_4_Phy(void)
{
    Phy_Init();

    Phy_RegisterSapHandlers(Pd_Mac_SapHandler, Plme_Mac_SapHandler, 0);
    Phy_RegisterSapHandlers(Pd_Mac_SapHandler, Plme_Mac_SapHandler, 1);

    PHY_register_ext_cmd_handler(PHY_ext_cmd_handler, 0);
    PHY_register_ext_cmd_handler(PHY_ext_cmd_handler, 1);

    /* Initialize Phy RPMSG support */
    if (HAL_RpmsgInit((hal_rpmsg_handle_t)phyRpmsgHandle, &phyRpmsgConfig) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
        return;
    }

    if (HAL_RpmsgInstallRxCallback((hal_rpmsg_handle_t)phyRpmsgHandle, PhyRpmsgRxCallback, NULL) != kStatus_HAL_RpmsgSuccess)
    {
        assert(0);
        return;
    }

    /* Initialize Asp RPMSG support */
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

#ifdef MAC_ENABLED
    void init_mac();
    init_mac();
#endif

    RADIO_CTRL->RF_CLK_CTRL |= RADIO_CTRL_RF_CLK_CTRL_ZBLL_CLK_EN_OVRD(1);
}
