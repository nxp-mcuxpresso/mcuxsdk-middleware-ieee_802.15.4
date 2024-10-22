/*! *********************************************************************************
* Copyright 2021-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "PhyWlanCoex.h"
#include "fsl_adapter_rpmsg.h"
#include "fsl_adapter_rfimu.h"

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */

static hal_rpmsg_return_status_t PhyWlanCoexRpmsgRxCallback(void *param, uint8_t *data, uint32_t len);

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */

__attribute__((section(".smu_cpu12_shared_var")))
volatile static cpu12_shared_var_t __attribute__((aligned(4))) cpu12_shared_var;

static RPMSG_HANDLE_DEFINE(phyCoexRpmsgHandle);
const static hal_rpmsg_config_t phyCoexRpmsgConfig = {
    .local_addr    = 0x21,
    .remote_addr   = 0x12,
    .imuLink       = kIMU_LinkCpu1Cpu2
};

static bool_t initialized = FALSE;

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

void PhyWlanCoex_Init(void)
{
    if(initialized == FALSE)
    {
        /* Initialize RPMSG link with WLAN CPU */ 
        if (kStatus_HAL_RpmsgSuccess != HAL_RpmsgInit((hal_rpmsg_handle_t)phyCoexRpmsgHandle,
                                                      (hal_rpmsg_config_t *)&phyCoexRpmsgConfig))
        {
            assert(0);
        }

        if (HAL_RpmsgInstallRxCallback((hal_rpmsg_handle_t)phyCoexRpmsgHandle, PhyWlanCoexRpmsgRxCallback, NULL) != kStatus_HAL_RpmsgSuccess)
        {
            assert(0);
        }

        /* Waiting for the link to be up */
        while(HAL_ImuLinkIsUp(phyCoexRpmsgConfig.imuLink) != kStatus_HAL_RpmsgSuccess);
        initialized = TRUE;
    }
}

void PhyWlanCoex_ChannelUpdate(uint8_t channel)
{
    coex_MsgSubtype_t subtype = coexMsgWl2OTChannelInfo_c;

    assert(initialized == TRUE);

    if(channel != cpu12_shared_var.otToWlan.OTChanUsed)
    {
        /* Update the channel used in the shared memory (only if it changed) */
        cpu12_shared_var.otToWlan.OTChanUsed = channel;

        /* Indicate WLAN CPU the channel has been updated */
        (void)HAL_RpmsgSend(phyCoexRpmsgHandle, (uint8_t*)&subtype, 1U);
    }
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */

static hal_rpmsg_return_status_t PhyWlanCoexRpmsgRxCallback(void *param, uint8_t *data, uint32_t len)
{
    /* TODO: WLAN could indicate 15.4 some coex infos */

    return kStatus_HAL_RL_RELEASE;
}