/*! *********************************************************************************
* Copyright 2026 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include "PhyInterface.h"

#define PHY_API_MAJOR_VERSION       1
#define PHY_API_MINOR_VERSION       0
#define PHY_API_PATCH_VERSION       0


static api_version_t api_version = {
    .msg_type = gGetApiVersion_c,
    .ctx_id   = 0,                  /* all interfaces use the same API version */
    .major    = PHY_API_MAJOR_VERSION,
    .minor    = PHY_API_MINOR_VERSION,
    .patch    = PHY_API_PATCH_VERSION
};

api_version_t *phy_get_api_version()
{
    return &api_version;
}
