/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Phy.h"
#include "nxp2p4_xcvr.h"
#include "fwk_platform.h"

/*
 * This function allows an override on a need basis of the XCVR parameters.
 * It is called after the XCVR initialization and also after a switch is
 * done from f.i. BLE to 15.4.
 *
 */
void PhyPlatformHwInit(void)
{
    XCVR_SetXtalTrim(PLATFORM_GetXtal32MhzTrim(FALSE));

#if defined(HDI_MODE) && (HDI_MODE == 1)
    RADIO_CTRL->FPGA_CTRL |= RADIO_CTRL_FPGA_CTRL_HDI_MODE_MASK;
    /* Select alternate data rate (only for fpga now) */
    RADIO_CTRL->FPGA_CTRL |= RADIO_CTRL_FPGA_CTRL_DATA_RATE_SEL(0x1U);
#endif /* defined(HDI_MODE) */
}
