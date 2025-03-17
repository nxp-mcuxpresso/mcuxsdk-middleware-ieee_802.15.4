/*
 * Copyright 2025 NXP
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
#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    XCVR_SetXtalTrim(PLATFORM_GetXtal32MhzTrim(FALSE));
#endif

#if defined(FPGA_TARGET) && (FPGA_TARGET == 1)
    /* Select alternate data rate (only for fpga now) */
    RADIO_CTRL->FPGA_CTRL |= RADIO_CTRL_FPGA_CTRL_DATA_RATE_SEL(0x1U);
#endif /* defined(HDI_MODE) */
}
