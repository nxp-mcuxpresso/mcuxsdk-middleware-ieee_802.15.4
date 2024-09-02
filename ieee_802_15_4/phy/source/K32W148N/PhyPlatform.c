/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Phy.h"

/*
 * This function allows an override on a need basis of the XCVR parameters.
 * It is called after the XCVR initialization and also after a switch is
 * done from f.i. BLE to 15.4.
 * 
 * Currently it is left empty, since there are no parameters to override.
 *
 */
void PhyPlatformHwInit(void)
{

}