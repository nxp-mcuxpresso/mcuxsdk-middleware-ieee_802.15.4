/*! *********************************************************************************
* Copyright 2025 NXP
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include "PhyPpme.h"
#include "fsl_device_registers.h"


void PhyPpme_EnablePatternA(ppme_pattern_t *set)
{
    uint32_t reg;

    ZLL->PROG_PKT_FLTR_LO_PATT_A = (uint32_t)set->pattern;
    ZLL->PROG_PKT_FLTR_UP_PATT_A = (uint32_t)(set->pattern >> 32);

    ZLL->PROG_PKT_FLTR_LO_MSK_A = (uint32_t)(set->mask);
    ZLL->PROG_PKT_FLTR_UP_MSK_A = (uint32_t)(set->mask >> 32);

    reg  = ZLL->PROG_PKT_FLTR_CTRL_A;

    reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_FRM_TYPE_A_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_FRM_VER_A_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_OFST_A_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_EN_A_MASK);
    reg |= (ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_FRM_TYPE_A(set->frame_type) |
            ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_FRM_VER_A(set->frame_version) |
            ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_OFST_A(set->offset) |
            ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_EN_A(1));

    if (set->packet_reject)
    {
        reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_REJECT_PKT_A_MASK);
        reg |= ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_REJECT_PKT_A(1);
    }

    if (set->enable_irq)
    {
        reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_MAT_IRQ_MODE_A_MASK);
        reg |= ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_MAT_IRQ_MODE_A(1);
    }

    reg |= ZLL_PROG_PKT_FLTR_CTRL_A_PPME_FILTERFAIL_ENABLE(1);

    ZLL->PROG_PKT_FLTR_CTRL_A = reg;
}

void PhyPpme_DisablePatternA()
{
    uint32_t reg;

    reg  = ZLL->PROG_PKT_FLTR_CTRL_A;
    reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_A_PKT_FLTR_EN_A_MASK);
    ZLL->PROG_PKT_FLTR_CTRL_A = reg;
}

void PhyPpme_EnablePatternB(ppme_pattern_t *set)
{
    uint32_t reg;

    ZLL->PROG_PKT_FLTR_LO_PATT_B = (uint32_t)set->pattern;
    ZLL->PROG_PKT_FLTR_UP_PATT_B = (uint32_t)(set->pattern >> 32);

    ZLL->PROG_PKT_FLTR_LO_MSK_B = (uint32_t)(set->mask);
    ZLL->PROG_PKT_FLTR_UP_MSK_B = (uint32_t)(set->mask >> 32);

    reg  = ZLL->PROG_PKT_FLTR_CTRL_B;

    reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FRM_TYPE_B_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FRM_VER_B_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_OFST_B_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_REJECT_AND_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_CONCAT_PFAB_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FLOAT_OFST_EN_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_REJECT_PKT_B_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_MAT_IRQ_MODE_B_MASK |
             ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_EN_B_MASK);

    reg |= (ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FRM_TYPE_B(set->frame_type) |
            ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FRM_VER_B(set->frame_version) |
            ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_OFST_B(set->offset) |
            ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_EN_B(1));

    if (set->packet_reject)
    {
        reg |= ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_REJECT_PKT_B(1);
    }

    if (set->enable_irq)
    {
        reg |= ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_MAT_IRQ_MODE_B(1);
    }

    if (set->flags & PHY_PPME_FLAGS_REJECT_AND)
    {
        reg |= ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_REJECT_AND(1);
    }

    if (set->flags & PHY_PPME_FLAGS_CONCAT)
    {
        reg |= ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_CONCAT_PFAB(1);
    }

    if (set->flags & PHY_PPME_FLAGS_FLOAT_OFFSET)
    {
        reg |= ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_FLOAT_OFST_EN(1);
    }

    ZLL->PROG_PKT_FLTR_CTRL_B = reg;
}

void PhyPpme_DisablePatternB()
{
    uint32_t reg;

    reg  = ZLL->PROG_PKT_FLTR_CTRL_B;
    reg &= ~(ZLL_PROG_PKT_FLTR_CTRL_B_PKT_FLTR_EN_B_MASK);
    ZLL->PROG_PKT_FLTR_CTRL_B = reg;
}

void PhyPpme_ClearOffsetErrorIntA()
{
    uint32_t reg;

    reg = ZLL->PROG_PKT_FLTR_INTR_MSK_STS;
    reg &= ~(ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_B_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_B_MASK);
    reg |= ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_A_MASK;
    ZLL->PROG_PKT_FLTR_INTR_MSK_STS = reg;
}

void PhyPpme_ClearPatternMatchIntA()
{
    uint32_t reg;

    reg  = ZLL->PROG_PKT_FLTR_INTR_MSK_STS;
    reg &= ~(ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_B_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_B_MASK);
    reg |= ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_A_MASK;
    ZLL->PROG_PKT_FLTR_INTR_MSK_STS = reg;
}

void PhyPpme_ClearOffsetErrorIntB()
{
    uint32_t reg;

    reg = ZLL->PROG_PKT_FLTR_INTR_MSK_STS;
    reg &= ~(ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_B_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_B_MASK);
    reg |= ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_B_MASK;
    ZLL->PROG_PKT_FLTR_INTR_MSK_STS = reg;
}

void PhyPpme_ClearPatternMatchIntB()
{
    uint32_t reg;

    reg  = ZLL->PROG_PKT_FLTR_INTR_MSK_STS;
    reg &= ~(ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_B_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_A_MASK |
             ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_OFST_ERR_B_MASK);
    reg |= ZLL_PROG_PKT_FLTR_INTR_MSK_STS_PKT_FLTR_IRQ_B_MASK;
    ZLL->PROG_PKT_FLTR_INTR_MSK_STS = reg;
}
