/*! *********************************************************************************
* Copyright 2025 NXP
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include "PhySam.h"
#include "fsl_device_registers.h"


void PhySam_PartitionTable(uint8_t saa0_start, uint8_t sap1_start, uint8_t saa1_start)
{
    uint32_t reg = ZLL->SAM_CTRL;

    /* clear pre-existing partition (if any) */
    reg &= ~(ZLL_SAM_CTRL_SAA0_START_MASK |
             ZLL_SAM_CTRL_SAP1_START_MASK |
             ZLL_SAM_CTRL_SAA1_START_MASK |
             ZLL_SAM_CTRL_SAP0_EN_MASK |
             ZLL_SAM_CTRL_SAA0_EN_MASK |
             ZLL_SAM_CTRL_SAP1_EN_MASK |
             ZLL_SAM_CTRL_SAA1_EN_MASK);

    /* Impose restrictions on the start of each partition */
    sap1_start = MAX(sap1_start, saa0_start);   /* Force SAP1_START >= SAA0_START */
    saa1_start = MAX(saa1_start, sap1_start);   /* Force SAA1_START >= SAP1_START */

    /* Write all the partition starts in the register; it doesn't matter if the partition
     * is enabled or not; if it is not it will not use the coresponding partition start
     */
    reg |= (ZLL_SAM_CTRL_SAA0_START(saa0_start) |
            ZLL_SAM_CTRL_SAP1_START(sap1_start) |
            ZLL_SAM_CTRL_SAA1_START(saa1_start));

    /* enable conditions for each paritition */
    if (saa0_start > 0)                 /* then SAP0 is enabled */
    {
        reg  |= ZLL_SAM_CTRL_SAP0_EN(1);
    }

    if (sap1_start > saa0_start) /* then SAA0 is enabled */
    {
        reg |= ZLL_SAM_CTRL_SAA0_EN(1);
    }

    if (saa1_start > sap1_start) /* then SAP1 is enabled */
    {
        reg |= ZLL_SAM_CTRL_SAP1_EN(1);
    }

    /* NOTE: the only condition to disable SAA1 partition is to make it 1 entry long */
    if (saa1_start != 0xFF) /* then SAA1 is always enabled */
    {
        reg |= ZLL_SAM_CTRL_SAA1_EN(1);
    }

    ZLL->SAM_CTRL = reg;

    /* Invalidate all the entries just for good measure */
    reg  = ZLL->SAM_TABLE;
    reg &= ~ZLL_SAM_TABLE_INVALIDATE_ALL_MASK;
    reg |= ZLL_SAM_TABLE_INVALIDATE_ALL(1);
    ZLL->SAM_TABLE = reg;
}

void PhySam_WriteEntry(uint8_t index, uint16_t checksum)
{
    uint32_t reg = ZLL->SAM_TABLE;

    reg &= ~(ZLL_SAM_TABLE_SAM_INDEX_MASK |
             ZLL_SAM_TABLE_SAM_CHECKSUM_MASK |
             ZLL_SAM_TABLE_SAM_INDEX_WR_MASK |
             ZLL_SAM_TABLE_SAM_INDEX_EN_MASK |
             ZLL_SAM_TABLE_SAM_INDEX_INV_MASK);

    reg |= (ZLL_SAM_TABLE_SAM_INDEX(index) |
            ZLL_SAM_TABLE_SAM_CHECKSUM(checksum) |
            // ZLL_SAM_TABLE_SAM_INDEX_WR(1) |
            0x40000000 |
            ZLL_SAM_TABLE_SAM_INDEX_EN(1));

    ZLL->SAM_TABLE = reg;
}

void PhySam_ReadEntry(uint8_t index, uint16_t *checksum)
{
    uint32_t reg = ZLL->SAM_TABLE;

    reg &= ~(ZLL_SAM_TABLE_SAM_INDEX_MASK |
             ZLL_SAM_TABLE_SAM_INDEX_WR_MASK);
    reg |= ZLL_SAM_TABLE_SAM_INDEX(index);

    ZLL->SAM_TABLE = reg;

    *checksum = ((ZLL->SAM_TABLE & ZLL_SAM_TABLE_SAM_CHECKSUM_MASK) >> ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT);
}

void PhySam_InvalidateEntry(uint8_t index)
{
    uint32_t reg = ZLL->SAM_TABLE;

    ZLL->SAM_TABLE = reg;
}

void PhySam_ComputeMode2Checksum(uint16_t panid, uint16_t addr, uint16_t *checksum)
{
    *checksum = panid + addr;
}

void PhySam_ComputeMode3Checksum(uint16_t panid, uint64_t addr, uint16_t *checksum)
{
    *checksum = panid + (uint16_t)(addr) + (uint16_t)(addr >> 16) + (uint16_t)(addr >> 32) + (uint16_t)(addr >> 48);
}

bool_t PhySam_Sap0AddrPresent(uint8_t *index)
{
    bool_t is_present = !!((ZLL->SAM_ADDR_PRESENT & ZLL_SAM_ADDR_PRESENT_SAP0_ADDR_PRESENT_MASK) >>
                                                  ZLL_SAM_ADDR_PRESENT_SAP0_ADDR_PRESENT_SHIFT);

    if (is_present)
    {
        *index = ((ZLL->SAM_MATCH & ZLL_SAM_MATCH_SAP0_MATCH_MASK) >> ZLL_SAM_MATCH_SAP0_MATCH_SHIFT);
    }

    return is_present;
}

bool_t PhySam_Sap1AddrPresent(uint8_t *index)
{
    bool_t is_present = !!((ZLL->SAM_ADDR_PRESENT | ZLL_SAM_ADDR_PRESENT_SAP1_ADDR_PRESENT_MASK) >>
                                                  ZLL_SAM_ADDR_PRESENT_SAP1_ADDR_PRESENT_SHIFT);

    if (is_present)
    {
        *index = ((ZLL->SAM_MATCH | ZLL_SAM_MATCH_SAP1_MATCH_MASK) >> ZLL_SAM_MATCH_SAP1_MATCH_SHIFT);
    }

    return is_present;
}

bool_t PhySam_Saa0AddrAbsent(uint8_t *index)
{
    bool_t is_absent = !!((ZLL->SAM_ADDR_PRESENT | ZLL_SAM_ADDR_PRESENT_SAA0_ADDR_ABSENT_MASK) >>
                                                 ZLL_SAM_ADDR_PRESENT_SAA0_ADDR_ABSENT_SHIFT);

    if (!is_absent)
    {
        *index = ((ZLL->SAM_MATCH | ZLL_SAM_MATCH_SAA0_MATCH_MASK) >> ZLL_SAM_MATCH_SAA0_MATCH_SHIFT);
    }

    return is_absent;
}

bool_t PhySam_Saa1AddrAbsent(uint8_t *index)
{
    bool_t is_absent = !!((ZLL->SAM_ADDR_PRESENT | ZLL_SAM_ADDR_PRESENT_SAA1_ADDR_ABSENT_MASK) >>
                                                 ZLL_SAM_ADDR_PRESENT_SAA1_ADDR_ABSENT_SHIFT);

    if (!is_absent)
    {
        *index = ((ZLL->SAM_MATCH | ZLL_SAM_MATCH_SAA1_MATCH_MASK) >> ZLL_SAM_MATCH_SAA1_MATCH_SHIFT);
    }

    return is_absent;
}

static void force_free_index_refresh()
{
     uint32_t reg = ZLL->SAM_TABLE;
     reg |= ZLL_SAM_TABLE_FIND_FREE_IDX_MASK;
     ZLL->SAM_TABLE = reg;
}


void PhySam_Sap0FirstFreeIndex(uint8_t *index)
{
    force_free_index_refresh();

     *index = ((ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_MASK) >>
                                    ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_SHIFT);
}

void PhySam_Sap1FirstFreeIndex(uint8_t *index)
{
    force_free_index_refresh();

     *index = ((ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_MASK) >>
                                    ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_SHIFT);
}

void PhySam_Saa0FirstFreeIndex(uint8_t *index)
{
    force_free_index_refresh();

     *index = ((ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_MASK) >>
                                    ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_SHIFT);
}

void PhySam_Saa1FirstFreeIndex(uint8_t *index)
{
    force_free_index_refresh();

     *index = ((ZLL->SAM_FREE_IDX & ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_MASK) >>
                                    ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_SHIFT);
}
