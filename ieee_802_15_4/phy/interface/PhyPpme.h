/*! *********************************************************************************
* Copyright 2025 NXP
*
* \file Platform specific definitions. This file contains the default values for
* platform files.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_PPME_H__
#define __PHY_PPME_H__

#include "EmbeddedTypes.h"

#define PHY_PPME_FRAME_TYPE_BEACON          (1 << 0)
#define PHY_PPME_FRAME_TYPE_DATA            (1 << 1)
#define PHY_PPME_FRAME_TYPE_ACK             (1 << 2)
#define PHY_PPME_FRAME_TYPE_COMMND          (1 << 3)
#define PHY_PPME_FRAME_TYPE_RESERVED        (1 << 4)
#define PHY_PPME_FRAME_TYPE_MULTIPURPOSE    (1 << 5)
#define PHY_PPME_FRAME_TYPE_FRAK            (1 << 6)
#define PHY_PPME_FRAME_TYPE_EXTENDED        (1 << 7)

#define PHY_PPME_FRAME_VERS_0           (1 << 0)
#define PHY_PPME_FRAME_VERS_1           (1 << 1)
#define PHY_PPME_FRAME_VERS_2           (1 << 2)
#define PHY_PPME_FRAME_VERS_3           (1 << 3)

/*
 * PHY_PPME_FLAGS_REJECT_AND    - rejection happens pattern A AND B match
 * PHY_PPME_FLAGS_CONCAT        - A and B are concatenated into a combined pattern
 * PHY_PPME_FLAGS_FLOAT_OFFSET  - pattern B's offset is floating (meaning that engine B
 *                                is going to try to match the pattern B from the end of
 *                                pattern A to the end of the packet - i.e. the offset
 *                                for B is not needed)
 */
#define PHY_PPME_FLAGS_REJECT_AND       (1 << 0)
#define PHY_PPME_FLAGS_CONCAT           (1 << 1)
#define PHY_PPME_FLAGS_FLOAT_OFFSET     (1 << 2)

/*
 * Structure containing all the pattner parremeters that should be provided:
 * - pattern:       the actual pattern needed to be expected
 * - mask:          every bit of this mask that is set to '1' will be ignored
 *                  from the pattern
 * - offset:        the offset at which the pattern will be searched at;
 *                  offset = 0 is at the end of the ASH (start of the payload)
 * - frame_type:    the frame types that the pattern will be searched for
 *                  it is an or of types PHY_PPME_FRAME_TYPE_xyz
 *                  ex: if we only want to search for the pattern only in data
 *                  frames and command frames then this fields should be:
 *                  frame_type = PHY_PPME_FRAME_TYPE_DATA | PHY_PPME_FRAME_TYPE_COMMAND
 * - frame_version: the frame version of the packets we want to search the pattern
 *                  on. For example if we want to search only on frames with
 *                  version 0 and 1 then this field should be:
 *                  frame_version = PHY_PPME_FRAME_VERS_0 | PHY_PPME_FRAME_VERS_1
 * - enable_irq:    set to True if you want the matching to generate the
 *                  PKT_FLTR_IRQ_A or PKT_FLTR_IRQ_B (depening on what pattern
 *                  engine you use - A or B)
 * - packet_reject: set to True if you want on pattern mismatch to get a
 *                  FilterFail_IRQ
 * - flags:         concatenate PHY_PPME_FLAGS_xyz defines to enable specific
 *                  functionality for engine B; this field is not applicable for
 *                  engine A. See description for each flags.
 */
typedef struct {
    uint64_t    pattern;
    uint64_t    mask;
    uint8_t     offset;
    uint8_t     frame_type;
    uint8_t     frame_version;
    uint8_t     enable_irq;
    uint8_t     packet_reject;
    uint8_t     flags;
} ppme_pattern_t;

/* Enable Pattern Matching Register Set A.
 *
 * Description for each parameter is at the structure level
 *
 * NOTE: Current HW implementation has an issue: if you want to only
 *   use a matching engine (A or B) you should enable both and then
 *   disable the one you don't want to use.
 */
void PhyPpme_EnablePatternA(ppme_pattern_t *set);

void PhyPpme_DisablePatternA();

/* Enable Pattern Matching Register Set B
 *
 * Description for each parameter is at the structure level
 *
 * NOTE: Current HW implementation has an issue: if you want to only
 *   use a matching engine (A or B) you should enable both and then
 *   disable the one you don't want to use.
 */
void PhyPpme_EnablePatternB(ppme_pattern_t *set);

void PhyPpme_DisablePatternB();

void PhyPpme_ClearOffsetErrorIntA();

void PhyPpme_ClearPatternMatchIntA();

void PhyPpme_ClearOffsetErrorIntB();

void PhyPpme_ClearPatternMatchIntB();

#endif /* __PHY_PPME_H__ */
