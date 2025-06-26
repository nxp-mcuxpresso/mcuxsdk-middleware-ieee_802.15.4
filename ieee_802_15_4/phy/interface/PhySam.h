/*! *********************************************************************************
* Copyright 2025 NXP
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_SAM_H__
#define __PHY_SAM_H__

#include "PhyTypes.h"

/*
 * Partition the SAM table into SAP0, SAA0, SAP1, SAA1. Therefore only three limits are
 * needed:
 * - saa0_start - first entry of saa0
 * - sap1_start - first entry of sap1
 * - saa1_start - first entry of saa1
 * sap0_start is automatically set to 0.
 *
 * Each limit is a value betwee 0 - 255. And the values should be in relation:
 * 0 <= saa0_start <= sap1_start <= saa1_start <= 255
 *
 * If a limit is equal to the previous one (or lower) the previous partition will not
 * be present. Example:
 *
 * if saa0_start = 0 then SAP0 will not exist.
 *
 * The above relation is automatically imposed by the function so there could be no
 * error. This design decision has been chosen so that the function would not return
 * anything, so we a cleaner simpler code.
 */
void PhySam_PartitionTable(uint8_t saa0_start, uint8_t sap1_start, uint8_t saa1_start);

/*
 * Write an entry in the SAM table.
 *
 * It's up to the user to make sure that the entry is in the correct section
 * (SAP0/1 or SAA0/1).
 */
void PhySam_WriteEntry(uint8_t index, uint16_t checksum);

/*
 * Read an entry from the SAM table.
 *
 * It's up to the user to make sure that the entry is from the correct section
 * (SAP0/1 or SAA0/1).
 */
void PhySam_ReadEntry(uint8_t index, uint16_t *checksum);

void PhySam_InvalidateEntry(uint8_t index);

/* Compute the checksum from addresses of mode 2. The result (checksum) is used
 * to populate the SAM table.
 */
void PhySam_ComputeMode2Checksum(uint16_t panid, uint16_t addr, uint16_t *checksum);

/* Compute the checksum from addresses of mode 3. The result (checksum) is used
 * to populate the SAM table.
 */
void PhySam_ComputeMode3Checksum(uint16_t panid, uint64_t addr, uint16_t *checksum);

/* Function that checks if the last received packet found an entry in the SAP0.
 * This function should be then used by SW (on after RX) to send an ACK with correct
 * value for FP.
 */
bool_t PhySam_Sap0AddrPresent(uint8_t *index);

/* Function that checks if the last received packet found an entry in the SAP1.
 * This function should be then used by SW (on after RX) to send an ACK with correct
 * value for FP.
 */
bool_t PhySam_Sap1AddrPresent(uint8_t *index);

/* Function that checks if the last received packet found an entry in the SAA0.
 * This function should be then used by SW (on after RX) to send an ACK with correct
 * value for FP.
 */
bool_t PhySam_Saa0AddrAbsent(uint8_t *index);

/* Function that checks if the last received packet found an entry in the SAA1.
 * This function should be then used by SW (on after RX) to send an ACK with correct
 * value for FP.
 */
bool_t PhySam_Saa1AddrAbsent(uint8_t *index);

/* Returns the first free index in the SAP0 table */
void PhySam_Sap0FirstFreeIndex(uint8_t *index);

/* Returns the first free index in the SAP1 table */
void PhySam_Sap1FirstFreeIndex(uint8_t *index);

/* Returns the first free index in the SAA0 table */
void PhySam_Saa0FirstFreeIndex(uint8_t *index);

/* Returns the first free index in the SAA0 table */
void PhySam_Saa1FirstFreeIndex(uint8_t *index);

#endif /* __PHY_SAM_H__ */
